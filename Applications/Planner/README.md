# Planner 路径规划模块

> 面向 T-Car（STM32F407 + 麦轮底盘 + OPS 平面定位）的全局路径规划模块。
> 车体是 **30cm × 30cm 方形**（半对角线 21.22cm），本版本按方形车体建模：
> 平移段固定朝向、只在"旋转安全点"原地转体，避免方形车在 40cm/35cm 通道里
> 旋转扫入禁区（30×30 方形转 45° 需要 42.43cm 宽，物理不可能）。
> 原地旋转的支点由底盘运动学决定（`Applications/chassis/chassis_config.h` 的
> `CHASSIS_ROTATION_CENTER_OFFSET_X/Y`，须等于导航位姿点 G）；G 默认是 OPS 位姿点，
> 可用 `CHASSIS_BODY_REF_OFFSET_X/Y` 把 G 改为车体参考点（推荐 = 车体几何中心），
> 车体不对称/有凸出时还需按实测配置本文件的 `PLANNER_ROTATION_SWEEP_RADIUS_CM`，见 §3.2。

## 1. 目录结构

```
Applications/Planner/
├── planner_switch.h          # 规划器总开关（PLANNER_ENABLED）
├── planner_config.h          # 全部可调参数（地图/方形几何/旋转安全/A* 代价/执行参数）
├── planner_port.h/.c         # 移植层：定位输入、规划调度、航点+旋转执行、执行期安全监测
├── utils/
│   ├── Cstar_map_core.h/.c   # 地图层：栅格位图、预定义障碍矩形、欧氏清障场、方形带符号净空场
│   ├── planner_algo.h/.c     # 算法层：方形净空 A*、路径连续化/净空优化、旋转方案、校验
│   └── priority_queue.h/.c   # A* 最小堆（打包 uint32 比较，零动态内存）
├── tools/planner_tools.c     # shell 调试命令（planner 命令组）
├── pytools/planner_viewer.py # PC 端路径可视化（粘贴 dry 日志即可画图）
└── tests/                    # 主机端测试/基准（gcc 编译，不参与固件构建）
```

分层原则：`algo` / `map_core` 是纯逻辑（不依赖 FreeRTOS/导航），可 PC 单元测试；
只有 `planner_port` 接触 RTOS、导航、定位和 shell。

## 2. 规划流水线

```
Planner_GoTo_fromName(name) / Planner_GoToDirect(target)
        │  (阻塞式：一次请求 = 一条路径 + 旋转方案 + 整条执行)
        ▼
planner_run_once()
  1. Loc_Get() 读定位（失败重试 50×100ms）得到起点 x,y,yaw
  2. 场地坐标 (0.01m) → 地图坐标（+MAP_OFFSET）
  3. Planner_PlanPathWithYaw(start, goal, 当前yaw, 目标yaw)   ← planner_algo.c
       a. MapCore_RebuildClearance()：重建欧氏清障场 + 方形带符号净空场
          （吸收雷达动态障碍；HOME 等贴边点的格中心值可能为负，靠端点容差/反向搜索收尾处理）
       b. 按朝向计算方形投影半长 H(yaw)=15*(|cos|+|sin|) ∈ [15, 21.22]cm
       c. 分级尝试：soft（方形净空 ≥4cm，40cm 通道居中恰好 5cm）
                    → hard（采样门限 ≥2cm，35cm 通道居中 2.5cm；1cm 采样+0.25cm 校验容差，真实净空约 ≥1.5cm，留跟踪误差余量）
          每个级别：A*（搜索净空=0 保栅格连通，贴障碍高代价）
                   → 路径连续化+逐点净空优化（把窄通道里的折线推到通道中心）
                   → 直线化（按 tier required 校验每段）
                   → 逐段采样方形净空校验（一般路段允许比 required 低 0.5cm，
                     端点 25cm 豁免区允许更低，见安全模型）
       d. 旋转方案：若目标 yaw 与当前 yaw 差 >1.5°：
            - 起点本身旋转安全（欧氏清障 ≥23.22cm）→ 起点先转好，之后按目标朝向平移；
            - 否则在路径上找第一个旋转安全点插入（路径顶点）；路径上没有安全点
              → 在地图里找附近安全点作为 via 绕行点，规划 起点→via→终点 两段；
            - 都找不到 → 规划失败 fail_reason=no-safe-rotation-point（不会冒险旋转）
          stats 输出 rotation_index/heading_before/heading_after/旋转点清障等
  4. 执行（planner_port.c）：
       - rotation_index=0：先在起点原地转到 heading_after；
       - 平移航点按阶段下发：旋转点之前的航点用 heading_before，之后用 heading_after
         （麦轮平移/蟹行，航段内不转体）；
       - 到旋转点后原地转到 heading_after（该点规划时已验证旋转安全）；
       - 每 10ms 执行期监测：轨迹偏离 >10cm、平移阶段按实际 yaw 的方形净空过低、
         旋转阶段欧氏清障 <21.72cm（半对角线 21.22 + 0.5cm 扫掠包络）→ 立即停车并重规划；
         原地旋转角速度降到 90°/s 减小漂移；旋转阶段不用轴对齐保守方形监测
         （该模型在 45° 会误报，真实旋转扫掠是半径 21.22cm 的圆）；
       - 剩余路程进入 50cm 减速区后逐点降速到 40cm/s；tight 路线限速 80cm/s。
  5. 终点精度校验：位置 ±1.2cm、朝向 ±2.5°；
       - 目标点旋转安全且需要时才原地补 yaw；目标点旋转不安全（如 STORAGE/
         PROCESSING）不会原地补转，日志给 warning（平移段已按目标朝向保持）。
  6. 任一航点失败/安全告警 → 从当前位姿重新规划（PLANNER_EXEC_RETRY_MAX 次）。
```

## 3. 安全模型（方形车体 30×30）

| 层级 | 约束 | 实现位置 |
| --- | --- | --- |
| 平移碰撞（硬） | 方形车体中心不得进入"障碍矩形按半长 H 在 x/y 各外扩"的中心禁区；轴对齐朝向（0/90/180/270°）H=15cm 精确，任意朝向用保守投影 H(ψ)≤21.22cm。采样门限：一般路段方形净空 ≥2cm（1cm 采样 + 0.25cm 校验容差，真实净空约 ≥1.5cm，留实车跟踪误差） | `MapCore_PointSquareClearanceCm/SquareSegmentFree`（A* 节点/边、路径直线化、最终校验） |
| 平移碰撞（软） | 尽量保持方形净空 ≥4cm（40cm 通道居中=5cm，35cm 通道=2.5cm 故判 tight） | A* 清障平方惩罚（`PLANNER_CLEARANCE_PENALTY_MAX`） |
| 原地旋转 | 旋转扫过所有朝向，需要中心欧氏清障 ≥ 旋转扫掠半径 + 2cm（默认 21.22+2 = **23.22cm**；车体有凸出时把 `PLANNER_ROTATION_SWEEP_RADIUS_CM` 按实测最大外缘半径配置）。旋转中心由底盘运动学配置（`chassis_config.h`）为 OPS 位姿点。40cm 通道内禁止旋转（方形车 45° 位姿需要 42.43cm 宽）。旋转时降速 90°/s，执行期按"扫掠半径+0.5cm"的扫掠包络监测（该包络覆盖任意朝向，不再用轴对齐保守方形误判 45°） | `MapCore_RotationSafe`、算法层旋转点选择、执行期旋转阶段监测 |
| 端点豁免 | 任务点由系统给定、规划器不能改（实测 HOME/STORAGE/PROCESSING 方形净空=0 恰好相切）。采样点到起终点欧氏距离 ≤25cm 时允许端点容差（hard: 0，允许最多 1cm 模型侵入吸收定位误差；soft: 2cm） | `Planner_SquarePathValidate`、A* 端点邻域、执行期端点监测 |
| 执行兜底 | 轨迹偏离 / 实际 yaw 的方形净空过小 / 旋转阶段清障不足 / 导航状态机卡死（规划器侧硬超时）→ 停车 + 重规划；目标点不安全时拒绝原地补转 | `wait_waypoint()`、`verify_goal_precision()` |

精度/安全设计要点：

- **方形是危险的是"角"**：圆形半径 15cm 模型在障碍角处低估了 6.2cm（半对角线
  21.22 vs 15），旧模型会贴着角走/在角附近转体。现在碰撞用 Minkowski 扩张精确
  判定，拐角/窄缝/旋转都按方形算。
- **绝不在窄通道内旋转**：旋转安全点要求欧氏清障 ≥23.22cm；40cm 通道中心清障
  20cm（不满足）、35cm 通道 17.5cm（不满足），所以旋转只会发生在开阔区；
  需要中途换朝向时算法会绕到安全点（必要时 via 绕行）再转。
- **分级退化**：起终点本身贴障碍/窄通道导致软目标不可达时，退化为 tight 路线
  （采样门限 2cm、1cm 采样），执行时限速；30cm 缝（方形车净空=0，只能"刚好塞满"）不会
  被当作正常通道——A* 高代价绕行；若确实无安全路线则规划失败而不是冒险通过。
- **两段式终点**：平移不转体，到点后再按需对齐 yaw；目标点旋转不安全时不做
  原地补转（否则会扫入障碍），以日志 warning 提示朝向误差。
- **精度**：路径拐点连续化优化后仍落在自由空间；终点位置用精确坐标 + 微调，
  平移段固定朝向减少麦轮打滑/耦合。

### 3.1 场地几何事实（重要）

| 几何 | 数值 | 含义 |
| --- | --- | --- |
| 车体 30×30cm | 半长 15cm，半对角线 21.22cm | 默认旋转扫掠半径 21.22cm（车体有凸出时改 `PLANNER_ROTATION_SWEEP_RADIUS_CM`） |
| 40cm 通道 | 居中时两侧各 5cm；可允许的最大偏航约 25.5°（再大方形角扫到墙） | 通道内不能旋转，但任意 90° 倍数朝向可直行 |
| 35cm 通道 | 居中两侧各 2.5cm | hard tier（tight）可通过，不能旋转 |
| 30cm 缝隙 | 方形 30cm 刚好塞满，净空=0 | 视为危险缝隙，A* 会绕行；不建议作为常规通道 |
| 任务点方形净空（zone1/2） | HOME=0、QrCode=10、MATL_GRAB=6、MATL_STORAGE=0、MATL_PROCESSING=0（cm，刚好相切） | 端点靠容差进出；终点朝向我向平移段保证，不原地补转 |

### 3.2 旋转中心与底盘运动学的配合（车体不对称/有凸出时必读）

原地旋转的安全判定与执行监测都以 **旋转中心** 为圆心，而旋转中心由底盘运动学决定
（`Applications/chassis/chassis_config.h` 的 `CHASSIS_ROTATION_CENTER_OFFSET_X/Y`，
默认 0 = 四轮几何中心）。导航/规划使用的车体位姿点 G：默认等于 OPS 位姿点，可用
`CHASSIS_BODY_REF_OFFSET_X/Y`（G 相对 OPS 点的偏移）改为车体参考点；任务点坐标、
30×30 方形平移模型、旋转安全判定全部以 G 为中心。因此要求：

1. **旋转中心 = 导航位姿点 G**：推荐把 G 配置成 **车体几何中心**（这样旋转中心=车身中心、
   扫掠最小、车体不整体平移）。运动学 `CHASSIS_ROTATION_CENTER_OFFSET_X/Y` 必须等于 G−O；
   两者不一致时原地转体会让位姿点画圈、车体整体平移，位置环与旋转指令互相拉扯。
   测量/配置见 `Applications/chassis/README.md` §4.2/§4.3/§4.5。
2. **扫掠半径按真实外廓配置**：`PLANNER_ROTATION_SWEEP_RADIUS_CM` 取"导航位姿点/旋转中心 G
   到车体最外缘（含凸出部分、机械臂、外壳）的最大距离"，不是 30×30 方形半对角线。
   例：车体（含凸出）长 34cm、宽 30cm，旋转中心在几何中心 → `sqrt(17²+15²) ≈ 22.67cm`，
   对应旋转安全欧氏门限从 23.22cm 提高到 24.67cm。`Planner_Init` 启动日志会打印
   `rotSweep/rotClear`，现场可直接核对生效值。
3. 该半径只影响 **原地旋转**；平移碰撞仍按 `PLANNER_CAR_SIDE_CM` 的方形模型计算。
   若车体实际外廓明显大于 30×30（尤其凸出方向要平移过窄通道），需同步评估
   `PLANNER_CAR_SIDE_CM`（会整体变保守）或调整任务点位/通道走向。

## 4. 关键参数（planner_config.h）

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `PLANNER_CAR_SIDE_CM` | 30.0 | 方形车体边长（平移碰撞模型） |
| `PLANNER_BODY_HALF_CM` | 15.0 | 半边长（推导） |
| `PLANNER_BODY_HALF_DIAGONAL_CM` | 21.22 | 30×30 方形半对角线（旋转扫掠半径默认值） |
| `PLANNER_SAFE_MARGIN_CM` | 4.0 | soft 方形净空目标（40cm 通道=5cm 刚好） |
| `PLANNER_SQUARE_HARD_MARGIN_CM` | 2.0 | hard 采样门限（实车 1cm 门限 + 2cm 采样时路径净空仅 ~1cm，跟踪误差 0.7cm 就触发执行期急停；提高到 2cm 并改 1cm 采样） |
| `PLANNER_ENDPOINT_SQUARE_TOL_CM` | 2.0 | 端点容差（hard 端点允许到 0） |
| `PLANNER_ENDPOINT_PENETRATION_TOL_CM` | 1.0 | 端点豁免区最多允许 1cm 模型侵入（吸收定位误差） |
| `PLANNER_ENDPOINT_ZONE_CM` | 25.0 | 端点豁免区半径（到起终点欧氏距离） |
| `PLANNER_VALIDATION_TOL_CM` | 0.25 | 一般路段采样校验容差（1cm 采样） |
| `PLANNER_ROTATION_CLEARANCE_CM` | 23.22 | 原地旋转所需欧氏清障（扫掠半径+2cm；默认半对角线+2cm） |
| `PLANNER_ROTATION_SWEEP_RADIUS_CM` | = 半对角线 21.22 | 原地旋转扫掠半径（cm）：旋转中心/OPS 位姿点到车体最外缘最大距离，车体有凸出时按实测填（见 §3.2） |
| `PLANNER_ROTATION_MARGIN_CM` | 2.0 | 旋转安全余量 |
| `PLANNER_ROTATION_SPEED_DEG_S` | 90 | 原地旋转角速度（比平移 220°/s 慢，减小旋转漂移） |
| `PLANNER_ROTATION_ACCEL_DEG_S2` | 180 | 原地旋转角加速度 |
| `PLANNER_ROTATION_MONITOR_EXTRA_CM` | 0.5 | 旋转阶段监测余量（中心清障 < 扫掠半径+0.5cm 立即停车） |
| `PLANNER_HEADING_DEADBAND_DEG` | 1.5 | yaw 差小于此值不旋转 |
| `PLANNER_GOAL_YAW_ALIGN_IF_SAFE` | 1 | 仅目标点旋转安全时允许原地补 yaw |
| `PLANNER_ROTATION_TIMEOUT_BASE_MS` / `_PER_DEG_MS` | 1500 / 25 | 原地旋转超时估算 |
| `PLANNER_CLEARANCE_PENALTY_MAX` | 200 | 贴障碍代价上限（×10），越大越不愿钻 30cm 缝 |
| `PLANNER_REFINE_SEG_CM` / `_MAX_OFFSET_CM` / `_STEP_CM` / `_MAX_PASSES` / `_TARGET_EXTRA_CM` | 10 / 6 / 2 / 3 / 1.0 | 路径连续化/逐点净空优化参数（目标 = tier 要求 + 1cm） |
| `PLANNER_PATH_SAMPLE_STEP_CM` | 1.0 | A* 边/直线化/最终校验采样步长（2cm 会漏掉窄通道 0.7cm 净空低谷） |
| `PLANNER_WAYPOINT_SPEED_CM_S` | 120 | 巡航速度 |
| `PLANNER_FINAL_SPEED_CM_S` | 40 | 终点减速速度 |
| `PLANNER_SLOWDOWN_DIST_CM` | 50 | 进站减速区长度 |
| `PLANNER_FINAL_ARRIVE_DIST_CM` | 1.0 | 终点位置阈值 |
| `PLANNER_FINAL_ALIGN_YAW_DEG` | 2.0 | 原地旋转/终点朝向阈值 |
| `PLANNER_FINAL_POS_TOL_CM` / `_YAW_TOL_DEG` | 1.2 / 2.5 | 终点精度校验阈值 |
| `PLANNER_TRACK_DEVIATION_CM` | 10.0 | 执行期轨迹偏离告警（0=关闭） |
| `PLANNER_TRACK_MIN_SQUARE_CLEARANCE_CM` | 0.3 | 平移阶段执行期方形净空告警（端点豁免区允许到 -1cm）；旋转阶段不查该指标，改用旋转扫掠包络（21.72cm） |
| `PLANNER_EXEC_RETRY_MAX` | 1 | 执行失败后重规划次数 |

### 调参建议

1. **先确认车体真实尺寸/旋转中心**：`PLANNER_CAR_SIDE_CM` 按平移外廓实测填写；
   原地旋转中心与扫掠半径按 `Applications/chassis/README.md` §4/§5 标定后填
   `CHASSIS_ROTATION_CENTER_OFFSET_X/Y` 与 `PLANNER_ROTATION_SWEEP_RADIUS_CM`。
   改大 1cm，40cm 通道的可旋转余量会明显变小。
2. 通道较宽（≥50cm）：保持 soft 4cm 目标；需要更快可先降 `PLANNER_SAFE_MARGIN_CM`
   到 2~3（仍保持 hard 门限 2cm 不变），不要动方形尺寸。
3. 现场定位噪声大（终点实测误差 >2cm）：先看日志 `final pose error`；端点
   `PLANNER_ENDPOINT_PENETRATION_TOL_CM` 可临时放宽，但不要用"放大到达阈值"掩盖。
4. 某点位规划失败：用 `planner clr x y` 查欧氏+方形净空；若方形净空为负或 0，
   说明点位压到方形中心禁区，应改点位（往开阔侧挪 2~3cm）而不是放宽规划器。
5. 需要改变朝向但日志报 `no-safe-rotation-point`：说明路径附近确实没有
   ≥23.22cm 的开阔区，不要放宽旋转门限——那等于允许方形车在通道里扫墙。

## 5. 调试命令（串口 shell，`planner` 命令组）

| 命令 | 说明 |
| --- | --- |
| `planner map` | 打印地图栅格：`#` 静态障碍、`@` 动态障碍、`.` 空闲 |
| `planner clr <x_cm> <y_cm>` | 查询该点欧氏清障 + 方形净空（30×30 轴对齐，cm） |
| `planner set_obs <x_cm> <y_cm> <0|1>` | 手动添加/清除动态障碍格（会立即重建距离场） |
| `planner stat` | 最近一次规划/执行报告：航点数/路径长/方形净空/欧氏清障/tier/fail_reason/旋转方案/耗时/终点误差 |
| `planner dry <id>` / `planner dry x y yaw` | 空跑：只规划并打印旋转方案与每个航点的方形净空，不下发导航 |
| `planner to <id>` / `planner go x y yaw` | 规划并执行到点位/坐标 |
| `planner init <1|2>` | 重新载入启动区地图（规划器空闲时） |
| `lcd plan_view [thick|thin]` | 在 LCD 上显示地图 + 最近一次路径 |

`dry` 日志可直接粘贴到 `pytools/planner_viewer.py` 画路径图。

## 6. 测试与验证

### 6.1 主机端回归（gcc，不依赖硬件）

`tests/` 下提供独立于实现的几何真值校验器、基准/回归/单测程序（详见 `tests/README.md`）：

- `baseline_report_v2.md`：旧版（git HEAD）40 条任务点对路径在**方形车体精确 SAT**
  度量下的结果：**26/40 条穿障、最大穿透 6.52cm、14 条需要终点补转的路径中 10 条
  终点旋转不安全**（旧圆模型/终点原地转对 30×30 方形车是危险的）；
- `after_report_v2.md`：v2 对比：40/40 规划成功、一般路段方形 SAT 碰撞 **0**、
  最大穿透 **0.000cm**、端点豁免区侵入超限 0、旋转方案 14 个（起点 4/中途 6/终点 4）
  且旋转点独立欧氏清障最小 **23.50cm**（≥23.22）、soft 4/tight 36、
  平均路径长 231.7cm（旧 179.1cm，+29%：绕开 30cm 缝/留出方形角安全余量）、
  平均/最大规划耗时 1.48ms/35.60ms（主机 -O2）；
- `new_runner`（7 PASS/0 FAIL）、`edge_runner`（23 PASS/0 FAIL，含 40/35/30cm 通道、
  任意 yaw、三种旋转分支、动态障碍、确定性、零长特例）、`profile_runner`
  （16 PASS/0 FAIL）全部通过；C 侧与 Python 独立 SAT 交叉校验 40 条路径 0 不一致；
- 30cm 缝语义：全段都在端点豁免区（≤25cm）时可 tight 相切通过；长段按安全优先
  clean fail（不冒险），这是方形车 30×30 与 30cm 缝的真实几何冲突。

构建产物放 `%TEMP%`，不进固件构建。

### 6.2 固件编译

Planner 文件属于 EIDE/MDK 工程，正常编译即可（AC5，C99）。本次修改文件：

- 修改文件：`planner_config.h`、`Cstar_map_core.h/.c`、`planner_algo.h/.c`、
  `planner_port.h/.c`、`tools/planner_tools.c`；
- 集成侧小改动：`Drivers/User/RPI/rpi_port.c`（雷达格写为动态障碍类型）、
  `Drivers/User/LCD/tools/lcd_tools.c`（显示动态障碍；路径宽度按半车宽）、
  `Core/Src/freertos.c`（Planner 任务优先级 Realtime2 → **High2**，低于电机控制
  High4/电机接收 High3，保证规划计算不饿死运动控制/急停链路）；
- 构建配置清理：移除 `.eide/eide.yml` 与 `MDK-ARM/Hurri-Cargo_Code.uvprojx` 中已删除的
  `path_smoother.c` 残留条目；
- 全量 AC5 重编 112 个 .c（0 error/0 warning，Planner 5 文件独立编译 0 warning）+
  armlink 链接通过：ROM **213.75KB / 1MB**，RAM **110.93KB / 128KB**
  （IRAM1 105952/114688 = 92.4%，IRAM2 7640/16384）。

### 6.3 实车验证清单（上车前，重要）

1. `planner map` 确认障碍布局与实际场地一致（启动区 1/2 分别确认）。
2. `planner clr` 检查每个任务点的方形净空：HOME/STORAGE/PROCESSING 当前为 0
   （刚好相切，启动时 Planner_Init 会打 warning）；若现场摆放/定位有偏差会判
   ENDPOINT 失败或执行告警；**建议把这些点位往自由空间挪 2~3cm**
   （改 `nav_map_data.c` 的点位，不是改规划器）。
3. `planner dry <点>` 逐点检查旋转方案与航点方形净空；确认中途旋转点都在开阔区
   （日志 `rotate@wpN ... clear=...` 应 ≥23.22cm；`rotation=none` 或 `rotate@wp0` 正常）。
4. 低速（先把 `PLANNER_WAYPOINT_SPEED_CM_S` 调到 60~80）跑完整任务流程，观察
   日志 `square clearance`、`rotation unsafe`、`track deviation`、`final pose error`，
   并实测最坏规划耗时（主机 avg 1.48ms/max 35.60ms；MCU 现场看 `plan=...ms` 日志，
   若超过 ~300ms 再评估进一步优化 via 候选数/refine 参数）。
5. 重点观察需要换 yaw 的段（zone1 HOME↔STORAGE/PROCESSING 等）：车应先到旋转
   安全点停稳、原地转、再直行/蟹行，不能在 40cm/35cm 通道里边走边转。
6. 确认无误后再逐步恢复速度、收紧终点容差。

## 7. 已知限制

- 地图为静态预定义障碍 + 可选的雷达动态障碍；没有实时避障传感器闭环（雷达默认
  关闭，`RPI_RADAR_ENABLED=0`）。动态障碍依赖树莓派雷达扫描后写入地图。
- 定位依赖 OPS；定位丢失时规划任务按失败退出（不会用旧位姿冒险）。
- 30cm 缝隙对 30×30 方形车只能"刚好塞满"（净空=0），规划器会按危险区域绕行；
  如果场地确实必须穿过 30cm 缝，需要更小的车体/更小的方形尺寸参数，或人工引导
  对齐，不能靠规划器保证安全余量。
- 栅格分辨率 10cm：A* 只能给出格中心折线，窄通道（35cm→2.5cm 净空）依赖
  "连续化+逐点净空优化"把折线推到通道中心；若通道中心带 <2cm（约 32cm 通道），
  优化后可能仍达不到 hard 门限而规划失败（安全优先，不冒险）。
- 路径会比旧版明显变长：方形车不能穿 30cm 缝（净空=0）、旋转要绕到开阔区，
  40 条任务点对平均长度 231.7cm（旧 179.1cm，+29%），个别点对（如
  MATL_PROCESSING↔MATL_GRAB）从 176cm 增到 ~356cm；这是安全余量的代价。
- 规划计算量：主机 -O2 平均 1.48ms、最坏 35.60ms（via 绕行）；MCU 预计几十~几百 ms，
  Planner 任务优先级已降到电机控制之下，规划期间不会阻塞运动控制/急停链路，
  但会拉长"请求→开始移动"的响应；现场用 `planner stat` 的 `plan=...ms` 观察。
- 任意非 90° 倍数的出发 yaw：若起点本身不能安全旋转且通道较窄，可能规划失败
  （fail-safe）；正常任务链路每个点位 yaw 都是 90° 倍数，不会触发。
- 任务点方形净空为 0（HOME/STORAGE/PROCESSING）时依赖端点容差与"终点不补转"；
  实车定位/摆放误差大时会触发 ENDPOINT 失败或执行期安全停车——这是模型与场地
  尺寸的真实冲突，建议改点位或按实测车体尺寸调 `PLANNER_CAR_SIDE_CM`。
- 平移碰撞模型是 30×30 对称方形，不区分车体凸出的具体方向；车体实际外廓不对称时
  该模型偏乐观。旋转侧可先按 §3.2 配置 `PLANNER_ROTATION_SWEEP_RADIUS_CM` 兜住
  "原地转体扫掠"；若凸出方向还要平移过窄通道，需把 `PLANNER_CAR_SIDE_CM` 往大改
  （整体变保守）或避开相关通道，不能只靠旋转半径配置。
- 执行期监测是最后兜底，不能替代机械/电气急停。

## 8. 相对旧实现的主要改动

| 问题（旧实现：半径 15cm 圆） | 修复（v2：30×30 方形） |
| --- | --- |
| 圆模型在障碍角处低估 6.2cm：方形半对角线 21.22cm，旧模型贴着角走/角附近转体 | 方形 Minkowski 扩张精确碰撞；拐角按方形算 |
| 终点原地转 yaw：30×30 方形在 40cm/35cm 通道旋转需要 42.43cm 宽，物理不可能，旧版会扫入禁区 | 旋转只在欧氏清障 ≥23.22cm 的安全点进行；平移段固定朝向，先转后走/中途安全点转 |
| 40cm 中间过道里"边走边转" | 麦轮固定朝向平移/蟹行；路径上无安全点则 via 绕行到开阔区再转 |
| 清障模型是圆，窄通道量化后可能贴边 | A* 搜索用 0 净空保连通 → 路径连续化+逐点净空优化 → 按 tier 方形净空校验 |
| 30cm 缝被当成普通通道 | 方形车净空=0 判高代价绕行；无安全路线则失败而不是冒险 |
| 执行期只监测欧氏清障 | 按实际 yaw 的方形净空监测 + 旋转阶段扫掠半径监测（可配置，默认 21.22cm）+ 端点豁免 |
| 终点无条件原地补 yaw | 目标点旋转不安全（STORAGE/PROCESSING）时拒绝原地补转，只用平移段朝向保证 |
| 旧版 `(int)(0.15/0.1)=1` 半径截断、按整格膨胀等历史问题 | 保留 v1 的精确几何/距离场/分级退化/航点剖面纯函数修复 |
