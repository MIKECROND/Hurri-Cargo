# 规划器评测 / 回归工具（tests/harness）

本目录是**独立于 Planner 实现**的主机评测与回归工具，分两条链路：

1. **重构前基线**：从 git 提交 `6413fac` 导出旧源码快照，在 PC 上复现旧版 `Planner_Init + astar_plan + simplify_path`
   的完整流程，量化基线表现（见 [baseline_report.md](baseline_report.md)）；
2. **重构版评测**：直接编译工作区的 `MapCore_Init + Planner_PlanPath`，用**同一套独立几何真值指标**重跑
   同样的 40 条有向点对 + 边界/回归用例，并补一组**执行剖面纯函数单元测试**（执行层段长/剩余路程 Blocker 回归），
   生成前后对比报告（见 [after_report.md](after_report.md)）。

> 工具只复制**障碍物世界坐标数据**（`reference_obstacles.h` / 旧 `planner_config.h`），几何校验完全独立实现，
> 不调用 Planner 的网格/清障距离场判定，避免“用被测实现自证”。
> `new_runner.c` 启动时还会逐格校验“独立障碍数据 vs MapCore 清障场”，偏差 >0.1cm 直接判失败，防止两边数据脱节。
>
> 所有输出都写到 `%TEMP%\planner_harness\`，不写仓库 `build/`。
>
> 报告说明：`baseline_report.md` 第 1~8 节、`after_report.md` 第 0~6 节由脚本自动生成；两份报告最后一个
> 人工小节（根因/结论）重跑脚本会被覆盖，保留时请先另存。
>
> 本机 harness 使用 `-Python python`，避免 WindowsApps 的 Python 存根。

## 0. v2 方形车体（30×30cm）评测——当前版本

> v2 起规划器改为 **30×30cm 方形车体**模型（v1 为“半径 15cm 圆”）：平移段保持固定朝向，只在
> `rotation_index` 处原地旋转（旋转点欧氏清障必须 ≥ 半对角线 21.22 + 2 = **23.22cm**）。
> 本目录 harness 已迁移到 v2 API：`MapCore_Init(zone,grid,clarity,square_clarity)`、
> `Planner_PlanPathWithYaw(start,goal,start_yaw,goal_yaw,...)`，并新增**独立精确 SAT** 真值
> （30×30 正方形按执行航向旋转 vs 轴对齐障碍矩形/地图边界；不调用 MapCore 碰撞判定）。

### 0.1 快速开始

```powershell
# 编译 3 个 runner + 跑 40 条带 yaw 的任务点对 + v2 边界用例 + 执行剖面单测，
# 并生成 v2 摘要/前后对比/旧基线方形复核报告（全部输出在 %TEMP%\planner_harness_v2）
powershell -NoProfile -ExecutionPolicy Bypass -File Applications\Planner\tests\harness\run_new.ps1
```

| 输出 | 说明 |
|---|---|
| `%TEMP%\planner_harness_v2\new_out\newv2_zone{1,2}.csv` | 40 条任务点对：stats + 独立 SAT + 旋转/朝向/重复点断言 |
| `%TEMP%\planner_harness_v2\new_out\newv2_paths_zone{1,2}.csv` | 每航点：世界坐标/执行航向/rotation_index/独立方形净空 |
| `%TEMP%\planner_harness_v2\new_out\acceptance_v2.csv` | v2 验收断言（PASS/FAIL/INFO） |
| `%TEMP%\planner_harness_v2\new_out\edge_results_v2.csv` | v2 边界/回归用例（E1~E11，PASS/FAIL） |
| `%TEMP%\planner_harness_v2\new_out\profile_results.csv` | 执行剖面纯函数单测（16 项，PASS/FAIL） |
| `%TEMP%\planner_harness_v2\baseline_report_v2.md` | 旧基线（HEAD 6413fac）路径按**方形真值**复核（自动生成） |
| `%TEMP%\planner_harness_v2\after_report_v2.md` | v2 前后对比：旧基线方形真值 vs v2 + 验收/边界用例（自动生成） |
| `%TEMP%\planner_harness_v2\new_report_v2.md` | v2 40 条任务点对摘要（自动生成） |

仓库内快照：`results/new_v2/*.csv`、[baseline_report_v2.md](baseline_report_v2.md)、[after_report_v2.md](after_report_v2.md)。

### 0.2 v2 验收标准（run_new.ps1 硬断言，任一 FAIL 退出码非 0）

1. **40/40 任务点对全部规划成功**（yaw 取自 `nav_map_data.c`：zone1 HOME/QrCode/MATL_GRAB=0°、MATL_STORAGE=90°、
   MATL_PROCESSING=180°；zone2 全 0°）；失败必须记录 `stats.fail_reason`。
2. **一般路段方形 SAT 不相交**：路径按执行语义逐段 1cm 采样，旋转正方形 vs 轴对齐障碍/地图边界，
   signed clearance ≥ 0（含 0.02cm 数值容差；0 表示 30cm 通道相切）。
3. **端点豁免区模型侵入 ≤ 1.0cm**：到起点/终点**欧氏距离 ≤ 25cm** 的采样点允许 ≤1cm 模型侵入
   （hard tier `endpoint_allowed = -1cm`）；一般路段不允许穿透。
4. **旋转安全**：每个旋转点到障碍/边界独立欧氏清障 ≥ 23.22cm；含起点先转（rot=0）、
   中途旋转、到终点再转（branch 1b，rot=len-1）。终点原地补转仅当终点旋转安全，端口层
   `verify_goal_precision` 已被评审确认为该安全行为（不修改规划输出）。
5. **朝向语义**：`heading_before = 起始实际 yaw`、`heading_after = 目标 yaw`
   （无需旋转时两者均=目标 yaw；|Δyaw| 在 `PLANNER_HEADING_DEADBAND_DEG` 内不旋转）。
6. **起终点精确 / 相邻点无重复（dist>0.01cm）**：唯一合法例外=起终点几乎重合（<1cm）时的 2 点零长路径，
   `new_runner` 单独识别并跳过重复断言（`zero_len_path=1`）；其它任何重复都算 FAIL。
7. **stats 一致**：`stats.waypoints/length_cm` 与独立测量一致（长度差 ≤0.5cm）。

v2 当前结果（工作树，2026-10-31 冻结版）：**40/40 成功、SAT 碰撞 0、最大穿透 0.000cm、
端点侵入超限 0、旋转方案 14（最小独立旋转清障 23.50cm）、soft 4 / tight 36、平均长度 231.7cm、
平均航点 5.15、规划耗时 avg≈1.48ms / max≈35.6ms；验收 7 PASS/0 FAIL；边界用例 23 PASS/0 FAIL；
profile 16 PASS/0 FAIL**。旧基线在方形真值下 **26/40 条穿障、最大穿透 6.52cm**，旋转补转不安全 10 条。

### 0.3 v2 已覆盖的评审修复与已知语义

- **重复航点**：`insert_rotation_point` 的 `k==steps` 顶点情形改为直接复用已有顶点，
  `plan_translation_path` 增加 `dedupe_path()`；重跑 0 重复（唯一合法零长例外见上）。
- **端点容差**：hard tier `endpoint_allowed` 修正为 `max(required-2, -1)` = **-1cm** 生效；
  端点 25cm 豁免区内允许 ≤1cm 模型侵入，一般路段仍要求真实净空 ≥0。
- **任意目标 yaw**（30/37/45°）+ 窄通道：走 branch 1b，`rotation_index=len-1`（到终点再转），
  由 edge E7d 覆盖。
- **30cm 通道语义（Lead 已确认，不要求改 Planner）**：30×30 车在 30cm 缝净空=0，只能相切通过；
  若穿越段全部落在端点 25cm 豁免区（短通道）可 tight 通过（E5c），更长 30cm 通道一般路段硬门限
  0.5cm 不可满足 → clean fail（reason=1/2，安全优先正确行为，E5d 记录该语义）。
- **旧 v1 端点 (100,100)** 对 30×30 方形不可达（中心侵入方框角 5cm > 端点容差）→ v2 正确拒绝（E10，
  若任务需要该点需 Lead 重新定义任务点/容差）。

> 以下第 1~9 节为 **v1（半径 15cm 圆）历史链路**的说明，产物 `baseline_report.md` / `after_report.md`
> 保留不删，可继续用 `run_baseline.ps1` + 旧 `run_new.ps1` 逻辑复现；v2 以本节为准。

## 1. 目录结构

```
Applications/Planner/tests/
├── README.md                      本文件：方法、用法、字段、验收标准
├── baseline_report.md             重构前（HEAD 6413fac）基线报告（自动生成 + 人工根因节）
├── after_report.md                重构前后对比报告（v1 圆模型历史版本，自动生成 + 人工结论节）
├── baseline_report_v2.md          v2：旧基线方形车体真值复核报告（自动生成）
├── after_report_v2.md             v2：前后对比（旧基线方形真值 vs v2 + 验收/边界用例，自动生成）
├── results/                       结果快照（可直接对比；重跑会覆盖 %TEMP% 中的新结果）
│   ├── baseline_zone1/2.csv       重构前 40 条点对的汇总指标
│   ├── baseline_paths_zone1/2.csv 重构前每条路径的航点明细（世界坐标/清障/段长/转角）
│   ├── baseline_rects_zone1/2.csv 独立几何真值用的障碍矩形（世界坐标 cm）
│   ├── baseline_points_zone1/2.csv 点位名称与坐标
│   └── new/                       重构版结果快照
│       ├── new_zone1/2.csv        与 baseline_zone*.csv 完全相同的表头（raw_* 列填 -1）
│       ├── new_stats_zone1/2.csv  tier / expanded_nodes / stats 与独立真值对比
│       ├── new_paths_zone1/2.csv  重构版航点明细
│       ├── acceptance.csv         验收断言结果（new_runner 输出）
│       ├── edge_results.csv       边界/回归用例结果（edge_runner 输出）
│       └── profile_results.csv    执行剖面单元测试结果（profile_runner 输出）
│   └── new_v2/                    v2 方形车体结果快照
│       ├── newv2_zone1/2.csv      40 条任务点对：stats + 独立 SAT + 旋转/朝向/重复点断言
│       ├── newv2_paths_zone1/2.csv 每航点：世界坐标/执行航向/rotation_index/独立方形净空
│       ├── acceptance_v2.csv      v2 验收断言（PASS/FAIL/INFO）
│       ├── edge_results_v2.csv    v2 边界/回归用例（E1~E11）
│       └── profile_results.csv    执行剖面单元测试（16 项）
└── harness/
    ├── reference_metrics.h/.c     独立几何真值/指标库（v2 含精确 SAT 方形车体真值）
    ├── reference_obstacles.h      独立几何真值用的障碍世界坐标数据 + 任务点位/yaw
    ├── summarize.py               CSV -> markdown 报告（v1 默认；--v2 出 v2 摘要）
    ├── compare.py                 前后对比报告生成器（v1 默认；--v2 委托 v2_report.py）
    ├── v2_report.py               v2 报告生成器：独立 Python SAT + 旧基线复核 + v2 对比
    ├── run_baseline.ps1           一键：v1 基线快照 + 编译 + 跑 + 汇总
    ├── run_new.ps1                一键（v2）：编译 3 个 runner + 跑 + 生成 v2 摘要/对比/基线报告
    ├── new_runner.c               v2 评测驱动（PlanPathWithYaw + 独立精确 SAT + 验收断言）
    ├── edge_runner.c              v2 边界/回归用例驱动（E1~E11，PASS/FAIL 硬断言）
    ├── profile_runner.c           执行剖面纯函数单元测试（P1~P7）
    └── baseline/
        ├── snapshot_baseline.py   从 git 6413fac 导出旧源码快照（SHA-256 校验）
        └── baseline_runner.c      旧版复现驱动（map 初始化 + astar_plan + simplify_path）
```

## 2. 快速开始

```powershell
# ① 重构版：编译 new_runner/edge_runner/profile_runner，跑 40 条点对 + 边界用例 + 执行剖面单测，
#     生成 new_report.md 与 after_report.md
powershell -NoProfile -ExecutionPolicy Bypass -File Applications\Planner\tests\harness\run_new.ps1

# ② 重构前基线：导出 HEAD 源码快照 + 编译 + 跑 40 条点对，重建 baseline 8 个 CSV
powershell -NoProfile -ExecutionPolicy Bypass -File Applications\Planner\tests\harness\run_baseline.ps1
```

两个脚本的输出都在 `%TEMP%\planner_harness\`：

| 脚本 | 输出 |
|---|---|
| `run_new.ps1` | `new_build\new_runner.exe`、`new_build\edge_runner.exe`、`new_build\profile_runner.exe`、`new_out\new_*.csv`、`new_out\acceptance.csv`、`new_out\edge_results.csv`、`new_out\profile_results.csv`、`new_report.md`、`after_report.md` |
| `run_baseline.ps1` | `baseline_src\`（HEAD 快照）、`build\baseline_runner.exe`、`out\baseline_*.csv`、`baseline_report.md` |

## 3. 重构版评测（new_runner / edge_runner）

### 3.1 编译命令

```powershell
$p = 'Applications\Planner'
$h = 'Applications\Planner\tests\harness'
D:\devtool\MinGW\bin\gcc.exe -std=c99 -O2 -Wall -Wextra `
    -DPLANNER_HOST_TEST -msse2 -mfpmath=sse `
    -I $p -I "$p\utils" -I $h `
    "$p\utils\Cstar_map_core.c" "$p\utils\planner_algo.c" "$p\utils\priority_queue.c" `
    "$h\reference_metrics.c" "$h\new_runner.c" `
    -o "$env:TEMP\planner_harness\new_build\new_runner.exe" -lm
# edge_runner 同理，把 new_runner.c 换成 edge_runner.c
```

**必须** `-DPLANNER_HOST_TEST`（去掉 ARM 专有 section 属性）、`-msse2 -mfpmath=sse`（对齐 Cortex-M4 单精度语义）、`-lm`；
不要编译 `planner_port.c`（依赖 FreeRTOS/cmsis_os2）。三个 runner 用同一组源文件编译，只换最后的主文件：
`new_runner.c` / `edge_runner.c` / `profile_runner.c`（均传 `--out-dir DIR`，分别输出 40 条点对 CSV+`acceptance.csv`、
`edge_results.csv`、`profile_results.csv`）。

### 3.2 new_runner 输出与验收断言

- 主 CSV `new_zone<z>.csv` 前 24 列**与基线完全同表头**（`raw_*` 因新接口不暴露 A* 原始逐格路径而填 -1），
  方便 `compare.py` 直接对齐；`new_stats_zone<z>.csv` 额外输出 `tier / expanded_nodes / stats.min_clearance_cm`
  以及与独立真值的差。
- 验收断言（任一失败 `new_runner` 退出码非 0，结果写入 `acceptance.csv`）：

| # | 断言 | 门限 |
|---:|---|---|
| 1 | 40/40 条有向点对规划成功 | 40/40 |
| 2 | 每条路径独立真值最小清障 | ≥ 14.9cm（15cm 半径留 0.1cm 数值容差） |
| 3 | 起点与请求坐标一致 | ≤ 1e-3 cm |
| 4 | 终点与请求坐标一致 | ≤ 1e-3 cm |
| 5 | 相邻航点无重复 | 最小相邻间距 > 1e-3 cm |
| 6 | `Planner_PathValidate(path,len,15.0,&min)` 通过且与独立真值一致 | 差 ≤ 0.6cm |
| 7 | `stats.length` 与独立测量长度一致 | 差 ≤ 0.5cm |
| 8 | 独立障碍数据与 `MapCore_PointClearanceCm` 逐格一致 | 偏差 ≤ 0.1cm |

- **门限语义（code freeze 最终定义）**：`PLANNER_ENDPOINT_CLEARANCE_TOL_CM = 1.0`、`PLANNER_VALIDATION_TOL_CM = 0.5`、
  `PLANNER_ENDPOINT_ZONE_CM = 25.0`。
  任务 40 点位对要求独立真值最小清障 **≥15.0cm**；通用/边界路径按**分区门限**：
  - 一般路段（到起点/终点欧氏距离 >25cm）：tight ≥**14.5cm**（15 − VALIDATION_TOL 0.5）、soft ≥**18.5cm**（19 − 0.5）；
  - **端点豁免区**（路径采样点到**精确起点/终点的欧氏距离 ≤ `PLANNER_ENDPOINT_ZONE_CM` = 25cm**，不是沿路径弧长）：
    tight ≥**14.0cm**（15 − ENDPOINT_TOL 1.0）、soft ≥**18.0cm**（19 − 1.0），用于起终点本身贴墙的场景
    （如 HOME 距场边条 15cm、(100,100) 距方框角 14.142cm）。
  - A* 端点放宽与校验同口径：仅对“两端都在同一端点邻域球（≤25cm）内”的边使用端点容差。

> 预期结果（当前实现）：40/40 成功，独立真值最小清障 **15.00cm**，0 条 <14.5cm；
> 平均航点 3.27、平均路径长 185.7cm；tier soft 4 / tight 36。
> `edge_runner` 边界/回归用例 **16 PASS / 0 FAIL**（含新增 T8 端点容差回归）；
> `profile_runner` 执行剖面单元测试 **16 PASS / 0 FAIL**（见 3.4）。

### 3.3 edge_runner 边界/回归用例

| 用例 | 内容 | 判定 |
|---|---|---|
| T1a | 10cm 平直微调（世界 (10,75)→(10,85)） | 返回 2 点路径且 `tier=soft` |
| T1b | HOME→QrCode（105cm） | 不得被当成短路径直达：`expanded_nodes>0`、长度>20cm |
| T2a/T2b | 目标在障碍方框/墙内部（(60,145)/(105,215)） | `PlanPath` 返回 0 |
| T2c | 任务书示例 (105,105) | INFO：该点实测清障 21.21cm、不在障碍内（见第 8 节勘误） |
| T3a/T3b | 起点 (500,0) / 目标 (-300,0) 在地图外 | 返回 0 |
| T4a~T4d | 动态障碍：zone1 QrCode→MATL_STORAGE 中点加 2 个 `MAP_CELL_DYNAMIC_OBSTACLE` 格 | 旧路径失效（清障 <15cm）；重规划成功且新路径独立真值（含动态格）≥14.9cm；路径确实改变；清除动态格后恢复原路径 |
| T5 | 同一对点连跑 5 次 | waypoints/length/min_clearance 完全一致（diff=0） |
| T6/T6b | zone1 HOME 与其余 4 点双向 8 条路径 | 全部成功、独立真值 ≥14.9cm；HOME 端点清障 14.9~15.1cm |
| T7 | 每 zone 的 soft/tight 分布 | 打印 tier 与实测清障关系；tight 最小清障 ≥14.9cm、soft ≥17cm（<18.5 告警） |
| T8 | 端点容差回归：`(10,105)<->(100,100)`（终点本身清障 14.142cm <15cm） | 双向规划成功；按分区门限校验：一般路段 ≥14.5cm、端点豁免区（到精确起点/终点欧氏距离 ≤25cm）≥14.0cm |

### 3.4 profile_runner 执行剖面纯函数单元测试（执行层 Blocker 回归）

背景：`planner_port.c` 曾把 `Planner_PointSegmentDistanceCm`（点到线段距离）误当"两点段长"，导致所有中间航点
段长/剩余路程恒 0、速度恒 40cm/s、航点被跳过。修复方式是把执行剖面逻辑抽成算法层纯函数并在 `planner_port.c` 使用：
`Planner_PointDistanceCm` / `Planner_BuildRemainingCm` / `Planner_WaypointSpeedCmS` /
`Planner_SegmentSpeedLimitCmS` / `Planner_WaypointTimeoutMs`。`profile_runner.c` 在主机上直接测试这些函数：

| 用例 | 内容 | 判定 |
|---|---|---|
| P1a/P1b | `Planner_PointDistanceCm`：世界 (0,0)-(300,400)cm；同一点 | =500.0cm（±1e-3）；=0 |
| P1c | 与 `Planner_PointSegmentDistanceCm` 语义区分 | p=线段起点时点-线段距离=0，而两点段长=100cm |
| P2a/P2b | `Planner_BuildRemainingCm`：路径 (0,0)→(100,0)→(100,100)cm | remaining={200,100,0}；out_cap 不足/len≤0/空指针返回 false |
| P3a/P3b | `Planner_WaypointSpeedCmS` | remaining 0/25/50/100 → 40/80/120/120；tight 上限 80；扫描不出现负值 |
| P4a/P4b/P4c | `Planner_WaypointTimeoutMs` | (100cm,120cm/s)→3500ms（±1）；speed≤0 → 60000ms 上限（不除零）；seg=0→2000ms |
| P6a/P6b | `Planner_SegmentSpeedLimitCmS` | seg=30→72.25cm/s；seg=300→120 封顶；seg=0.2/0→40 下限；扫描单调不减且 ∈[40,120] |
| P5a/P5b | **旧 Blocker 回归**：多段路径 remaining 必须非零且严格递减、差值=各段长 | 合成 3 段 {210,160,80,0}；真实规划路径 remaining[0]=`stats.length_cm`、逐段递减（旧实现会全 0） |
| P7a/P7b | 执行速度 = `min(WaypointSpeed, SegmentSpeedLimit)`，真实路径整条剖面 | 短末段 30cm 时被限到 72.25；真实 4 点路径剖面 [80,74.2,80,40]cm/s，首点>40、末点=40、非全 40（旧 bug 全 40） |

> 当前结果：**16 PASS / 0 FAIL**，由 `run_new.ps1` 纳入整体 PASS/FAIL 汇总（失败退出码非 0），结果写入
> `profile_results.csv`，对比报告第 5b 节展示。

## 4. 重构前基线复现（HEAD 6413fac）

基线版本锁定为提交 **`6413facd5c62eb9f7bd5f50cc5852df9e02f2e95`**（重构前 main 的 HEAD）。
`snapshot_baseline.py` 内置 11 个文件的 SHA-256；任何文件与基线不一致都会报 `HASH-MISMATCH` 并退出非 0。

```powershell
$base = Join-Path $env:TEMP 'planner_harness'
# 1) 导出旧源码快照（到 %TEMP%\planner_harness\baseline_src）
py Applications\Planner\tests\harness\baseline\snapshot_baseline.py
# 2) 编译（源码来自快照，驱动/指标来自仓库）
D:\devtool\MinGW\bin\gcc.exe -std=c99 -O2 -Wall -Wextra -msse2 -mfpmath=sse `
    -I "$base\baseline_src" -I Applications\Planner\tests\harness `
    "$base\baseline_src\planner_algo.c" "$base\baseline_src\Cstar_map_core.c" `
    "$base\baseline_src\priority_queue.c" `
    Applications\Planner\tests\harness\reference_metrics.c `
    Applications\Planner\tests\harness\baseline\baseline_runner.c `
    -o "$base\build\baseline_runner.exe"
# 3) 跑 40 条点对
& "$base\build\baseline_runner.exe" --out-dir "$base\out"
# 4) 生成基线报告
py Applications\Planner\tests\harness\summarize.py --results-dir "$base\out" `
    --label "HEAD 6413fac（重构前基线）" --out "$base\baseline_report.md"
```

基线复现驱动与 HEAD 逐行一致的部分：`Planner_Init`（矩形填充 + `(int)((max-1e-4)/res)` 细节）、
`astar_plan`（快照编译，`prev_path=NULL, is_return=false`）、`simplify_path/segment_is_free`（逐行复制）。

## 5. 主机浮点精度要求（**必须** `-msse2 -mfpmath=sse`）

`D:\devtool\MinGW\bin\gcc.exe` 是 32 位 MinGW，默认用 x87 做浮点，中间结果按 80 位扩展精度保存。
旧版地图填充里有 `int gx0 = (int)(min_x / g_map.res);` 这类**除法后直接截断**的写法：

- Cortex-M4（AC6，单精度 IEEE）：`4.5f / 0.1f` 正确舍入为 `45.0f` → 格号 45；
- x87（80 位扩展精度）：除法得到 `44.9999993…` → 截断成 **44**。

差一格会让墙/方框多占或少占一列网格，实测会导致基线 **zone1 8 条 + zone2 4 条**点对出现**假性 A\* 无解**。
因此本工具所有 gcc 命令都带 `-msse2 -mfpmath=sse`，与目标机单精度语义对齐；
`run_baseline.ps1` / `run_new.ps1` 已固化该参数，勿删。

## 6. 评测方法

### 6.1 独立几何真值（`reference_metrics.c` + `reference_obstacles.h`）

- 由预定义障碍四边形（世界坐标，每 4 点一组）按 min/max 构造**轴对齐矩形**，与地图网格无关；
  世界坐标 = 地图坐标 - (2.4, 2.4)m，统一换算成 **cm**。
- `point_to_rect_distance`：点到轴对齐矩形的**精确欧氏距离**（点在矩形内为 0）。
- 对路径折线按 **≤2cm** 步长采样（每段 `steps = ceil(len/2cm)`，含两端点），逐采样点求到所有矩形的最小距离：
  - `min_clearance_cm`：全程最小清障距离；
  - `collision`：`min_clearance_cm < 15cm`（机器人半径 15cm）；
  - 同时统计每段长度/段内最小清障、每个航点清障、拐点转角（度）、终点误差（末点应精确等于目标点）。
- 动态障碍用例会把动态格（10cm 方格）转成世界坐标 `RefRect` 追加进障碍集合，独立测量“静态+动态”清障。
- 一致性兜底：`new_runner.c` 启动时逐格比较 `MapCore_PointClearanceCm(格中心)` 与独立模型
  （含地图边界项），偏差 >0.1cm 判失败，确保两份障碍数据没有脱节（当前最大偏差 0.0000cm）。

### 6.2 分类阈值（报告用）

| 分类 | 判据 | 含义 |
|---|---|---|
| 明确侵入 | `min_clearance < 14.5cm` | 相对车体半径有 >0.5cm 的确定侵入（扣除 2cm 采样与浮点噪声） |
| 零余量 | `14.5cm ≤ min_clearance < 15.05cm` | 名义清障正好 = 车体半径；本场地多条 30cm 通道 = 30cm 车宽、HOME 端点离场边条 15cm |
| 有余量 | `min_clearance ≥ 15.05cm` | 真正留有余量的路径 |

“严格碰撞”严格按 `min_clearance < 15.0cm` 判定（CSV `collision` 列）。注意：名义清障正好 15.0cm 的路径会因
world↔map 浮点换算在 14.99999~15.00001cm 抖动，报告同时给出“严格碰撞”和“明确侵入”两列，避免把数值抖动误读为穿障。

## 7. 输出字段

### 7.1 汇总 CSV（`*_zone<z>.csv`，基线/重构版前 24 列同表头）

| 字段 | 说明 |
|---|---|
| `zone,start,goal` | 启动区、起点名、终点名 |
| `ok` | 规划是否成功（0/1） |
| `astar_pts` / `waypoints` | A* 原始逐格点数 / 直线化后航点数（新版 `astar_pts=-1`） |
| `length_cm` | 路径总长 |
| `min_clearance_cm` | 全程最小清障距离 |
| `collision` | `min_clearance_cm < 15cm`（0/1） |
| `min_clr_wp_idx` / `min_clr_seg` / `min_clr_rect` | 最小清障所在航点 / 路径段 / 最近障碍矩形下标 |
| `min_clr_x_cm, min_clr_y_cm` | 最小清障发生点世界坐标 |
| `segs_over_radius` / `wp_over_radius` | 含 <15cm 采样点的段数 / 清障 <15cm 的航点数 |
| `goal_error_cm` | 路径末点与目标点距离（正常 ≈0） |
| `max_turn_deg` / `sum_turn_deg` | 内部航点最大转角 / 转角绝对值之和 |
| `raw_min_clearance_cm, raw_collision` 等 | 基线专用：A* 原始逐格路径同口径指标（新版填 -1/0） |

### 7.2 其他 CSV

- `*_paths_zone<z>.csv`：`zone,start,goal,wp_idx,world_x_cm,world_y_cm,clearance_cm,seg_len_cm,turn_deg`（每个航点一行）
- `new_stats_zone<z>.csv`：`tier, expanded_nodes, stats_waypoints, stats_length_cm, stats_min_clearance_cm,
  ref_min_clearance_cm, stats_minus_ref_cm, validate_ok, validate_min_cm, start_err_cm, goal_err_cm, min_adjacent_cm`
- `acceptance.csv`：`check,result,detail`（new_runner 验收断言）
- `edge_results.csv`：`test,name,result,detail`（edge_runner 用例，PASS/FAIL/INFO/WARN）
- `profile_results.csv`：`test,name,result,detail`（profile_runner 执行剖面纯函数单测，PASS/FAIL）
- `*_rects_zone<z>.csv` / `*_points_zone<z>.csv`：独立几何真值障碍矩形 / 点位表

## 8. 重构前后主要结论（详见 [after_report.md](after_report.md)）

| 指标 | 重构前 | 重构后 |
|---|---|---|
| 规划成功 | 40/40 | 40/40 |
| 明确侵入（<14.5cm） | 22 条 | **0 条** |
| 零余量（14.5~15.05cm） | 14 条 | 24 条（名义清障 15.0cm 的场地几何下限） |
| 有余量（>15.05cm） | 4 条 | 16 条 |
| 最小清障 | 11.18cm（zone2 QrCode→MATL_PROCESSING @ (95,100)cm） | **15.00cm**（0 条明确侵入） |
| 平均航点 / 最大航点 | 3.60 / 6 | 3.27 / 6 |
| 平均路径长 | 179.1cm | 185.7cm（+3.7%，换取安全余量） |
| tier 分布 | — | soft 4 / tight 36 |
| 端点容差回归（T8） | — | **PASS**：一般路段 15.000cm（≥14.5）/ 端点豁免区（欧氏≤25cm）14.142cm（≥14.0） |
| 执行剖面单元测试 | — | **16 PASS / 0 FAIL**（含执行层段长/剩余路程 Blocker 回归） |

> **code freeze 最终重跑**：端点放宽/校验口径已统一为“到起点/终点的欧氏距离”，`PLANNER_ENDPOINT_ZONE_CM = 25.0`；
> 任务 40 点位对的路径/清障/航点/长度与冻结前**逐字段一致**（最小清障仍 15.00cm），仅 zone1/zone2 各 2 条路径的
> `expanded_nodes` 因 A* 端点口径调整 -1（MATL_STORAGE→HOME、MATL_PROCESSING→HOME）；新增 T8（欧氏≤25cm 端点区）
> 验证端点容差场景；三 runner 全 PASS（9 / 16 / 16）。

**规格勘误（已与 Lead 沟通中）**：任务书示例“目标在障碍内部（世界 (105,105) 十字柱子区域）”与实际预定义障碍数据不符——
该点位于 30cm 十字通道中心，实测清障 21.21cm，`Planner_PlanPath` 正常返回 4 点路径；
真正的“障碍内部”用例改用 (60,145)（方框内）与 (105,215)（墙内），均正确返回 0。

## 9. 已知限制

- 障碍物几何取自预定义矩形（世界坐标），不含雷达实时障碍（动态障碍用例单独模拟）；
  评测的是“静态地图 + 点位”下的规划质量。
- 采样步长 2cm，理论最大漏检约 1cm（最近点落在两采样点之间）；因此报告用 14.5cm 作为“明确侵入”门限，
  避免把边界噪声当成穿障。
- 转角指标是几何折线转角，不含车体运动学约束（麦轮小车无最小转弯半径，但仍有偏摆角速度限制）。
- 报告中的 `%TEMP%` 绝对路径每次运行会变；`results/` 下是人工复制的快照。
- 主机 float 与目标机 single 的差异已用 `-msse2 -mfpmath=sse` 对齐；仍未覆盖 ARM `--fpmode=fast` 的 FTZ 行为
  （评测数值均为正规数，影响可忽略）。
