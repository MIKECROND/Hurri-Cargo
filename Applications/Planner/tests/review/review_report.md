# Planner 重构独立评审报告（对抗性代码评审 + 主机端独立验证，最终定稿）

- 评审对象：`Applications/Planner` 重构（工作区相对 `HEAD 6413fac` 的改动）+ 集成改动
  `Drivers/User/RPI/rpi_port.c`、`Drivers/User/LCD/tools/lcd_tools.c`
- 评审人：独立评审员（非实现者），未修改任何 Planner/驱动源码
- 评审日期：2026-10-31；**评审期间实现者迭代 5 个版本，本报告数字均以最终冻结版为准**
- 最终冻结源码（SHA-256）：
  `planner_port.c` 6EEACBAA…、`planner_algo.c` 14A2D549…、`planner_algo.h` 28954024…、
  `planner_config.h` A0F32A34…、`Cstar_map_core.c` BA1800D3…、`planner_tools.c` 904D32EC…、
  `README.md` C3E70ECF…（README 已同步：端点容差 1.0、校验容差 0.5、端点豁免区 25cm、
  一般路段 ≥14.5cm / 端点区 ≥14.0cm）
- 复现：`powershell -NoProfile -ExecutionPolicy Bypass -File Applications\Planner\tests\review\run_review.ps1`
  （只写 `%TEMP%`；测试程序源码均在 `Applications/Planner/tests/review/`）

---

## 0. 结论摘要

**GO（有条件）：无 Blocker、无 Major；建议按 §5 的实车回归清单完成受控验证后上车。**

| 级别 | 数量 | 结论 |
|---|---|---|
| **Blocker** | **0** | 首轮发现的执行层段长误用（所有航点不下发）已修复并复核：16 项纯函数数值断言全过、armcc 0 告警。 |
| **Major** | **0** | 首轮 5 项 Major 全部修复并复核：A\* 收尾切角/假失败、直达弱校验、终点侧端点容差、安全门限与文档不一致、工程文件残留。最后一轮残留假无解（R1~R4）也已修复（v5 实测 0/4600）。 |
| **Minor** | 5 | 优先队列满静默丢节点；Planner 优先级高于电机控制任务（A\* 期间阻塞）；雷达写动态格未重建清障场（执行期监测漏检，雷达默认关）；`get_grid_val/set_grid_val` 无越界保护；`planner stat`/失败报告残留旧值。 |
| **Nit** | 5 | `Planner_PathSimplify(NULL,5)` 返回 5；LCD `MapToScreen` 越界坐标 uint16 回绕；zone1 围栏注释与数据不符；`Planner_Init` 失败无上层状态；重规划后 `start_w_extend` 未更新。 |

**最终冻结版关键验证数字**（PC 端独立几何，不调用 Planner 校验函数）：
- 4600 次规划（静态 3000 + 动态 1600）：4558 成功、42 失败（全部经独立 BFS 确认为真不可达），
  **假无解 0**；分区安全门限（到起/终点欧氏距离 ≤25cm 的端点区 ≥14.0cm，其余 ≥14.5cm）
  **违规采样 0、违规路径 0**；实测一般路段最小清障 **15.000cm**、端点区 14.142cm。
- 任务 40 条点位对 **40/40 成功**、最小 15.000cm；首尾端点 bit 级精确、`stats` 与独立真值 0 差异。
- 地图几何/2bit 打包/越界/canary/清障场一致性全部通过（与独立几何 0.0000cm 差异）。
- armcc AC5 5 个 Planner `.c` **0 warning**；`.eide/eide.yml`、`.uvprojx` 已无 `path_smoother.c` 残留。
- 说明：fuzz 中仍有 1622/4600 条路径“最小清障 <15cm”，这是 v5 文档化的采样容差/端点豁免语义
  （一般 ≥14.5、端点区 ≥14.0）下的正常值，**不属于分区门限违规**；任务点位对全部 ≥15.000cm。

---

## 1. 编译 / 环境验证

| 对象 | 编译器/选项 | 结果 |
|---|---|---|
| 5 个 Planner `.c`（冻结版复核） | armcc AC5（`.planner_verify/compile_planner.ps1`，Cortex-M4.fp，-O2，C99） | **5/5 OK，0 warning** |
| `lcd_tools.c` / `rpi_port.c`（雷达关） | armcc AC5 | OK，0 warning |
| `rpi_port.c`（`-DRPI_RADAR_ENABLED=1`，覆盖动态障碍类型改动行） | armcc AC5 | OK，0 warning |
| 变体 `-DPLANNER_ENABLED=0` / `-DPLANNER_PLATFORM=1`(RASPBERRY) / 组合 | armcc AC5 | 5/5 OK，0 warning |
| 3 个算法/地图 `.c` | gcc `-Wall -Wextra` | 0 warning |
| 全工程 AC5 112 个 `.c` + armlink | **由 Lead 执行**：链接成功，ROM 206.8KB/1MB、RAM 105.5KB/128KB、IRAM1 103.2KB/112KB、IRAM2 4.9KB/16KB | **本次评审未独立复跑全工程构建**（仅独立复跑 5+2 个文件）；建议上车前再干净构建一次确认产物时间戳。 |

主机测试编译：`gcc -std=c99 -O2 -Wall -Wextra -DPLANNER_HOST_TEST -msse2 -mfpmath=sse …-lm`
（独立几何数据取自 HEAD 6413fac 障碍物世界坐标，与实现数据逐字节比对一致；动态障碍按 10cm 方格独立建模）。

---

## 2. 问题清单（冻结版）

### 2.1 Blocker / Major：无（已修复项复核证据见 §2.2）

### 2.2 已修复项（v1→v5 复核证据）

| 原问题 | 修复后复核（冻结版实测） |
|---|---|
| **Blocker**：`planner_port.c` 段长/剩余路程误用（恒 0 → 所有航点被跳过） | 改为 `Planner_PointDistanceCm`/`Planner_BuildRemainingCm`/`WaypointSpeed/Timeout` 纯函数；`port_distance_repro.c` 16 项断言 ALL PASS（remaining=200/100/0、rem0→40cm/s、tight→80、timeout 100cm@120→3500ms）；armcc 0 告警。 |
| **M1**：A\* 起点容差收尾切角 / 同格强制直达 / 可达报无解 | `(205,60)->(205,155)` n=4/min **15.000**；`(90,20)->(90,105)` n=4/min 14.252（端点区）；`(105,120)->(100,110)` n=9/min 14.142；A\* 早收 2 点路径 min=15.000、<14.9cm=0。 |
| **M2**：直达门限过弱 | 端点清障充足时直达按完整 `required` 校验；A\* 早收 2 点路径 min=15.000；直达分支残余 2cm 采样 min=14.142，全部在端点豁免区（≥14.0，无违规）。 |
| **M3**：终点侧端点容差无效 | `(10,105)->(100,100)` 与反向均成功 n=3/min 14.142；终点清障∈[13,15)cm 专项 26/26 成功。 |
| **M4**：安全门限与文档不一致 | 端点容差 2.0→1.0、校验容差 1.0→0.5、端点豁免区 20→25cm，README 同步为“一般路段 ≥14.5cm / 端点区 ≥14.0cm”；实测分区违规 0。 |
| **残留 R1~R4**（最后一轮 Major）：A\* 端点放宽与 PathValidate 豁免口径不一致 → 4/4600 假无解 | 统一按“到起/终点欧氏距离 ≤25cm”判定端点区并收紧 A\* 放宽（边两端同属一个端点邻域球才整条用 14cm）；冻结版实测：`(155,190)->(80,105)` n=5/min15.000、`(205,190)->(105,80)` n=6/min15.000、`(150,205)->(80,105)` n=6/min15.000、`(170,190)->(25,-10)` n=5/min15.000；4600 次 fuzz **假无解 0**。 |
| **M5**：工程文件残留 `path_smoother.c` | `.eide/eide.yml` 与 `.uvprojx` 已无该条目（本机 grep 复核）；全量构建由 Lead 执行成功（见 §1）。 |
| **Minor（已修）**：`wait_waypoint` 无总体超时 | 每个航点/终点对齐/精度补偿均带 `deadline_tick` 兜底超时。 |
| **Minor（已修）**：LCD 粗路径只画半径一半 | `lcd_tools.c:239` 已改为 `PHYSICAL_RADIUS / res * CELL`。 |

### 2.3 仍存在的 Minor / Nit（不阻塞上车，建议后续迭代）

| 级别 | 位置 | 问题 | 建议 |
|---|---|---|---|
| Minor | `planner_algo.c:329-332` | 优先队列满时 `pq_push` 失败只 `continue`，但此前已写入 `s_g_score/s_came_from`；该节点之后若无更低代价路径不会再入队 → 可能无解/非最优。冻结版实测最大 push 214（未触发）；`MAX_PQ_SIZE=2500` 相对 2304 格 + 重复入队余量不大。 | 溢出时记录并返回失败（或扩大队列/decrease-key），不要静默丢节点。 |
| Minor | `Core/Src/freertos.c:148-150` vs `:99-101` | Planner `osPriorityRealtime2` 高于 Motor_Ctrl `High4`/Motor_Receive `High3`/Scan_Get；A\* 无 `osDelay`，计算期间阻塞电机控制任务。急停命令入队后要由低优先级任务真正发出，最坏延迟 ≈ A\* 耗时（host 最大 0.70ms，MCU 未实测）。 | Planner 优先级降到电机/导航任务之下，或 A\* 循环内定期 `osDelay(1)`。 |
| Minor | `Drivers/User/RPI/rpi_port.c:450` + `Cstar_map_core.c:459-474` | 雷达写动态格后未调用 `MapCore_RebuildClearance()`；`MapCore_PointClearanceCm()` 的动态距离用 `s_dynamic_cells[]` 缓存（仅 Rebuild 更新）→ 执行期监测（`planner_port.c:195`）在两次规划之间看不到新障碍格（雷达默认关闭）。 | 雷达会话结束/写入后调用一次 Rebuild（或动态查询直接扫位图）。 |
| Minor | `Cstar_map_core.c:42-43, 286-302` | 动态格 >512 时点清障退化为每次全图扫描 2304 格；实测 600 格时 host `Rebuild=1.9ms`、单次 `PlanPath=3.2ms`（正常 0.035/0.10ms），MCU 可能百 ms 级（雷达开启时影响实时性）。 | 溢出保留整图扫描一次/桶索引，或限制雷达写入量。 |
| Minor | `Cstar_map_core.h:74-94` | `get_grid_val/set_grid_val` 公开 inline 无越界检查（当前调用点均已校验）；接口误用时静默越界写。 | 加断言/受控 API；注释明确调用方责任。 |
| Minor | `planner_tools.c:204-224` | `planner stat` 失败后仍显示上一次成功报告；minClr<15 无 WARN。 | 失败清空/标记 report；低于半径打印 WARN。 |
| Nit | `planner_algo.c:403` | `Planner_PathSimplify(NULL,5,...)` 返回 5（不防御空指针；内部调用安全）。 | 返回 0。 |
| Nit | `lcd_tools.c:176-178` | `MapToScreen` 对地图外坐标会算出负值再转 `uint16` 回绕（离屏时可能画乱线）。 | clamp 到屏幕范围。 |
| Nit | `Cstar_map_core.c:110-129` 注释 | zone1 围栏注释与实际矩形不一致；数据本身与 HEAD 一致、功能正确。 | 修正注释。 |
| Nit | `planner_port.c:595-610` / `freertos.c:555-564` | `Planner_Init` 失败只打 dmesg_fail，后续请求失败且无“规划器不可用”状态。 | 置全局错误标志并快速返回。 |
| Nit | `planner_port.c:506-590` | 重规划后 `start_w_extend` 未更新；`g_planner_report` 失败时保留旧值。 | 失败清零 report、每轮更新 start。 |

---

## 3. 实测验证结果（冻结版，PC 端独立几何）

测试程序（`Applications/Planner/tests/review/`）：`fuzz_check.c`（模糊/边界/地图一致性/独立 BFS/计时）、
`diag_stages.c`（分阶段诊断）、`port_distance_repro.c`（执行层纯函数回归）、`run_review.ps1`（一键复跑）。

### 3.1 地图核心 / 几何一致性

| 检查 | 结果 |
|---|---|
| `MAX_CELLS=2304`、`MAX_MAP_BYTES=576`、2bit/格 | 匹配（576×4=2304） |
| 全格 2bit 写入/回读（含 (47,47) 最大索引）/ canary | 0 错误 / 未越界 |
| 点清障 vs 独立矩形几何（2304 格中心逐格） | max diff **0.0000cm** |
| 格清障缓存 vs `floor(独立清障+1e-3)` | 0 不一致 |
| 静态矩形/障碍格数 | zone1 10/362、zone2 10/358，与独立数据一致 |
| 动态格溢出（600 格 > 512 上限） | 点清障 max diff 0.0001cm（逻辑正确，性能见 Minor） |
| `SegmentMinClearanceCm` vs 0.1cm 密集采样（400 段） | 最大高估 0.135cm（未超 1-Lipschitz 界） |

### 3.2 A\* / 直线化 / 校验 / 任务点对

| 指标 | 结果 |
|---|---|
| 任务 40 条有向点对（2 zone × 5 点） | **40/40 成功**，独立最小清障 15.000cm，tight 36/40 |
| 首尾端点 bit 级等于请求 | 4600 条路径 0 违例 |
| 中间点均为格中心 / 相邻重复点 / 点数 >MAX_PATH_LEN / 非有限坐标 | 0 / 0 / 0 / 0 |
| `stats.waypoints/min_clearance/length` vs 独立真值 | 0 不一致；min/length max diff **0.000cm** |
| 输出容量（cap=n / n-1 / 2） | 正确（n→成功、n-1→0、2→0，无截断/off-by-one） |
| `Planner_PathSimplify` 原地压缩 vs 独立等价实现（300 组随机长路径+退化输入） | 0 不一致、0 越界（证明不读被覆盖元素） |
| `Planner_PathResample` 容量/异常输入 | 正确（cap 不足返回 0，不截断） |

### 3.3 模糊测试统计（4600 次：静态 3000 + 动态 1600；含 2~20 个随机动态障碍格）

| 项目 | 数值 |
|---|---|
| 成功 / 失败 | 4558 / 42（失败全部经独立 BFS 复核为**真不可达**，均在动态障碍分割轮次） |
| **假无解（可达却返回 0）** | **0 / 4600** |
| **分区门限违规**（到起/终点欧氏 ≤25cm 的端点区 ≥14.0，其余 ≥14.5） | **违规采样 一般段=0、端点区=0；违规路径=0** |
| 实测最小清障 | 一般路段 15.000cm、端点区 14.142cm；全局 14.142cm（<13cm=0、夹障碍中=0） |
| <14.9cm / <15.0cm 路径数 | 53 / 1622（均为文档化“端点区 ≥14 / 采样容差”语义内的正常值，非违规） |
| soft / tight 档位 | soft 811（min 18.028cm）、tight 3747（min 14.142cm） |
| 动态障碍进入清障几何（抽查） | 0 不一致；`dyn_field_mismatch=0` |
| 端点清障∈[13,15)cm 专项 / 直达分支 | 26/26 成功；直达分支 min 14.142、端点区门限违规 0 |

### 3.4 边界用例（冻结版全 PASS）

| 用例 | 结果 |
|---|---|
| 起点==终点（HOME）/ 0.2cm、1cm 极短位移 | 2 点路径、首尾精确 ✔ |
| 目标/起点在静态障碍内、地图外 | 返回 0 ✔ |
| 起点被动态障碍环包围（正向/反向） | 返回 0 ✔ |
| HOME(15cm 贴墙) ↔ 任务点（16 条）/ 30cm 走廊往返 | 全成功，min=15.000cm ✔ |
| 动态墙留 30cm 缺口 / 堵死唯一缺口 | 穿缺口成功（min=15.000）/ 返回 0 不穿越 ✔ |
| 终点清障 14.142cm 双向 | 成功（n=3，min=14.142）✔ |
| **历史回归用例（M1/M2/M3 + R1~R4）** | 10/10 **REPRO-OK**：`(205,60)->(205,155)` 4wp/15.000、`(90,20)->(90,105)` 4wp/14.252、`(105,120)->(100,110)` 9wp/14.142、`(105,180)->(115,190)` 3wp/15.657、`(10,105)->(100,100)`↔ 双向 3wp/14.142、R1 5wp/15.000、R2 6wp/15.000、R3 6wp/15.000、R4 5wp/15.000；分区违规=0 ✔ |

### 3.5 计时（host 参考；MCU 未实测）

```
1000 次 Planner_PlanPath(含内部 RebuildClearance, 距离>=30cm):
  min=0.038ms median=0.106ms p95=0.331ms max=0.698ms mean=0.135ms
MapCore_RebuildClearance 单次 = 0.035ms
动态格溢出(600 格): Rebuild=1.9ms, 单次 PlanPath=3.2ms
```

---

## 4. 评审重点逐项结论

1. **碰撞安全正确性**：地图 2bit 索引/打包、`MAX_MAP_BYTES/MAX_CELLS`、越界写（canary）、静态矩形保守填充、
   清障场重建、动态格溢出、点/线段清障公式与地图边界——**均正确**（独立几何 0.0000cm 差异）；
   `1e-3` 容差仅 0.001cm，无安全隐患；分区安全门限实测违规 0。
2. **A\* 正确性**：反向搜索+收尾重建顺序正确；输出容量无 off-by-one；octile 启发函数可采纳且一致；
   `uint16` g 值上界约 37600 <65535，不溢出；简化原地压缩安全（测试+证明）；端点放宽与最终校验已统一
   （欧氏 25cm 口径），假无解 0。残余：优先队列满静默丢节点（Minor）。
3. **双级安全语义**：soft 19 / hard 15 的设计目标 + 文档化的采样容差（一般 ≥14.5、端点区 ≥14.0）一致；
   `stats` 与实际完全一致；实测一般路段 ≥15.000、端点区 ≥14.142，无违规；任务点位对全部 ≥15.000cm。
4. **执行层（planner_port.c）**：Blocker 已修复；`s_waypoint` 静态缓存、两段式终点、终点精度补偿、
   abort/dry_run、mutex/event 协议无新问题；`wait_waypoint` 兜底超时已加；剩余 Minor：Planner 优先级高于
   电机控制任务、雷达动态格未重建清障场；未发现除零、数组越界、printf 格式问题。
5. **配置/一致性**：单位一致（cm/m）；编译期断言存在；`PLANNER_ENABLED=0`/`PLATFORM_RASPBERRY` 可编译；
   `PHYSICAL_RADIUS` 兼容别名有注释；冻结版 README 与配置一致（容差 1.0/0.5、端点豁免区 25cm、14.5/14cm）。
6. **集成**：`rpi_port.c` 动态障碍类型改动正确且带越界/雷达开关保护；`lcd_tools.c` 显示动态障碍正确、
   粗路径宽度已修；`freertos.c` 已调用 `Planner_Init`；EIDE/MDK 工程文件已无 `path_smoother.c` 残留。

---

## 5. GO/NO-GO 与实车回归建议

**结论：GO（有条件）。** 无 Blocker、无 Major；主机端 4600 次独立 fuzz + 边界/回归用例全部满足验收标准
（假无解 0、分区门限违规 0、40 任务点位对 40/40、min=15.000cm）；5 个 Planner `.c` armcc 0 warning。

上车前/首发运行建议：
1. **构建**：在 EIDE/Keil 里做一次干净全量构建，确认 `.axf/.hex` 时间戳更新（Lead 已执行 112 个 `.c` +
   armlink 成功，本评审未独立复跑）。
2. **低速实车**（先把 `PLANNER_WAYPOINT_SPEED_CM_S` 调到 60~80）：
   `planner map` 核对障碍布局 → 每个任务点 `planner dry` 检查航点/最小清障 → 跑完整任务流程，观察
   `track deviation` / `clearance` 告警与 `final pose error`。
3. **执行层重点观察**：航点是否逐个下发（原 Blocker 症状是“只朝终点直冲”）、超时/降速是否符合预期、
   两段式终点 1cm/2° 与精度补偿在 OPS 噪声下的表现。
4. **雷达**：默认关闭；若启用，先核对树莓派“地图格号”协议（未加 MAP_OFFSET），并确认雷达写入后清障场/
   执行期监测的刷新（Minor）。
5. **后续迭代**（不阻塞）：修 Minor（PQ 满、Planner 优先级、雷达 Rebuild、get_grid_val 越界保护、
   planner stat）、补动态障碍闭环测试。

---

## 6. 附：最终复现清单

| # | 场景 | 输入（世界 cm / zone1） | 冻结版现状 |
|---|---|---|---|
| 1 | 执行层段长（原 Blocker） | 任意多航点路径 | **已修复**：纯函数 16 项断言全过，航点正常下发 |
| 2 | 历史假无解 R1~R4（原残留 Major） | `(155,190)->(80,105)`、`(205,190)->(105,80)`、`(150,205)->(80,105)`、`(170,190)->(25,-10)` | **已修复**：n=5/6/6/5，min=15.000cm，分区违规 0 |
| 3 | M1 回归 | `(205,60)->(205,155)` / `(90,20)->(90,105)` / `(105,120)->(100,110)` | **已修复**：4wp/15.000、4wp/14.252、9wp/14.142 |
| 4 | M2 回归 | `(105,180)->(115,190)` | **已修复**：3wp/min=15.657 |
| 5 | M3 回归 | `(10,105)->(100,100)` 双向 | **已修复**：3wp/min=14.142 |
| 6 | 安全门限 | 4600 次随机点对（静态+动态） | 假无解 **0**；分区违规 **0**；一般 min 15.000、端点区 min 14.142 |
| 7 | 构建配置 | EIDE/MDK | `path_smoother.c` 条目已删除；全工程构建请现场复核 |
