# 规划器重构前后对比报告：HEAD 6413fac（重构前） → 重构版（工作区）

> 本报告由 `tests/harness/compare.py` 基于两套 runner 的 CSV 自动生成；
> 两侧路径均由 `reference_metrics.c` 的**独立几何真值**测量（不读 Planner 网格/清障距离场）。
> 基线数据：`Applications/Planner/tests/results`；重构版数据：`%TEMP%\planner_harness\new_out`
> （快照见 `tests/results/new/`）。

## 0. 结论摘要

- 规划成功：重构前 **40/40** → 重构后 **40/40**（失败：0 → 0）。
- 明确侵入（最小清障 < 14.5cm，真实穿障）：重构前 **22** 条 → 重构后 **0** 条。
- 严格按 < 15.0cm 判定的“碰撞”条数：30 → 9；两侧的这部分差异都是“名义清障正好 15.0cm”的零余量路径（30cm 通道 = 30cm 车宽 / HOME 端点离场边条 15cm）在 world↔map 浮点换算后测得 14.99999cm，并非真实侵入。
- 分类口径：明确侵入（<14.5cm）22 → 0；零余量（14.50~15.05cm）14 → 24；有余量（>15.05cm）4 → 16。
- 严格安全（全程最小清障 ≥ 15.0cm）：重构前 **10/40** → 重构后 **31/40**。
- 全局最小清障：重构前 **QrCode -> MATL_PROCESSING（11.18cm @ (95.0,100.0)cm）** → 重构后 **HOME -> QrCode（15.00cm @ (0.0,0.0)cm）**。
- 平均航点：3.60 → **3.27**；最大航点：6 → 6；平均路径长：179.1cm → **185.7cm**；终点误差最大：0.0000cm → 0.0000cm。
- **新实现全部 40 条路径按独立真值测量的最小清障为 15.00cm（≥ 15.0cm）：其中 24 条为名义 15.0cm 的零余量路径、16 条有余量，0 条明确侵入（<14.5cm）。**
- tier 分布（重构后）：zone1 soft=2 / tight=18；zone2 soft=2 / tight=18。
- new_runner 验收断言：**9 PASS / 0 FAIL**。
- edge_runner 边界/回归用例：**16 PASS / 0 FAIL**。
- profile_runner 执行剖面单元测试（执行层段长/剩余路程 Blocker 回归）：**16 PASS / 0 FAIL**。

## 1. 前后汇总对比

| 指标 | 重构前（HEAD 6413fac（重构前）） | 重构后（重构版（工作区）） | 变化 |
|---|---:|---:|---:|
| 规划成功 | 40/40 | 40/40 | +0 |
| 严格安全(≥15cm) | 10/40 | 31/40 | +21 |
| 明确侵入(<14.5cm) | 22 | 0 | -22 |
| 零余量(14.5~15.05cm) | 14 | 24 | +10 |
| 有余量(>15.05cm) | 4 | 16 | +12 |
| 严格碰撞(<15cm，含零余量浮点抖动) | 30 | 9 | -21 |
| 最小清障 | QrCode -> MATL_PROCESSING（11.18cm @ (95.0,100.0)cm） | HOME -> QrCode（15.00cm @ (0.0,0.0)cm） | - |
| 平均航点 | 3.60 | 3.27 | -0.33 |
| 最大航点 | 6 | 6 | +0 |
| 平均路径长cm | 179.1 | 185.7 | +6.5 |
| 终点误差最大cm | 0.0000 | 0.0000 | - |

> 注：新实现（`Planner_PlanPath`）不对外暴露 A* 原始逐格路径，`astar_pts=-1`、`raw_*` 列不可用，因此“A* 原始路径清障/直线化损失”只对基线有意义；重构版的安全指标按最终下发的路径测量。

## 2. 逐点对差异（仅显著项：|Δ清障|≥0.5cm 或航点变化 或 |Δ长度|≥5cm）

- 显著差异 **24/40** 条；其中清障提升 24 条、清障下降 0 条；无显著差异 16 条。

| zone | 点对 | 最小清障 前→后 cm | Δ清障 cm | 航点 前→后 | Δ长度 cm | 重构后 tier |
|---:|---|---:|---:|---|---:|---|
| 1 | HOME -> MATL_PROCESSING | 13.44 → 15.00 | +1.56 | 6 → 4 | +9.8 | tight |
| 1 | HOME -> MATL_STORAGE | 13.44 → 15.00 | +1.56 | 6 → 4 | +9.8 | tight |
| 1 | MATL_GRAB -> MATL_STORAGE | 14.14 → 15.81 | +1.67 | 4 → 4 | +1.7 | tight |
| 1 | MATL_GRAB -> QrCode | 17.89 → 20.12 | +2.23 | 4 → 4 | +1.8 | soft |
| 1 | MATL_PROCESSING -> HOME | 14.14 → 15.00 | +0.86 | 6 → 4 | +7.1 | tight |
| 1 | MATL_PROCESSING -> MATL_STORAGE | 14.14 → 15.81 | +1.67 | 4 → 4 | +7.8 | tight |
| 1 | MATL_STORAGE -> HOME | 14.14 → 15.00 | +0.86 | 6 → 4 | +7.1 | tight |
| 1 | MATL_STORAGE -> MATL_GRAB | 14.14 → 15.81 | +1.67 | 4 → 4 | +1.6 | tight |
| 1 | MATL_STORAGE -> MATL_PROCESSING | 14.14 → 15.81 | +1.67 | 4 → 4 | +7.8 | tight |
| 1 | QrCode -> MATL_GRAB | 17.89 → 20.12 | +2.23 | 4 → 4 | +1.8 | soft |
| 2 | HOME -> MATL_GRAB | 13.44 → 15.00 | +1.56 | 6 → 4 | +3.6 | tight |
| 2 | HOME -> MATL_PROCESSING | 14.23 → 15.00 | +0.77 | 2 → 3 | +0.1 | tight |
| 2 | HOME -> MATL_STORAGE | 13.21 → 15.00 | +1.79 | 6 → 6 | +181.2 | tight |
| 2 | MATL_GRAB -> HOME | 14.14 → 15.00 | +0.86 | 6 → 4 | +0.9 | tight |
| 2 | MATL_GRAB -> MATL_STORAGE | 14.14 → 15.81 | +1.67 | 4 → 4 | +1.7 | tight |
| 2 | MATL_GRAB -> QrCode | 14.14 → 20.12 | +5.98 | 4 → 4 | -5.8 | soft |
| 2 | MATL_PROCESSING -> HOME | 14.23 → 15.00 | +0.77 | 2 → 3 | +0.1 | tight |
| 2 | MATL_PROCESSING -> MATL_STORAGE | 14.14 → 15.81 | +1.67 | 4 → 4 | +7.8 | tight |
| 2 | MATL_PROCESSING -> QrCode | 11.18 → 15.81 | +4.63 | 4 → 4 | -0.4 | tight |
| 2 | MATL_STORAGE -> HOME | 13.21 → 15.00 | +1.79 | 6 → 3 | +13.2 | tight |
| 2 | MATL_STORAGE -> MATL_GRAB | 14.14 → 15.81 | +1.67 | 4 → 4 | +1.6 | tight |
| 2 | MATL_STORAGE -> MATL_PROCESSING | 14.14 → 15.81 | +1.67 | 4 → 4 | +7.8 | tight |
| 2 | QrCode -> MATL_GRAB | 14.14 → 20.12 | +5.98 | 4 → 4 | -5.8 | soft |
| 2 | QrCode -> MATL_PROCESSING | 11.18 → 15.81 | +4.63 | 4 → 4 | -0.4 | tight |

## 3. tier 分布与实测清障关系（重构后）

| zone | soft 条数 | soft 清障范围 cm | soft 平均 cm | tight 条数 | tight 清障范围 cm | tight 平均 cm | 失败 |
|---:|---:|---|---:|---:|---|---:|---:|
| 1 | 2 | 20.12 ~ 20.12 | 20.12 | 18 | 15.00 ~ 15.81 | 15.27 | 0 |
| 2 | 2 | 20.12 ~ 20.12 | 20.12 | 18 | 15.00 ~ 15.81 | 15.27 | 0 |

> soft = 满足“半径 + 4cm 安全余量”的推荐路径；tight = 起终点贴障碍时退化为仅满足 15cm 半径的路径。tight 路径的实测最小清障可等于 15cm（端点本身贴墙），soft 路径应明显更大。

## 4. 验收断言（new_runner，重构后）

| 检查项 | 结果 | 详情 |
|---|---|---|
| 40/40 规划成功 | PASS | 成功 40/40（失败 0） |
| 每条路径独立真值最小清障>=14.9cm | PASS | 最小 15.00cm（MATL_STORAGE -> HOME @ (105.0,42.8)cm），低于门限路径数 0 |
| 起点与请求精确一致(<=1e-003 cm) | PASS | 最大误差 0.000017cm |
| 终点与请求精确一致(<=1e-003 cm) | PASS | 最大误差 0.000017cm |
| 路径相邻点无重复(间距>1e-003 cm) | PASS | 最小相邻间距 14.1421cm |
| Planner_PathValidate(15cm)通过且与独立真值差<=0.6cm | PASS | validate失败 0 条，最大差 0.0000cm |
| stats.length 与独立测量长度差<=0.5cm | PASS | 最大差 0.0000cm |
| 独立障碍数据 vs MapCore 清障场逐格偏差<=0.1cm | PASS | 最大偏差 0.0000cm（zone1=0.0000，zone2=0.0000） |
| 逐路径验收断言总失败项 | PASS | 累计 0 项 |
| tier 分布 | INFO | zone1 soft=2 tight=18；zone2 soft=2 tight=18 |

## 5. 边界/回归用例（edge_runner，重构后）

| 用例 | 名称 | 结果 | 详情 |
|---|---|---|---|
| T1a | 10cm 短距离微调返回 2 点且 tier=soft | PASS | 路径点数=2(期望2), tier=0(期望soft=0), exp=0, len=10.00cm |
| T1b | >20cm 的 HOME->QrCode 走 A* 不被误判为短路径 | PASS | 路径点数=2, A*展开=10(期望>0), len=105.48cm(期望>20), tier=1 |
| T2a | 目标在障碍方框内部 → 返回 0 | PASS | goal=(60,145) 在方框 [35,90]x[120,175] 内部, PlanPath 返回 0 |
| T2b | 目标在障碍墙内部 → 返回 0 | PASS | goal=(105,215) 在横墙 [76,134]x[210,225] 内部, PlanPath 返回 0 |
| T2c | 任务书示例 (105,105) 的实际几何判定 | INFO | world(105,105): 独立真值清障=21.21cm, MapCore清障=21.21cm, 在障碍内=0, PlanPath返回=4（结论：该坐标是 30cm 十字通道中心，不是障碍内部；任务书示例与实际障碍数据不符） |
| T3a | 起点在地图外 → 返回 0 | PASS | start world=(500,0)cm → 地图 x=7.4m 超出 [0,4.8)m, 返回 0 |
| T3b | 目标在地图外 → 返回 0 | PASS | goal world=(-300,0)cm → 地图 x=-0.6m 超出 [0,4.8)m, 返回 0 |
| T4a | 加入动态障碍后旧路径已不满足 15cm 硬约束 | PASS | 动态障碍 2 格 @ grid(34,34)+(35,34) ≈ world(105.0,105.0)cm; 旧路径清障 15.00cm→0.00cm |
| T4b | 动态障碍下重规划成功且新路径独立真值≥14.9cm | PASS | 新路径点数=6(原2) len=343.61cm(原180.00) tier=1; 独立真值(含动态)最小清障=15.81cm; 路径已改变=1 |
| T4c | 动态障碍后新路径与旧路径不同（确实绕行） | PASS | 新路径点数=6(原2) len=343.61cm(原180.00) tier=1; 独立真值(含动态)最小清障=15.81cm; 路径已改变=1 |
| T4d | 清除动态障碍后恢复原路径能力 | PASS | 清除后路径点数=2(原2) len=180.00cm(原180.00) min清障=15.00cm(原15.00) tier=1 |
| T5 | 重复规划确定性（5 次结果完全一致） | PASS | 连跑 5 次: waypoints=2, len diff=0.000000cm, min_clr diff=0.000000cm, 坐标最大差=0.000000cm, 异常次数=0 |
| T6 | zone1 HOME 端点贴墙（15cm）路径全部有效 | PASS | HOME 相关路径 8/8 条成功且独立真值≥14.9cm; HOME 端点独立真值清障范围 [15.000, 15.000]cm |
| T6b | HOME 端点清障≈15cm（距场边条 15cm） | PASS | HOME 端点独立真值清障 [15.000, 15.000]cm（期望 14.9~15.1） |
| T7 | zone1 的 tier 分布 | INFO | soft=2(范围 20.12~20.12, 平均 20.12, <18.5cm 0 条), tight=18(范围 15.00~15.81, 平均 15.27), fail=0 |
| T7 | tier 分布与实测清障关系符合预期 | PASS | soft_min=20.12cm(期望≥17, ≥18.5 更佳), tight_min=15.00cm(期望≥14.9), fail=0 |
| T7 | zone2 的 tier 分布 | INFO | soft=2(范围 20.12~20.12, 平均 20.12, <18.5cm 0 条), tight=18(范围 15.00~15.81, 平均 15.27), fail=0 |
| T7 | tier 分布与实测清障关系符合预期 | PASS | soft_min=20.12cm(期望≥17, ≥18.5 更佳), tight_min=15.00cm(期望≥14.9), fail=0 |
| T8 | 端点容差场景 (10,105)<->(100,100) 规划成功且满足分区门限 | PASS | 终点(100,100)实测清障=14.142cm（端点容差场景=1）；dir0 (10,105)->(100,100): 点数=3 len=92.07cm tier=1 一般段min=15.000cm(门限≥14.5, 超差0) 端点豁免区(欧氏≤25cm)min=14.142cm(门限≥14.0, 超差0)；dir1 (100,100)->(10,105): 点数=3 len=92.07cm tier=1 一般段min=15.000cm(门限≥14.5, 超差0) 端点豁免区(欧氏≤25cm)min=14.142cm(门限≥14.0, 超差0) |

## 5b. 执行剖面单元测试（profile_runner，重构后）

背景：`planner_port.c` 曾把 `Planner_PointSegmentDistanceCm`（点到线段距离）误当段长，导致段长/剩余路程恒 0、中间航点被跳过。修复后执行剖面抽成算法层纯函数，本组用例直接测试：`Planner_PointDistanceCm` / `Planner_BuildRemainingCm` / `Planner_WaypointSpeedCmS` / `Planner_SegmentSpeedLimitCmS` / `Planner_WaypointTimeoutMs`，并回归“多段路径 remaining 非零且单调递减、速度剖面非恒 40”。

结果：**16 PASS / 0 FAIL**。

| 用例 | 内容 | 结果 | 详情 |
|---|---|---|---|
| P1a | 两点距离 = 500cm（3-4-5 直角三角形） | PASS | world(0,0)-(300,400)cm → 500.0000cm（期望 500.0000 ±0.001） |
| P1b | 两点距离：零距离 = 0 | PASS | 同一点距离 → 0.000000cm（期望 0） |
| P1c | 点-线段距离(0) 与 两点段长(100) 语义区分 | PASS | p=线段起点: 点-线段距离=0.000000cm（期望0）, 两点距离=100.0000cm（期望100） |
| P2a | 剩余路程 = {200,100,0}cm | PASS | 路径(0,0)→(100,0)→(100,100)cm: remaining={200.0000, 100.0000, 0.0000}（期望 {200,100,0}） |
| P2b | 剩余路程：容量不足/非法长度/空指针返回 false | PASS | out_cap=2→返回0, len=0→返回0, len=-1→返回0, out=NULL→返回0（期望全 false） |
| P3a | 减速区/巡航/tight 限速速度剖面（40/80/120） | PASS | remaining/tight→speed: 0/F→40, 25/F→80, 50/F→120, 100/F→120, 0/T→40, 25/T→80, 50/T→80, 100/T→80（共 8 个用例） |
| P3b | 速度剖面：不出现负值 / tight 不超过 80cm/s | PASS | remaining∈[-100,300] 扫描: soft 速度 ∈ [40.00, 120]，tight 最大 80.00（期望 soft≥40 且无负值、tight≤80） |
| P4a | 超时估算公式 = 3500ms | PASS | seg=100cm, speed=120cm/s → 3500ms（期望 2000 + 100/120*1000*1.8 = 3500 ±1） |
| P4b | 超时：speed≤0 除零保护 + 60000ms 上限 | PASS | speed=0→60000ms, speed=-5→60000ms, seg=10000/speed=1→60000ms（期望均 60000 上限，无除零） |
| P4c | 超时：零段长只保留 2000ms 固定余量 | PASS | seg=0, speed=120 → 2000ms（期望 2000 固定余量） |
| P6a | 段长限速：sqrt(2*a*s) 反推 + [40,120] clamp | PASS | seg=30→72.2496cm/s（期望72.25）, seg=300→120.00（期望120封顶）, seg=0.2→40.00（期望40下限）, seg=0→40.00 |
| P6b | 段长限速：单调性与安全区间 | PASS | seg∈[0,500] 扫描: 速度单调不减、∈[40,120]，最大 120.00cm/s |
| P5a | 多段路径 remaining 非零且严格递减（Blocker 回归） | PASS | 3 段路径 remaining={210.0000,160.0000,80.0000,0.0000}（期望 {210,160,80,0}），严格递减=1，差值=各段长=1（旧实现会全 0） |
| P5b | 真实规划路径 remaining[0]=规划长度且严格递减（端到端回归） | PASS | 真实规划 zone1 HOME→MATL_PROCESSING: 点数=4, stats.length=277.4378cm, remaining[0]=277.4378cm, remaining[len-1]=0.0000, 全部段长>0 且单调递减=1 |
| P7a | 执行速度 = min(剩余路程速度, 段长限速)（短末段限速 72.25） | PASS | 2 段路径 remaining={130,30,0}: 中间点 WaypointSpeed=88.00, SegmentLimit=72.25, 实际=min=72.25（期望 72.25，说明段长限速生效） |
| P7b | 真实路径执行剖面：首点>40、末点=40、非全 40（过冲修复 + 旧 Blocker 回归） | PASS | 真实路径 4 点: 执行速度剖面=[80.0,74.2,80.0,40.0]cm/s；首点 80.0(>40)、末点 40.0(=40)、非全 40=1（旧 bug 会全 40） |

## 6. 复现命令

```powershell
# 重新生成重构前基线 8 个 CSV（%TEMP%\planner_harness\out）
powershell -NoProfile -ExecutionPolicy Bypass -File Applications\Planner\tests\harness\run_baseline.ps1

# 编译并运行重构版 runner + 边界用例 + 执行剖面单测，生成 new_*.csv / acceptance.csv / edge_results.csv / profile_results.csv
powershell -NoProfile -ExecutionPolicy Bypass -File Applications\Planner\tests\harness\run_new.ps1

# 单独生成对比报告
py Applications\Planner\tests\harness\compare.py `
    --baseline-dir Applications\Planner\tests\results `
    --new-dir $env:TEMP\planner_harness\new_out --out after_report.md
```

> 主机 gcc 必须带 `-msse2 -mfpmath=sse`（对齐 Cortex-M4 单精度语义）；`run_baseline.ps1` / `run_new.ps1` 已固化该参数。

## 7. 人工结论与建议（补充；重跑 compare.py 会覆盖本节）

1. **安全性达标**：重构前的 22 条真实穿障（8 条 30cm 通道切角 + 14 条 14.14cm 角点擦碰）已全部消除；
   重构后 40/40 路径独立真值最小清障为 **15.00cm**，0 条 <14.5cm。
   其中 24 条为名义 15.0cm 的**零余量**路径（30cm 通道 = 车宽、HOME 端点离场边条 15cm），这是场地几何决定的物理下限，
   不是规划缺陷；另 16 条有余量（15.81~20.12cm）。9 条“严格 <15cm”是 world↔map 浮点换算（1e-4cm 量级）造成的数值抖动，无物理侵入。
2. **效率变化可接受**：平均航点 3.60→3.27（-9.2%），平均路径长 179.1→185.7cm（+3.7%）；用约 +6.5cm 的路径换取
   全部路径 ≥15cm 安全，代价合理。唯一明显变长的是 zone2 HOME→MATL_STORAGE（+181cm）：旧路径直穿 30cm 窄缝，
   新实现绕行保证硬约束，属预期行为。
3. **tier 分布**：soft 4 条（QrCode↔MATL_GRAB 双向，清障 20.12cm），tight 36 条；tight 均由端点贴墙
   （HOME 15cm、MATL_STORAGE/PROCESSING 20cm）导致，实现已对 tight 路径限速 80cm/s。
4. **动态障碍回归**：zone1 QrCode→MATL_STORAGE 中点加 2 格动态障碍后，旧路径清障 15.00→0.00cm（失效）；
   重规划得到 6 点/343.61cm 路径，独立真值（含动态格）最小清障 15.81cm；清除动态格后恢复 180.00cm 原路径。
5. **执行剖面回归**：执行层 Blocker——“点到线段距离被误当两点段长，导致段长/剩余路程恒 0、中间航点被跳过”
   修复后，`profile_runner` 的 16 项纯函数单测全部通过：两点距离/点-线段距离语义区分（500cm / 0 / 100cm）；
   `remaining={200,100,0}` 且多段路径严格递减、真实规划路径 `remaining[0]=stats.length=277.4378cm`（旧实现会全 0）；
   速度剖面 40/80/120、tight ≤80；段长限速 seg=30→72.25、seg=300→120、seg=0.2→40；超时 3500ms、
   speed≤0 走 60000ms 上限不除零；真实路径执行速度剖面 [80, 74.2, 80, 40]cm/s（首点>40、末点=40、非全 40，
   旧 bug 会全 40）。已纳入 `run_new.ps1` 硬断言。
6. **code freeze 最终口径：端点放宽/校验统一为“到起点/终点的欧氏距离”，`PLANNER_ENDPOINT_ZONE_CM = 25.0`（本轮定稿）**：
   `PLANNER_ENDPOINT_CLEARANCE_TOL_CM = 1.0`、`PLANNER_VALIDATION_TOL_CM = 0.5`。
   门限：一般路段（到两端欧氏距离 >25cm）tight ≥14.5cm / soft ≥18.5cm；端点豁免区（到起点或终点欧氏距离 ≤25cm）
   tight ≥14.0cm / soft ≥18.0cm；A* 端点放宽仅对“两端都在同一端点邻域球（≤25cm）内”的边使用端点容差，
   保证 A* 放行路径必过校验。
   最终重跑：任务 40 点位对的路径/清障/航点/长度与此前**逐字段一致**（最小清障仍 15.00cm、平均航点 3.27、
   平均路径长 185.7cm、soft 4 / tight 36），`expanded_nodes` 因 A* 端点口径调整较上一轮 -1
   （zone1/zone2 各 2 条：MATL_STORAGE→HOME、MATL_PROCESSING→HOME），未出现任何 FAIL。
   新增 T8：终点 (100,100) 本身清障 14.142cm（<15cm）属端点容差场景，`(10,105)<->(100,100)` 双向均规划成功，
   一般路段最小清障 15.000cm（≥14.5）、端点豁免区（欧氏≤25cm）最小清障 14.142cm（≥14.0），用于防端点容差回归。
7. **规格勘误（需确认）**：任务书示例“目标在障碍内部（世界 (105,105) 十字柱子区域）”与实际预定义障碍数据不符：
   该点实测清障 21.21cm、不在任何障碍矩形内，PlanPath 正常返回 4 点路径。真正的“障碍内部”用例改用
   (60,145)（方框内）与 (105,215)（墙内），均正确返回 0。
8. **结论（code freeze 最终重跑）**：new_runner 9 PASS/0 FAIL、edge_runner 16 PASS/0 FAIL、profile_runner 16 PASS/0 FAIL，
   三 runner 全 PASS；任务点位对指标与冻结前一致，端点放宽/校验口径已统一为欧氏距离 ≤25cm 并由 T8 覆盖。
   建议保持当前实现，实车重点验证 tight 路径（30cm 通道）的定位与跟踪误差，以及短末段（<50cm）按段长限速后的停稳精度。
