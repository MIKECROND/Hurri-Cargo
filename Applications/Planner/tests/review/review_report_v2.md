# Planner v2（30×30 方形车体）独立评审报告（v2.4 最终版）

- 评审对象：`Applications/Planner` v2 重构（方形 Minkowski 净空 + 只在旋转安全点转 + 平移固定朝向）
  + 集成改动；评审期间实现者按 Lead 通知迭代（本报告以 v2.4 最终工作树为准）
- 评审人：独立评审员（非实现者）；未修改 Planner/驱动源码，只写 `tests/review/**` 与 `%TEMP%`
- 日期：2026-10-31（v2 首轮 + v2.1/v2.2 Major 修复复核 + v2.3 执行层热修复 + **v2.4 参数整定/失败日志复核**）
- **冻结源码 SHA-256**（v2.4 最终工作树；以当前工作树重算）：
  - **`planner_config.h` `A37DAAC3E32E7C35…`（v2.4 参数整定）**
  - `utils/Cstar_map_core.c` `4DD0D0BF…`、`utils/Cstar_map_core.h` `10991859…`
  - `utils/planner_algo.c` `4C65BAADE7178B7A…`（v2.4 未动）、`utils/planner_algo.h` `B04F8BD2…`
  - **`planner_port.c` `2723812766D08BAE…`（新增失败诊断日志）**、`planner_port.h` `5D02BE50…`
  - `tools/planner_tools.c` `7A2DDF3C…`、`utils/priority_queue.c` `2ECFA8D3…`
  - 集成侧 `Core/Src/freertos.c` `9EC38C5B…`（Planner 优先级 High2）
- 最终全量构建（Lead，干净 112 `.c` + armlink，v2.4）：compile 112/112、failed 0、LINK OK；
  ROM 213.73KB/1MB、RAM 110.93KB/128KB、IRAM1 105952/114688=**92.4%**；
  （v2.4 LINK bytes/Code/RO 精确值待 Lead 补，v2.3 为 2161040B/133220/84864）
- 复现：`powershell -NoProfile -ExecutionPolicy Bypass -File Applications\Planner\tests\review\run_review.ps1`
  （仅写 `%TEMP%`；测试源码均在 `tests/review/`）
- v2.2 复跑说明：**所有模块 exit=0**（含 `targeted`），armcc 5 文件 0 warning，`problems=0`；
  M3 两个 −0.5cm 端点特例由失败转为通过、−1.5cm 超容差用例 clean fail（reason=ENDPOINT）。
- v2.3 复核说明：规划算法（algo/map_core）零改动；本次只复核执行层热修复，armcc 5 文件 **0 warning**
  已独立复跑；主机 harness 指标不受影响（见 §7）。
- v2.4 复核说明：参数整定已重跑 targeted + 3000 fuzz + timing；armcc 5 文件 **0 warning**；
  无安全违例，但发现**端点 −1cm 容差被 hard=2.0 抵消**（endpoint_allowed=0，见 §8），
  需 Lead 确认或一行修复；dynamic 未重跑（地图/算法未变）。

---

## 0. 结论

**GO（v2.4，条件式）。无 Blocker、无安全违例；v2.4 参数整定方向正确（路径余量显著加厚、1cm 采样
补上漏检），但发现 1 个非安全的功能回归需 Lead 决策：hard=2.0 使端点 −1cm 渗透容差失效
（endpoint_allowed=2.0−2.0=0，见 §8）。若任务点（HOME/STORAGE/PROCESSING 净空=0）按实测 40/40
可过且不需要 OPS/摆放 0~1cm 容差，则 v2.4 可 GO；若需保留该容差，需改一行后复跑。**

- **M1（执行层安全）已关闭**：`verify_goal_precision()` 逐分支复核，不可旋转终点任何分支都
  不会下发 `ARRIVE_CHECK_BOTH`/`goal_yaw` 目标。
- **M2（资源/实时性）已关闭**：Planner 优先级 = High2（低于 Motor_Ctrl High4/Motor_Receive High3）；
  `s_via_b`/`s_refined_path` 已删；最终全量链接 IRAM1 92.4%（余 8736B）、IRAM2 余 8744B、
  RAM 110.93KB/128KB；host 规划 avg 1.2~2.8ms / max ~32-34ms（随机器波动），目标板 High2 下
  规划期间不阻塞电机控制/急停，最坏耗时留待实车记录。
- **M3（端点 −1cm 容差未接入 A\*）已关闭**：`plan_translation_path()` 调 A\* 第 5 参已改为
  `endpoint_allowed_cm`（`planner_algo.c:736-739`）。独立复现：
  - `(50,192.5,0°)→(105,195.5,0°)`（净空≈−0.5cm）→ **n=3、tier=1、fail=0、min_sq=-0.50**，独立校验通过；反向同样通过；
  - `(50,192.5,0°)→(105,196.6,0°)`（净空≈−1.5cm 超容差）→ **clean fail，fail_reason=1(ENDPOINT)**；
  - 40/40 任务点对回归全过。
- **无 Blocker**：3119 次规划成功 2260 条，独立 SAT/方形净空/旋转/端点/heading 校验 **0 违例**；
  dup=0；40/40 任务点对通过。
- 其余为 Minor/Nit（不影响 GO，建议后续清理，见 §2.3）。

---

## 1. 编译 / 环境验证

| 对象 | 工具/选项 | 结果 |
|---|---|---|
| 5 个 Planner `.c` | armcc AC5，Cortex-M4.fp，-O2，C99（v2.2 独立重跑） | **5/5 OK，0 warning** |
| `Core/Src/freertos.c`（Priority 改动） | armcc AC5（v2.2 独立重跑） | **1/1 OK，0 warning** |
| 全工程 112 个 `.c` + armlink | Lead 干净全量构建（v2.2 最终） | **compile 112/112、failed 0；LINK OK 2159848 bytes；ROM 213.75KB/1MB；RAM 110.93KB/128KB；IRAM1 105952/114688=92.4%（余 8736B）；IRAM2 7640/16384（余 8744B）** |
| 算法/地图 3 个 `.c` | gcc `-Werror` 级别 `-Wall -Wextra` | 0 warning |
| 主机测试 | gcc `-std=c99 -O2 -Wall -Wextra -DPLANNER_HOST_TEST -msse2 -mfpmath=sse` | 0 warning |

## 2. 问题清单

### 2.1 Blocker：无

独立验证（见 §3）在 **2260** 条成功规划上得到：端点 bit 级一致、heading/rotation_index 语义一致、
旋转点欧氏清障全部 ≥23.22（最小 23.235cm）、逐 1cm 独立 SAT 碰撞 0、方形净空采样 0 违规、
**相邻重复点 dup=0**、端点 −0.5cm 模型侵入（设计允许 ≤1cm）路径可通过且独立校验通过。

### 2.2 Major

**M1（Major1，安全）`verify_goal_precision()` 可在旋转不安全终点原地转 yaw —— v2.1 已修复，复核通过**
- 原位置：`planner_port.c`（旧版 `:303-355`）。
- v2.1 新逻辑（`planner_port.c:274-368`）逐分支复核结论：
  - yaw 修正只在 `can_rotate_at_goal = MapCore_RotationSafe(goal_map) && PLANNER_GOAL_YAW_ALIGN_IF_SAFE`
    为真时执行（`:305-316`），并把 `rotate_in_place(goal_map, goal_yaw, ...)` 放在该分支内；
  - 终点不可旋转时，yaw 超差仅 warning（`:317-320`），随后只做位置微调：`keep_yaw = pose.pose.yaw`
    + `ARRIVE_CHECK_DISTANCE` + `yaw_tol=180°`（`:322-345`）——**不会**下发 goal_yaw/BOTH 目标；
  - 位置达标、仅 yaw 超差且终点不可旋转：接受并返回 true（`:348-350`）；
  - 修正次数用尽后的最终判定同样区分 `can_rotate_at_goal`：可旋转终点 yaw 超差判失败，
    不可旋转终点接受 yaw 误差（`:353-367`）。
- 结论：**任何分支都不会在不可旋转终点触发导航层原地转体，M1 关闭**。

**M2（Major2，资源/实时性）—— v2.2 已关闭**
- 已修：Planner 任务优先级 `Realtime2 → osPriorityHigh2`（`Core/Src/freertos.c:153`，低于 Motor_Ctrl
  High4 / Motor_Receive High3；armcc 单独编译 freertos.c 0 warning）；`s_via_b` 删除、via 第二段
  直接写入输出缓冲（省 ~3.2KB BSS，配合已删的 s_refined_path）。
- 最终数字：IRAM1 105952/114688=**92.4%**（余 8736B）、IRAM2 7640/16384（余 8744B）、
  RAM 110.93KB/128KB；全量 112 `.c` + armlink 成功。
- 耗时：host 40 任务点对 avg 1.2~2.4ms、max ~32-34ms（随机器波动；本评审全模块运行平均 2.8ms/次、
  含 200 次随机 yaw/via 重规划）；目标板 High2 下规划期间不阻塞电机控制/急停，最坏耗时留待
  实车记录（见 §5 条件）。
- 关闭结论：资源与调度风险已消除（余量 8.7KB IRAM1/8.7KB IRAM2）；无未关闭项。

**M3（Major3，功能）hard tier 端点 −1cm 渗透容差未接入 A\* —— v2.2 已修复，复核通过**
- 原位置：`planner_algo.c:736-739`（`plan_translation_path` 调 `plan_segment` 的第 5 参）。
- 修复：第 5 参由 `0.0f` 改为 `endpoint_allowed_cm`，端点容差（含 hard −1cm）现在进入
  `plan_segment` 的起终点检查、端点邻域 `node_need/edge_need/start_need/short_need`（`:177-187`）。
  已按行复核：`plan_segment` 的 seed/found 视线均用该值；A\* 其余节点仍 required=0 保连通，
  最终 `Planner_SquarePathValidate` 仍按 tier 门限（一般段 required−0.5、端点区 endpoint_allowed）把关。
- v2.2 独立复现（全过）：
  - `zone1 (50,192.5,0°) → (105,195.5,0°)`（净空≈−0.5cm，25cm 端点豁免区）→ **n=3、tier=1、
    fail=0、min_sq=-0.50**，独立 SAT/方形/旋转校验通过；反向同样通过；
  - `(50,192.5,0°) → (105,196.6,0°)`（净空≈−1.5cm 超容差）→ **clean fail，fail_reason=1(ENDPOINT)**；
  - 40/40 任务点对回归通过、0 SAT 碰撞、0 重复点。
- 状态：**已修复，M3 关闭**。

### 2.3 Minor / Nit

| 级别 | 位置 | 问题 | 建议 |
|---|---|---|---|
| Minor | `planner_algo.c` `plan_one_tier` 末尾（case3 失败处） | `fail_reason` 被无条件覆盖成 `NO_ROTATION_POINT`：终点本身方形净空不足 / 端点不可达时也报“找不到旋转点”。实测：goal=(105,205,yaw=-123°)（清障 5cm，任何朝向都压障碍）→ 返回 fail=3。 | 保留最早/更根本的 `seg_fail`（ENDPOINT/NO_SOLUTION 优先）；仅在路径可规划但无旋转点时用 NO_ROTATION。 |
| Minor | `planner_algo.c` `refine_path_clearance`（`in[i]==in[i+1]`） | **v2.1 已消除**：`dedupe_path()` 在 `plan_translation_path` 输出前统一去除相邻零长段；fuzz 3119 条规划 **dup=0**（start==goal 的 2 点零长特例按合法路径处理，未再计为重复）。 | 已修，保留 dedupe 兜底即可。 |
| Minor | `planner_algo.c` `plan_segment` 直达/容量 | `MAX_PATH_LEN=400`；A* 原始路径可达 400 格，`refine_path_clearance` 最坏 ~2×输入，raw>200 格时 refine 返回 0（no-solution）。静态地图路径通常 <150 格，属病态输入风险。 | 容量不足时回退“A* 原始路径直接简化/校验”，而不是直接失败；或提高 MAX_PATH_LEN/限制搜索长度。 |
| Minor | `planner_algo.c:736-739` | **v2.2 已修复**：端点容差已传入 `plan_segment`，−0.5cm 特例通过、−1.5cm clean fail（见 M3）。 | 已修，保留 targeted 两个特例防回归。 |
| Minor | `planner_port.c:221-229` | 旋转阶段欧氏监测门限 = `half_diagonal - 0.5` = 20.72cm < 物理扫掠半径 21.22cm（允许 0.5cm 侵入）；中途旋转点位置阈值 2cm，最坏可把规划时的 23.22cm 余量吃到 21.22cm。 | 旋转阶段门限改成 `half_diagonal`（或 21.0 内）并把中途旋转到达阈值收紧到 1cm。 |
| Minor | `planner_algo.c:1151-1177` 零长特例 | start/goal 坐标几乎重合（dist<1cm）时直接返回 2 点路径，**不检查该位姿当前方形净空**（只在 stats 里记录 min_sq，tier 可能为 hard/负净空）。语义上是“原地不动/原地转”，执行层不会产生平移、旋转又要求起点旋转安全，因此不会引入新碰撞；但若外部把车摆进模型禁区，规划器会返回“成功”而不告警。 | 可在 stats/日志中保留负净空 warning，或在特例分支要求 min_sq ≥ endpoint_allowed 才返回成功（若允许原地不动则仅 warning）。 |
| Minor | `Cstar_map_core.c:1001-1041` | `MapCore_FindRotationSafePoint()` 已实现但全工程无调用（algo 在 `plan_via_rotation` 内自己扫描），属死代码。 | 删除或复用（减少重复扫描实现）。 |
| Nit | `planner_port.h` 文件头 / 若干注释 | 文件头仍描述“Planner_PlanPath、两段式终点对齐”，v2 已是 `PlanPathWithYaw` + 旋转阶段；README/注释与代码有轻微不同步。 | 同步注释。 |
| Nit | `planner_algo.c` `Planner_PathValidate` 类注释 | README 称“2cm 采样下真实净空 ≥0（不碰）”，但 validate 允许 `required-0.5`（hard=0.5），最坏真实采样下限约 -0.5cm；本次 dense SAT 未发现实际碰撞，仍建议把注释改为“采样门限”而非“保证不碰”。 | 修正措辞/或采样步长减半。 |

## 3. 独立对抗性验证结果（PC，独立几何）

独立几何：只用 HEAD 障碍物世界坐标实现点到 AABB/地图边界欧氏距离、方形带符号净空
（模型口径独立实现）、30×30 旋转正方形 vs AABB 的 SAT 精确碰撞（含穿透深度）、
旋转安全点欧氏判定。被测实现只通过公开 API 接触。

### 3.1 地图核心一致性

| 检查 | 结果 |
|---|---|
| `MapCore_Init` 后 rect 数/障碍格数（zone1 10/362、zone2 10/358） | 与独立数据一致 |
| 2304 格中心欧氏清障 / 方形净空（H0=15）vs 独立几何 | max diff **0.0000cm / 0.0000cm** |
| 格方形净空场 vs floor(独立净空)（自由格，饱和 [-100,100]） | diff>1cm 数 **0** |
| 2bit 打包全格写入/回读 + 三缓冲 canary（全格写后） | 0 错误、无越界 |
| 动态格写入后 `MapCore_RebuildClearance` + 方形净空抽查 | 0 不一致（`dyn_field_mismatch=0`） |

### 3.2 随机模糊（可摆放位姿：自身朝向方形净空 ≥0）

| 项目 | zone1 | zone2 | 合计 |
|---|---|---|---|
| 静态随机（随机起终点/任意 yaw） | 1500 计划 / 1142 成功 | 1500 / 1052 | 3000 计划 / **2194 成功** |
| 其中失败原因 | ep 0 / nosol 1 / norot 357 | ep 0 / nosol 2 / norot 446 | ep 0 / nosol 3 / **norot 803** |
| 动态障碍（每轮 2~20 随机格，10 轮，随机 yaw） | 73 / 45 | 46 / 21 | 119 / **66 成功** |

**所有成功规划的独立校验结果（2260 条，含 M3 端点 −0.5cm 特例）**：
- 首尾端点 bit 级一致：0 违例；heading_before/after 与 start/goal yaw：0 违例；
- rotation_index 越界：0；旋转点独立欧氏清障最小 **23.235cm**（≥23.22）：0 违例；
- 按执行 semantics（按 rotation_index 分段取 yaw）逐 1cm 采样：
  **SAT 碰撞 0、端点区 SAT 穿透 >1cm 0、方形净空一般段违规 0、端点区违规 0**；
- 实测最小方形净空：一般路段 **0.651cm**（门限 required-0.5=0.5）、端点区 **-0.500cm**
  （v2.2 门限 endpoint_allowed=-1.0，仍在设计允许的 ≤1cm 模型侵入内）；
- `stats.min_square_clearance`/`min_clearance` 与独立测量不一致：**1**——这是中间旋转方案下
  stats 按保守 `half_max` 计算、而独立校验按执行 heading 计算的误差（2 例，差 ≤0.9cm，
  stats 更保守），非安全缺陷；其余 2259 条完全一致；
- 相邻重复点：**0/3119**（`dedupe_path` 修复生效；start==goal 的 2 点零长特例按合法路径处理）；
  NaN/越界/waypoints 不一致：0。

**失败率高的原因（不是 bug）**：任意随机 yaw 的投影半长 H∈[15,21.22]，通过 30/35cm 通道需要
≤30/35cm 宽，任意 yaw 大多物理不可行；旋转点要求欧氏 ≥23.22 只存在于开阔区；且 hard tier 还要求
采样净空 ≥0.5。故 **803/3000** 的 `no-rotation` 失败主要来自“该朝向在此地图/通道物理不可行/无安全
旋转点”，属 fail-safe，与 README 已声明的限制一致。`endpoint` 失败由 v2.1 的 1 降到 **0**
（M3 端点 −1cm 容差生效）。

**1 条可疑假无解**：`(15,200,-8.8°)->(205,145,-9.8°)`（yaw 差 1°<deadband，无旋转需求）：
5cm 独立网格 A*（同保守模型、门限 0）可达，但规划返回 no-solution。由于该独立网格用 need=0，
而 Planner hard tier 要求采样 ≥0.5cm，属“分辨率/门限差”而非确定的实现缺陷；记为残余风险，不作为
Major。若需严格，可用与 hard 门限一致的独立 A*（need=0.5）复核。

### 3.3 专项用例（`fuzz_check targeted` / `diag_stages`）

| 用例 | 结果 |
|---|---|
| 40 条任务点位对（2 zone × 5 点，含 HOME/STORAGE/PROCESSING、目标 yaw 90/180） | **40/40 成功且独立校验通过**（v2.2 复跑仍 40/40） |
| 35cm 横向通道直线穿越（yaw 0） | n=2，min_sq=2.00cm，tight，通过 |
| 30cm 十字通道穿越（yaw 0） | 规划 n=7 绕行（未从 30cm 缝硬穿），独立校验通过（README 预期行为） |
| 30cm 通道内要求 90° 旋转 | **失败 fail=NO_ROTATION_POINT**（不会在窄通道旋转，fail-safe，符合设计） |
| 同点不同 yaw（点不可旋转） | 成功：经 via 安全点绕行，rotation 点欧氏 28.28cm |
| 同点不同 yaw @旋转安全点（v2.1 零长特例） | **n=2、rotation_index=0、stats min_sq=5.00**，独立校验通过（起点原地转） |
| 起点可旋转（开阔区）+ 100° 目标 yaw（v2.1） | **n=2、rot=0、min_sq=7.62**，通过（先转后直线平移） |
| 任意目标 yaw 37° / 开阔区 | 成功：回退“到终点再转”（rotation_index=len-1，goal 旋转安全） |
| 任意目标 yaw −123°（开阔终点，v2.1） | **n=3、rot=2（=len-1）、min_sq=2.01**，通过（到终点再转） |
| 端点 −0.5cm 模型侵入（25cm 豁免区，v2.2 M3 修复） | **通过：n=3、tier=1、fail=0、min_sq=-0.50**，独立校验通过；反向同样通过 |
| 端点 −1.5cm 模型侵入（超 -1cm 容差，v2.2） | **clean fail：n=0、fail_reason=ENDPOINT(1)**，符合设计 |
| 目标在障碍内 / 不可摆放位姿 | 返回 0（fail-safe）；fail_reason 仍被覆盖为 no-rotation（Minor） |
| out_cap=2..5 极小容量 | 返回 0 或无越界；末尾 canary 未破坏 |
| 重复规划同输入 | n/rotation/heading 完全一致 |

### 3.4 计时（host 参考）

`PlanPathWithYaw` 200 次（随机点对/任意 yaw，含内部 RebuildClearance）v2.2 平均 **2.8ms/次**
（Lead 的 40 任务点对：avg 1.2~2.4ms、max ~32-34ms，随机器波动）。MCU 未实测；High2 下规划期间
不阻塞电机控制/急停，最坏耗时留待实车记录（§5）。

## 4. 重点结论逐项回答

1. **A\* 反向端点逻辑（HOME/Processing 净空=0、格中心 -5cm）**：净空场/欧氏场与独立几何完全一致；
   40/40 任务点对（含 HOME/STORAGE/PROCESSING）成功；端点容差与 pen 语义在规划/执行两侧一致
   （规划 hard 端点 −1cm 容差已接入 A\*，执行监测另允许 -1cm；端点 −0.5cm 特例通过、−1.5cm clean fail）。
2. **A\* 搜索净空=0 vs 最终安全**：未发现“搜索可走但最终漏出碰撞路径”——后续 refine/simplify/dedupe 后
   按 tier 完整 validate；独立 1cm SAT 在 2260 条成功路径上 0 碰撞。搜索仅用于连通性（端点邻域按
   endpoint_allowed 放宽、其余 required=0），最终门限（soft 4/ hard 1，采样容差 0.5；端点 soft 2 /
   hard −1）由 `Planner_SquarePathValidate` 把关。
3. **refine 的逐点优化**：候选点自身净空 ≥0 且评分包含两段内部采样与自身净空；不修改端点；
   容量越界有检查；`PLANNER_REFINE_MAX_PASSES=2`、偏移/步长有界 → 无死循环/无端点破坏；独立校验通过。
4. **旋转方案**：`rotation_index` 语义（0=起点、i=中间、len-1=终点）与端口执行一致；旋转点独立
   欧氏 ≥23.22 全部满足；via 拼接长度/容量/rot_idx 正确（v2.1 改为第二段直接写输出尾部，已复核
   `len_a/out_cap/total` 边界）；heading_before/after 与执行使用一致（起点 0 段/终点 len-1 段特例正确；
   新增零长特例 rotation_index=0 与执行层先转后走一致）。
5. **执行期**：航点 yaw 选择、旋转阶段监测、端点豁免、超时/abort/dry_run 路由与 v1 一致；
   **M1 已修复复核**：不可旋转终点不会收到 goal_yaw/BOTH 目标；旋转阶段门限 20.72 偏松（Minor）。
6. **数值/溢出/UB/C99**：armcc AC5（5 Planner 文件 + freertos.c）**0 warning**；
   `cell_square_clearance_cm` 用 int8 基值+浮点偏移；`map_cell_in_bounds` 前后都有；A* g/f 有
   uint16 饱和；未发现除零/越界/未初始化；`MapCore_Init` NULL 缓冲有检查；
   `dedupe_path` 边界（len<2/输出退化）有保护。IRAM1 余量 8.7KB / IRAM2 余 8.7KB（M2 关闭）。
7. **文档与代码**：README 已更新为方形语义；仍有 2 处轻微不一致（fail_reason 覆盖、
   旋转监测门限/port.h 头注释），不影响功能。

## 5. GO/NO-GO 与上车条件

**GO（v2.2 最终结论）。** v2.2 已满足：
1. M1（执行层安全）修复并逐分支复核通过；
2. M2（资源/调度）关闭：IRAM1 105952/114688=92.4%（余 8736B）、IRAM2 余 8744B、
   Planner High2 低于电机控制任务；
3. M3（端点 −1cm 容差接入 A\*）修复：端点 −0.5cm 特例通过、−1.5cm clean fail ENDPOINT；
4. 独立复跑（`run_review.ps1`）全模块 exit=0、`problems=0`：0 安全违例、40/40 任务点对、
   dup=0、armcc 5 文件 0 warning、全量 112 `.c` + armlink OK；
5. targeted/随机模糊 3119 次（2260 成功）独立 SAT/方形净空/旋转校验 0 违例。

**保留的上车条件（非代码阻断）**：
1. 实车低速（60~80cm/s）全流程：`planner map`→逐点 `planner dry`→HOME/STORAGE/PROCESSING
   全任务，观察 square clearance/rotation/track deviation/final pose error 日志；
2. 记录目标板最坏 `PlanPathWithYaw` 耗时（via 分支）与实车定位噪声下的终点精度；
3. 上车前对最终产物再做一次干净全量构建确认（当前 Lead 已构建，本评审引用的数字即此产物）。

## 6. 剩余风险（物理现实，非代码缺陷）

- ENDPOINT 豁免：v2.2 已支持端点区 ≤1cm 模型侵入（端点 −0.5cm 特例通过、−1.5cm 超容差
  clean fail ENDPOINT）。实车任务点若因摆放/OPS 偏差侵入模型 0~1cm 现在可规划；超过 1cm
  仍会 ENDPOINT 失败（fail-safe）。
- HOME/STORAGE/PROCESSING 方形净空=0（恰好相切）：任何定位/摆放误差都可能触发端点失败或执行期
  停车；30cm 通道对 30×30 车“刚好塞满”（采样门限 0.5 不允许），规划默认绕行/失败。
- 任意非 90° 倍数 yaw：H 最大 21.22cm，需要 ≥42.4cm 通道；本场地大量 30/35cm 通道，规划失败属
  fail-safe；任务链路 yaw 均为 90° 倍数，不受影响。
- 原地旋转要求欧氏 ≥23.22cm，只在开阔区；中途旋转点位置阈值 2cm 会消耗部分余量（Minor 建议收紧）。
- 执行期监测是最后兜底，不能替代机械/电气急停；OPS 定位噪声下 1.2cm/2.5° 终点精度需实车标定。

---

## 7. v2.3 执行层热修复复核（实车旋转假告警）

### 7.1 实车现象与根因（Lead 提供，代码复核一致）

- 现象：GRAB→PROCESSING 规划 6 wp/357cm，rotate@0（−1°→180°，clear=25.4cm）；执行旋转时
  `wait_waypoint` 报 `square clearance -1.03cm < -1.00cm, emergency stop`；重规划后在 −40° 起点旋转
  同样触发 → Mission Failed。
- 根因（复核成立）：旋转阶段仍按“实际 yaw 的轴对齐保守方形”
  `MapCore_PointSquareClearanceCm(pose, H(actual_yaw))` 监测。45° 时 H=21.21cm 的 AABB 扩张在
  障碍**角**附近会明显低估真实旋转安全：GRAB 处真实扫掠圆清障 25.8cm，但 45° 方形保守模型可算出
  -0.2~-1.0cm，叠加漂移/定位误差后触发 -1.03cm < -1.00 的端点门限 → 误停车；重规划又选起点旋转，
  同样误判。旧的旋转门限 `half_diagonal-0.5=20.72cm` 比真实扫掠包络 21.22cm 还晚 0.5cm，本身偏松。

### 7.2 v2.3 改动（只改 `planner_port.c` / `planner_config.h`，algo/map_core 零改动）

1. `planner_config.h`（`EFDD6F66…`）：新增 `PLANNER_ROTATION_SPEED_DEG_S 90`、
   `PLANNER_ROTATION_ACCEL_DEG_S2 180`、`PLANNER_ROTATION_MONITOR_EXTRA_CM 0.5`。
2. `planner_port.c`（`4764EA41…`）：
   - `wait_waypoint()`：**旋转阶段跳过平移方形监测**（`if (!rotation_phase && ...)`）；旋转安全改用
     真实扫掠包络 `circle_cm < PLANNER_BODY_HALF_DIAGONAL_CM + PLANNER_ROTATION_MONITOR_EXTRA_CM`
     （21.22+0.5=**21.72cm**）才停车；平移阶段方形监测与端点 −1cm 逻辑保持原样（逐行比对未变）。
   - 新增 `send_waypoint_ex(..., angular_speed, angular_accel, ...)`；平移 `send_waypoint()` 仍用原
     `PLANNER_WAYPOINT_ANGULAR_SPEED_DEG_S(220)/ACCEL(350)`，仅 `rotate_in_place()` 用
     `90°/s`、`180°/s²`。
   - 旋转阶段轨迹偏离监测保留：`rotate_in_place` 仍调用 `wait_waypoint(wp_map, wp_map, ..., true)`，
     外层条件含 `rotation_phase`，偏差 = `Planner_PointSegmentDistanceCm(pose, wp, wp)`（零长段实现为
     到点距离，已确认 `ab2<=1e-9` 时 t=0）。

### 7.3 独立复核结论（Lead 提的 4 点）

1. **旋转阶段跳过方形监测是否安全：安全。** 30×30 方形外接圆半径
   `sqrt(15²+15²)=21.2132cm`（配置用 21.22 更保守）。旋转阶段要求中心欧氏清障
   `>=21.72cm`，则任意朝向的方形都完整落在半径 21.2132cm 的圆内 → 与障碍/地图边界至少
   `21.72-21.2132≈0.507cm` 间隙，**不可能碰撞**；且比真实接触包络提前 0.5cm 停车（旧 20.72cm
   门限确实偏松，本次一并修正）。`MapCore_PointClearanceCm` 已确认包含静态矩形 + 动态 10cm 格 +
   地图边界（出图返回 0），与方形净空场的数据范围一致，无“漏掉动态障碍”缺口；10ms 监测周期在
   40cm/s 最大漂移速度下最多 0.4cm/周期 < 0.5cm 余量。
2. **平移阶段方形监测与端点 −1cm 不变：确认未变。** `if (!rotation_phase && ...)` 内逻辑
   （含 `g_plan_path` 端点 25cm 豁免、`allowed=-PLANNER_ENDPOINT_PENETRATION_TOL_CM`）逐行与 v2.2
   一致；端点 `-1cm` 仅在 `rotation_phase==false` 时生效。
3. **`send_waypoint_ex` 参数/速度隔离：确认正确。** 全文件仅 2 个调用点：`send_waypoint()`（平移，
   默认 220/350）和 `rotate_in_place()`（90/180）；无其它路径使用慢速参数，普通航点行为不变。
   旋转超时沿用 `Planner_RotationTimeoutMs(delta)=1500+25*delta`（180°→6s），慢速 90°/s 下 180°
   约 2.0s + 0.5s 加速 < 6s，不会因降速误超时。
4. **旋转阶段轨迹偏离监测保留：确认保留。** 旋转阶段仍每 10ms 检查
   `Planner_PointSegmentDistanceCm(pose, wp, wp) > PLANNER_TRACK_DEVIATION_CM(10cm)` 并急停；
   慢速旋转时位置漂移同时受此 + 21.72cm 包络双兜底。

**编译/构建**：armcc AC5 5 个 Planner `.c` 独立复跑 **5/5 OK、0 warning**（含修改后的
`planner_config.h`/`planner_port.c`）；Lead 全量 112 `.c` + armlink OK，IRAM1 92.4% 与 v2.2 相同。
规划侧零改动 → v2.2 的 40/40 任务点对、host harness 指标继续有效。

### 7.4 残余风险（Minor，不阻断）

- 中途旋转点到达阈值仍为 `PLANNER_WAYPOINT_ARRIVE_DIST_CM=2cm`：若实际停在规划旋转点旁
  1.5~2cm（朝障碍方向），初始圆清障可能 23.22−2=21.22cm < 21.72 而立即触发停车。导航通常
  会继续朝旋转点收敛，且触发是 fail-safe（停车重规划），但建议把中途旋转到达阈值收到 1cm
  或旋转前先做 100~200ms 位置收敛。
- `Loc_Get` 失败时该周期跳过监测（所有执行期监测的公共既有行为）；建议定位丢失时直接减速/停车
  而非继续旋转。
- `wait_waypoint` 在 `NAV_STATE_COMPLETE` 时先返回再监测（10ms 窗口，余量足够）：建议完成时补一次
  终态清障检查，成本极低。

**v2.3 结论：无 Blocker，GO 维持。** 实车修复方向正确（用真实扫掠圆替代轴对齐保守方形），
且新门限严格更安全；建议实车验证时重点观察 GRAB/PROCESSING 起点的旋转日志。

---

## 8. v2.4 参数整定 + 失败诊断日志复核

### 8.1 变更清单（当前工作树）

- `planner_config.h`（`A37DAAC3…`）：`PLANNER_SQUARE_HARD_MARGIN_CM 1.0→2.0`、
  `PLANNER_PATH_SAMPLE_STEP_CM 2.0→1.0`、`PLANNER_VALIDATION_TOL_CM 0.5→0.25`、
  `PLANNER_REFINE_MAX_PASSES 2→3`、`PLANNER_REFINE_TARGET_EXTRA_CM 0.5→1.0`。
- `planner_port.c`（`27238127…`）：规划失败日志新增 `startSq/goalSq/half/nodes` 诊断（仅日志）。
- `planner_algo.c`/`Cstar_map_core.c` 未改（哈希不变），规划搜索/几何逻辑不变，仅阈值/采样变化。

### 8.2 参数自洽性检查（Lead 问题 1）

- **一般路段安全余量更厚**：hard required=2.0，validation_tol=0.25 → 采样门限 **1.75cm**；
  采样步长 1cm（Lipschitz 误差 ≤0.5cm）→ 真实净空最坏 **≥1.25cm > 0**（v2.2 为 ≥0）。
  独立 1cm 采样复核：3000 次 fuzz 成功路径最小方形净空 **2.000cm**（门限 1.75），
  40/40 任务点对通过，一般路段 0 违例。
- **35cm 通道（居中 2.5cm）仍可过**：实测 `(50,192)→(160,192)` n=2、min_sq=2.00、tight 通过；
  refine 目标 hard+1.0=3.0cm 在 35cm 通道物理上限 2.5cm 处饱和，不导致失败（refine 只优化不强制）。
- **30cm 缝（净空 0）继续绕行**：实测 30cm 十字通道穿越 n=11（v2.2 为 n=7，hard 2.0 后绕得更远），
  独立校验通过；不会硬穿 30cm 缝。
- **端点体系：25cm 豁免区仍成立，但 −1cm 渗透容差在 v2.4 失效（见 8.3 M4）**：
  `endpoint_allowed = hard(2.0) − ENDPOINT_SQUARE_TOL(2.0) = 0`，代码只在 <−1 时钳位 →
  允许值 0（相切），不再是 −1；soft tier 仍为 4−2=2。端点 25cm 区判定逻辑本身未变。
- **A\* 连通性**：搜索仍 required=0（0 净空保连通），最终校验按新门限，故 30cm 缝“可搜索但不可
  成行”；未发现搜索/校验口径冲突。

### 8.3 新发现 M4（Major，功能语义回归，非安全）：端点 −1cm 渗透容差失效

- 位置：`planner_algo.c:1185-1189`。`required_cm=2.0` 时 `endpoint_allowed=0.0`，
  钳位条件 `<-PENETRATION_TOL(-1)` 不触发 → hard 端点只允许“相切（0）”，
  **v2.1/v2.2 的“端点区允许 ≤1cm 模型侵入”不再生效**。
- 独立复现（v2.4 代码）：原 M3 两个特例 `(50,192.5,0°)↔(105,195.5,0°)`（端点净空 −0.5cm）
  现在返回 **n=0、fail=ENDPOINT**；3000 fuzz 中成功路径端点最小方形净空 **+0.002cm**
  （v2.2 曾出现 −0.500cm），即已无任何负渗透端点被接受。
- 影响：任务点 HOME/STORAGE/PROCESSING 的坐标方形净空=0（相切）仍能规划（40/40 通过）；
  受影响的是“实车摆放/OPS 偏差使任务点或起点侵入模型 0~1cm”的场景——v2.2 专门修 M3 支持该窗口，
  v2.4 参数把它静默关闭（方向更安全，但属功能回归，需产品确认）。
- 建议（二选一，均一行）：
  1) 保留 −1cm：hard tier 显式 `endpoint_allowed_cm = -PLANNER_ENDPOINT_PENETRATION_TOL_CM;`
     （或新增 `PLANNER_HARD_ENDPOINT_ALLOWED_CM (-1.0f)`），soft 仍 `required−2`；
  2) 确认收紧：把 README/设计文档改为“hard=2.0 后端点仅允许相切，不再支持模型侵入”，
     并确认任务点摆放精度足够（40/40 实测支持）。
- 安全方向上 M4 不是 Blocker（更严格），但**这是一个验收语义变化，建议上车前确认**。

### 8.4 性能检查（Lead 问题 2）

- 采样步长 2→1cm：A* 每条边 `MapCore_SquareSegmentFree` 采样点数 ×2；同时 simplify/validate
  采样 ×2、refine passes 2→3（×1.5）→ 规划总耗时约 2~3×。
- host 实测（200 次随机点对/任意 yaw，含 RebuildClearance）：**5.68ms/次**（v2.2 同口径 2.8ms），
  约 2×；Lead 独立测量 40/40 avg 3.3ms/max 75ms。
- 估算 MCU：约 300~500ms/次（Lead 估计）；Planner 任务优先级已 **High2**（低于 Motor_Ctrl High4 /
  Motor_Receive High3），规划为原地阻塞式，不阻塞电机控制/急停链路；但 GoTo 请求延迟与
  `PLANNER_ASTAR_RETRY_MAX=3` 次重试的最坏总时长（~1.5s）需实车确认可接受。
- 结论：性能风险可控，无 Blocker；建议实车记录 plan_ms（report 已有字段）与最坏 via 分支耗时。

### 8.5 失败日志字段检查（Lead 问题 3）

- `planner_port.c:586-600`：格式串 13 个转换符与 13 个实参一一对应：
  `%s`(fail_reason_text)、6×`%.1f/%.0f`(start/goal xy+deg，float 变参提升 double)、
  `%d`(PLANNER_ASTAR_RETRY_MAX int)、`%.2f/%.2f`(startSq/goalSq)、`%.1f/%.1f`(half)、
  `%u`(`(unsigned int)stats.expanded_nodes`) —— **类型/数量匹配，无格式化 UB**。
- 该日志只在规划失败路径执行一次（非 10ms 周期），`logPrintln` 开销可忽略；字段能区分
  “端点方形净空被卡死（startSq/goalSq 低）”与“搜索无解（nodes 大）”，达到诊断目的。
- 提示：`check_mission_point_clearance()` 的 warning 阈值仍用 `PLANNER_SQUARE_HARD_MARGIN_CM`
  （现 2.0），5 个任务点净空=0 → 每次初始化都 warning；属预期告警（可接受），若嫌吵可改为
  `< PLANNER_ENDPOINT_PENETRATION_TOL_CM` 或直接看端点相切语义（Nit）。

### 8.6 v2.4 复跑统计

| 项目 | 结果 |
|---|---|
| 3000 次静态随机（任意 yaw，可摆放位姿） | 成功 **2194**（no-rotation 803、no-solution 2、endpoint 1），安全违例 **0** |
| 成功路径独立校验 | SAT 碰撞 0、方形净空违规 0、端点/heading/rotation 0、旋转点欧氏最小 23.259cm、dup=0 |
| 最小方形净空 | 一般段 **2.000cm**（门限 1.75）、端点 **+0.002cm**（当前门限 0） |
| 40/40 任务点对 | 全过；35cm 通道 min_sq=2.00 tight；30cm 缝 n=11 绕行；通道内旋转 fail-safe |
| targeted | **exit=2**：仅因 8.3 的 2 个端点 −0.5cm 用例按 v2.2 期望已不再通过（非安全失败） |
| armcc AC5 | 5 个 Planner `.c` **0 warning** |
| host timing | 5.68ms/次（200 次随机，含 RebuildClearance） |

### 8.7 v2.4 结论

**GO（条件式）**：参数整定使一般路段真实净空从 ≥0 提升到 ≥1.25cm、修复了 2cm 采样漏检
（实车 0.29cm 急停根因），35cm 通道可控、30cm 缝安全绕行，无安全违例，armcc/全量构建通过。
**唯一需决策项**：hard=2.0 使端点 −1cm 渗透容差静默失效（M4，非安全、方向更严格）。
若任务点均为 0 净空相切、可接受不再支持 0~1cm 摆放偏差 ⇒ **GO**；
若需保留该容差 ⇒ 按 8.3 建议改一行后复跑 targeted（预期 exit=0）再 GO。

---

## 附：本目录交付物
- `fuzz_check.c`：v2 API 独立对抗性验证（SAT/方形净空/旋转/动态障碍/专项/容量/计时/端点渗透容差）
- `diag_stages.c`：v2 分阶段诊断 + “存在合法旋转方案”独立存在性证明
- `port_distance_repro.c`：执行层纯函数回归（纯函数签名兼容，16 项断言全过）
- `run_review.ps1`：一键复跑（v2.2 本机验证：全模块 exit=0、armcc 0 warning、`problems=0`；
  v2.3 执行层改动不参与 host harness，规划指标沿用 v2.2）
