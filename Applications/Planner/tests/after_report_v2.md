# 重构前后对比报告（v2：30×30 方形车体）：HEAD 6413fac → v2 工作区

> 车体模型：30×30cm 方形（半边长 15cm，半对角线 21.22cm）；旋转安全欧氏门限 23.22cm。
> 独立真值：精确 SAT（旋转正方形 vs 轴对齐障碍矩形 + 地图边界），逐段 1cm 采样。
> 执行语义：平移段保持航向，仅 rotation_index 处原地旋转；航点 i<=rotation_index 用 heading_before，
> 否则 heading_after；rotation_index=len-1 表示到终点后再转（branch 1b）。
> 端点豁免区=到起点/终点欧氏 ≤25cm：hard tier 允许 ≤1.0cm 模型侵入（endpoint_allowed=-1cm，
> 一般路段仍要求真实净空 ≥0）；起终点几乎重合（<1cm）时允许 2 点零长路径（唯一合法重复）。
> 基线：v1 HEAD 路径按旧执行语义（除最后一段保持 start_yaw，最后一段 goal_yaw）重测方形真值。
> 生成：`tests/harness/v2_report.py --v2`。

## 0. 摘要

- 规划成功：v2 **40/40**（失败 0）。
- 方形穿障路径（一般路段 SAT 相交或端点区侵入 >1cm）：旧基线 **26/40** 条 → v2 **0** 条。
- 最大穿透深度：旧基线 **6.52cm** → v2 **0.000cm**。
- 全局最小方形 signed clearance：旧基线 -6.52cm → v2 **0.00cm**（0=相切，30cm 通道两端即为 0）。
- 端点豁免区侵入超限：v2 **0** 条；旧基线同一规则下端点区超限采样 **0** 个。
- 旋转方案：**14** 个（起点旋转 4 / 终点补转 4 / 中途 6）；最小独立旋转清障 **23.50cm**，不安全 **0** 个。
- tier 分布：soft **4** / tight **36**。平均长度 **231.7cm**；规划耗时 avg **1.48ms** / max **35.60ms**。
- 相邻重复航点：0 条（修复前 5 条，Lead 已修复；当前 min 相邻间距 4.977cm）。
- 旧基线终点补转：14 条需要终点补转，其中独立欧氏清障 <23.22cm（不安全）**10** 条。
- Python SAT 交叉校验 C 侧指标：40 条路径，不一致 **0** 条。

## 1. 汇总对比

| 指标 | 旧基线（v1 HEAD，方形真值） | v2 工作区 |
|---|---:|---:|
| 规划成功 | 40/40 | 40/40 |
| 方形穿障路径（一般相交或端点超限） | 26/40 | 0 |
| 一般路段相交采样 / 端点超限采样 | 504 / 0 | 0 / 0 |
| 最大穿透深度 | 6.52cm | 0.000cm |
| 最小方形 signed clearance | -6.52cm | **0.00cm** |
| 旋转方案数 | 无（v1 无旋转规划） | 14 |
| 最小独立旋转清障 | 终点补转不安全 10 条 | 23.50cm（全部≥23.22） |
| soft / tight | — | 4 / 36 |
| 平均路径长 | 179.1cm | 231.7cm |
| 平均/最大规划耗时 | — | 1.48ms / 35.60ms |

## 2. 任务点对明细（旧基线 vs v2）

| zone | 点对 | yaw | 旧:航点/长度 | 旧:SAT相交/端点超限/最大穿透 | v2:航点/长度 | v2:rot | v2:最小signed | v2:最大穿透 | v2:tier |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---|
| 1 | HOME -> MATL_GRAB | 0→0 | 2 / 105.9cm | 0 / 0 / 0.00cm | 3 / 106.1cm | 无 | 0.00 | 0.00 | tight |
| 1 | HOME -> MATL_PROCESSING | 0→180 | 6 / 267.7cm | 31 / 0 / 4.78cm | 6 / 289.4cm | 1 | 0.00 | 0.00 | tight |
| 1 | HOME -> MATL_STORAGE | 0→90 | 6 / 267.7cm | 31 / 0 / 4.78cm | 7 / 284.6cm | 2 | 0.00 | 0.00 | tight |
| 1 | HOME -> QrCode | 0→0 | 2 / 105.5cm | 0 / 0 / 0.00cm | 3 / 110.7cm | 无 | 0.00 | 0.00 | tight |
| 1 | MATL_GRAB -> HOME | 0→0 | 2 / 105.9cm | 0 / 0 / 0.00cm | 3 / 106.1cm | 无 | 0.00 | 0.00 | tight |
| 1 | MATL_GRAB -> MATL_PROCESSING | 0→180 | 2 / 176.0cm | 0 / 0 / 0.00cm | 6 / 356.6cm | 0(起点) | 0.00 | 0.00 | tight |
| 1 | MATL_GRAB -> MATL_STORAGE | 0→90 | 4 / 170.1cm | 14 / 0 / 4.67cm | 5 / 178.6cm | 0(起点) | 0.00 | 0.00 | tight |
| 1 | MATL_GRAB -> QrCode | 0→0 | 4 / 172.5cm | 6 / 0 / 1.52cm | 4 / 183.5cm | 无 | 6.00 | 0.00 | soft |
| 1 | MATL_PROCESSING -> HOME | 180→0 | 6 / 270.4cm | 23 / 0 / 4.67cm | 6 / 287.8cm | 3 | 0.00 | 0.00 | tight |
| 1 | MATL_PROCESSING -> MATL_GRAB | 180→0 | 2 / 176.0cm | 0 / 0 / 0.00cm | 6 / 355.1cm | 5(终点) | 0.00 | 0.00 | tight |
| 1 | MATL_PROCESSING -> MATL_STORAGE | 180→90 | 4 / 164.1cm | 14 / 0 / 4.67cm | 7 / 177.9cm | 3 | 0.00 | 0.00 | tight |
| 1 | MATL_PROCESSING -> QrCode | 180→0 | 4 / 172.7cm | 6 / 0 / 1.52cm | 5 / 177.2cm | 4(终点) | 0.00 | 0.00 | tight |
| 1 | MATL_STORAGE -> HOME | 90→0 | 6 / 270.4cm | 23 / 0 / 4.67cm | 6 / 282.8cm | 3 | 0.00 | 0.00 | tight |
| 1 | MATL_STORAGE -> MATL_GRAB | 90→0 | 4 / 170.1cm | 14 / 0 / 4.67cm | 5 / 177.1cm | 4(终点) | 0.00 | 0.00 | tight |
| 1 | MATL_STORAGE -> MATL_PROCESSING | 90→180 | 4 / 164.1cm | 14 / 0 / 4.67cm | 7 / 177.9cm | 3 | 0.00 | 0.00 | tight |
| 1 | MATL_STORAGE -> QrCode | 90→0 | 2 / 180.0cm | 0 / 0 / 0.00cm | 6 / 355.2cm | 5(终点) | 0.00 | 0.00 | tight |
| 1 | QrCode -> HOME | 0→0 | 2 / 105.5cm | 0 / 0 / 0.00cm | 3 / 110.7cm | 无 | 0.00 | 0.00 | tight |
| 1 | QrCode -> MATL_GRAB | 0→0 | 4 / 172.5cm | 6 / 0 / 1.52cm | 4 / 183.3cm | 无 | 6.00 | 0.00 | soft |
| 1 | QrCode -> MATL_PROCESSING | 0→180 | 4 / 172.7cm | 6 / 0 / 1.52cm | 5 / 178.9cm | 0(起点) | 0.00 | 0.00 | tight |
| 1 | QrCode -> MATL_STORAGE | 0→90 | 2 / 180.0cm | 0 / 0 / 0.00cm | 6 / 356.9cm | 0(起点) | 0.00 | 0.00 | tight |
| 2 | HOME -> MATL_GRAB | 0→0 | 6 / 273.7cm | 31 / 0 / 4.78cm | 4 / 287.2cm | 无 | 0.00 | 0.00 | tight |
| 2 | HOME -> MATL_PROCESSING | 0→0 | 2 / 106.9cm | 18 / 0 / 2.79cm | 5 / 109.9cm | 无 | -0.00 | 0.00 | tight |
| 2 | HOME -> MATL_STORAGE | 0→0 | 6 / 267.8cm | 49 / 0 / 4.67cm | 6 / 469.9cm | 无 | 0.00 | 0.00 | tight |
| 2 | HOME -> QrCode | 0→0 | 2 / 105.5cm | 0 / 0 / 0.00cm | 3 / 109.1cm | 无 | 0.00 | 0.00 | tight |
| 2 | MATL_GRAB -> HOME | 0→0 | 6 / 276.4cm | 23 / 0 / 4.67cm | 4 / 291.4cm | 无 | 0.00 | 0.00 | tight |
| 2 | MATL_GRAB -> MATL_PROCESSING | 0→0 | 2 / 176.0cm | 0 / 0 / 0.00cm | 6 / 360.7cm | 无 | -0.00 | 0.00 | tight |
| 2 | MATL_GRAB -> MATL_STORAGE | 0→0 | 4 / 170.1cm | 14 / 0 / 4.67cm | 4 / 182.7cm | 无 | 0.00 | 0.00 | tight |
| 2 | MATL_GRAB -> QrCode | 0→0 | 4 / 180.1cm | 14 / 0 / 4.67cm | 4 / 183.3cm | 无 | 5.47 | 0.00 | soft |
| 2 | MATL_PROCESSING -> HOME | 0→0 | 2 / 106.9cm | 18 / 0 / 2.79cm | 5 / 109.9cm | 无 | -0.00 | 0.00 | tight |
| 2 | MATL_PROCESSING -> MATL_GRAB | 0→0 | 2 / 176.0cm | 0 / 0 / 0.00cm | 5 / 356.5cm | 无 | -0.00 | 0.00 | tight |
| 2 | MATL_PROCESSING -> MATL_STORAGE | 0→0 | 4 / 164.1cm | 14 / 0 / 4.67cm | 6 / 179.1cm | 无 | -0.00 | 0.00 | tight |
| 2 | MATL_PROCESSING -> QrCode | 0→0 | 4 / 172.4cm | 22 / 0 / 6.52cm | 4 / 178.5cm | 无 | -0.00 | 0.00 | tight |
| 2 | MATL_STORAGE -> HOME | 0→0 | 6 / 267.8cm | 49 / 0 / 4.67cm | 7 / 467.6cm | 无 | 0.00 | 0.00 | tight |
| 2 | MATL_STORAGE -> MATL_GRAB | 0→0 | 4 / 170.1cm | 14 / 0 / 4.67cm | 5 / 177.2cm | 无 | 0.00 | 0.00 | tight |
| 2 | MATL_STORAGE -> MATL_PROCESSING | 0→0 | 4 / 164.1cm | 14 / 0 / 4.67cm | 7 / 177.7cm | 无 | -0.00 | 0.00 | tight |
| 2 | MATL_STORAGE -> QrCode | 0→0 | 2 / 180.0cm | 0 / 0 / 0.00cm | 6 / 359.7cm | 无 | 0.00 | 0.00 | tight |
| 2 | QrCode -> HOME | 0→0 | 2 / 105.5cm | 0 / 0 / 0.00cm | 3 / 109.1cm | 无 | 0.00 | 0.00 | tight |
| 2 | QrCode -> MATL_GRAB | 0→0 | 4 / 180.1cm | 14 / 0 / 4.67cm | 4 / 183.3cm | 无 | 6.00 | 0.00 | soft |
| 2 | QrCode -> MATL_PROCESSING | 0→0 | 4 / 172.4cm | 22 / 0 / 6.52cm | 5 / 178.9cm | 无 | -0.00 | 0.00 | tight |
| 2 | QrCode -> MATL_STORAGE | 0→0 | 2 / 180.0cm | 0 / 0 / 0.00cm | 5 / 361.0cm | 无 | 0.00 | 0.00 | tight |

## 3. 旋转方案明细

| zone | 点对 | rotation_index | 点数 | heading_before | heading_after | 独立旋转清障cm | 位置 |
|---:|---|---:|---:|---:|---:|---:|---|
| 1 | HOME -> MATL_STORAGE | 2 | 7 | 0 | 90 | 27.46 | 中途旋转 |
| 1 | HOME -> MATL_PROCESSING | 1 | 6 | 0 | 180 | 23.50 | 中途旋转 |
| 1 | QrCode -> MATL_STORAGE | 0 | 6 | 0 | 90 | 25.00 | 起点先转 |
| 1 | QrCode -> MATL_PROCESSING | 0 | 5 | 0 | 180 | 25.00 | 起点先转 |
| 1 | MATL_GRAB -> MATL_STORAGE | 0 | 5 | 0 | 90 | 25.81 | 起点先转 |
| 1 | MATL_GRAB -> MATL_PROCESSING | 0 | 6 | 0 | 180 | 25.81 | 起点先转 |
| 1 | MATL_STORAGE -> HOME | 3 | 6 | 90 | 0 | 25.81 | 中途旋转 |
| 1 | MATL_STORAGE -> QrCode | 5 | 6 | 90 | 0 | 25.00 | 到终点再转 |
| 1 | MATL_STORAGE -> MATL_GRAB | 4 | 5 | 90 | 0 | 25.81 | 到终点再转 |
| 1 | MATL_STORAGE -> MATL_PROCESSING | 3 | 7 | 90 | 180 | 25.81 | 中途旋转 |
| 1 | MATL_PROCESSING -> HOME | 3 | 6 | 180 | 0 | 25.81 | 中途旋转 |
| 1 | MATL_PROCESSING -> QrCode | 4 | 5 | 180 | 0 | 25.00 | 到终点再转 |
| 1 | MATL_PROCESSING -> MATL_GRAB | 5 | 6 | 180 | 0 | 25.81 | 到终点再转 |
| 1 | MATL_PROCESSING -> MATL_STORAGE | 3 | 7 | 180 | 90 | 25.81 | 中途旋转 |

## 4. v2 验收断言（acceptance_v2.csv）

| 检查项 | 结果 | 详情 |
|---|---|---|
| 40/40 任务点对全部规划成功 | PASS | 成功 40/40，失败 0 |
| 一般路段方形 SAT 不相交 | PASS | SAT 碰撞点对=0，全局最小 signed=-0.0000cm（>=-0.02 视为相切安全） |
| 端点豁免区模型侵入 <=1.0cm | PASS | 超限点对=0（端点区=到起终点欧氏<=25cm，允许侵入<=1.0cm） |
| 旋转点独立欧氏清障 >=23.22cm | PASS | 旋转方案=14，不安全=0，最小独立旋转清障=23.50cm（门限23.22cm） |
| 朝向方案语义正确（before=起始yaw / after=目标yaw） | PASS | 语义错误点对=0（不旋转时两者均=目标 yaw） |
| 起终点精确一致 / 相邻点无重复 | PASS | 端点异常点对=0，相邻点重复点对=0，合法零长路径特例=0（起终点<1cm，允许两点相同） |
| stats.waypoints/length 与实测一致 | PASS | 不一致点对=0（长度差<=0.5cm） |
| tier 分布 | INFO | soft=4 tight=36（zone1 2/18, zone2 2/18） |
| 旋转方案与耗时 | INFO | 旋转方案=14（起点旋转 4 / 终点补转 4）；平均长度=231.7cm；规划耗时 avg=1.48ms max=35.60ms |

- 验收断言：**7 PASS / 0 FAIL**。

## 5. v2 边界/回归用例（edge_results_v2.csv）

| 用例 | 名称 | 结果 | 详情 |
|---|---|---|---|
| E1 | 10cm 短距直达 2 点 + soft + SAT 安全 | PASS | 点数=2(期望2) tier=0(期望soft=0) rot=65535 len=10.00cm SAT_min=10.000 穿透=0.000 |
| E2 | 目标在障碍内部 → clean fail(ENDPOINT) | PASS | goal=(60,145) 在方框 [35,90]x[120,175] 内：返回=0 fail_reason=1(期望ENDPOINT=1) |
| E3a | 起点在地图外 → clean fail | PASS | start world=(500,0)cm 在地图外：返回=0 fail_reason=1（期望非0、不崩溃） |
| E3b | 目标在地图外 → clean fail | PASS | goal world=(-300,0)cm 在地图外：返回=0 fail_reason=1（期望非0、不崩溃） |
| E10 | 旧 v1 端点 (100,100) 对 30×30 方形不可达 → clean fail | PASS | world(100,100) 距方框角 14.142cm，30×30 方形中心在此侵入方框角 5cm > 端点容差 1cm：返回=0 fail_reason=1（30cm 方形车几何不可达，属正确拒绝） |
| E4 | zone1 HOME 端点 8 条路径全部成功且方形 SAT 安全 | PASS | HOME 相关 8/8: 成功=8 安全=8 旋转方案=4 | SAT_min=0.00cm 最小旋转清障=23.50cm |
| E5a | 40cm 通道 yaw 0/90/180 全部成功、soft 且 SAT 安全 | PASS | 40cm 通道 yaw 0/90/180：成功=3/3 安全=3/3 soft=3/3 SAT_min=5.00cm(len 10.0/10.0/10.0) |
| E5b | 35cm 通道成功、tight 且 SAT 安全 | PASS | 35cm 通道：成功=1 tier=1(期望tight=1) SAT_min=2.50cm 穿透=0.000 旋转=65535 |
| E5c | 30cm 短通道（端点豁免区内）可 tight 通过 | PASS | 30cm 短通道(len20cm，全在25cm端点区)：成功=1 tier=1 SAT_min=-0.000cm 穿透=0.000 |
| E5d | 30cm 长通道穿越 → clean fail（安全优先正确行为） | PASS | 30cm 长通道(80cm)：clean fail fail_reason=2（端点=1/无解=2）——安全优先正确行为，不要求改 Planner |
| E6a | 任意起始 yaw(10/30/45°) 开阔点成功且 SAT 安全 | PASS | 开阔点 yaw 10/30/45 直达：成功=3/3 安全=3/3 SAT_min=3.79cm |
| E6b | HOME 起点任意 yaw 干净失败并给出 fail_reason | PASS | HOME(yaw10)->QrCode：返回=0 fail_reason=1（方形扫入边界，期望 clean fail=ENDPOINT） |
| E6c | 旋转安全起点任意 yaw → 起点旋转后平移且安全 | PASS | (10,75,yaw45)->(10,85,yaw0)：成功=1 rot=0(期望0) 旋转清障=25.00cm SAT_min=10.00cm |
| E7a | 终点再转分支（起点不可转+终点旋转安全） | PASS | HOME(yaw0)->(10,75,yaw90)：成功=1 rot=2(n-1=2) 终点旋转清障=25.00cm SAT_min=0.00cm |
| E7b | 中途旋转方案语义正确且旋转点独立清障>=23.22cm | PASS | HOME->MATL_STORAGE(yaw0->90)：成功=1 rot=2 旋转清障=27.46cm SAT_min=0.00cm hb=0.0 ha=90.0 |
| E7c | 所有旋转点独立欧氏清障 >=23.22cm（不在窄通道内旋转） | PASS | zone1 20 点对：旋转方案=14 不安全=0 最小独立旋转清障=23.50cm（门限 23.22cm） |
| E7d | 任意目标 yaw 经窄通道→到终点再转分支（rotation_index=len-1） | PASS | 任意目标 yaw(30/37/45°）HOME→(10,75)：成功=3/3 均到终点再转=1 安全=3/3 rot_idx=2/2/2 旋转清障=25.00/25.00/25.00cm |
| E8 | 同一对点重复规划结果稳定（5 次完全一致） | PASS | 连跑 5 次：waypoints=6 len=356.9064cm rot=0 tier=1 min_sq=-0.0000 结果一致=1 |
| E11a | 起终点重合 → 合法 2 点零长路径（允许两点相同） | PASS | 同点(10,75,0)->(10,75,0)：成功=1 点数=2 两点间距=0.000cm 长度=0.000cm（合法零长路径特例，允许重复） |
| E11b | 起终点重合需转向 → 起点原地转且旋转安全 | PASS | 同点(10,75,45)->(10,75,0)：成功=1 点数=2 rot=0 独立旋转清障=25.00cm len=0.000cm |
| E9a | 加入动态障碍后旧路径已不满足方形安全 | PASS | 动态格=2 @ grid(34,25)+(35,25)≈world(105.0,15.0)cm：旧路径 SAT 碰撞=50 最大穿透=10.00cm |
| E9b | 动态障碍下重规划成功且新路径方形 SAT 安全 | PASS | 重规划：成功=1 点数=8(原6) len=364.9cm(原356.9) rot=0 SAT(含动态)安全=1 路径改变=1 |
| E9c | 清除动态障碍后恢复原路径能力 | PASS | 清除动态格后：成功=1 len=356.9cm(原356.9) SAT_min=0.00cm |

- 边界用例：**23 PASS / 0 FAIL**（0 INFO/WARN）。

## 6. 结论与风险

1. **方形车体安全达标**：v2 40/40 成功；一般路段方形 SAT 相交 **0** 条、最大穿透 **0.000cm**；
   旧基线在同一 SAT 真值下 26/40 条穿障（一般相交或端点超限）、最大穿透 6.52cm，说明 v1 圆模型对 30×30 方形确实过于乐观。
2. **旋转安全**：14 个旋转方案全部安排在独立欧氏清障 ≥23.22cm 处（最小 23.50cm），
   含起点先转、中途旋转与到终点再转（branch 1b）三种；未在 40/35cm 窄通道内旋转。
3. **端点语义**：HOME 处方形净空=0（贴场边条）属 tight 端点；30cm 通道净空=0，
   仅当穿越段完全落在 25cm 端点豁免区时允许通过；更长 30cm 通道（一般路段硬门限 0.5cm 不可满足）
   返回 clean fail（reason=1/2），属安全优先的正确行为（Lead 已确认，不需修改 Planner）。
   端点 hard tier 的 `endpoint_allowed` 已修正为 -1cm（此前误 clamp 到 0），本报告独立 SAT 按
   “一般路段不穿透、端点区允许 ≤1cm”校验，40/40 任务点对端点侵入超限 = 0。
4. **已修复问题**：评测期间发现并上报“旋转点插入产生相邻重复航点”（5/40 条）；Lead 修复后
   `insert_rotation_point` 直接复用已有顶点，并新增 `dedupe_path()` 清理相邻零长段；重跑后
   40/40 任务点对 0 重复（唯一合法例外：起终点重合 <1cm 的 2 点零长路径，已在 edge E11 覆盖）。
5. **性能**：规划 avg 1.48ms / max 35.60ms（主机，-O2）；平均路径长 231.7cm（绕行保证方形安全）。
6. **残余风险/需 Lead 决策**：旧 v1 端点 (100,100) 对 30×30 方形不可达（中心会侵入方框角 5cm，
   超出端点 1cm 容差），v2 正确拒绝；若任务场景确实需要该点，需要重新定义任务点或放宽端点容差，
   否则实车会 clean fail。

## 7. 复现

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File Applications\Planner\tests\harness\run_new.ps1
py Applications\Planner\tests\harness\v2_report.py --v2 \
    --baseline-dir Applications\Planner\tests\results --new-dir %TEMP%\planner_harness_v2\new_out \
    --out-baseline Applications\Planner\tests\baseline_report_v2.md \
    --out-after Applications\Planner\tests\after_report_v2.md
```
