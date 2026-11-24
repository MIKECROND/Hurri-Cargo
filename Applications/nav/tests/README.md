# 导航控制主机端仿真（nav host-side simulator）

本目录是**独立于固件的 nav 控制主机端仿真/回归工具**：在 PC 上编译**真实的 `nav_core.c`**（以及真实
`kinematics.c`、`nav_math.c`），用参数化的被控对象模型 + OPS 定位模型驱动它，量化"速度/精度/延迟影响"，
用于：

1. 没有实车时对比控制方案、回归 nav 改动（把 `nav_core.c` 的真实行为逐 5ms 步进）；
2. 量化**电机命令链端到端延迟**（0/10/20/50/100ms）对到位时间、超调、精度、振荡的影响；
3. 扫描 `nav par` 运行时整定参数与 `nav_config.h` 编译期宏，给出有数据支撑的推荐值；
4. 输出 `nav_sim_report.md`（延迟-精度-时间曲线、参数敏感度排序、STOPPING 等待对比、模型不确定度）。

> 仿真模型是**理想地面 + 一阶轮速滞后 + 纯延迟**，不是实车标定：结论用于比较方案/找趋势/缩小实车整定范围，
> 绝对数值需实车校核。模型假设与不确定项见下文第 4 节。

## 1. 快速开始（一条命令）

```powershell
powershell -NoProfile -ExecutionPolicy Bypass `
    -File Applications\nav\tests\harness\run_sim.ps1
```

默认输出到 `%TEMP%\nav_harness\`（不写仓库 `build/`）：

| 输出 | 说明 |
|---|---|
| `out\runs_baseline.csv` | 冻结基线（改动前 nav_core.c）场景 × 延迟结果，每行一个用例 |
| `out\runs_current.csv` | 当前工作区 nav_core.c（Nav_Update 新版）同套件结果 |
| `out\runs_current_unc.csv` | 模型不确定度矩阵（轮速滞后/打滑/OPS 帧率/周期抖动/STOP 制动/OPS 延迟） |
| `out\runs_sw_*.csv` | 运行时整定参数扫描（`nav par` 同名参数，无需重编译） |
| `out\runs_mx_*.csv` | 编译期宏扫描（`NAV_*` 强制包含覆盖头重编译 nav_core.c） |
| `out\<tag>_<scenario>_d<delay>.csv` | 代表性场景 trace（每控制周期一行，含真实位姿/指令/测量） |
| `out\nav_sim_report.md` | 自动生成的 Markdown 汇总报告 |
| `build\`、`baseline_src\` | 中间产物与冻结基线源码 |

常用参数：

```powershell
... -SkipBuild          # 复用已编译 exe
... -NoSweep            # 只跑基线/延迟套件（最快，~30s）
... -NoUncertainty      # 跳过不确定度矩阵
... -NoTrace            # 不输出 trace
... -Verbose            # 兼容 PS 通用参数；打印详细构建/运行信息
... -NavLog             # 每个用例打印一行摘要（同时打开 nav 日志）
... -OutDir D:\tmp\nav  # 换输出目录
```

也可以直接跑单个用例/场景（便于实车问题复现）：

```powershell
$env:TEMP\nav_harness\build\navsim_current.exe --out-dir $env:TEMP\nav_harness\out `
    --tag manual --suite single --scenario line100_70 --delay 50 --verbose
# 参数扫描（当前版本支持 Nav_SetTunable，不用重编译）：
...\navsim_current.exe --out-dir ... --tag sw_test --suite quick --tune yaw_kp=2.4 --tune stop_settle_ms=200
```

## 2. 目录结构

```
Applications/nav/tests/
├── README.md                        本文件（方法/用法/模型假设）
├── baseline/MANIFEST.sha256         冻结基线来源 commit 与源码 SHA-256（脚本每次校验）
└── harness/
    ├── stubs/                       主机端桩头：cmsis_os2.h / FreeRTOS.h / task.h / log.h / zdt_v5_cfg.h
    ├── sim_config.h                 被控对象/OPS/时序参数定义与默认值
    ├── sim_plant.h/.c               麦轮底盘 + 轮速滞后 + 延迟 + 打滑 + 里程积分（真实 Kinematics_Inverse）
    ├── sim_ops.h/.c                 OPS 测量（帧率/延迟/量化/噪声）+ Loc_Get + OPS_Is_Stale 仿真实现
    ├── sim_platform.h/.c            CMSIS/FreeRTOS/MotionControl/log/地图桩 + 电机命令链延迟模型
    ├── sim_runner.c                 场景编排、指标统计、CSV/trace 输出、驱动 Nav_Update 或旧版 Nav_Task
    ├── report.py                    CSV -> Markdown 报告
    └── run_sim.ps1                  一键构建+运行+报告
```

## 3. 仿真链路与工作原理

```
nav_core.c（真实代码，每 5ms 一次）
  -> MotionControl_SetVelocity(vx_body, vy_body, w)          [sim_platform.c 记录命令]
  -> 每 1ms 采样命令进环形缓冲；t 时刻执行 t-cmd_delay 的命令 [纯延迟，模拟总线/队列/驱动器]
  -> Kinematics_Inverse（真实 kinematics.c）                  [机体系速度 -> 四轮角速度]
  -> 整数 RPM 量化（复刻 MotionControl：|rad/s|*60/2π 截断）  [低速度分辨率约 0.38cm/s/档]
  -> 电机加速度斜坡（nav 下发 car_acc 折算，默认=acc/R）      [Emm 速度模式 acc 档位等效]
  -> 轮速一阶滞后 tau（默认 60ms）                            [机械+速度环]
  -> 正运动学还原机体系速度 + 打滑系数(0~10%)                  [地面里程]
  -> 世界系位姿积分（连续 yaw，不归一化）
  -> OPS 测量：帧率 20/50/100Hz、延迟、位置量化 0.1mm、角度噪声
  -> Replicate nav_local.c：帧间保持 + yaw 一阶低通 -> Loc_Get / OPS_Is_Stale
  -> 回到 nav_core.c
```

**驱动方式**

- **当前版本**（`nav_core.h` 有 `void Nav_Update(void)`）：构建时定义 `SIM_HAVE_NAV_UPDATE`，每个控制周期
  先推进 5ms 仿真（被控对象+OPS+场景状态机），再调用一次 `Nav_Update()`；与固件 `Nav_Task` 的
  `osDelay(5); Nav_Update();` 时序一致。`Nav_GetDebug()` 直接用于 trace。
- **冻结基线**（改动前的 `nav_core.c`，只有无限循环 `Nav_Task`）：用 `osDelay()` 桩推进仿真时钟并在
  场景结束时 `setjmp/longjmp` 退出，控制代码仍是 100% 真实 `nav_core.c`。

**基线快照**：`run_sim.ps1` 从 git commit `b6f3ddb`（清单见 `baseline/MANIFEST.sha256`，脚本内同样固化
这些 SHA-256）导出改动前的 `nav_core.c / nav_core.h / nav_config.h` 到
`%TEMP%\nav_harness\baseline_src\`，逐个校验哈希后再编译。
这样即使工作区代码继续变化，"改动前"基线仍可复现（需要该 commit 在仓库历史中）。

## 4. 场景与指标

| 场景 | 内容 |
|---|---|
| `line100_30/50/70/100` | 直线 100cm，目标速度 30/50/70/100cm/s（acc=110/dec=70，与 map 点位一致） |
| `line100_yaw90` | 100cm 平移 + 同时把航向转到 90°（远距离航向弱增益场景） |
| `short5` | 短距 5cm（起步即在近目标区） |
| `diag100` | 斜向 100cm（70.71,70.71） |
| `rot90` / `rot180` | 原地旋转 90°/180°（纯航向环） |
| `pass_straight` | PASS 不停车通过点：40cm 处 4cm 触发 Nav_Retarget 切 100cm 目标 |
| `pass_turn` | PASS 后 90° 转弯 + Nav_SetYawFast（过弯场景） |
| `seq_3pt` | 连续 3 段导航（到点 COMPLETE 后 GO 下一段，含平移+原地旋转） |
| `line30_100` | 30cm 高速（100cm/s）短冲 |

指标（每行 CSV，一个 leg 从导航指令下发开始统计）：到位时间 `arrive_total_ms`、最终位置/航向误差
`final_pos_err_cm`/`final_yaw_err_deg`、到达瞬间真实剩余距离 `leg_true_dist_cm`（>阈值=提前判到/滑移）、
超调 `max_overshoot_cm`、航向超调 `max_yaw_overshoot_deg`、横向偏差 `lat_max_cm`/`lat_rms_cm`、
目标附近振荡 `near_osc_count`、指令翻转 `cmd_reversals`、STOPPING 尾延迟 `leg_settle_tail_ms`、
PASS 不停车最小速度 `pass_min_speed_cm_s`、控制周期 min/max/mean、以及 PASS/多段场景的逐段
`leg_arrive_ms`/`leg_pos_err_cm` 等（分号分隔）。完整字段见 CSV 表头。

## 5. 模型默认值与不确定性（重要）

| 参数 | 默认 | 依据/不确定度 |
|---|---|---|
| `cmd_delay_ms` | 0（套件扫 0/10/20/50/100） | 电机总线命令链端到端延迟；修复前可能 100ms 级，目标是 <15ms |
| 轮速一阶滞后 `wheel_tau_s` | 60ms | 任务给定范围 40~80ms；不确定度套件扫 40/60/80/120ms |
| 整数 RPM 量化 | 开 | 真实 MotionControl 速度模式就是整数 RPM 截断（低速度 0.38cm/s 一档） |
| 电机加速度斜坡 | 开 | nav 下发 car_acc 经 Emm 档位折算，模型取 acc/R（档位误差 ±2%） |
| STOP 制动 | 4000cm/s² + 20ms | **最大不确定项**：0xFE 立即停止后实际停车距离未知；扫描 800/4000/8000cm/s² × 20/60ms |
| 打滑 `slip` | 0 | 理想地面；扫描 0/5/10% |
| OPS 帧率/延迟 | 50Hz / 20ms | 扫描 20/50/100Hz、延迟 5/20/50ms |
| OPS 量化/噪声 | 0.01cm/0.01° + yaw 0.05°σ | OPS 标称 0.1mm；噪声固定种子可复现 |
| 控制周期抖动 | 关 | 模拟命令队列阻塞 Nav_Task：0~8ms 随机@35%，延迟 0/20/50ms 对比 |
| 坐标/旋转中心 | 取 `chassis_config.h`（当前 0/0） | 若实车标定了 OPS 安装偏移，仿真仍是同一套运动学 |

## 6. 参数扫描与推荐值的用法

- **运行时整定参数**（新版 nav，`Nav_SetTunable` 名称与 `nav par` 一致）：`run_sim.ps1` 的 `sw_*`
  CSV 就是逐个参数取值对比；报告第 3 节按"综合代价"排序敏感度并给推荐值。
  手工扫描：`navsim_current.exe --suite quick --tune yaw_kp=2.4 --tag sw_yaw_kp_2.4 ...`
- **编译期宏**（不能运行时改的，如 `NAV_ARRIVE_CHECK_COUNT`、`NAV_YAW_ZERO_CROSS_LOCK_MS`）：
  脚本生成强制包含头（先 `#include "nav_config.h"` 再 `#undef/#define`）重编译 `nav_core.c`，
  tag `mx_<宏>_<值>`；**不修改生产代码**。
- 报告"综合代价"= 到位时间(s) + 2×位置误差(cm) + 0.05×yaw误差(deg) + 0.3×超调(cm) + 0.1×振荡次数。
  该权重体现"又快又准"的偏好，可按需在 `report.py` 调整。

## 7. 已知限制

- 模型是单点近似：没有车体惯性/轮胎侧滑动力学、没有电机闭环细节、没有 OPS 丢帧（用帧龄模型代替）；
  STOP 制动、轮速滞后是最主要的两个不确定参数（见第 5 节）。
- 场景都是无现场障碍的直线/斜线/原地旋转；不包含 Planner 路径与任务层时序（mission 的 PASS 触发用本
  仿真的简化版：距点 4cm 触发 Nav_Retarget）。
- 仿真 CPU 上串行执行，`taskENTER_CRITICAL` 等为桩；不验证真实中断/并发。
- 只编译 nav/chassis 运动学相关真实源码；chassis 的电机命令队列行为被"纯延迟+周期抖动"模型替代
  （哪个模型更准取决于 motor-bus 修复后的实测时延）。
- 本目录不改生产代码、不碰串口/烧录；构建产物只写 `%TEMP%`。
