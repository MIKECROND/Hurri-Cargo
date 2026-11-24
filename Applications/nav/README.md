# Applications/nav 模块说明

## 1. 模块职责

`Applications/nav` 负责车体定位融合、地图点管理、目标导航控制循环以及现场调试入口。控制层以非阻塞方式运行，由上层任务按固定控制周期调用；主机端 harness 使用同一份真实控制代码做行为回归。

模块边界：

- `localization/nav_local.c/.h`：读取带时间戳的位姿，供导航控制与调试工具使用。
- `utils/nav_math.c/.h`：角度归一化、位姿参考点偏移换算以及基础限幅。
- `utils/nav_map.c/.h`：目标点表初始化、增删查改、按启动区选择预定义点位。
- `utils/nav_core.c/.h`：导航状态机、速度规划、航向环、停稳判定与调试快照。
- `tools/nav_tools.c`：Letter-Shell 调试命令组 `nav` 与 `map`。
- `nav_config.h`：编译期默认参数，只读；运行时整定值由控制层结构体承载。
- `tests/harness/`：主机端仿真与报告工具，用于验证真实控制代码的行为一致性。

## 2. 对外 API

核心控制接口见 `utils/nav_core.h`：

- `Nav_GoTo()`：按目标点 ID 启动导航。
- `Nav_GoToDirect()`：按目标位姿启动导航。
- `Nav_GoTo_fromName()`：按点位名称启动导航。
- `Nav_Retarget()`：运行中替换目标点，保持速度与 PID 状态连续。
- `Nav_Stop()`：请求停车并进入停稳流程。
- `Nav_GetState()` / `Nav_GetDebug()` / `Nav_GetTunables()` / `Nav_SetTunable()`：状态查询与运行时整定。
- `Nav_SetYawFast()`：设置过弯场景的航向快速跟随标志。
- `Nav_Update()`：执行一个导航控制周期，不调用阻塞延时。

地图接口见 `utils/nav_map.h`，包括 `Map_Init()`、`Map_AddPoint()`、`Map_RemovePoint()`、`Map_UpdatePoint()`、`Map_GetPoint()`、`Map_GetPointByName()` 与预定义点位访问接口。定位接口见 `localization/nav_local.h`。

## 3. 参数边界

所有默认值集中在 `nav_config.h`，按用途分为：

- 控制节拍与定位新鲜度：`NAV_UPDATE_TIME`、`NAV_OPS_STALE_TIMEOUT_MS`。
- 位置环：`NAV_STARTUP_SPEED`、`NAV_ALIGN_XY_KP`、`NAV_NEAR_TARGET_FACTOR`、`NAV_NEAR_TARGET_GAIN`、`NAV_TRAJ_SPEED_CAP`、`NAV_POS_DEADBAND_CM`、`NAV_VEL_LAG_S`。
- 航向环：`NAV_YAW_PID_KP/KI/KD`、`NAV_YAW_PID_D_FILTER_TAU_S`、`NAV_ALIGN_YAW_MAX/MIN`、`NAV_YAW_ACCEL_LIMIT`、`NAV_YAW_GAIN_FAR/FAST`、`NAV_YAW_FEEDFORWARD_WEIGHT`、`NAV_YAW_FEEDBACK_WEIGHT`。
- 到达与停稳：`NAV_ARRIVE_CHECK_COUNT`、`NAV_STOP_SETTLE_MS`、`NAV_STOP_WINDOW_MS`、`NAV_STOP_FRESH_OPS_MS`、`NAV_STOP_MAX_RESUME`、`NAV_STOP_EXTRA_WAIT_MS`。
- 实测速度估计：`NAV_MEAS_VEL_TAU_S`、`NAV_MEAS_VEL_STALE_MS`、`NAV_USE_MEAS_SPEED_DECEL`。
- 地图工具：`NAV_MAP_TOOL`。

运行时可通过 `nav par <name> <value>` 修改 `NavTunables_t` 中的字段；控制层对增益、权重、速度和等待时间做安全范围检查，非法输入会被拒绝。运行时修改仅在本次上电有效，复位后恢复 `nav_config.h` 默认值。

## 4. 调试入口

`tools/nav_tools.c` 导出两个 Shell 命令组：

- `nav pose`：实时显示当前位姿，按 Ctrl+C 退出。
- `nav go <id> <view>`：按目标点 ID 导航，`view` 为是否同时打开位姿显示。
- `nav to <x> <y> <yaw>`：按指定坐标与航向导航。
- `nav stop` / `nav state`：停止导航 / 查询状态。
- `nav par`：打印运行时整定参数；`nav par <name> <value>` 在线修改一个参数。
- `nav trace [period_ms]`：输出 CSV 格式的导航内部状态，按 Ctrl+C 退出。
- `map list|get|set|rm|add|init`：地图点位管理工具，受 `NAV_MAP_TOOL` 控制。

## 5. 验证方式

本机完整 harness 命令：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File Applications\nav\tests\harness\run_sim.ps1 -Python python
```

必须用 `-Python python`；本机 `python3` 会命中 WindowsApps 存根。harness 会从冻结基线 commit 提取原始控制代码，与工作区当前控制代码分别编译，运行同一组场景与参数扫描，并生成 CSV 与 `nav_sim_report.md`。行为保持验收要求关键指标逐行一致。

## 6. 行为保持边界

本轮重构只允许注释、命名、类型可读性整理和文档补充。以下内容禁止改动：

- `nav_config.h` 中全部增益、阈值、时间常数与开关值。
- `nav_map_data.c` 中所有点位、运动参数与到达参数。
- OPS 位姿新鲜度与停稳判定时序，包括 `OPS_GetFrameAgeMs` 的既有语义。
- PID、梯形速度规划、航向前馈/反馈融合、停稳复核与恢复补位逻辑。
- harness 基线 CSV、MANIFEST 与冻结 baseline commit。

P1-01 限幅复用未在本批实施：`nav_math.h` 已有 `static inline clamp()`，但抽取现有分散限幅会引入逻辑等价判断风险；本批无整机编译环境，harness 是唯一行为证据，因此按“存疑不做”的授权保持原实现。
