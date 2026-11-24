# DIP_SW 启动区选择驱动

## 1. 模块职责

`Drivers/User/DIP_SW` 负责读取拨码开关状态、消抖并锁存启动区，供地图初始化与 Planner 初始化使用。

主要接口：

- `DIP_Switch_Init()`：在 RTOS 调度器启动后调用，内部完成采样与消抖，锁存启动区。
- `DIP_Switch_GetStartZone()`：返回锁存后的启动区；运行中拨动开关不会改变结果。
- `DIP_Switch_ZoneName()`：启动区枚举到可读名称的转换。

启动区枚举 `Start_Zone_t`：`START_ZONE_UNKNOWN`、`START_ZONE_1`、`START_ZONE_2`。

## 2. 关键参数边界

参数在 `DIP_Switch.h`：

- `DIP_SWITCH_DEBUG_MODE`：当前保持原值，允许通过 shell 命令 `dip` 强制指定启动区。量产改零属于 P2-08，**本轮不实施**。
- `DIP_SWITCH_DEBOUNCE_MS`：稳定判定所需电平保持时间。
- `DIP_SWITCH_RETRY_MAX`：稳定采样失败时的最大重试次数。

初始化阻塞时间约等于消抖时间乘重试次数；调试模式下若已由 shell 强制指定启动区，则跳过采样直接采用该值。

## 3. 禁止改动项

- `DIP_SWITCH_DEBUG_MODE` 默认值、`DIP_SWITCH_DEBOUNCE_MS`、`DIP_SWITCH_RETRY_MAX`。
- `s_debug_zone` 的类型与 shell 强制指定语义。
- `START_ZONE_x` 与 `MAP_ZONE_x` 的既有取值关系。B05 已将 `Planner_Init()` 入参从 `START_ZONE_x` 改为数值相等的 `MAP_ZONE_x`，且该分支在 `PLANNER_ENABLED=0` 下不生效；其余使用点保持原样。
- 消抖状态机与锁存时序；启动区在运行期不得随拨码变化。

## 4. 调试入口

Shell 命令 `dip [get|1|2|auto]`：

- `dip get`：查询当前启动区及是否处于强制模式。
- `dip 1` / `dip 2`：强制使用启动区 1 / 2。
- `dip auto`：取消强制，恢复按拨码电平判定。

启动时若调试模式生效，会打印一次强制区号；抖动超限时打印告警，不重复刷屏。

## 5. 验证方式

- 静态验证：`git diff` 确认仅注释/命名整理；`DIP_Switch.h` 数值与 `Planner_Init()` 调用语义不变；数字多重集门禁通过。
- 现场验证：分别拨到两区并复位，用 `dip get` 与地图加载结果确认区号一致；用 `dip 1/2/auto` 验证强制与恢复。
- 当前环境无 ARM 整机，拨码电气特性与消抖边界需后续实车确认。
