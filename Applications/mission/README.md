# Applications/mission · 总任务流程编排

> 对应实现：`mission.c/.h`、`tools/mission_tools.c`、
> `utils/{mission_arm,mission_code,mission_nav,mission_route,mission_timing,mission_ui}`。

## 1. 职责

总任务（比赛流程）的编排层：把「扫码 → 抓取 → 粗加工 → 暂存 → 回家」各阶段，
按当前启动区与轮次组织成任务码与路线，调用 nav / arm / RPI / SCANNER / LCD 完成整车动作。
本模块**只做编排**，不实现底层导航控制、机械臂位姿解算或显示驱动。

| 子文件 | 职责 |
| :- | :- |
| `mission.c` | 任务主线程与阶段状态机、对外运行控制 API（`mission_set_running` 等） |
| `utils/mission_code.c` | 任务码（`xxx+xxx+xxx+xxx`）校验、解析与缓存 |
| `utils/mission_route.c` | 固定路线表（数据层）：按启动区/轮次选路线，不含流程控制 |
| `utils/mission_nav.c` | 导航封装：启动→等待→重试→中止→到位精修 |
| `utils/mission_arm.c` | 机械臂动作编排：按任务码逐物料调用 `Arm_Action_*` |
| `utils/mission_ui.c` | 任务数据到 LCD 的显示映射 |
| `utils/mission_timing.h` | 分阶段耗时统计（现场调参用轻量仪表） |
| `tools/mission_tools.c` | shell 调试命令 `mission` 的注册与处理 |

## 2. 关键参数边界

所有标定值、任务点坐标、导航增益、机械臂位姿**集中在 `mission_config.h`（本轮禁改）**，
以及 `Applications/nav/nav_config.h`、`Applications/arm/arm_config.h`。
本模块代码中出现的常量分两类：

- **禁改（保持原位原值）**：任务点名（`MISSION_TARGET_*`）、路线点位与临时点、
  导航速度/对齐/通过点参数（`MISSION_*_SPEED_*` / `*_DIST_*` / `*_TIMEOUT_MS`）、
  二次标定位姿（`MISSION_CAL_ARM_*`）、延时定值（`MISSION_SETTLE_MS`）、超时与重试上限。
  这些值来自实车整定，改动即改变可观测行为。
- **可整理（值不变）**：缓冲区容量、纯结构性常量、局部命名、注释与 Doxygen。
  提取为具名宏时必须逐字符保留原数值与后缀（如 `200` 不得改为 `200U`）。

任务码约定：`4 段 × 3 位`，段间 `+` 分隔，每段取值 `0~999`；
抓取行为颜色码 `1~6`（对应 `RPI_Color_t`），规则详见 `mission_arm.c/.h` 头部注释。

## 3. 禁止改动项（红线）

- `mission_config.h`：**整体只读**，任何数值/注释均不改。
- `Applications/nav/nav_map_data.c`：地图点位整体只读。
- 任务码帧格式、shell 命令字与输出文本（对外契约）：不改。
- `mission_route.c` 中的路线数组、点位与 `MISSION_NAV_MODE_*` 取值：不改。
- shell 命令字 `mission run/stop/time/code` 及其参数语义：不改。
- 导航/机械臂的时序、优先级、超时判定：不改。

## 4. 调试入口（shell 命令）

串口 shell 输入 `mission`：

| 命令 | 作用 |
| :- | :- |
| `mission run` | 启动总任务（计时起点） |
| `mission stop` | 停止总任务并置中止标志 |
| `mission time` | 查询本次/最近一次任务运行时长（ms） |
| `mission code` | 查看当前任务码 |
| `mission code <s1> <s2> <s3> <s4>` | 一次设置 4 段任务码（每段 `0~999`） |
| `mission code <row> <value>` | 只修改第 `row` 段（`1~4`） |

分阶段耗时统计由 `mission_timing.h` 提供，开关与汇总宏见 `mission_config.h`
（`MISSION_TIMING_ENABLE` / `MISSION_TIMING_SUMMARY_ENABLE`）；日志仅在段结束时输出，
不在控制链路稳态打印。

## 5. 验证方式

本模块无独立主机 harness。行为保持以「静态审阅 + `git diff` 逐行核对 +
`tools/check_refactor.py --check all` 五类门禁」为准；关键数值以数字 token 多重集与
HEAD 基线比对，确保标定值与任务点坐标零变化。
