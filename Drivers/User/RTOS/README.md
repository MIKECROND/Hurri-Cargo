# RTOS 实用工具模块

## 1. 模块职责

`Drivers/User/RTOS` 提供系统信息查询、任务状态查看、系统时间查看、手动错误触发与事件标志位定义。

主要文件：

- `rtos_utils.c/.h`：shell 命令实现与帮助文本。
- `Events.h`：全局事件标志组句柄与事件位定义。

对外行为：

- `os mem`：显示堆/内存状态。
- `os task`：显示任务名、状态、优先级、栈余量与 CPU 占用。
- `os time`：显示 tick 频率、运行时间与系统时钟树频率。
- `os error`：手动触发 `Error_Handler()`，仅用于调试。
- `top`：持续刷新任务状态，按 Ctrl+C 退出。
- `Task_Info()`：任务信息快照核心实现。

## 2. 关键参数边界

- `task_ids[32]`：任务枚举容量；`os task` 在任务数大于容量时只展示可枚举部分。
- `OS_Top()` 刷新周期与 `osDelay(100)`：属于交互式显示节奏，不作为控制时序。
- `OS_HELP`：shell 帮助输出格式，属于既有命令契约。
- `Events.h` 中 `System_StatusHandle`、`Planner_EventHandle` 及各事件位（`SYS_INIT_COMPLETE`、`APP_NEED_USART`、`UART*_TX_IDLE`、`NAV_EVT_*`、`SCAN_EVT_BARCODE` 等）是模块间握手接口，位值不可改。

B05 已将 `Task_Info()` 循环变量改为 `uint32_t` 并加入零任务提前返回，避免 `uint8_t` 下溢；正常路径（任务数至少两个）的排序、输出顺序与格式保持不变。

## 3. 禁止改动项

- `Events.h` 任何事件位、句柄名与位含义。
- `os task`/`top` 的输出字段顺序、格式宽度与刷新周期。
- 审计 C7 已修项：日志互斥 `s_log_mutex`、UART TX 互斥、`my_printf` 静态缓冲、Shell `osDelay(1)`、SCANNER `ErrorCallback`、OPS 150ms 帧龄语义。相关文件只允许注释/命名/`static` 正确性核对，严禁“重新实现”。
- P1-02（全局 `g_` 收敛为 `static` + 访问函数）属跨文件结构改动，本轮降级为建议，不实施（见 `docs/refactor/VERIFICATION.md`）。

## 4. 调试入口

```text
os mem
os task
os time
top
os error
```

`os task` 与 `top` 会短暂置位 `APP_NEED_USART`，退出时清除，避免与导航/调试输出争用串口。`os error` 会触发错误分支，仅用于故障演练。

## 5. 验证方式

- 静态验证：`git diff` 确认仅注释/类型整理；`Events.h` 位定义与 `OS_HELP` 文本不变；数字多重集门禁通过。
- 现场验证：复位后运行 `os task`，确认任务列表与 B05 前逐行一致；任务数为零时不出现 `255` 下溢输出；`top` 可用 Ctrl+C 正常退出。
- 当前环境无 ARM 整机，栈余量/CPU 占用与 `--callgraph` 复核属后续 P2（P2-04）。
