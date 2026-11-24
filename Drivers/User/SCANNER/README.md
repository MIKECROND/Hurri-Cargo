# Drivers/User/SCANNER · 条码扫描器驱动

> 对应实现：`scan_driver.c/.h`、`tools/scan_tools.c`。

## 1. 职责

通过 UART5 接收条码扫描枪输出，按字节间超时组帧，经 StreamBuffer 送给消费者，
对外提供"取最新条码 / 取新条码（单次置位）"接口及扫码输出使能控制。
本模块只做"收帧 + 组帧 + 缓存"，不解析条码业务含义。

| 文件 | 职责 |
| :- | :- |
| `scan_driver.h` | 对外接口与缓冲配置（帧长、超时、StreamBuffer 容量） |
| `scan_driver.c` | UART5 空闲中断 DMA 收帧、错误回调与 rearm、组帧、StreamBuffer 写入 |
| `tools/scan_tools.c` | shell 调试命令 `scan`（`on`/`off`/`get`） |

## 2. 关键参数边界

- UART5 DMA 模式为 **NORMAL**（非 CIRCULAR），由 `Scan_UART_ErrorCallback` 出错后重新装填；
  模式、回调注册与 rearm 逻辑属审计已修项，**不改/勿回退**。
- 组帧参数：`FRAME_TIMEOUT_MS`（字节间超时）、`SCAN_GROUP_WINDOW_MS`（分组窗口）、
  `SCAN_STREAM_BUF_SIZE` / `SCAN_STREAM_TRIGGER_LVL`（StreamBuffer）——保持原值。
- 条码缓冲区 `SCAN_BARCODE_MAX_LEN` / `SCAN_FRAME_MAX_SIZE` 为结构性容量，保持原值。

## 3. 禁止改动项（红线）

- UART5 DMA NORMAL 模式、StreamBuffer 读写、`Scan_UART_ErrorCallback` 注册与出错 rearm：不改。
- `SCAN_EVT_BARCODE` 的**单次置位语义**（事件位被取走后清零，避免重复上报）：不改。
- 条码解析格式、事件/回调时序：不改。

## 4. 调试入口

串口 shell：

| 命令 | 作用 |
| :- | :- |
| `scan on` | 打开扫码输出 |
| `scan off` | 关闭扫码输出 |
| `scan get` | 读取当前条码 |

## 5. 验证方式

无独立主机 harness。行为保持以静态审阅 + `git diff` 逐行核对 + `tools/check_refactor.py --check all` 门禁为准；
DMA 模式/事件语义/缓冲数值以 grep 与数字 token 多重集前后比对，确保零变化。
