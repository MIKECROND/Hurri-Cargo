# Drivers/User/LCD · ILI9488 屏显模块

> 对应实现：`lcd_port.c/.h`、`lcd/ili9488.*`、`utils/**`、`tools/lcd_tools.c`。

## 1. 职责

驱动 ILI9488 TFT 屏，为业务层提供"任务完成 / 扫码 / 状态"等文本与图形显示。
分层：IO 接口层（`utils/io_spi/lcd_io_spi_hal.c`）→ 屏幕驱动与图形库（`utils/stm32_adafruit_lcd.c`、`lcd/ili9488.c`）→ 端口封装（`lcd_port.c`）。

| 文件 | 职责 |
| :- | :- |
| `lcd_port.h/.c` | 对外统一初始化与控制入口（IO → 驱动 → 背光） |
| `lcd/ili9488.*` | ILI9488 控制器底层驱动（配置宏 + 绘图接口） |
| `utils/stm32_adafruit_lcd.*` | 移植的 BSP 图形库（文本/形状/位图/读屏） |
| `utils/io_spi/**` | SPI HAL LCD IO 实现 |
| `utils/Fonts/**` | 字库数据（**禁改**） |
| `tools/lcd_tools.c` | LCD 调试 shell 命令 |

## 2. 关键参数边界

- 初始化顺序：IO 接口 → 屏幕驱动/图形库 → 背光；须在 `MX_GPIO_Init()` / `MX_SPI1_Init()` 之后调用。
- SPI1 读写速率切换、数据方向与传输控制（`lcd_io_spi_hal.c`）：属实车时序，**不改**。
- `ILI9488_*` 配置宏（接口/方向/色序/位深/分辨率）：保持原值（见 `lcd/ili9488.h`）。
- 字库数据 `utils/Fonts/**`：整体只读，不改。

全屏阻塞刷新约 59ms 属已知行为（审计 #16），列为 P2 需实车评估，**本轮不纳入**。

## 3. 禁止改动项（红线）

- SPI1 时序、刷新实现、背光控制时序：不改。
- `utils/Fonts/**` 字库数据、BMP 位图结构：不改。
- 移植来源文件（`ili9488.*`、`stm32_adafruit_lcd.*`、`bmp.h`、`lcd_io.h`、`lcd_io_spi_hal.*`）：禁止覆盖上游，仅允许补注释。
- ILI9488 配置宏数值：不改。

## 4. 调试入口

LCD 调试 shell 命令见 `tools/lcd_tools.c`；显示内容由任务层（`mission_ui.c`）经 `lcd_port.h` 接口驱动。

## 5. 验证方式

无独立主机 harness。行为保持以静态审阅 + `git diff` 逐行核对 + `tools/check_refactor.py --check all` 门禁为准；
SPI 时序grep、字库/配置数值以数字 token 多重集前后比对，确保零变化。
