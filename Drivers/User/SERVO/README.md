# Drivers/User/SERVO · 舵机模块

> 对应实现：`servo_port.c/.h`、`BUS_SERVO/**`、`PWM_SERVO/**`。

## 1. 职责

对上提供统一的舵机角度设置接口（按逻辑 ID），对下屏蔽总线舵机与 PWM 舵机差异：
ID 1/2 为总线舵机（走 USART3 总线协议），ID 3/4 为 PWM 舵机（走定时器 PWM）。

| 文件 | 职责 |
| :- | :- |
| `servo_port.h/.c` | 统一端口层：逻辑 ID → 总线/PWM 分派，角度设置 |
| `BUS_SERVO/utils/**` | 总线舵机驱动与协议帧封装（USART3） |
| `PWM_SERVO/utils/**` | PWM 舵机驱动（PWM_SERVO_ENABLED 开关） |

## 2. 关键参数边界

- 统一 ID 分配：`SERVO_PORT_BUS_ID_MIN/MAX`（1/2）、`SERVO_PORT_PWM_ID_MIN/MAX`（3/4）——保持原值。
- 总线舵机协议帧、USART3 波特率、电压回传语义：均为实车契约，**不改**。
- PWM 舵机首次调用自动初始化 PWM 输出；`PWM_SERVO_ENABLED=0` 时 3/4 号调用被空实现忽略——保持现状。

## 3. 禁止改动项（红线）

- 总线舵机协议帧格式、命令字、USART3 波特率、电压回传语义：不改。
- PWM 通道映射与占空比换算：不改。
- 对外 ID 分配宏（`SERVO_PORT_*_ID_*`）：不改。

## 4. 调试入口

机械臂 shell 命令 `arm flip [deg|cal|restore]` 控制相机翻转舵机（与标定链路同一接口）；
总线层调试见 `BUS_SERVO/tools/servo_tools.c`、`PWM_SERVO/tools/pwm_servo_tools.c`。

## 5. 验证方式

无独立主机 harness。行为保持以静态审阅 + `git diff` 逐行核对 + `tools/check_refactor.py --check all` 门禁为准；
协议字节/波特率/ID 数值以 grep 与数字 token 多重集前后比对，确保零变化。
