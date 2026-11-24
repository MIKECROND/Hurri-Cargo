# MISC 扩展消息显示模块

## 1. 模块职责

`Drivers/User/MISC` 提供启动阶段的分步状态消息显示，用固定格式提示“等待/成功/失败”，便于确认初始化流程停在哪一步。

主要接口：

- `Show_dmesg(status, string)`：输出一条带状态前缀的消息。
- `SHOW_DMESG(status, string)`：宏封装；`MESSAGE_ENABLE=0` 时展开为空。

消息状态 `dmesg_status`：

- `dmesg_wait`：输出 `[wait] <string>`。
- `dmesg_ok`：输出 `[ ok ]` 并回车换行。
- `dmesg_fail`：输出 `[fail]` 并回车换行。

## 2. 关键参数边界

- `MESSAGE_ENABLE`：编译期总开关，当前保持原值。关闭时所有 `SHOW_DMESG()` 调用不产生输出，也不应留下未使用变量告警。
- 输出格式字符串 `[wait]` / `[ ok ]` / `[fail]` 属于既有调试输出契约，不属于可重命名范围。
- 底层 `my_printf` 使用静态缓冲与互斥保护，其 `static` 属性和互斥逻辑属于审计 C7 已修项，严禁回退或“重新实现”。

## 3. 禁止改动项

- `MESSAGE_ENABLE` 默认值与条件编译结构。
- 三种消息状态前缀、回车换行位置与 `my_printf` 调用时序。
- `log` 互斥与 UART TX 互斥逻辑；本模块只使用既有输出通道，不新增锁或日志调用。
- 不得把启动消息改造成高频稳态打印（见 `docs/log-usage-policy.md`）。

## 4. 调试入口

在启动流程中按步骤调用：

```c
SHOW_DMESG(dmesg_wait, "Init nav");
/* ... */
SHOW_DMESG(dmesg_ok, "");
```

失败分支使用 `SHOW_DMESG(dmesg_fail, "")`。观察串口启动日志即可定位初始化阶段，无需新增 shell 命令。

## 5. 验证方式

- 静态验证：`git diff` 确认仅注释新增；`MESSAGE_ENABLE`、状态枚举与格式字符串不变；`my_printf` 的 static/互斥属性未回退。
- 现场验证：正常上电观察 `[wait] → [ ok ]` 顺序；构造失败分支观察 `[fail]`；确认无重复刷屏。
- 当前环境无 ARM 整机，串口实际输出需后续烧录确认。
