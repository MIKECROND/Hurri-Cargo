# BATTERY 电池电压监测模块

## 1. 模块职责

`Drivers/User/BATTERY` 通过总线舵机回传值周期性估计电池电压，提供低压/读取失败告警与实时查看入口。本模块不直接访问 ADC；审计 #14 的 ADC 死代码清理属 P2-07，涉 CubeMX 生成文件，**本轮不实施**。

主要接口：

- `Battery_Init()`：创建后台采样任务；禁用时为空实现，不占用任务栈。
- `Battery_GetVoltage(uint16_t *voltage)`：读取最近一次有效电压（mV）。

## 2. 关键参数边界

参数在 `Battery.h`：

- `BATTERY_ENABLED`：模块总开关；关闭时任务与采样均不创建。
- `BATTERY_SERVO_ID`：用于读取电压的总线舵机 ID，当前为物料盘舵机。
- `BATTERY_SERVO_SCALE` / `BATTERY_SERVO_OFFSET_MV`：电压换算比例与偏置，按实测校准。
- `BATTERY_SAMPLE_AVG` / `BATTERY_SAMPLE_INTERVAL_MS`：单次采样平均次数与采样周期。
- `BATTERY_MIN_VALID_MV`：有效电压下限，低于该值视为读取失败或未接电池。
- `BATTERY_THRESHOLD`：低电压告警阈值。
- `BATTERY_WARN_INTERVAL_MS`：告警最小间隔，避免刷屏。

后台任务等待 `SYS_INIT_COMPLETE` 后再采样；无效电压不更新显示值，低压与读取失败共用节流窗口。

## 3. 禁止改动项

- 上述所有校准与告警宏值，尤其 `BATTERY_SERVO_SCALE`、`BATTERY_SERVO_OFFSET_MV`、`BATTERY_THRESHOLD`。
- 舵机 ID 选择、平均次数、采样周期与告警节流语义。
- 等待 `SYS_INIT_COMPLETE`、`APP_NEED_USART` 置位/清位的串口占用时序。
- `Core/**`、`Drivers/User/ADC/**` 与 CubeMX 生成内容；P2-07 仅记录建议。

## 4. 调试入口

Shell 命令 `battery`：

- `battery time <ms>`：修改当前采样周期，仅影响运行中实例，复位后恢复宏默认值。
- `battery time`：查看当前采样周期。
- `battery view`：持续显示电压，按 Ctrl+C 退出；显示期间会置位 `APP_NEED_USART` 并在退出时清除。
- 无参数调用打印用法；未初始化时提示模块未初始化。

## 5. 验证方式

- 静态验证：`git diff` 确认仅注释新增；电压标定/告警宏值不变；数字多重集门禁通过。
- 现场验证：用 `battery view` 对比万用表读数；短时拔掉/断开舵机确认“unavailable”提示；低压告警只按节流间隔出现。
- 当前环境无 ARM 整机，电压标定与告警门限需后续实车确认。
