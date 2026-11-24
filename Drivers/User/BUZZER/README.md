# BUZZER 蜂鸣器驱动

## 1. 模块职责

`Drivers/User/BUZZER` 负责无源/有源蜂鸣器的 PWM 发声、开机与启动区提示音、码盘提示音异步播放，以及钢琴调试模式。

主要文件：

- `buzzer.c/.h`：驱动实现与对外接口。
- `buzzer_sheet.c`：开机曲、区号曲与按键音符数据，视为**标定/数据文件**。

对外接口见 `buzzer.h`：

- `Buzzer_Init()`：初始化并播放开机提示音与启动区提示音。
- `Buzzer_On()` / `Buzzer_Off()`：持续发声与停声。
- `Buzzer_Beep()`：阻塞式鸣叫指定时长。
- `Buzzer_PlayOpsTuneAsync()`：非阻塞播放码盘提示音，内部临时创建任务。
- `Buzzer_EnsureInit()`：确保已初始化，避免重复播放开机曲。
- `Buzzer_KeyToFreq()`：按键字符到音符频率的查询，供钢琴模式使用。

## 2. 关键参数边界

参数集中在 `buzzer.h`：

- `BUZZER_ACTIVE_MODE`：蜂鸣器类型；无源模式可通过 PWM 调频，有源模式仅支持开关。
- `BUZZER_TIM_CLK_HZ` / `BUZZER_TIM_PRESCALER`：TIM4 时钟与预分频，决定频率分辨率。
- `BUZZER_MIN_FREQ_HZ` / `BUZZER_MAX_FREQ_HZ`：频率截断范围，超出范围按边界处理。
- `BUZZER_DEFAULT_FREQ_HZ`：默认发声频率。

`buzzer.c` 中开机提示音与启动区提示音的时长/间隔属于现场时序：

- 无源模式下 `Buzzer_Init()` 总阻塞约 3~3.5s（审计 #9，P2-02）。
- 有源模式仅短鸣一声，频率参数无效。
- 码盘提示音通过独立异步任务播放，不阻塞调用者。

## 3. 禁止改动项

- `buzzer_sheet.c` 中乐谱数组、音符表与开机曲/区号曲数据。
- `BUZZER_ACTIVE_MODE`、定时器时钟/预分频、频率上下限与默认频率。
- 开机曲与启动区提示音的阻塞时长、播放间隔与调用时序。
- `Buzzer_PlayOpsTuneAsync()` 的任务创建/自删除语义，以及 `Buzzer_EnsureInit()` 的幂等语义。

本轮仅为注释/Doxygen 整理。审计 #9 的“上电蜂鸣器阻塞改异步”属于 P2-02，需实车验证上电时序，**本轮不实施**。

## 4. 调试入口

Shell 命令 `buzzer`：

- `buzzer status`：查看当前发声状态与频率。
- `buzzer on [freq_hz]` / `buzzer off`：持续发声与停声。
- `buzzer beep [ms]` 或 `buzzer beep [freq] [ms]`：单次鸣叫。
- `buzzer sweep`：无源模式下扫频查找谐振点；有源模式提示不适用。
- `buzzer piano`：键盘钢琴模式，按 ESC 退出；有源模式提示不适用。

## 5. 验证方式

- 静态验证：`git diff` 确认仅注释新增，乐谱数据与时序常量无变化；`check_refactor.py --check all` 全绿。
- 现场验证：用 `buzzer status/on/off/beep/sweep/piano` 逐项确认发声、停声与音调；确认开机曲与启动区提示音顺序不变。
- 当前环境无 armcc，不能做 ARM 整机编译/烧录；实车声学与上电时序验证属后续 P2。
