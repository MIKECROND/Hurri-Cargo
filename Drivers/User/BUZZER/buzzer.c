/**
 * @file buzzer.c
 * @author T-Car Team
 * @brief 蜂鸣器驱动实现
 *
 * 使用 TIM4_CH2(PB7) 驱动蜂鸣器，由 BUZZER_ACTIVE_MODE 选择两种工作方式：
 *   - 无源(BUZZER_ACTIVE_MODE=0)：输出 PWM 方波，发声频率 = PWM 频率，占空比 50%
 *   - 有源(BUZZER_ACTIVE_MODE=1)：输出恒定高电平开关，频率参数无效
 *
 * 空闲电平: PWM 通道配置为 PWM1 + 极性高(TIM4 OCPolarity=HIGH)，停止发声时比较值清 0，
 * 管脚保持低电平；即"蜂鸣器未使用时对应管脚为低电平"，不会给蜂鸣器/驱动三极管持续通电。
 * 可调参数见 buzzer.h。
 */

#include "buzzer.h"
#include "tim.h"
#include "DIP_Switch.h"
#include "cmsis_os2.h"
#include "shell.h"
#include "log.h"
#include <stdlib.h>
#include <string.h>

#if !BUZZER_ACTIVE_MODE
/** 分频后的计数频率(Hz) */
#define BUZZER_COUNTER_CLK_HZ (BUZZER_TIM_CLK_HZ / (BUZZER_TIM_PRESCALER + 1U))
#endif

static bool buzzer_inited = false;
static bool buzzer_on = false; /* 当前是否正在发声 */
#if !BUZZER_ACTIVE_MODE
static uint32_t buzzer_freq_hz = 0; /* 当前发声频率(Hz)，截断后的实际值 */
#endif

#if !BUZZER_ACTIVE_MODE
/* ------------------------- 音符与开机提示音(无源) ------------------------- */

/**
 * 音符频率表(Hz)，十二平均律 A4=440Hz，覆盖 C2 ~ C7
 * 命名规则：NOTE_<音名><八度>，升号用 S 后缀，如 NOTE_CS4 表示 C#4
 */
#define NOTE_REST 0U
#define NOTE_C2 65U
#define NOTE_CS2 69U
#define NOTE_D2 73U
#define NOTE_DS2 78U
#define NOTE_E2 82U
#define NOTE_F2 87U
#define NOTE_FS2 92U
#define NOTE_G2 98U
#define NOTE_GS2 104U
#define NOTE_A2 110U
#define NOTE_AS2 117U
#define NOTE_B2 123U

#define NOTE_C3 131U
#define NOTE_CS3 139U
#define NOTE_D3 147U
#define NOTE_DS3 156U
#define NOTE_E3 165U
#define NOTE_F3 175U
#define NOTE_FS3 185U
#define NOTE_G3 196U
#define NOTE_GS3 208U
#define NOTE_A3 220U
#define NOTE_AS3 233U
#define NOTE_B3 247U

#define NOTE_C4 262U
#define NOTE_CS4 277U
#define NOTE_D4 294U
#define NOTE_DS4 311U
#define NOTE_E4 330U
#define NOTE_F4 349U
#define NOTE_FS4 370U
#define NOTE_G4 392U
#define NOTE_GS4 415U
#define NOTE_A4 440U
#define NOTE_AS4 466U
#define NOTE_B4 494U

#define NOTE_C5 523U
#define NOTE_CS5 554U
#define NOTE_D5 587U
#define NOTE_DS5 622U
#define NOTE_E5 659U
#define NOTE_F5 698U
#define NOTE_FS5 740U
#define NOTE_G5 784U
#define NOTE_GS5 831U
#define NOTE_A5 880U
#define NOTE_AS5 932U
#define NOTE_B5 988U

/** 高音 C6 ~ C7 */
#define NOTE_C6 1047U
#define NOTE_CS6 1109U
#define NOTE_D6 1175U
#define NOTE_DS6 1245U
#define NOTE_E6 1319U
#define NOTE_F6 1397U
#define NOTE_FS6 1480U
#define NOTE_G6 1568U
#define NOTE_GS6 1661U
#define NOTE_A6 1760U
#define NOTE_AS6 1865U
#define NOTE_B6 1976U
#define NOTE_C7 2093U

/** 单个音符：freq_hz 为 0 表示休止符 */
typedef struct {
    uint16_t freq_hz;
    uint16_t duration_ms;
} Buzzer_Note_t;

/** 开机提示音 **/
static const Buzzer_Note_t sys_startup_tune[] = {
  {NOTE_C5, 120}, {NOTE_E5, 120}, {NOTE_G5, 120}, {NOTE_C5, 120},
  {NOTE_E5, 120}, {NOTE_G5, 120}, {NOTE_C5, 120}, {NOTE_E5, 120},
  {NOTE_G5, 120}, 
};

/** 码盘提示音 **/
static const Buzzer_Note_t ops_start_tune[] = {
    {NOTE_E5, 120}, {NOTE_E5, 120}, {NOTE_REST, 120}, {NOTE_E5, 120},
    {NOTE_REST, 120}, {NOTE_C5, 120}, {NOTE_E5, 120}, {NOTE_REST, 120},
    {NOTE_G5, 120}, {NOTE_REST, 120}, {NOTE_REST, 120}, {NOTE_REST, 120},
    {NOTE_G4, 120}, {NOTE_REST, 120}, {NOTE_REST, 120}, {NOTE_REST, 120},
};

/** 设置为启动区一提示音 **/
static const Buzzer_Note_t Zone_1_startup_tune[] = {
  {NOTE_E5, 90},  {0, 50},
  {NOTE_E5, 90},  {0, 500}
};

/** 设置为启动区二提示音 **/
static const Buzzer_Note_t Zone_2_startup_tune[] = {
  {NOTE_G5, 150}, {0, 40},
  {NOTE_E5, 150}, {0, 40},
  {NOTE_C5, 300}, {0, 500}
};

/**
 * @brief 播放单个音符
 * @param freq_hz 音符频率(Hz)，0 表示休止
 * @param duration_ms 持续时长(ms)
 */
static void Buzzer_PlayNote(uint32_t freq_hz, uint32_t duration_ms) {
    if (freq_hz == 0) {
        Buzzer_Off();
    } else {
        Buzzer_On(freq_hz);
    }
    osDelay(duration_ms);
}

/**
 * @brief 播放一段乐谱
 * @note  每个音符后插入 20ms 休止做分音，避免连成一片
 * @param tune 乐谱数组
 * @param count 音符个数
 */
static void Buzzer_PlayTune(const Buzzer_Note_t *tune, uint32_t count) {
    for (uint32_t i = 0; i < count; i++) {
        Buzzer_PlayNote(tune[i].freq_hz, tune[i].duration_ms);
        Buzzer_Off();
        osDelay(20);
    }
}

/**
 * @brief 播放开机提示音
 */
static void Buzzer_PlayStartupTune(void) {
    Buzzer_PlayTune(sys_startup_tune, sizeof(sys_startup_tune) / sizeof(sys_startup_tune[0]));
}

/** 开机提示音结束后到启动区提示音之间的间隔(ms) */
#define BUZZER_ZONE_TUNE_DELAY_MS 1000U

/**
 * @brief 播放 DIP 开关所选启动区的提示音
 * @note  启动区未知时静默
 */
static void Buzzer_PlayZoneTune(void) {
    switch (DIP_Switch_GetStartZone()) {
    case START_ZONE_1:
        Buzzer_PlayTune(Zone_1_startup_tune, sizeof(Zone_1_startup_tune) / sizeof(Zone_1_startup_tune[0]));
        break;
    case START_ZONE_2:
        Buzzer_PlayTune(Zone_2_startup_tune, sizeof(Zone_2_startup_tune) / sizeof(Zone_2_startup_tune[0]));
        break;
    default:
        break;
    }
}

/* ------------------------- 键盘弹奏(无源) ------------------------- */

/** 每个按键的音符持续时长(ms) */
#define BUZZER_PIANO_NOTE_MS 200U

/** 退出弹奏模式的按键(默认 ESC) */
#define BUZZER_PIANO_EXIT_CHAR '\x1b'

/**
 * 键盘键位绑定表(用户自行修改)
 *
 * 每项为 { 按键字符, 音符频率 }，进入 buzzer piano 后按下对应字符即发出该音符；
 * 乐谱(sheet)解析同样复用此表，可自由增删或重新绑定到任意 NOTE_xxx。
 *
 * 默认键位(音高由低到高)：
 *   C2 ~ B2：数字行，自然音 '1'~'7'，升号用对应的 Shift 符号 '!' '@' '#' '$' '%'
 *   C3 ~ B5：沿用原键位('8'~'0' 起，依次跨 qwerty 各排)
 *   C6 ~ C7：剩余字母键 b B c C l m M n N v V x z
 */
#define BUZZER_KEY_NOTE_TABLE                                                                                          \
    {'1', NOTE_C2}, {'!', NOTE_CS2}, {'2', NOTE_D2}, {'@', NOTE_DS2}, {'3', NOTE_E2}, {'4', NOTE_F2}, {'$', NOTE_FS2}, \
        {'5', NOTE_G2}, {'%', NOTE_GS2}, {'6', NOTE_A2}, {'^', NOTE_AS2}, {'7', NOTE_B2}, {'8', NOTE_C3},              \
        {'*', NOTE_CS3}, {'9', NOTE_D3}, {'(', NOTE_DS3}, {'0', NOTE_E3}, {'q', NOTE_F3}, {'Q', NOTE_FS3},             \
        {'w', NOTE_G3}, {'W', NOTE_GS3}, {'e', NOTE_A3}, {'E', NOTE_AS3}, {'r', NOTE_B3}, {'t', NOTE_C4},              \
        {'T', NOTE_CS4}, {'y', NOTE_D4}, {'Y', NOTE_DS4}, {'u', NOTE_E4}, {'i', NOTE_F4}, {'I', NOTE_FS4},             \
        {'o', NOTE_G4}, {'O', NOTE_GS4}, {'p', NOTE_A4}, {'P', NOTE_AS4}, {'a', NOTE_B4}, {'s', NOTE_C5},              \
        {'S', NOTE_CS5}, {'d', NOTE_D5}, {'D', NOTE_DS5}, {'f', NOTE_E5}, {'g', NOTE_F5}, {'G', NOTE_FS5},             \
        {'h', NOTE_G5}, {'H', NOTE_GS5}, {'j', NOTE_A5}, {'J', NOTE_AS5}, {'k', NOTE_B5}, {'l', NOTE_C6},              \
        {'L', NOTE_CS6}, {'z', NOTE_D6}, {'Z', NOTE_DS6}, {'x', NOTE_E6}, {'c', NOTE_F6}, {'C', NOTE_FS6},             \
        {'v', NOTE_G6}, {'V', NOTE_GS6}, {'b', NOTE_A6}, {'B', NOTE_AS6}, {'n', NOTE_B6}, {                            \
        'm', NOTE_C7                                                                                                   \
    }

/** 键位与音符的映射项 */
typedef struct {
    char key;
    uint16_t freq_hz;
} Buzzer_KeyNote_t;

/** 键位绑定表实例，内容由 BUZZER_KEY_NOTE_TABLE 决定 */
static const Buzzer_KeyNote_t buzzer_key_notes[] = {BUZZER_KEY_NOTE_TABLE};

/**
 * @brief 键盘弹奏模式：读取按键并播放对应音符
 * @note  键位绑定见 BUZZER_KEY_NOTE_TABLE，按 BUZZER_PIANO_EXIT_CHAR 退出
 */
static void Buzzer_PianoMode(void) {
    extern Shell shell;

    logPrintln("Piano mode: press keys to play, press ESC to quit");

    char ch;
    for (;;) {
        if (shell.read(&ch, 1) == 0) {
            osDelay(5);
            continue;
        }
        if (ch == BUZZER_PIANO_EXIT_CHAR) break;

        for (uint32_t i = 0; i < sizeof(buzzer_key_notes) / sizeof(buzzer_key_notes[0]); i++) {
            if (buzzer_key_notes[i].key == ch) {
                Buzzer_Beep(buzzer_key_notes[i].freq_hz, BUZZER_PIANO_NOTE_MS);
                break;
            }
        }
    }

    Buzzer_Off();
    logPrintln("Piano mode quit");
}
#else
/* ------------------------- 开机提示音(有源) ------------------------- */

/** 有源蜂鸣器无法发声调，仅上电后短鸣一声 */
#define BUZZER_STARTUP_BEEP_MS 150U

/**
 * @brief 播放开机提示音
 */
static void Buzzer_PlayStartupTune(void) {
    Buzzer_On(0);
    osDelay(BUZZER_STARTUP_BEEP_MS);
    Buzzer_Off();
}
#endif /* !BUZZER_ACTIVE_MODE */

/**
 * @brief 确保蜂鸣器已初始化
 */
void Buzzer_EnsureInit(void) {
    if (!buzzer_inited) {
        Buzzer_Init();
    }
}

/**
 * @brief 按键位绑定表查询按键对应的音符频率
 * @param key 按键字符
 * @param freq_hz 输出音符频率(Hz)
 * @return true 已绑定
 * @return false 未绑定
 */
bool Buzzer_KeyToFreq(char key, uint16_t *freq_hz) {
#if BUZZER_ACTIVE_MODE
    (void)key;
    (void)freq_hz;
    return false; /* 有源蜂鸣器无音调概念 */
#else
    if (freq_hz == NULL) return false;

    for (uint32_t i = 0; i < sizeof(buzzer_key_notes) / sizeof(buzzer_key_notes[0]); i++) {
        if (buzzer_key_notes[i].key == key) {
            *freq_hz = buzzer_key_notes[i].freq_hz;
            return true;
        }
    }
    return false;
#endif
}

/**
 * @brief 初始化蜂鸣器
 * @return true 初始化成功
 * @return false 初始化失败
 */
bool Buzzer_Init(void) {
    MX_TIM4_Init();

    if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2) != HAL_OK) {
        return false;
    }

    __HAL_TIM_SET_PRESCALER(&htim4, BUZZER_TIM_PRESCALER);
    Buzzer_Off();

    buzzer_inited = true;

    /* 开机提示音结束后间隔 1s 再播报启动区提示音 */
    Buzzer_PlayStartupTune();
#if !BUZZER_ACTIVE_MODE
    osDelay(BUZZER_ZONE_TUNE_DELAY_MS);
    Buzzer_PlayZoneTune();
#endif

    return true;
}

/**
 * @brief 开始发声
 * @param freq_hz 发声频率(Hz)，有源模式下被忽略
 */
void Buzzer_On(uint32_t freq_hz) {
    if (!buzzer_inited) return;

#if BUZZER_ACTIVE_MODE
    (void)freq_hz; /* 有源蜂鸣器自带振荡电路，频率参数无效 */
    /* 比较值拉满，极性高时输出近似恒定高电平(有源蜂鸣器发声) */
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, __HAL_TIM_GET_AUTORELOAD(&htim4));
#else
    if (freq_hz < BUZZER_MIN_FREQ_HZ) freq_hz = BUZZER_MIN_FREQ_HZ;
    if (freq_hz > BUZZER_MAX_FREQ_HZ) freq_hz = BUZZER_MAX_FREQ_HZ;

    /* 计数时钟 = 1MHz，计数周期 = 1MHz / 发声频率 */
    uint32_t period = BUZZER_COUNTER_CLK_HZ / freq_hz;
    if (period == 0) period = 1;

    uint32_t arr = period - 1; /* 自动重装载值 */
    __HAL_TIM_SET_AUTORELOAD(&htim4, arr);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, period / 2); /* 50% 占空比 */

    buzzer_freq_hz = freq_hz;
#endif

    buzzer_on = true;
}

/**
 * @brief 停止发声
 * @note 比较值置 0; PWM1 + 极性高(见 tim.c 的 TIM4 OCPolarity)时输出恒为低电平,
 *       即蜂鸣器未使用/空闲时对应管脚保持低电平, 不会给蜂鸣器/驱动三极管持续通电。
 */
void Buzzer_Off(void) {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
    buzzer_on = false;
}

/**
 * @brief 阻塞式鸣叫指定时长
 * @param freq_hz 发声频率(Hz)，有源模式下被忽略
 * @param duration_ms 持续时长(ms)
 */
void Buzzer_Beep(uint32_t freq_hz, uint32_t duration_ms) {
    Buzzer_On(freq_hz);
    osDelay(duration_ms);
    Buzzer_Off();
}

/** 码盘提示音播放任务的栈大小(字节) */
#define BUZZER_OPS_TASK_STACK_SIZE (128 * 4)

/**
 * @brief 播放码盘提示音(阻塞)
 * @note  无源模式下约 1s；有源模式下仅短鸣一声
 */
static void Buzzer_PlayOpsTune(void) {
#if BUZZER_ACTIVE_MODE
    Buzzer_On(0);
    osDelay(BUZZER_STARTUP_BEEP_MS);
    Buzzer_Off();
#else
    Buzzer_PlayTune(ops_start_tune, sizeof(ops_start_tune) / sizeof(ops_start_tune[0]));
#endif
}

/**
 * @brief 码盘提示音播放任务：播放完成后删除自身
 */
static void Buzzer_OpsTune_Task(void *argument) {
    (void)argument;

    Buzzer_PlayOpsTune();
    osThreadExit();
}

/**
 * @brief 非阻塞播放码盘提示音
 * @note  内部临时创建任务播放, 播放完成后任务自行删除
 */
void Buzzer_PlayOpsTuneAsync(void) {
    const osThreadAttr_t attr = {
        .name = "BuzzerOps",
        .stack_size = BUZZER_OPS_TASK_STACK_SIZE,
        .priority = osPriorityLow,
    };

    if (osThreadNew(Buzzer_OpsTune_Task, NULL, &attr) == NULL) {
        logWarning("Failed to create buzzer ops tune task");
    }
}

/* ------------------------- Shell 调试命令 ------------------------- */

static void Buzzer_PrintUsage(void) {
    logPrintln("Usage: buzzer <command>");
    logPrintln("  status              show current state");
#if BUZZER_ACTIVE_MODE
    logPrintln("  on                  start tone");
    logPrintln("  off                 stop tone");
    logPrintln("  beep [ms]           beep once (default 100 ms)");
#else
    logPrintln("  on [freq_hz]        start tone (default %d Hz)", BUZZER_DEFAULT_FREQ_HZ);
    logPrintln("  off                 stop tone");
    logPrintln("  beep [freq] [ms]    beep once (default %d Hz, 100 ms)", BUZZER_DEFAULT_FREQ_HZ);
    logPrintln("  sweep               sweep 500~5000 Hz to find resonance");
    logPrintln("  piano               keyboard piano mode (C3~B5)");
#endif
}

/**
 * @brief 蜂鸣器调试命令
 * @param argc 参数个数
 * @param argv 参数列表
 */
static void Buzzer_Tool(int argc, char *argv[]) {
    /* 调试用：未初始化时自动初始化一次 */
    if (!buzzer_inited && !Buzzer_Init()) {
        logWarning("Buzzer init failed");
        return;
    }

    if (argc < 2) {
        Buzzer_PrintUsage();
        return;
    }

    if (strcmp(argv[1], "status") == 0) {
#if BUZZER_ACTIVE_MODE
        logPrintln("Buzzer(active): %s", buzzer_on ? "on" : "off");
#else
        logPrintln("Buzzer(passive): %s, freq: %d Hz", buzzer_on ? "on" : "off", buzzer_freq_hz);
#endif
    } else if (strcmp(argv[1], "on") == 0) {
        uint32_t freq = (argc >= 3) ? (uint32_t)atoi(argv[2]) : BUZZER_DEFAULT_FREQ_HZ;
        Buzzer_On(freq);
        logPrintln("Buzzer on");
    } else if (strcmp(argv[1], "off") == 0) {
        Buzzer_Off();
        logPrintln("Buzzer off");
    } else if (strcmp(argv[1], "beep") == 0) {
        uint32_t freq = (argc >= 3) ? (uint32_t)atoi(argv[2]) : BUZZER_DEFAULT_FREQ_HZ;
        uint32_t ms = (argc >= 4) ? (uint32_t)atoi(argv[3]) : 100;
        logPrintln("Beep %d ms", ms);
        Buzzer_Beep(freq, ms);
    } else if (strcmp(argv[1], "sweep") == 0) {
#if BUZZER_ACTIVE_MODE
        logPrintln("Active buzzer: sweep not applicable");
#else
        logPrintln("Sweeping 500~5000 Hz...");
        for (uint32_t freq = 500; freq <= 5000; freq += 100) {
            Buzzer_On(freq);
            osDelay(60);
        }
        Buzzer_Off();
        logPrintln("Sweep done");
#endif
    } else if (strcmp(argv[1], "piano") == 0) {
#if BUZZER_ACTIVE_MODE
        logPrintln("Active buzzer: piano not applicable");
#else
        Buzzer_PianoMode();
#endif
    } else {
        Buzzer_PrintUsage();
    }
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_DISABLE_RETURN,
                 buzzer,
                 Buzzer_Tool,
                 Buzzer Tool);
