/**
 * @file buzzer.h
 * @author T-Car Team
 * @brief 无源蜂鸣器驱动接口
 *
 * 硬件连接：PB7 -> TIM4_CH2，通过 PWM 输出方波驱动无源蜂鸣器，
 * 输出频率即蜂鸣器发声频率，需给到接近其谐振频率时声音最响。
 *
 * 空闲电平：TIM4_CH2 配置为 PWM1 + 极性高（tim.c / .ioc 的 TIM4.OCPolarity_2=HIGH），
 * 停止发声时 PWM 比较值清 0 -> PB7 输出低电平；即蜂鸣器未使用/空闲时管脚保持低电平。
 * 可调参数见 buzzer.h。
 */

#ifndef __BUZZER_H__
#define __BUZZER_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <stdbool.h>
#include <stdint.h>

/* ------------------------- 配置宏 ------------------------- */

/**
 * 蜂鸣器类型选择：
 *   0 = 无源蜂鸣器，用 PWM 调频驱动，可发声调/播放开机小曲
 *   1 = 有源蜂鸣器，内部自带振荡电路，仅支持恒定电平开关，频率参数无效
 */
#define BUZZER_ACTIVE_MODE 0

/** TIM4 计数器时钟频率(Hz)：APB1=42MHz，定时器时钟倍频后为 84MHz */
#define BUZZER_TIM_CLK_HZ 84000000U

/** 预分频系数，分频后计数频率为 1MHz，即频率分辨率为 1us */
#define BUZZER_TIM_PRESCALER 83U

/** 支持的发声频率范围(Hz)，超出范围自动截断 */
#define BUZZER_MIN_FREQ_HZ 20U
#define BUZZER_MAX_FREQ_HZ 20000U

/** 默认发声频率(Hz)，无源蜂鸣器谐振点通常在 2~4kHz */
#define BUZZER_DEFAULT_FREQ_HZ 2700U

/* ------------------------- 接口函数 ------------------------- */

/**
 * @brief 初始化无源蜂鸣器，并播放开机提示音与启动区提示音
 * @note  阻塞播放：开机提示音结束后间隔 1s，再播放 DIP 开关所选启动区的提示音；
 *        无源模式下总时长约 3~3.5s，有源模式下仅短鸣一声
 * @return true 初始化成功
 * @return false 初始化失败
 */
bool Buzzer_Init(void);

/**
 * @brief 以指定频率开始发声
 * @param freq_hz 发声频率(Hz)，超出 [BUZZER_MIN_FREQ_HZ, BUZZER_MAX_FREQ_HZ] 时自动截断
 * @note  有源模式(BUZZER_ACTIVE_MODE=1)下频率参数被忽略，仅输出恒定高电平
 */
void Buzzer_On(uint32_t freq_hz);

/**
 * @brief 停止发声
 * @note  停止后 PWM 比较值清 0，管脚输出低电平（蜂鸣器未使用时不通电）；
 *        需停掉"未使用"蜂鸣器的管脚电平时调用本函数即可。
 */
void Buzzer_Off(void);

/**
 * @brief 阻塞式鸣叫指定时长
 * @param freq_hz 发声频率(Hz)，有源模式下被忽略
 * @param duration_ms 持续时长(ms)
 */
void Buzzer_Beep(uint32_t freq_hz, uint32_t duration_ms);

/**
 * @brief 非阻塞播放码盘提示音(首次收到码盘数据时调用)
 * @note  内部临时创建任务播放, 播放完成后任务自行删除, 不阻塞调用者
 */
void Buzzer_PlayOpsTuneAsync(void);

/**
 * @brief 确保蜂鸣器已初始化
 * @note  未初始化时才执行初始化(含开机提示音)，已初始化则什么都不做
 */
void Buzzer_EnsureInit(void);

/**
 * @brief 按键位绑定表查询按键对应的音符频率
 * @param key 按键字符，与 buzzer.c 内的 BUZZER_KEY_NOTE_TABLE 一致，如 's' -> NOTE_C5
 * @param freq_hz 输出该按键对应的音符频率(Hz)
 * @return true 该按键已绑定音符
 * @return false 未绑定(有源模式下恒为 false)
 */
bool Buzzer_KeyToFreq(char key, uint16_t *freq_hz);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __BUZZER_H__ */
