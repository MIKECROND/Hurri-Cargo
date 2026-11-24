/**
 * @file pwm_servo_driver.h
 * @brief PWM 舵机驱动头文件
 *
 * 硬件连接：TIM9_CH1 -> PE5(舵机 1)，TIM9_CH2 -> PE6(舵机 2)
 * 定时器配置：168MHz / (167 + 1) = 1MHz 计数，ARR = 19999 -> 50Hz(20ms)
 * 因此比较值单位即 1us，可直接用脉宽(us)控制舵机
 *
 * 是否启用由 PWM_SERVO_ENABLED 控制：关闭时不初始化 TIM9，接口退化为空实现
 */

#ifndef __PWM_SERVO_DRIVER_H__
#define __PWM_SERVO_DRIVER_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "tim.h"

/** 功能开关：0 关闭 PWM 舵机(不初始化 TIM9，舵机保持卸力)；允许构建系统 -D 覆盖 */
#ifndef PWM_SERVO_ENABLED
#define PWM_SERVO_ENABLED 0
#endif

/** 舵机通道数量(与 TIM9 已配置的通道一一对应) */
#ifndef PWM_SERVO_CH_NUM
#define PWM_SERVO_CH_NUM 2U
#endif

/** 角度范围(度) */
#ifndef PWM_SERVO_ANGLE_MIN
#define PWM_SERVO_ANGLE_MIN 0U
#endif
#ifndef PWM_SERVO_ANGLE_MAX
#define PWM_SERVO_ANGLE_MAX 270U
#endif

/** 角度两端对应的脉宽(us)，按实际舵机型号调整 */
#ifndef PWM_SERVO_PULSE_MIN_US
#define PWM_SERVO_PULSE_MIN_US 500U
#endif
#ifndef PWM_SERVO_PULSE_MAX_US
#define PWM_SERVO_PULSE_MAX_US 2500U
#endif

/**
 * @brief 初始化舵机 PWM 输出
 * @return true 初始化成功
 * @return false 初始化失败(PWM_SERVO_ENABLED 为 0 时恒返回 false)
 * @note 重复调用直接返回成功
 */
bool PWM_Servo_Init(void);

/**
 * @brief 停止 PWM 输出并取消初始化
 * @note 关闭后舵机卸力，再次调用 PWM_Servo_Init 可恢复输出
 */
void PWM_Servo_DeInit(void);

/**
 * @brief 查询是否已初始化
 * @return true 已初始化
 * @return false 未初始化
 */
bool PWM_Servo_IsInit(void);

/**
 * @brief 设置舵机角度
 * @param id 舵机编号(0 ~ PWM_SERVO_CH_NUM-1)
 * @param angle 目标角度(度)，超出 PWM_SERVO_ANGLE_MIN ~ PWM_SERVO_ANGLE_MAX 会被限幅
 * @note 未初始化或编号非法时直接返回
 */
void PWM_Servo_SetAngle(uint8_t id, uint16_t angle);

/**
 * @brief 直接按脉宽设置舵机(调试/标定用，跳过角度换算)
 * @param id 舵机编号(0 ~ PWM_SERVO_CH_NUM-1)
 * @param pulse_us 脉宽(us)，超出 PWM_SERVO_PULSE_MIN_US ~ PWM_SERVO_PULSE_MAX_US 会被限幅
 * @return true 设置成功
 * @return false 未初始化或编号非法
 */
bool PWM_Servo_SetPulseUs(uint8_t id, uint16_t pulse_us);

/**
 * @brief 读取舵机当前角度
 * @param id 舵机编号(0 ~ PWM_SERVO_CH_NUM-1)
 * @param angle 输出当前角度(度)
 * @return true 读取成功
 * @return false 未初始化、编号非法或出参为空
 */
bool PWM_Servo_GetAngle(uint8_t id, uint16_t *angle);

/**
 * @brief 读取舵机当前脉宽
 * @param id 舵机编号(0 ~ PWM_SERVO_CH_NUM-1)
 * @param pulse_us 输出当前脉宽(us)
 * @return true 读取成功
 * @return false 未初始化、编号非法或出参为空
 */
bool PWM_Servo_GetPulseUs(uint8_t id, uint16_t *pulse_us);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __PWM_SERVO_DRIVER_H__ */
