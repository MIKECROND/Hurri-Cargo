/**
 * @file servo_port.h
 * @author T-Car Team
 * @brief 舵机模块端口接口
 * 可调参数见 servo_port.h。
 */

#ifndef __SERVO_PORT_H__
#define __SERVO_PORT_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "servo_driver.h"
#include "servo_protocol.h"
#include "pwm_servo_driver.h"

/** 统一舵机 ID 分配：1、2 为总线舵机(总线物理 ID 与逻辑 ID 相同)，3、4 为 PWM 舵机 */
/* PWM_SERVO_ENABLED 为 0 时，3、4 号仍按 PWM 舵机分配，但调用会被驱动空实现忽略 */
#define SERVO_PORT_BUS_ID_MIN 1U
#define SERVO_PORT_BUS_ID_MAX (SERVO_PORT_BUS_ID_MIN + 1U)
#define SERVO_PORT_PWM_ID_MIN (SERVO_PORT_BUS_ID_MAX + 1U)
#define SERVO_PORT_PWM_ID_MAX (SERVO_PORT_PWM_ID_MIN + PWM_SERVO_CH_NUM - 1U)

/**
 * @brief 统一舵机角度设置接口
 * @param servo_id 舵机 ID：1、2 为总线舵机，3、4 为 PWM 舵机
 * @param angle 目标角度(度)
 * @note PWM 舵机首次调用时自动初始化 PWM 输出；ID 非法时直接忽略
 */
void Servo_Port_SetAngle(uint8_t servo_id, float angle);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __SERVO_PORT_H__ */
