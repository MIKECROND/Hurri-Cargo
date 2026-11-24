/**
 * @file servo_port.c
 * @author T-Car Team
 * @brief 舵机模块端口实现
 *
 * 对上层提供统一的舵机角度接口：
 *   ID 1、2 -> 总线舵机(物理 ID 与逻辑 ID 相同)
 *   ID 3、4 -> PWM 舵机(TIM9_CH1/CH2)
 * 可调参数见 servo_port.h。
 */

#include "servo_port.h"

/**
 * @brief 角度限幅并四舍五入为整度
 * @param angle 输入角度
 * @return 限幅后的整度值
 */
static uint16_t Servo_Port_ToPwmAngle(float angle) {
    if (angle < (float)PWM_SERVO_ANGLE_MIN) {
        angle = (float)PWM_SERVO_ANGLE_MIN;
    } else if (angle > (float)PWM_SERVO_ANGLE_MAX) {
        angle = (float)PWM_SERVO_ANGLE_MAX;
    }

    return (uint16_t)(angle + 0.5f);
}

void Servo_Port_SetAngle(uint8_t servo_id, float angle) {

#if PWM_SERVO_ENABLED
    if ((servo_id >= SERVO_PORT_BUS_ID_MIN) && (servo_id <= SERVO_PORT_BUS_ID_MAX)) {
        /* 总线舵机走原有协议接口，速度/功率参数保持默认(0,0) */
        (void)Servo_SetServoAngle(servo_id, angle, 0, 0);
    } else if ((servo_id >= SERVO_PORT_PWM_ID_MIN) && (servo_id <= SERVO_PORT_PWM_ID_MAX)) {
        /* PWM 输出未启动时先初始化，避免依赖外部初始化顺序 */
        if (!PWM_Servo_IsInit() && !PWM_Servo_Init()) {
            return;
        }

        PWM_Servo_SetAngle((uint8_t)(servo_id - SERVO_PORT_PWM_ID_MIN), Servo_Port_ToPwmAngle(angle));
    }
#else
    Servo_SetServoAngle(servo_id, angle,0,0);
#endif
}
