/**
 * @file pwm_servo_driver.c
 * @brief PWM 舵机驱动实现
 *
 * TIM9 计数时钟为 1MHz(168MHz / 168)，ARR = 19999，
 * 因此比较值单位即 1us，周期 20ms，可直接按脉宽控制舵机。
 */

#include "pwm_servo_driver.h"

#if !PWM_SERVO_ENABLED

/* ================= 关闭分支 ================= */

/* 关闭时保留空实现，上层(servo_port / arm)无需关心开关状态 */
bool PWM_Servo_Init(void) {
    return false;
}

void PWM_Servo_DeInit(void) {
}

bool PWM_Servo_IsInit(void) {
    return false;
}

void PWM_Servo_SetAngle(uint8_t id, uint16_t angle) {
    (void)id;
    (void)angle;
}

bool PWM_Servo_SetPulseUs(uint8_t id, uint16_t pulse_us) {
    (void)id;
    (void)pulse_us;

    return false;
}

bool PWM_Servo_GetAngle(uint8_t id, uint16_t *angle) {
    (void)id;
    (void)angle;

    return false;
}

bool PWM_Servo_GetPulseUs(uint8_t id, uint16_t *pulse_us) {
    (void)id;
    (void)pulse_us;

    return false;
}

/* ================= 启用分支 ================= */
#else /* PWM_SERVO_ENABLED */

/** 舵机编号与 TIM 通道的映射表，顺序与硬件接线一致 */
static const uint32_t servo_channel_map[PWM_SERVO_CH_NUM] = {
    TIM_CHANNEL_1,
    TIM_CHANNEL_2,
};

static bool is_init = false;

/**
 * @brief 角度换算为定时器比较值(单位 1us)
 * @param angle 目标角度(度)
 * @return 对应脉宽的比较值
 */
static uint16_t PWM_Servo_AngleToCompare(uint16_t angle) {
    int32_t clamped = (int32_t)angle;

    if (clamped < (int32_t)PWM_SERVO_ANGLE_MIN) {
        clamped = (int32_t)PWM_SERVO_ANGLE_MIN;
    } else if (clamped > (int32_t)PWM_SERVO_ANGLE_MAX) {
        clamped = (int32_t)PWM_SERVO_ANGLE_MAX;
    }

    /* 线性插值：pulse = min + (angle - angle_min) * (max - min) / (angle_max - angle_min) */
    uint32_t pulse_us = PWM_SERVO_PULSE_MIN_US + (uint32_t)(clamped - (int32_t)PWM_SERVO_ANGLE_MIN) *
                                                     (PWM_SERVO_PULSE_MAX_US - PWM_SERVO_PULSE_MIN_US) /
                                                     (PWM_SERVO_ANGLE_MAX - PWM_SERVO_ANGLE_MIN);

    return (uint16_t)pulse_us;
}

/**
 * @brief 定时器比较值(单位 1us)反算为角度
 * @param pulse_us 当前脉宽
 * @return 对应角度(度)
 */
static uint16_t PWM_Servo_CompareToAngle(uint16_t pulse_us) {
    int32_t pulse = (int32_t)pulse_us;

    if (pulse < (int32_t)PWM_SERVO_PULSE_MIN_US) {
        pulse = (int32_t)PWM_SERVO_PULSE_MIN_US;
    } else if (pulse > (int32_t)PWM_SERVO_PULSE_MAX_US) {
        pulse = (int32_t)PWM_SERVO_PULSE_MAX_US;
    }

    return (uint16_t)((int32_t)PWM_SERVO_ANGLE_MIN + (pulse - (int32_t)PWM_SERVO_PULSE_MIN_US) *
                                                         (int32_t)(PWM_SERVO_ANGLE_MAX - PWM_SERVO_ANGLE_MIN) /
                                                         (int32_t)(PWM_SERVO_PULSE_MAX_US - PWM_SERVO_PULSE_MIN_US));
}

bool PWM_Servo_Init(void) {
    if (is_init) {
        return true;
    }

    MX_TIM9_Init();

    if (HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1) != HAL_OK) {
        return false;
    }

    if (HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2) != HAL_OK) {
        (void)HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_1);
        return false;
    }

    is_init = true;

    return is_init;
}

void PWM_Servo_DeInit(void) {
    if (!is_init) {
        return;
    }

    for (uint8_t id = 0; id < PWM_SERVO_CH_NUM; id++) {
        (void)HAL_TIM_PWM_Stop(&htim9, servo_channel_map[id]);
    }

    is_init = false;
}

bool PWM_Servo_IsInit(void) {
    return is_init;
}

void PWM_Servo_SetAngle(uint8_t id, uint16_t angle) {
    (void)PWM_Servo_SetPulseUs(id, PWM_Servo_AngleToCompare(angle));
}

bool PWM_Servo_SetPulseUs(uint8_t id, uint16_t pulse_us) {
    if (!is_init || id >= PWM_SERVO_CH_NUM) {
        return false;
    }

    if (pulse_us < PWM_SERVO_PULSE_MIN_US) {
        pulse_us = PWM_SERVO_PULSE_MIN_US;
    } else if (pulse_us > PWM_SERVO_PULSE_MAX_US) {
        pulse_us = PWM_SERVO_PULSE_MAX_US;
    }

    __HAL_TIM_SET_COMPARE(&htim9, servo_channel_map[id], pulse_us);

    return true;
}

bool PWM_Servo_GetPulseUs(uint8_t id, uint16_t *pulse_us) {
    if (!is_init || id >= PWM_SERVO_CH_NUM || pulse_us == NULL) {
        return false;
    }

    *pulse_us = (uint16_t)__HAL_TIM_GET_COMPARE(&htim9, servo_channel_map[id]);

    return true;
}

bool PWM_Servo_GetAngle(uint8_t id, uint16_t *angle) {
    uint16_t pulse_us = 0;

    if (angle == NULL || !PWM_Servo_GetPulseUs(id, &pulse_us)) {
        return false;
    }

    *angle = PWM_Servo_CompareToAngle(pulse_us);

    return true;
}

#endif /* PWM_SERVO_ENABLED */
