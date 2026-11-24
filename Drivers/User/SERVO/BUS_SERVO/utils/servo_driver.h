/**
 * @file servo_driver.h
 * @brief 舵机模块驱动接口
 */

#ifndef __SERVO_DRIVER_H__
#define __SERVO_DRIVER_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <stdint.h>
#include "servo_uart.h"

extern StreamBufferHandle_t Servo_Rx_StreamHandle;

void Servo_Init(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __SERVO_DRIVER_H__ */
