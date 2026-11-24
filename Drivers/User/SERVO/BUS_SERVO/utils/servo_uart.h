/**
 * @file servo_uart.h
 * @brief 舵机模块串口接口
 */

#ifndef __SERVO_UART_H__
#define __SERVO_UART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "Events.h"
#include "stream_buffer.h"

extern UART_HandleTypeDef huart3;

/* 最大应答帧 21 字节(帧头2+cmd1+len1+content16+checksum1);
   同步读取 2 个舵机可能连续到达 2 帧, 这里留足余量, 避免 DMA 提前填满后停摆 */
#define USART3_RX_BUF_SIZE 64

void Servo_UART_TxCallback(UART_HandleTypeDef *huart);
void Servo_UART_RxCallback(UART_HandleTypeDef *huart, uint16_t Size);
void Servo_UART_ErrorCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
} /* __cplusplus */
#endif

#endif
