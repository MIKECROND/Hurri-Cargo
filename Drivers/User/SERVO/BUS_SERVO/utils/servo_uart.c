/**
 * @file servo_uart.c
 * @brief 舵机模块串口实现
 */

#include "servo_uart.h"
#include "log.h"

uint8_t rx3Buffer[USART3_RX_BUF_SIZE];

/**
 * @brief 舵机串口发送回调函数
 * @param huart 串口句柄
 */
void Servo_UART_TxCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
        osEventFlagsSet(System_StatusHandle, UART3_TX_IDLE);
    }
}

/**
 * @brief 舵机串口接收回调函数
 * @param huart 串口句柄
 * @param Size 接收数据大小
 * @note  DMA 正常模式下 IDLE 表示一帧结束; 缓冲区填满(TC)也必须处理,
 *        否则 HAL 已停止 DMA 而本回调不重装, USART3 接收会永久停摆。
 */
void Servo_UART_RxCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance != USART3) {
        return;
    }

    /* 半传输事件不代表帧结束: DMA 仍在填充后半段, 此时 TxState/RxState 仍为 BUSY,
       重装填必然返回 HAL_BUSY, 直接返回等待 IDLE/TC */
    if ((huart->RxEventType != HAL_UART_RXEVENT_IDLE) && (huart->RxEventType != HAL_UART_RXEVENT_TC)) {
        return;
    }

    if ((Size > 0U) && (Size <= USART3_RX_BUF_SIZE)) {
        extern StreamBufferHandle_t Servo_Rx_StreamHandle;

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        (void)xStreamBufferSendFromISR(Servo_Rx_StreamHandle, rx3Buffer, Size, &xHigherPriorityTaskWoken);

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    /* 本次接收已结束(HAL 已把 RxState 置回 READY), 重装填以接收下一帧 */
    if (HAL_UARTEx_ReceiveToIdle_DMA(huart, rx3Buffer, USART3_RX_BUF_SIZE) != HAL_OK) {
        logWarning("servo uart rx re-arm failed");
    }
}

/**
 * @brief 舵机串口错误回调函数
 * @param huart 串口句柄
 */
void Servo_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
        __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_PE | UART_FLAG_FE | UART_FLAG_NE | UART_FLAG_ORE);
        huart->RxState = HAL_UART_STATE_READY;
        huart->gState = HAL_UART_STATE_READY;
        if (HAL_UARTEx_ReceiveToIdle_DMA(huart, rx3Buffer, USART3_RX_BUF_SIZE) != HAL_OK) {
            logWarning("servo uart rx re-arm failed after error");
        }
    }
}
