/**
 * @file servo_driver.c
 * @brief 舵机模块驱动实现
 */

#include "servo_driver.h"
#include "cmsis_os2.h"
#include "log.h"
#include "servo_protocol.h"
#include "main.h"
#include "stream_buffer.h"
#include "usart.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "Events.h"

extern uint8_t rx3Buffer[];
StreamBufferHandle_t Servo_Rx_StreamHandle = NULL;

/**
 * @brief 舵机模块初始化
 * @note USART3 配置为 115200 8N1(与原库默认波特率一致), 接收使用 DMA + IDLE
 */
void Servo_Init(void) {
    /* 先把协议层事务锁建好: 后续多个任务共用舵机总线时, 读类事务需要串行化 */
    Servo_ProtocolInit();

    MX_USART3_UART_Init();

    /* 接收缓冲与回调必须先于 DMA 装填: 装填后随时可能触发 IDLE/TC 事件,
       回调未注册时会落到弱定义的空实现上, DMA 不会重装, 接收会静默失效 */
    Servo_Rx_StreamHandle = xStreamBufferCreate(512, 1);
    configASSERT(Servo_Rx_StreamHandle != NULL);

    osEventFlagsClear(System_StatusHandle, UART3_TX_IDLE);

    if (HAL_UART_RegisterCallback(&huart3, HAL_UART_TX_COMPLETE_CB_ID, Servo_UART_TxCallback) != HAL_OK) {
        Error_Handler();
    }

    /* 注册接收事件回调函数 */
    if (HAL_UART_RegisterRxEventCallback(&huart3, Servo_UART_RxCallback) != HAL_OK) {
        Error_Handler();
    }

    /* 注册错误回调函数 */
    if (HAL_UART_RegisterCallback(&huart3, HAL_UART_ERROR_CB_ID, Servo_UART_ErrorCallback) != HAL_OK) {
        Error_Handler();
    }

    /* 最后再装填接收, 保证回调/缓冲区已就绪 */
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx3Buffer, USART3_RX_BUF_SIZE) != HAL_OK) {
        logError("servo uart rx start failed");
        Error_Handler();
    }
}

/**
 * @brief 串口发送任务
 * @param argument 任务参数
 * @note  队列元素类型必须与 Servo_SendPackage_Common 入队的 Package_t 一致
 *        (见 Core/Src/freertos.c 中 Servo_Tx_Data 的创建)。
 */
void Servo_Tx_Task(void *argument) {
    (void)argument;
    extern osMessageQueueId_t Servo_Tx_DataHandle;

    static Package_t pkg;
    static uint8_t txBuffer[SERVO_TX_FRAME_MAX];
    uint16_t len;
    uint32_t waitResult;

    osEventFlagsWait(System_StatusHandle, SYS_INIT_COMPLETE, osFlagsNoClear, osWaitForever);
    for (;;) {
        if (osMessageQueueGet(Servo_Tx_DataHandle, &pkg, NULL, osWaitForever) != osOK) {
            continue;
        }

        len = Servo_PackageToBytes(&pkg, txBuffer, sizeof(txBuffer));
        if (len == 0U) {
            logError("servo tx frame too large: size=%u", (unsigned)pkg.size);
            continue;
        }

        osEventFlagsClear(System_StatusHandle, UART3_TX_IDLE);
        if (HAL_UART_Transmit_DMA(&huart3, txBuffer, len) != HAL_OK) {
            logError("servo tx dma start failed");
            (void)HAL_UART_AbortTransmit(&huart3);
            continue;
        }

        /* 必须带超时: DMA/UART 异常时 TX_COMPLETE 回调不会触发,
           否则本任务会永久挂起, 后续所有舵机命令都发不出去 */
        waitResult = osEventFlagsWait(System_StatusHandle, UART3_TX_IDLE, osFlagsWaitAny, pdMS_TO_TICKS(50));
        if ((waitResult & osFlagsError) != 0U) {
            logError("servo tx timeout");
            (void)HAL_UART_AbortTransmit(&huart3);
        }

        /* 注意: 这里不能加 osDelay 节拍。发送是异步的, 调用方在入队后立即等待应答;
           若本任务发送完还人为延时, 后面排队的命令会错过 100ms 应答窗口。 */
    }
}
