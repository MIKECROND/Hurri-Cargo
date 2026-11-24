/**
 * @file shell_port.c
 * @author Letter (NevermindZZT@gmail.com)
 * @brief
 * @version 0.1
 * @date 2019-02-22
 *
 * @copyright (c) 2019 Letter
 *
 */

#include "FreeRTOS.h"
#include "task.h"
#include "Events.h"
#include "shell_port.h"
#include "log.h"
#include "usart.h"
#include "string.h"
#include <stdbool.h>

Shell shell;
char shellBuffer[SHELL_BUFFER_SIZE];

#define USART1_RX_BUF_SIZE 32
static uint8_t rx1Buffer[USART1_RX_BUF_SIZE];

/* UART1 发送互斥：log(shellWriteEndLine) 与 shell 回显/命令输出可能来自不同任务，
 * 原实现用"清事件标志 + DMA + 无锁等标志"的方式并发写会互相踩：后到的任务可能
 * 清掉前一条的完成标志、HAL_UART_Transmit_DMA 返回 HAL_BUSY 被忽略，
 * 结果丢一段输出/两段输出交错。加锁后每条 write 原子完成（只有日志/命令输出，
 * 不在电机/导航控制链路上；单次等待上限 = 一条 DMA 的发送时间）。 */
static osMutexId_t s_shell_tx_mutex = NULL;
/* 单块 DMA 发送完成最长等待(ms)：1KB@1Mbps≈8.2ms；100ms 兜底避免 TX 完成回调
 * 丢失时 Shell 任务永久挂死（宁可丢一段输出也不要卡死 shell）。 */
#define SHELL_TX_WAIT_TIMEOUT_MS 100U

/**
 * @brief 用户shell写
 * @param data 数据
 * @param len 数据长度
 * @return 实际写入的数据长度
 */
static short userShellWrite(char *data, uint32_t len) {
    uint32_t total_sent = 0;
    const uint16_t MAX_DMA_LEN = 0xFFFF;  // DMA单次最大发送长度
    bool locked = false;

    if (s_shell_tx_mutex != NULL) {
        if (osMutexAcquire(s_shell_tx_mutex, osWaitForever) == osOK) {
            locked = true;
        }
    }

    while (len > 0) {
        // 计算本次发送长度，不超过DMA限制
        uint16_t send_len = len > MAX_DMA_LEN ? MAX_DMA_LEN : (uint16_t)len;

        // 发送数据
        osEventFlagsClear(System_StatusHandle, UART1_TX_IDLE);
        if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)data, send_len) != HAL_OK) {
            /* UART 忙/参数非法：不要再等标志（可能永远等不到），丢弃本块剩余数据返回 */
            break;
        }
        uint32_t flags = osEventFlagsWait(System_StatusHandle, UART1_TX_IDLE, osFlagsWaitAny, SHELL_TX_WAIT_TIMEOUT_MS);
        if ((flags & UART1_TX_IDLE) == 0) {
            /* 超时/事件错误：放弃剩余数据，避免 Shell 永久阻塞 */
            break;
        }

        // 更新进度
        total_sent += send_len;
        data += send_len;
        len -= send_len;
    }

    if (locked) {
        osMutexRelease(s_shell_tx_mutex);
    }
    return (short)total_sent;
}

/**
* @brief 用户shell读
* @param data 数据缓冲区
* @param len 数据长度
* @return 实际读取的数据长度
*/
static short userShellRead(char *data, unsigned short len) {
    extern osMessageQueueId_t Usart1_Rx_DataHandle;
   if (len == 0) return 0;

   if (osMessageQueueGet(Usart1_Rx_DataHandle, data, NULL, 0) == osOK) {
       return 1;
   }
   return 0;
}

void Shell_UART_TxCallback(UART_HandleTypeDef *huart){
    if (huart->Instance == USART1) {
        osEventFlagsSet(System_StatusHandle, UART1_TX_IDLE);
    }
}

/**
* @brief 用户shell接收事件回调函数
* @param huart 串口句柄
* @param Size 接收数据长度
*/
/* USART1 接收队列溢出丢字节计数：ISR 只累加，Shell_Task 里打印 */
static volatile uint32_t s_shell_rx_dropped = 0;

void Shell_UART_RxCallback(UART_HandleTypeDef *huart, uint16_t Size){
    if (huart->Instance == USART1) {
        if (huart->RxEventType == HAL_UART_RXEVENT_IDLE) {
        extern osMessageQueueId_t Usart1_Rx_DataHandle;

        // 发送接收数据到消息队列
        for (uint16_t i = 0; i < Size; i++) {
            osStatus_t state = osMessageQueuePut(Usart1_Rx_DataHandle, &rx1Buffer[i], NULL, 0);
            if (state != osOK) {
            /* 中断上下文禁止阻塞打印：HAL_UART_Transmit 在 ISR 里会阻塞（1Mbps 下几十字节
             * 也要几十 us，且与其它优先级 5 的中断互相影响）。这里只累加计数，
             * 由 Shell_Task（任务上下文）限速打印。 */
            s_shell_rx_dropped++;
            break; // 如果队列满了，跳出循环
            }
        }
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx1Buffer, USART1_RX_BUF_SIZE);
  }
}

/**
* @brief 用户shell错误回调函数
* @param huart 串口句柄
*/
void Shell_UART_ErrorCallback(UART_HandleTypeDef *huart){
    if (huart->Instance == USART1) {
        __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_PE | UART_FLAG_FE | UART_FLAG_NE | UART_FLAG_ORE);
        huart->RxState = HAL_UART_STATE_READY;
        huart->gState = HAL_UART_STATE_READY;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, rx1Buffer, USART1_RX_BUF_SIZE);
    }
}

/**
* @brief 用户shell初始化
*/
void userShellInit(void) {

    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx1Buffer, USART1_RX_BUF_SIZE);
    /* 初始化串口之后 */

    /* 注册发送完成回调函数 */
    if (HAL_UART_RegisterCallback(&huart1,
         HAL_UART_TX_COMPLETE_CB_ID,
         Shell_UART_TxCallback) != HAL_OK) {

        Error_Handler();
    }

    /* 注册接收事件回调函数 */
    if (HAL_UART_RegisterRxEventCallback(&huart1,
        Shell_UART_RxCallback) != HAL_OK) {

        Error_Handler();
    }

    /* 注册错误回调函数 */
    if (HAL_UART_RegisterCallback(&huart1,
         HAL_UART_ERROR_CB_ID,
         Shell_UART_ErrorCallback) != HAL_OK) {

        Error_Handler();
    }

   shell.write = userShellWrite;
   shell.read = userShellRead;

   /* 创建 UART1 发送互斥（scheduler 已运行，由 Sys_Init 任务调用） */
   s_shell_tx_mutex = osMutexNew(NULL);
   if (s_shell_tx_mutex == NULL) {
       /* 此时 logInit() 还没执行，用 my_printf 保证这条告警能打出来 */
       my_printf("[warn] shell: failed to create UART1 TX mutex, output may interleave\r\n");
   }

   shellInit(&shell, shellBuffer, SHELL_BUFFER_SIZE);
}

/**
* @brief 用户shell任务
* @param argument 任务参数
*/
void Shell_Task(void *argument) {
    (void)argument;

    osEventFlagsWait(System_StatusHandle, SYS_INIT_COMPLETE, osFlagsNoClear, osWaitForever);

    char data;
    for(;;) {
        osDelay(1);
        // 优先将数据发送给需要使用串口的应用程序
        if(osEventFlagsGet(System_StatusHandle) & APP_NEED_USART) continue;
        /* 一次唤醒尽量把 UART1 接收队列排空：原实现 osDelay(5) + 每次只取 1 字节，
         * 吞吐只有约 200B/s，而 USART1 是 1Mbps；粘贴/长命令(>32B)时 32 字节的
         * 接收队列必然溢出丢字符（表现为命令缺字符/不识别）。这里改为每 1ms 唤醒
         * 一次、每次最多连续处理 64 字节，命令分发延迟也从 ~5ms 降到 ~1ms。
         * Shell 任务优先级低（Normal2，低于 Mission/Nav/Motor），不会影响控制链路。 */
        for (uint8_t i = 0; i < 64; i++) {
            if (!shell.read(&data, 1)) break;
            shellHandler(&shell, data);
        }

        /* 接收队列溢出提示：ISR 只计数，这里（任务上下文）限速打印一次 */
        if (s_shell_rx_dropped != 0) {
            logWarning("USART1 rx queue overflow, %lu byte(s) dropped", (unsigned long)s_shell_rx_dropped);
            s_shell_rx_dropped = 0;
        }
    }
}

/**
 * @brief 软件重置
 * @param argc 参数数量
 * @param argv 参数列表
 */
static void Sys_Reset(int argc, char *argv[]) {
    if (argc > 1 && strcmp(argv[1], "-y") == 0) {
        logPrintln("system will reset after 1 seconds");
        osDelay(1000);
        HAL_NVIC_SystemReset();
    }
    osEventFlagsSet(System_StatusHandle, APP_NEED_USART);
    logPrintln("WARNING: System will be reset, Would you like to proceed? (y/n)");

    uint8_t byte;
    while (1) {
        if (shell.read((char*)&byte, 1)) {
            if (byte == 'y') {
                logPrintln("system will reset after 1 seconds");
                osDelay(1000);
                HAL_NVIC_SystemReset();
            } else {
                osEventFlagsClear(System_StatusHandle, APP_NEED_USART);
                break;
            }
        }
        osDelay(20);
    }
}
SHELL_EXPORT_CMD(
SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN)|SHELL_CMD_DISABLE_RETURN,
Reset, Sys_Reset, Rest System);
