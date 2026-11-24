/**
 * @file Events.h
 * @brief 系统事件定义
 * @author Pcb-yun (pcbyinyun@163.com)
 * 可调参数见 Drivers/User/RTOS/rtos_utils.h。
 */

#ifndef __EVENTS_H__
#define __EVENTS_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "cmsis_os2.h"

/**
 * @note 每个事件标志对象最多可以设置24个成员
 */

/*                  系统状态                    */
extern osEventFlagsId_t System_StatusHandle;
#define SYS_INIT_COMPLETE (1UL << 0) // 系统初始化完成
#define APP_NEED_USART (1UL << 1) // 需要使用串口
#define UART1_TX_IDLE (1UL << 2) // UART1发送空闲
#define UART2_RX_CPLT (1UL << 3) // UART2接收完成
#define UART6_TX_IDLE (1UL << 4) // UART6发送空闲
#define UART3_TX_IDLE (1UL << 5) // UART3发送空闲
#define UART3_RX_IDLE (1UL << 6) // UART3接收空闲
#define ADC1_CONVCPLT (1UL << 7) // ADC1转换完成
#define UART2_TX_IDLE (1UL << 12) // UART2发送完成(DMA/TC 中断)
#define UART2_RX_ACT (1UL << 13) // UART2收到任意字节(链路活动/静默判定)

/*             任务/导航/扫描状态               */
#define NAV_EVT_DONE (1UL << 8) // 导航到位
#define NAV_EVT_ERROR (1UL << 9) // 导航超时/失败
#define MISSION_ABORT (1UL << 10) // 总任务被主动停止
#define SCAN_EVT_BARCODE (1UL << 11) // 扫描到新条码

/*            规划器状态               */
extern osEventFlagsId_t Planner_EventHandle;
#define PLANNER_EVT_DONE (1U << 0)
#define PLANNER_EVT_ERROR (1U << 1)

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // __EVENTS_H__
