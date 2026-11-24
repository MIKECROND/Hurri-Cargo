/**
 * @file task.h
 * @brief 主机端仿真用 FreeRTOS task.h 最小桩
 *
 * nav 控制代码只用到临界区宏（保护运行时整定参数/调试快照）与头文件本身；
 * 主机端仿真为单线程顺序执行，临界区等效为空操作。
 */

#ifndef __TASK_H__
#define __TASK_H__

#include "FreeRTOS.h"

#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define taskENTER_CRITICAL_FROM_ISR() 0
#define taskEXIT_CRITICAL_FROM_ISR(x) ((void)(x))

#endif /* __TASK_H__ */
