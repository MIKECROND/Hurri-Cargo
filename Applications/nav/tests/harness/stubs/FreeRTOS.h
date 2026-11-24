/**
 * @file FreeRTOS.h
 * @brief 主机端仿真用 FreeRTOS 最小桩（只有动态内存接口被 nav 控制代码使用）
 */

#ifndef __FREERTOS_H__
#define __FREERTOS_H__

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef configASSERT
#define configASSERT(x) ((void)0)
#endif

void *pvPortMalloc(size_t size);
void vPortFree(void *ptr);

#ifdef __cplusplus
}
#endif

#endif /* __FREERTOS_H__ */
