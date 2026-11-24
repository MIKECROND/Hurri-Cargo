/**
 * @file cmsis_os2.h
 * @brief 主机端仿真用 CMSIS-RTOS2 最小桩（仅覆盖 nav 控制代码实际用到的 API）
 *
 * 说明：本文件位于 Applications/nav/tests/harness/stubs/，构建时用 -Istubs 放在真实
 * 头文件搜索路径之前，从而遮蔽固件里的 CMSIS-RTOS2 真实现；相应实现在 sim_platform.c。
 * 不要把这些桩加入生产代码。
 */

#ifndef __CMSIS_OS2_H__
#define __CMSIS_OS2_H__

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int32_t osStatus_t;
typedef void *osEventFlagsId_t;
typedef void *osMutexId_t;
typedef void *osThreadId_t;
typedef void *osMessageQueueId_t;

#define osOK ((osStatus_t)0)
#define osError ((osStatus_t)-1)

#define osWaitForever 0xFFFFFFFFU
#define osFlagsNoClear 0x00000002U

uint32_t osKernelGetTickCount(void);
osStatus_t osDelay(uint32_t ticks);

uint32_t osEventFlagsWait(osEventFlagsId_t ef_id, uint32_t flags, uint32_t options, uint32_t timeout);
uint32_t osEventFlagsSet(osEventFlagsId_t ef_id, uint32_t flags);

void osThreadExit(void);

osStatus_t osMutexAcquire(osMutexId_t mutex_id, uint32_t timeout);
osStatus_t osMutexRelease(osMutexId_t mutex_id);

#ifdef __cplusplus
}
#endif

#endif /* __CMSIS_OS2_H__ */
