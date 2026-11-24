/**
 * @file rtos_utils.c
 * @author Pcb-yun (pcbyinyun@163.com)
 * @brief RTOS实用命令集源文件
 * 可调参数见 Drivers/User/RTOS/rtos_utils.h。
 */

#include "rtos_utils.h"
#include "shell.h"
#include "log.h"
#include "cmsis_os2.h"
#include "stm32f4xx_hal.h"
#include "Events.h"
#include <string.h>

/**
 * @brief 显示内存分配信息
 */
static void Memory_Info(void) {
    HeapStats_t xHeapStats;
    vPortGetHeapStats(&xHeapStats);

    logPrintln(
        "Free heap: %u bytes\r\n" /* 当前可用的堆总大小——这是所有空闲块的总和，而不是可以分配的最大块 */
        "Min ever free: %u bytes\r\n" /* 自系统启动以来堆中可用内存的最小总量（所有可用块的总和） */
        "Largest free block: %u bytes\r\n" /* 调用vPortGetHeapStats()时堆中所有空闲块的最大大小（以字节为单位） */
        "Smallest free block: %u bytes\r\n" /* 调用vPortGetHeapStats()时堆中所有空闲块的最小大小（以字节为单位） */
        "Number of free blocks: %u\r\n" /* 调用vPortGetHeapStats()时堆中所有空闲块的数量 */
        "Successful allocations: %u\r\n" /* 成功调用pvPortMalloc()的次数 */
        "Successful frees: %u", /* 成功调用vPortFree()的次数 */
        xHeapStats.xAvailableHeapSpaceInBytes,
        xHeapStats.xMinimumEverFreeBytesRemaining,
        xHeapStats.xSizeOfLargestFreeBlockInBytes,
        xHeapStats.xSizeOfSmallestFreeBlockInBytes,
        xHeapStats.xNumberOfFreeBlocks,
        xHeapStats.xNumberOfSuccessfulAllocations,
        xHeapStats.xNumberOfSuccessfulFrees);
}

/**
 * @brief 显示任务状态信息
 */
static void Task_Info(void) {
    osThreadId_t task_ids[32];
    uint32_t task_count = osThreadEnumerate(task_ids, 32);
    if (!task_count) {  /* 0 任务防御：避免下方 task_count-1 在 uint8_t 下溢为 255 */
        logPrintln("no task");
        return;
    }

    // 获取总运行时间
    uint32_t ulTotalRunTime;

    // 获取任务数量
    uint32_t ulArraySize = uxTaskGetNumberOfTasks();
    TaskStatus_t *pxTaskStatusArray = pvPortMalloc(ulArraySize * sizeof(TaskStatus_t));

    if (pxTaskStatusArray != NULL) {
        ulArraySize = uxTaskGetSystemState(pxTaskStatusArray, ulArraySize, &ulTotalRunTime);

        // 定义任务信息结构体用于缓存
        typedef struct {
            osThreadId_t task_id;
            const char *task_name;
            osThreadState_t state;
            osPriority_t priority;
            uint32_t stack_space;
            uint32_t high_water;
            float cpu_usage;
        } TaskInfo_t;

        TaskInfo_t *task_info_array = pvPortMalloc(task_count * sizeof(TaskInfo_t));
        if (task_info_array != NULL) {
            // 缓存所有任务信息
            for (uint32_t i = 0; i < task_count; i++) {
                task_info_array[i].task_id = task_ids[i];
                task_info_array[i].task_name = osThreadGetName(task_ids[i]);
                task_info_array[i].state = osThreadGetState(task_ids[i]);
                task_info_array[i].priority = osThreadGetPriority(task_ids[i]);
                task_info_array[i].stack_space = osThreadGetStackSpace(task_ids[i]);

                // 获取任务高水位线和运行时间
                TaskStatus_t xTaskStatus;
                TaskHandle_t xTask = (TaskHandle_t)task_ids[i];
                vTaskGetInfo(xTask, &xTaskStatus, pdTRUE, eInvalid);
                task_info_array[i].high_water = xTaskStatus.usStackHighWaterMark;

                // 计算CPU使用率
                if (ulTotalRunTime > 0) {
                    task_info_array[i].cpu_usage = (float)xTaskStatus.ulRunTimeCounter / (float)ulTotalRunTime * 100.0f;
                } else {
                    task_info_array[i].cpu_usage = 0.0f;
                }
            }

            // 按照CPU占用率从高到低排序（冒泡排序）
            for (uint32_t i = 0; i < task_count - 1; i++) {
                for (uint32_t j = 0; j < task_count - 1 - i; j++) {
                    if (task_info_array[j].cpu_usage < task_info_array[j + 1].cpu_usage) {
                        TaskInfo_t temp = task_info_array[j];
                        task_info_array[j] = task_info_array[j + 1];
                        task_info_array[j + 1] = temp;
                    }
                }
            }

            // 打印排序后的任务信息
            logPrintln("ID        Name              State       Priority    Stack Space    High Water    CPU\r\n"
                       "----------------------------------------------------------------------------------------");

            for (uint32_t i = 0; i < task_count; i++) {
                const char *state_str;
                switch (task_info_array[i].state) {
                case osThreadInactive:
                    state_str = "Inactive";
                    break;
                case osThreadReady:
                    state_str = "Ready";
                    break;
                case osThreadRunning:
                    state_str = "Running";
                    break;
                case osThreadBlocked:
                    state_str = "Blocked";
                    break;
                case osThreadTerminated:
                    state_str = "Terminated";
                    break;
                default:
                    state_str = "Unknown";
                    break;
                }
                logPrintln("%p  %-16s  %-10s  %-10lu  %-10lu     %-12lu  %6.2f %%",
                           task_info_array[i].task_id,
                           task_info_array[i].task_name ? task_info_array[i].task_name : "<unknown>",
                           state_str,
                           task_info_array[i].priority,
                           task_info_array[i].stack_space,
                           task_info_array[i].high_water,
                           task_info_array[i].cpu_usage);
            }
            vPortFree(task_info_array);
        } else {
            logPrintln("Failed to allocate memory for task info array");
        }
        vPortFree(pxTaskStatusArray);
    } else {
        logPrintln("Failed to allocate memory for task status array");
    }
}

/**
 * @brief 显示系统时间信息
 */
static void Time_Info(void) {
    uint32_t tick_count = osKernelGetTickCount();
    uint32_t tick_freq = osKernelGetTickFreq();

    uint8_t hours = tick_count / (tick_freq * 3600);
    uint8_t minutes = (tick_count % (tick_freq * 3600)) / (tick_freq * 60);
    uint8_t seconds = (tick_count % (tick_freq * 60)) / tick_freq;
    uint16_t milliseconds = (tick_count % tick_freq) * 1000 / tick_freq;

    logPrintln("Tick Frequency: %lu Hz\r\n"
               "System Tick: %lu\r\n"
               "Uptime: %02lu:%02lu:%02lu.%03lu",
               tick_freq,
               tick_count,
               hours,
               minutes,
               seconds,
               milliseconds);

    uint32_t sysclk = HAL_RCC_GetSysClockFreq();
    uint32_t hclk = HAL_RCC_GetHCLKFreq();
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();

    logPrintln("System Clock: %lu Hz\r\n"
               "HCLK: %lu Hz\r\n"
               "PCLK1: %lu Hz\r\n"
               "PCLK2: %lu Hz",
               sysclk,
               hclk,
               pclk1,
               pclk2);
}

/**
 * @brief 持续显示任务信息
 */
static void OS_Top(void) {
    Shell *shell;
    uint8_t byte;
    shell = shellGetCurrent();
    if (shell == NULL) return;

    osEventFlagsSet(System_StatusHandle, APP_NEED_USART);
    uint8_t line = uxTaskGetNumberOfTasks() + 2;
    Task_Info();

    while (1) {
        logPrintln("\033[%dA", line + 1);
        Task_Info();
        if (shell->read((char *)&byte, 1)) {
            if (byte == 0x03) break;
        }
        osDelay(100);
    }
    logPrintln("\033[%dA\033[J\033[2A", line);
    osEventFlagsClear(System_StatusHandle, APP_NEED_USART);
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_DISABLE_RETURN,
                 top,
                 OS_Top,
                 View task information);

/**
 * @brief 手动触发错误
 */
static void OS_Error(void) {
    logPrintln("Manual error trigger");
    Error_Handler();
}

/**
 * @brief OS命令处理函数
 * @param argc 参数数量
 * @param argv 参数列表
 */
static void OS_Tool_Shell(int argc, char *argv[]) {
    if (argc < 2) {
        logPrintln(OS_HELP);
        return;
    }

    if (strcmp(argv[1], "mem") == 0) {
        Memory_Info();
    } else if (strcmp(argv[1], "task") == 0) {
        Task_Info();
    } else if (strcmp(argv[1], "time") == 0) {
        Time_Info();
    } else if (strcmp(argv[1], "error") == 0) {
        OS_Error();
    } else {
        logPrintln("Invalid command: %s\r\n" OS_HELP, argv[1]);
    }
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_DISABLE_RETURN,
                 os,
                 OS_Tool_Shell,
                 System information query tool);
