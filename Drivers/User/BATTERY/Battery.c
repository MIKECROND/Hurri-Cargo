/**
 * @file Battery.c
 * @author T-Car Team
 * @brief 电池电压监控模块源文件
 *
 * 电压来源: 总线舵机回传的供电电压(Servo_Monitor 的 voltage 字段, 单位 mV), 不再使用 ADC。
 *   - 舵机总线读命令已由协议层事务互斥串行化, 本任务与机械臂动作可并发安全访问;
 *   - 采样做 BATTERY_SAMPLE_AVG 次平均, 再用 BATTERY_SERVO_SCALE/OFFSET_MV 校准到电池端电压;
 *   - 读取失败/电压过低时保留上次有效值, 并按 BATTERY_WARN_INTERVAL_MS 限速告警。
 * 可调参数见 Battery.h。
 */

#include "battery.h"
#include "servo_protocol.h"
#include "shell.h"
#include "log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "Events.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#if !BATTERY_ENABLED

/* 任务符号必须存在，freertos.c 会无条件引用 */
void Battery_Get_Task(void *argument) {
    (void)argument;
    osThreadExit(); /* 立即退出，不占用栈运行 */
}

bool Battery_Init(void) {
    return false;
}

bool Battery_GetVoltage(uint16_t *voltage) {
    (void)voltage;
    return false;
}

/* ================= 启用分支 ================= */
#else /* BATTERY_ENABLED */

/**
 * @brief 电池电压监控数据结构体
 */
typedef struct {
    uint16_t voltage; // 最近一次有效电压 mV(已按 scale/offset 换算)
    uint16_t interval_ms; // 采样周期 ms
    bool valid; // 是否已采到有效电压
    uint32_t last_warn_tick; // 上次告警时间戳
} BatteryData_t;

static BatteryData_t *g_battery = NULL;
static bool is_init = false;

/**
 * @brief 从总线舵机读取一次供电电压并换算为电池电压
 * @param voltage 输出: 电池电压(mV)
 * @return true=读取成功; false=舵机无应答/读回值异常
 */
static bool Battery_ReadServoVoltage(uint16_t *voltage) {
    ServoData data[1];

    if (Servo_Monitor(BATTERY_SERVO_ID, data) != SERVO_STATUS_SUCCESS) {
        return false;
    }

    float mv = (float)data[0].voltage * BATTERY_SERVO_SCALE + (float)BATTERY_SERVO_OFFSET_MV;
    if (mv < 0.0f) {
        mv = 0.0f;
    }
    *voltage = (uint16_t)mv;
    return true;
}

/**
 * @brief 多次采样取平均(抑制舵机电压读数抖动)
 * @param voltage 输出: 平均电压(mV)
 * @return true=至少一次读取成功; false=全部读取失败
 */
static bool Battery_SampleAverage(uint16_t *voltage) {
    uint32_t sum = 0;
    uint8_t ok_count = 0;

    for (uint8_t i = 0; i < BATTERY_SAMPLE_AVG; i++) {
        uint16_t mv = 0;
        if (Battery_ReadServoVoltage(&mv)) {
            sum += mv;
            ok_count++;
        }
    }
    if (ok_count == 0) {
        return false;
    }

    *voltage = (uint16_t)(sum / ok_count);
    return true;
}

/**
 * @brief 初始化电池电压监控模块(电压来自舵机回传, 不再初始化 ADC)
 * @return 初始化结果
 * @note 模块本身总是可用; 是否已采到有效电压由 valid 标记(读取失败会在任务里重试并限速告警)。
 */
bool Battery_Init(void) {
    if (is_init) {
        return true;
    }

    g_battery = pvPortMalloc(sizeof(BatteryData_t));
    if (g_battery == NULL) {
        return false;
    }
    memset(g_battery, 0, sizeof(BatteryData_t));
    g_battery->interval_ms = BATTERY_SAMPLE_INTERVAL_MS;

    is_init = true;
    logInfo("battery: voltage source = servo ID %d (monitor voltage)", BATTERY_SERVO_ID);
    return true;
}

/**
 * @brief 电池电压更新任务(周期读取舵机回传电压)
 */
void Battery_Get_Task(void *argument) {
    (void)argument;

    osEventFlagsWait(System_StatusHandle, SYS_INIT_COMPLETE, osFlagsNoClear, osWaitForever);
    if (!is_init) {
        osThreadExit();
    }

    for (;;) {
        uint16_t voltage = 0;
        uint32_t now = osKernelGetTickCount();

        if (Battery_SampleAverage(&voltage) && voltage >= BATTERY_MIN_VALID_MV) {
            g_battery->voltage = voltage;
            g_battery->valid = true;

            if (g_battery->voltage <= BATTERY_THRESHOLD &&
                (now - g_battery->last_warn_tick) >= BATTERY_WARN_INTERVAL_MS) {
                g_battery->last_warn_tick = now;
                logWarning("Low battery voltage: %d mV (servo ID %d)", g_battery->voltage, BATTERY_SERVO_ID);
            }
        } else if ((now - g_battery->last_warn_tick) >= BATTERY_WARN_INTERVAL_MS) {
            /* 失败告警限速: 舵机未接/未上电时不要刷屏, 保留上次有效读数 */
            g_battery->last_warn_tick = now;
            logWarning("Battery voltage read failed/invalid (servo ID %d)", BATTERY_SERVO_ID);
        }

        osDelay(g_battery->interval_ms);
    }
}

/**
 * @brief 获取电池电压
 * @param voltage 电池电压指针
 * @return 是否成功获取(尚未采到有效电压时返回 false)
 */
bool Battery_GetVoltage(uint16_t *voltage) {
    if (!is_init || g_battery == NULL || voltage == NULL || !g_battery->valid) {
        return false;
    }
    *voltage = g_battery->voltage;
    return true;
}

/**
 * @brief 实时刷新显示电池电压
 */
static void Battery_ViewRealtime(void) {
    Shell *shell;
    uint8_t byte;
    shell = shellGetCurrent();
    if (shell == NULL) {
        return;
    }

    osEventFlagsSet(System_StatusHandle, APP_NEED_USART);
    logPrintln("Battery Voltage Viewer - Press ^C to exit\r\n"
               "Voltage: ----- mV");

    while (1) {
        if (g_battery->valid) {
            logPrintln("\033[1A\033[2K\rVoltage: %d mV (servo ID %d)", g_battery->voltage, BATTERY_SERVO_ID);
        } else {
            logPrintln("\033[1A\033[2K\rVoltage: unavailable (servo ID %d)", BATTERY_SERVO_ID);
        }
        if (shell->read((char *)&byte, 1)) {
            if (byte == 0x03) {
                break;
            }
        }
        osDelay(33);
    }

    logPrintln("\033[2A\033[J\033[2A");
    osEventFlagsClear(System_StatusHandle, APP_NEED_USART);
}

/**
 * @brief 电池工具
 */
static void Battery_Tool(int argc, char *argv[]) {
    if (!is_init || g_battery == NULL) {
        logWarning("Battery module not initialized");
        return;
    }

    if (argc == 1) {
        if (g_battery->valid) {
            logPrintln("Battery Voltage: %d mV (from servo ID %d monitor)", g_battery->voltage, BATTERY_SERVO_ID);
        } else {
            logPrintln("Battery Voltage: unavailable (servo ID %d not responding)", BATTERY_SERVO_ID);
        }
    } else if (argc <= 3) {
        if (strcmp(argv[1], "time") == 0) {
            if (argc == 2) {
                logPrintln("Current interval: %d ms", g_battery->interval_ms);
            } else if (argc == 3) {
                char *endptr;
                long val = strtol(argv[2], &endptr, 10);
                if (*endptr != '\0') {
                    logPrintln("Invalid time: %s", argv[2]);
                } else {
                    g_battery->interval_ms = (uint16_t)val;
                    logPrintln("time set to %d ms", g_battery->interval_ms);
                }
            } else {
                logPrintln("Usage: battery time [ms]");
            }
        } else if (strcmp(argv[1], "view") == 0) {
            Battery_ViewRealtime();
        } else {
            logPrintln("Usage: battery time|view");
        }
    } else {
        logPrintln("Usage: battery time|view");
    }
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_DISABLE_RETURN,
                 battery,
                 Battery_Tool,
                 Battery Tool);

#endif
