/**
 * @file DIP_Switch.c
 * @author T-Car Team
 * @brief DIP 开关驱动实现
 * 可调参数见 DIP_Switch.h / 对应 config。
 */

#include "DIP_Switch.h"
#include "cmsis_os2.h"
#include "log.h"
#include "main.h"
#include <stdint.h>

#if DIP_SWITCH_DEBUG_MODE
#include "shell.h"
#include "planner_port.h"
#include "nav_map.h"
#include <stdlib.h>
#include <string.h>
#endif

/* 启动区域缓存：仅在初始化时写入，之后只读 */
static Start_Zone_t s_start_zone = START_ZONE_UNKNOWN;

#if DIP_SWITCH_DEBUG_MODE
/* DEBUG 模式下 shell 强制指定的启动区域，START_ZONE_UNKNOWN 表示未指定，仍按开关电平判定 */
static Start_Zone_t s_debug_zone = START_ZONE_UNKNOWN;
#endif

/**
 * @brief 读取 DIP 开关电平并转换为启动区域
 * @return START_ZONE_1 开关拨到 1 侧（低电平）
 * @return START_ZONE_2 开关拨到 2 侧（高电平）
 */
static Start_Zone_t DIP_Switch_ReadZone(void) {
    return (HAL_GPIO_ReadPin(DIP_SW_GPIO_Port, DIP_SW_Pin) == GPIO_PIN_RESET) ? START_ZONE_1 : START_ZONE_2;
}

bool DIP_Switch_Init(void) {
#if DIP_SWITCH_DEBUG_MODE
    /* DEBUG 模式下已被 shell 强制指定时，直接采用该区域，跳过消抖采样 */
    if (s_debug_zone != START_ZONE_UNKNOWN) {
        s_start_zone = s_debug_zone;
        logPrintln("DIP switch debug mode: start zone forced to %d", (int)s_start_zone);
        return true;
    }
#endif

    Start_Zone_t zone = DIP_Switch_ReadZone();

    /* 连续采样一致才认为开关状态稳定，避免上电瞬间的机械抖动被误判 */
    for (uint32_t retry = 0; retry < DIP_SWITCH_RETRY_MAX; retry++) {
        osDelay(DIP_SWITCH_DEBOUNCE_MS);

        Start_Zone_t new_zone = DIP_Switch_ReadZone();
        if (new_zone == zone) {
            s_start_zone = zone;
            return true;
        }

        logWarning("DIP switch jitter detected");
        zone = new_zone;
    }

    s_start_zone = START_ZONE_UNKNOWN;
    logError("DIP switch state unstable, start zone unknown");
    return false;
}

Start_Zone_t DIP_Switch_GetStartZone(void) {
#if DIP_SWITCH_DEBUG_MODE
    /* DEBUG 模式下 shell 指定的区域优先，便于运行中切换启动区验证 */
    if (s_debug_zone != START_ZONE_UNKNOWN)
        return s_debug_zone;
#endif
    return s_start_zone;
}

#if DIP_SWITCH_DEBUG_MODE
/**
 * @brief 启动区域名称
 */
static const char *DIP_Switch_ZoneName(Start_Zone_t zone) {
    switch (zone) {
    case START_ZONE_1:
        return "1";
    case START_ZONE_2:
        return "2";
    default:
        return "unknown";
    }
}

/**
 * @brief DIP 开关调试命令
 * @param argc 命令参数数量
 * @param argv 命令参数数组
 */
static void DIP_Switch_Tool(int argc, char *argv[]) {
    if (argc < 2 || strcmp(argv[1], "get") == 0) {
        logPrintln("Start zone: %s (forced: %s)", DIP_Switch_ZoneName(DIP_Switch_GetStartZone()),
                   DIP_Switch_ZoneName(s_debug_zone));
        return;
    }

    if (strcmp(argv[1], "1") == 0) {
        s_debug_zone = START_ZONE_1;
        Map_Init(32, MAP_ZONE_1);
#if PLANNER_ENABLED
        Planner_Init(MAP_ZONE_1);
#endif
    } else if (strcmp(argv[1], "2") == 0) {
        s_debug_zone = START_ZONE_2;
        Map_Init(32, MAP_ZONE_2);
#if PLANNER_ENABLED
        Planner_Init(MAP_ZONE_2);
#endif
    } else if (strcmp(argv[1], "auto") == 0) {
        s_debug_zone = START_ZONE_UNKNOWN;
    } else {
        logPrintln("Usage: dip [get|1|2|auto]");
        return;
    }

    logPrintln("Start zone: %s (forced: %s)", DIP_Switch_ZoneName(DIP_Switch_GetStartZone()),
               DIP_Switch_ZoneName(s_debug_zone));
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_DISABLE_RETURN,
                 dip,
                 DIP_Switch_Tool,
                 DIP switch debug: dip [get|1|2|auto]);
#endif /* DIP_SWITCH_DEBUG_MODE */
