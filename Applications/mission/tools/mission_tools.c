/**
 * @file mission_tools.c
 * @author T-Car Team
 * @brief 总任务调试工具实现
 *
 * shell 命令 `mission`:
 *   - run / stop: 启动 / 停止总任务
 *   - code:      查看当前任务码
 *   - code <s1> <s2> <s3> <s4>: 一次设置 4 段任务码(每段 0~999)
 *   - code <row> <value>:       只修改第 row 段(1~4)
 *   - code <a+b+c+d>:           按条码字符串格式设置
 * 任务码行号: 1/2 = 第一轮抓取/放置, 3/4 = 第二轮抓取/放置。
 * 可调参数见 mission_config.h / Applications/mission/README.md。
 */

#include "shell.h"
#include "shell_cmd_group.h"
#include "log.h"
#include "cmsis_os.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include <stdint.h>
#include "Events.h"
#include "mission.h"
#include "mission_code.h"
#include "mission_config.h"
#include "mission_timing.h"

#if MISSION_SHELL == 1

/**
 * @brief 打印 `mission` 命令用法
 */
static void mission_print_usage(void) {
    logPrintln("Usage: mission COMMAND");
    logPrintln("commands:");
    logPrintln("  run                        Start mission");
    logPrintln("  stop                       Stop mission");
    logPrintln("  time                       Print mission run time + phase timing breakdown");
    logPrintln("  code                       Print current task code");
    logPrintln("  code <s1> <s2> <s3> <s4>   Set all 4 task-code segments (0~999 each)");
    logPrintln("  code <row> <value>         Set one segment (row 1~%u, value 0~999)", (unsigned)MISSION_TASK_ROW_NUM);
    logPrintln("  code <a+b+c+d>             Set by barcode-style string, e.g. 123+456+789+012");
    logPrintln("note: rows 1/2 = round1 grab/place, 3/4 = round2 grab/place");
}

/**
 * @brief 打印当前任务码(格式与扫码解析日志一致)
 */
static void mission_print_code(void) {
    logPrintln("TaskCode: 1.%03d 2.%03d 3.%03d 4.%03d",
               (int)Mission_TaskCodeGetValue(MISSION_ROUND1_GRAB),
               (int)Mission_TaskCodeGetValue(MISSION_ROUND1_STORAGE),
               (int)Mission_TaskCodeGetValue(MISSION_ROUND2_GRAB),
               (int)Mission_TaskCodeGetValue(MISSION_ROUND2_STORAGE));
    logPrintln("(1/2 = round1 grab/place, 3/4 = round2 grab/place)");
}

/**
 * @brief 解析一个 0~999 的十进制参数
 * @param text  参数字符串
 * @param value 输出: 解析结果
 * @return true=格式合法且在 0~999 内; false=非法
 */
static bool mission_parse_code_value(const char *text, uint16_t *value) {
    if (text == NULL || value == NULL || *text == '\0') return false;

    char *end = NULL;
    unsigned long v = strtoul(text, &end, 10);
    if (end == text || *end != '\0' || v > 999ul) return false;

    *value = (uint16_t)v;
    return true;
}

/**
 * @brief 把 4 段数值拼成 xxx+xxx+xxx+xxx 交给任务码解析(复用格式校验与统一日志)
 * @param values 4 段任务码(每段 0~999)
 * @return true=设置成功; false=拼接失败或格式非法
 */
static bool mission_apply_code(const uint16_t values[MISSION_TASK_ROW_NUM]) {
    /* 最长 "999+999+999+999" + 结尾 '\0' */
    char barcode[MISSION_TASK_ROW_NUM * MISSION_TASK_DIGIT_NUM + MISSION_TASK_ROW_NUM];
    int len = snprintf(barcode,
                       sizeof(barcode),
                       "%u+%u+%u+%u",
                       (unsigned)values[0],
                       (unsigned)values[1],
                       (unsigned)values[2],
                       (unsigned)values[3]);
    if (len <= 0 || (size_t)len >= sizeof(barcode)) return false;

    return Mission_TaskCodeParse((const uint8_t *)barcode);
}

/**
 * @brief `mission code ...`: 查看/修改任务码(调试用, 不经过扫码枪)
 * @param argc 参数数量(argv[0] = "code")
 * @param argv 参数数组
 */
static void mission_shell_code(int argc, char *argv[]) {
    if (argc == 1) {
        mission_print_code();
        return;
    }

    /* 单个参数且含 '+': 按条码字符串设置, 如 123+456+789+012 */
    if (argc == 2 && strchr(argv[1], '+') != NULL) {
        if (!Mission_TaskCodeParse((const uint8_t *)argv[1])) {
            logPrintln("Invalid task code: %s (expect xxx+xxx+xxx+xxx)", argv[1]);
            return;
        }
        mission_print_code();
        return;
    }

    /* 4 个数值: 一次设置 4 段 */
    if (argc == 5) {
        uint16_t values[MISSION_TASK_ROW_NUM];
        for (uint8_t i = 0; i < MISSION_TASK_ROW_NUM; i++) {
            if (!mission_parse_code_value(argv[i + 1], &values[i])) {
                logPrintln("Invalid segment %u: %s (expect 0~999)", (unsigned)(i + 1), argv[i + 1]);
                return;
            }
        }
        if (!mission_apply_code(values)) {
            logPrintln("Failed to update task code");
            return;
        }
        mission_print_code();
        return;
    }

    /* 2 个数值: 只修改第 row 段(row 1~4) */
    if (argc == 3) {
        uint16_t row_index = 0;
        uint16_t value = 0;
        if (!mission_parse_code_value(argv[1], &row_index) || row_index < 1 || row_index > MISSION_TASK_ROW_NUM ||
            !mission_parse_code_value(argv[2], &value)) {
            logPrintln(
                "Invalid row/value: %s %s (row 1~%u, value 0~999)", argv[1], argv[2], (unsigned)MISSION_TASK_ROW_NUM);
            return;
        }

        uint16_t values[MISSION_TASK_ROW_NUM];
        for (uint8_t i = 0; i < MISSION_TASK_ROW_NUM; i++) {
            values[i] = Mission_TaskCodeGetValue((MissionTaskRow_t)i);
        }
        values[row_index - 1] = value;

        if (!mission_apply_code(values)) {
            logPrintln("Failed to update task code");
            return;
        }
        mission_print_code();
        return;
    }

    mission_print_usage();
}

/**
 * @brief `mission`: 总任务调试命令入口
 */
static void mission_shell(int argc, char *argv[]) {
    if (argc < 2) {
        mission_print_usage();
        return;
    }

    if (strcmp(argv[1], "run") == 0) {
        logPrintln("Mission Start");
        mission_set_running(true);
    } else if (strcmp(argv[1], "stop") == 0) {
        logPrintln("Mission Stop");
        mission_set_running(false);
    } else if (strcmp(argv[1], "time") == 0) {
        /* 任务运行记时: 运行中打印实时时长, 结束后打印最近一次时长;
         * 随后打印分阶段耗时汇总表(运行中查询会把当前正在进行的阶段也算进去) */
        uint32_t elapsed_ms = 0;
        bool running = mission_get_run_time(&elapsed_ms);
        uint32_t rest = elapsed_ms;
        uint32_t hours = rest / 3600000u;
        rest %= 3600000u;
        uint32_t minutes = rest / 60000u;
        rest %= 60000u;
        uint32_t seconds = rest / 1000u;
        uint32_t millis = rest % 1000u;
        logPrintln("Mission run time: %u:%02u:%02u.%03u (%u ms), %s",
                   (unsigned)hours,
                   (unsigned)minutes,
                   (unsigned)seconds,
                   (unsigned)millis,
                   (unsigned)elapsed_ms,
                   running ? "running" : "stopped");
        MissionTiming_PrintSummary(elapsed_ms);
    } else if (strcmp(argv[1], "code") == 0) {
        mission_shell_code(argc - 1, &argv[1]);
    } else {
        logPrintln("invalid command: %s", argv[1]);
        mission_print_usage();
    }
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_DISABLE_RETURN,
                 mission,
                 mission_shell,
                 Start car mission);

#endif
