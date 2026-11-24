/**
 * @file mission_code.c
 * @author T-Car Team
 * @brief 任务码（条码）解析与存储实现
 * 可调参数见 mission_config.h / Applications/mission/README.md。
 */

#include "mission_code.h"
#include "log.h"
#include <stddef.h>

/* 从 xxx+xxx+xxx+xxx 条码解析出的任务码缓存 */
static uint16_t s_task_code[MISSION_TASK_ROW_NUM] = {0}; // 每段任务码整数值
static uint8_t s_task_digit[MISSION_TASK_ROW_NUM][MISSION_TASK_DIGIT_NUM] = {0}; // 每段拆出的独立数字

/**
 * @brief 判断条码是否符合 xxx+xxx+xxx+xxx 格式
 *        要求：恰好 MISSION_TASK_ROW_NUM-1 个 '+'、每段非空且只含数字 0-9
 * @param buf 条码字符串
 * @return true 符合格式 false 不符合
 */
static bool Barcode_IsQuad(const uint8_t *buf) {
    const char *p = (const char *)buf;
    int plus_cnt = 0;
    bool seg_has_digit = false;

    for (; *p != '\0'; p++) {
        if (*p == '+') {
            if (!seg_has_digit) return false; /* 空段 */
            plus_cnt++;
            seg_has_digit = false;
        } else if (*p >= '0' && *p <= '9') {
            seg_has_digit = true;
        } else {
            return false; /* 出现非数字字符 */
        }
    }

    /* 最后一段必须非空 */
    return seg_has_digit && (plus_cnt == MISSION_TASK_ROW_NUM - 1);
}

/**
 * @brief 打印当前任务码缓存（扫码解析与调试工具共用的统一输出格式）
 */
static void TaskCode_Log(void) {
    logPrintln("Mission TaskCode: 1. %d | 2. %d | 3. %d | 4. %d",
               s_task_code[0],
               s_task_code[1],
               s_task_code[2],
               s_task_code[3]);
}

/**
 * @brief 解析并写入任务码缓存（调用前必须已通过 Barcode_IsQuad 校验）
 * @param barcode 组合条码字符串（格式已校验）
 */
static void TaskCode_Store(const uint8_t *barcode) {
    const char *p = (const char *)barcode;

    for (uint8_t i = 0; i < MISSION_TASK_ROW_NUM; i++) {
        uint16_t value = 0;
        uint8_t digit_count = 0;

        /* 清空该段数字位，避免短段残留上次结果（MISSION_TASK_DIGIT_NUM=3，与原 3 行逐格等价） */
        for (uint8_t k = 0; k < MISSION_TASK_DIGIT_NUM; k++) {
            s_task_digit[i][k] = 0;
        }

        /* 数字段累加为整数，同时拆成独立数字 */
        while (*p >= '0' && *p <= '9') {
            if (digit_count < MISSION_TASK_DIGIT_NUM) {
                s_task_digit[i][digit_count] = (uint8_t)(*p - '0');
            }
            digit_count++;
            value = (uint16_t)(value * 10u + (uint16_t)(*p - '0'));
            p++;
        }
        s_task_code[i] = value;

        /* 前 MISSION_TASK_ROW_NUM-1 段后必定紧跟 '+'，最后一段后必定是字符串结尾（格式已校验） */
        if (i < MISSION_TASK_ROW_NUM - 1) {
            p++;
        }
    }

    TaskCode_Log();
}

bool Mission_TaskCodeParse(const uint8_t *barcode) {
    if (barcode == NULL || !Barcode_IsQuad(barcode)) {
        return false;
    }

    TaskCode_Store(barcode);
    return true;
}

uint8_t Mission_TaskCodeGetDigit(MissionTaskRow_t row, uint8_t index) {
    if ((uint32_t)row >= MISSION_TASK_ROW_NUM || index >= MISSION_TASK_DIGIT_NUM) {
        return 0;
    }
    return s_task_digit[row][index];
}

uint16_t Mission_TaskCodeGetValue(MissionTaskRow_t row) {
    if ((uint32_t)row >= MISSION_TASK_ROW_NUM) {
        return 0;
    }
    return s_task_code[row];
}
