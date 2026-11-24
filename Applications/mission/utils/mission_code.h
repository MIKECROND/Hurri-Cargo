/**
 * @file mission_code.h
 * @author T-Car Team
 * @brief 任务码（条码）解析与存储
 *
 * 纯逻辑模块：只负责 xxx+xxx+xxx+xxx 条码的校验、解析与缓存，
 * 不依赖 LCD / 导航 / 机械臂等业务模块，便于单独测试与复用。
 * 可调参数见 mission_config.h / Applications/mission/README.md。
 */

#ifndef __MISSION_CODE_H__
#define __MISSION_CODE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 任务码行号：每轮任务对应的"抓取颜色序列"和"放置颜色序列"
 */
typedef enum {
    MISSION_ROUND1_GRAB = 0, // 第一轮：原料区抓取颜色序列
    MISSION_ROUND1_STORAGE, // 第一轮：暂存区/粗加工区放置颜色序列
    MISSION_ROUND2_GRAB, // 第二轮：原料区抓取颜色序列
    MISSION_ROUND2_STORAGE, // 第二轮：暂存区/粗加工区放置颜色序列
} MissionTaskRow_t;

#define MISSION_TASK_ROW_NUM 4 // 任务码段数
#define MISSION_TASK_DIGIT_NUM 3 // 每段拆出的独立数字个数

/**
 * @brief 校验、解析并缓存 xxx+xxx+xxx+xxx 格式的任务码
 * @note  只有格式合法时才更新缓存，非法条码不会破坏上一次的有效任务码
 * @param barcode 扫描到的条码字符串
 * @return true 格式合法并已保存
 * @return false 格式非法
 */
bool Mission_TaskCodeParse(const uint8_t *barcode);

/**
 * @brief 读取任务码中某一段的第 index 个数字
 * @param row   任务码行号
 * @param index 段内数字下标（0 ~ MISSION_TASK_DIGIT_NUM-1）
 * @return uint8_t 数字 0~9；参数越界时返回 0
 */
uint8_t Mission_TaskCodeGetDigit(MissionTaskRow_t row, uint8_t index);

/**
 * @brief 读取任务码中某一段的整数值（调试/显示用）
 * @param row 任务码行号
 * @return uint16_t 该段整数值（按 3 位十进制解析，0~999）；行号越界时返回 0
 */
uint16_t Mission_TaskCodeGetValue(MissionTaskRow_t row);

#ifdef __cplusplus
}
#endif

#endif /* __MISSION_CODE_H__ */
