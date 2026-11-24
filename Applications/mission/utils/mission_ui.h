/**
 * @file mission_ui.h
 * @author T-Car Team
 * @brief 总任务 LCD 显示接口
 *
 * 只负责"任务数据 -> LCD 绘制"，不负责扫描、解析、导航；
 * 与任务流程解耦，便于单独替换显示方案。
 * 可调参数见 mission_config.h / Applications/mission/README.md。
 */

#ifndef __MISSION_UI_H__
#define __MISSION_UI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 在 LCD 上显示扫码结果
 * @param barcode 条码字符串（函数内部会临时改写并恢复该缓冲区）
 * @param valid   true：已通过任务码格式校验，分两行显示；
 *                false：格式非法，用 Font_note 自动换行显示全部内容
 */
void Mission_UiShowBarcode(uint8_t *barcode, bool valid);

/**
 * @brief 在 LCD 上显示任务完成计数
 * @param pick_count  正确抓取数
 * @param place_count 正确放置数
 */
void Mission_UiShowComplete(int pick_count, int place_count);

#ifdef __cplusplus
}
#endif

#endif /* __MISSION_UI_H__ */
