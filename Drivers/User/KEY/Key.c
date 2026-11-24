/**
 * @file Key.c
 * @author T-Car Team
 * @brief KEY_UP 按键驱动实现
 * 可调参数见 Key.h。
 */

#include "Key.h"
#include "main.h"
#include <stdint.h>

/* gpio.c 中 START 配置为下拉输入，按下时为高电平 */
#define KEY_LEVEL_PRESSED GPIO_PIN_SET

static bool s_initialized = false; /* 是否已完成初始化 */
static bool s_stable_pressed = false; /* 消抖后的稳定状态 */
static bool s_last_raw_pressed = false; /* 上一次原始采样状态 */
static uint32_t s_raw_change_tick = 0; /* 原始电平最近一次变化的时刻 */

/**
 * @brief 读取按键原始电平
 * @return true 按下
 * @return false 松开
 */
static bool Key_ReadRaw(void) {
    return HAL_GPIO_ReadPin(START_GPIO_Port, START_Pin) == KEY_LEVEL_PRESSED;
}

void Key_Init(void) {
    s_stable_pressed = Key_ReadRaw();
    s_last_raw_pressed = s_stable_pressed;
    s_raw_change_tick = HAL_GetTick();
    s_initialized = true;
}

bool Key_Up_Scan(void) {
    if (!s_initialized) {
        Key_Init();
    }

    uint32_t now = HAL_GetTick();
    bool raw_pressed = Key_ReadRaw();

    /* 原始电平发生变化：重新计时，本次不产生事件 */
    if (raw_pressed != s_last_raw_pressed) {
        s_last_raw_pressed = raw_pressed;
        s_raw_change_tick = now;
        return false;
    }

    /* 电平未变化或与稳定状态一致：无事件 */
    if (raw_pressed == s_stable_pressed) {
        return false;
    }

    /* 新电平保持时间不足消抖时长：继续等待 */
    if ((now - s_raw_change_tick) < KEY_DEBOUNCE_MS) {
        return false;
    }

    /* 电平已稳定，更新消抖状态；只在按下沿上报事件 */
    s_stable_pressed = raw_pressed;
    return raw_pressed;
}
