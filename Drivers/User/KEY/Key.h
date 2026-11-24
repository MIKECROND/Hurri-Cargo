/**
 * @file Key.h
 * @author T-Car Team
 * @brief START 按键驱动接口
 *
 * 模块职责：采样 START 按键电平、消抖并产生一次按下事件。
 * 采用非阻塞扫描方式，由调用方周期调用 Key_Up_Scan()，函数立即返回，
 * 不再像旧实现那样在函数内部阻塞等待按键松开。
 * 可调参数见 Key.h。
 */

#ifndef __KEY_H__
#define __KEY_H__

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * @brief 按键消抖时间(ms)：电平保持稳定达到该时长才被确认为有效
 */
#define KEY_DEBOUNCE_MS 20U

/**
 * @brief 初始化 START 按键
 * @note  记录初始电平。若初始化时按键已经按下，则不会产生按下事件，
 *        需要先松开再重新按下，避免上电时按键被卡住导致误触发。
 */
void Key_Init(void);

/**
 * @brief 扫描 START 按键（非阻塞）
 * @note  需周期调用（建议周期不大于 10ms），内部完成消抖与边沿检测；
 *        单次调用立即返回，不会阻塞调用任务。
 *        模块状态为静态变量, 非线程安全, 只应由同一个任务调用。
 * @return true 检测到一次消抖后的按下事件
 * @return false 无按下事件
 */
bool Key_Up_Scan(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __KEY_H__ */
