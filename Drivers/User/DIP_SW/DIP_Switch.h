/**
 * @file DIP_Switch.h
 * @author T-Car Team
 * @brief DIP 开关驱动接口（启动区域选择）
 *
 * 模块职责：采样 DIP 开关电平、消抖并给出启动区域。
 * 本模块只依赖 HAL 与 RTOS，不包含任何应用层头文件；
 * 按键输入见 Key 模块（Key.h）。
 * 可调参数见 DIP_Switch.h / 对应 config。
 */

#ifndef __DIP_SWITCH_H__
#define __DIP_SWITCH_H__

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * @brief DIP 开关选中的启动区域
 */
typedef enum {
    START_ZONE_UNKNOWN = 0, // 未初始化或状态不稳定
    START_ZONE_1, // 启动区 1
    START_ZONE_2, // 启动区 2
} Start_Zone_t;

/* ------------------------- 配置宏 ------------------------- */

/**
 * DEBUG 模式开关：
 *   0 = 关闭，启动区域完全由 DIP 开关电平决定（量产默认）
 *   1 = 开启，允许通过 shell 命令 dip 人为指定启动区域，忽略 DIP 开关实际电平
 *
 * 命令用法：dip [get|1|2|auto]
 *   get  - 查看当前启动区域
 *   1/2  - 强制使用启动区 1/2
 *   auto - 取消强制，恢复按 DIP 开关电平判定
 */
#define DIP_SWITCH_DEBUG_MODE 1

/**
 * @brief 消抖时间(ms)：两次采样一致且间隔达到该值才判定为稳定
 */
#define DIP_SWITCH_DEBOUNCE_MS 20U

/**
 * @brief 稳定采样失败时的最大重试次数
 */
#define DIP_SWITCH_RETRY_MAX 3U

/**
 * @brief 初始化 DIP 开关并锁存启动区域
 * @note  内部包含消抖采样，会阻塞约 DIP_SWITCH_DEBOUNCE_MS × DIP_SWITCH_RETRY_MAX，
 *        需在 RTOS 调度器启动后调用；
 *        DIP_SWITCH_DEBUG_MODE=1 且已被 shell 强制指定时跳过采样直接采用该区域
 * @return true 读取到稳定的开关状态
 * @return false 多次采样仍不稳定，启动区域置为 START_ZONE_UNKNOWN
 */
bool DIP_Switch_Init(void);

/**
 * @brief 获取启动区域
 * @note  结果在 DIP_Switch_Init() 中锁存，运行中拨动开关不会改变结果；
 *        DIP_SWITCH_DEBUG_MODE=1 时优先返回 shell 命令 dip 强制指定的区域
 * @return Start_Zone_t 启动区域
 */
Start_Zone_t DIP_Switch_GetStartZone(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __DIP_SWITCH_H__ */
