/**
 * @file ops.h
 * @author Pcb-yun (pcbyinyun@163.com)
 * @brief 平面定位模块头文件
 * 可调参数见 ops_cfg.h。
 */

#ifndef __OPS_H__
#define __OPS_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "ops_cfg.h"
#include <stdbool.h>
#include <stdint.h>

#define USART4_RX_BUF_SIZE 32

typedef struct {
    uint8_t data[USART4_RX_BUF_SIZE];
    uint8_t len;
} Uart4_RxBuf_t;

/**
 * @brief 定位数据结构体
 */
typedef struct {
#if OPS_USE_POS
    float x; // X坐标 (mm)
    float y; // Y坐标 (mm)
#endif
#if OPS_USE_YAW
    float yaw; // 航向 (deg)
#endif
#if OPS_USE_PITCH
    float pitch; // 俯仰
#endif
#if OPS_USE_ROLL
    float roll; // 滚转
#endif
#if OPS_USE_ANG_VEL
    float w_z; // 角速度
#endif
    uint32_t timestamp; // 时间戳
} OPSData_t;

bool OPS_Init(void);
bool OPS_Get(OPSData_t *pose);
void OPS_Zero(void);
bool OPS_Is_Ready(void);

/**
 * @brief 距最近一次"收到并解析完一整帧"的毫秒数
 * @return 0 = 刚收到；UINT32_MAX(0xFFFFFFFF) = 上电以来从未收到完整帧
 * @note  只读、无锁（32bit 单写者）；供导航/规划做 OPS 数据丢失判定（如 >150ms 判丢失）。
 *        不要用 OPSData_t.timestamp 判断新鲜度：那是"收到任意字节"的时间戳，
 *        且定位融合层会按自己的更新周期重写 PoseTimestamp.timestamp。
 */
uint32_t OPS_GetFrameAgeMs(void);

/**
 * @brief OPS 数据是否已过期（距最近完整帧的时间 > max_age_ms）
 * @param max_age_ms 允许的最大数据年龄(ms)
 * @return true = 过期/从未收到过完整帧
 */
bool OPS_Is_Stale(uint32_t max_age_ms);

#define OPS_SET_HELP                                                                                                   \
    "Usage: set COMMAND [value]\r\n"                                                                                   \
    "\r\n"                                                                                                             \
    "commands:\r\n"                                                                                                    \
    "  j         Set yaw angle\r\n"                                                                                    \
    "  x         Set X coordinate\r\n"                                                                                 \
    "  y         Set Y coordinate"

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __OPS_H__ */
