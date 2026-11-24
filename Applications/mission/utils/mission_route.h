/**
 * @file mission_route.h
 * @author T-Car Team
 * @brief 任务固定路线表（数据层）
 * 可调参数见 mission_config.h / Applications/mission/README.md。
 */

#ifndef __MISSION_ROUTE_H__
#define __MISSION_ROUTE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "mission_nav.h"

/**
 * @brief 扫码点路线（两区各一套）
 */
const MissionRoute_t *Mission_RouteQr(void);

/**
 * @brief 进入原料区路线（两区各一套，按启动区/轮次选择）
 * @param process_layer 轮次：0=第一轮, 1=第二轮
 */
const MissionRoute_t *Mission_RouteGrab(uint8_t process_layer);

/**
 * @brief 粗加工区路线（第一二轮共用同一条路线；两区各一套）
 */
const MissionRoute_t *Mission_RouteProcessing(void);

/**
 * @brief 暂存区路线（第一二轮共用同一条路线；两区各一套，二区入口临时点 y 加整场偏移）
 * @param process_layer 轮次：0=第一轮, 1=第二轮
 */
const MissionRoute_t *Mission_RouteStorage(uint8_t process_layer);

/**
 * @brief 回家路线
 */
const MissionRoute_t *Mission_RouteHome(void);

#ifdef __cplusplus
}
#endif

#endif /* __MISSION_ROUTE_H__ */
