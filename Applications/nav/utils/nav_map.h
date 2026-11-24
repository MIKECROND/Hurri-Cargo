/**
 * @file nav_map.h
 * @author Pcb-yun (pcbyinyun@163.com)
 * @brief 导航地图头文件
 * 可调参数见 nav_config.h / Applications/nav/README.md。
 */

#ifndef __NAV_MAP_H__
#define __NAV_MAP_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "stdint.h"
#include "stdbool.h"
#include "nav_math.h"

/**
 * @brief 地图点集区域编号（与 DIP 开关选中的启动区对应）
 */
#define MAP_ZONE_1 1u // 启动区1
#define MAP_ZONE_2 2u // 启动区2

/**
 * @brief 两区点位独立维护
 * @note  启动区1/2 的点位表在 nav_map_data.c 里各自独立（g_map_points_zone1 / g_map_points_zone2），
 */

/**
 * @brief 到达检测方式
 */
typedef enum {
    ARRIVE_CHECK_DISTANCE = 0, // 基于距离检测
    ARRIVE_CHECK_YAW, // 基于角度检测
    ARRIVE_CHECK_BOTH // 距离+角度检测
} ArriveCheckMode_t;

/**
 * @brief 目标点运动参数
 */
typedef struct {
    float target_speed; // 目标线速度 (cm/s)
    float target_angular_speed; // 目标角速度 (deg/s)
    float acceleration; // 加速度 (cm/s²)
    float deceleration; // 减速度 (cm/s²)
    float angular_acceleration; // 角加速度 (deg/s²)
} MotionParams_t;

/**
 * @brief 到达检测参数
 */
typedef struct {
    ArriveCheckMode_t check_mode; // 检测模式
    float distance_threshold; // 距离阈值 (cm)
    float yaw_threshold; // 角度阈值 (deg)
    uint16_t timeout_ms; // 超时时间 (ms)
} ArriveParams_t;

/**
 * @brief 目标点完整结构
 */
typedef struct {
    uint8_t id; // 目标点ID
    char name[16]; // 目标点名称
    Pose2D_t pose; // 目标位姿
    MotionParams_t motion; // 运动参数
    ArriveParams_t arrive; // 到达检测参数
} TargetPoint_t;

/**
 * @brief 地图数据
 */
typedef struct {
    uint8_t max_points; // 地图最大目标点数
    uint8_t point_count; // 当前目标点数量
} NavMapInfo_t;

bool Map_Init(uint8_t max_point_num, uint8_t zone);
NavMapInfo_t *Map_GetInfo(void);
bool Map_AddPoint(const TargetPoint_t *point);
bool Map_RemovePoint(uint8_t id);
bool Map_UpdatePoint(uint8_t id, TargetPoint_t *point);
TargetPoint_t *Map_GetPoint(uint8_t id);
TargetPoint_t *Map_GetPointByName(const char *name);
bool Map_LoadPoints(const TargetPoint_t *points, uint8_t count);

/* 预定义点位数据：按 PLANNER_ENABLED 自动返回规划器版或不使用规划器版（见 nav_map_data.c） */
const TargetPoint_t *Map_GetDataPoints(uint8_t zone);
uint8_t Map_GetDataPointCount(uint8_t zone);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __NAV_MAP_H__ */
