/**
 * @file nav_map_data.c
 * @brief 导航地图目标点数据定义（启动区1 / 启动区2 两套独立点位表）
 */

#include "nav_map.h"
#include "planner_switch.h"

/**
 * @brief 默认运动参数（两区点位共用；单点可覆盖）
 */
#define MOTION_DEFAULT                                                                                                 \
    {                                                                                                                  \
        .target_speed = 70.0f,                                                                                         \
        .target_angular_speed = 220.0f,                                                                                \
        .acceleration = 110.0f,                                                                                        \
        .deceleration = 70.0f,                                                                                         \
    }

/**
 * @brief 默认到达检测参数（两区点位共用；单点可覆盖）
 */
#define ARRIVE_DEFAULT                                                                                                 \
    {                                                                                                                  \
        .check_mode = ARRIVE_CHECK_BOTH,                                                                               \
        .distance_threshold = 1.5f,                                                                                    \
        .yaw_threshold = 1.0f,                                                                                         \
        .timeout_ms = 10000,                                                                                           \
    }

/**
 * @brief 点位初始化(坐标/朝向 + 默认运动/到达参数)
 * @param id_   点 id
 * @param name_ 点名(与任务路线表 / 另一张点位表保持一致)
 * @param x_    x 坐标(cm)
 * @param y_    y 坐标(cm)
 * @param yaw_  朝向(°)
 */
#define MAP_POINT(id_, name_, x_, y_, yaw_)                                                                            \
    {.id = (id_),                                                                                                      \
     .name = (name_),                                                                                                  \
     .pose = {.x = (x_), .y = (y_), .yaw = (yaw_)},                                                                    \
     .motion = MOTION_DEFAULT,                                                                                         \
     .arrive = ARRIVE_DEFAULT}

/* ==========================================================================
 * 启动区1 点位表
 * ========================================================================== */
static const TargetPoint_t g_map_points_zone1[] = {
    MAP_POINT(0, "HOME", 0.0f, 0.0f, 0.0f),
    MAP_POINT(1, "QrCode", 24.0f, 105.0f + 10.0f, 0.0f),
    MAP_POINT(2, "MATL_GRAB", 105.0f + 10.0f, 16.0f, 0.0f),
    MAP_POINT(3, "MATL_PROCESSING", 114.0f, 190.0f, 180.0f),
    MAP_POINT(4, "MATL_STORAGE", 195.0f, 100.0f, 90.0f),
    MAP_POINT(5, "OA1", 190.0f, 190.0f, 90.0f),
    MAP_POINT(6, "OA2", 195.0f, 20.0f, 0.0f),
};

/* ==========================================================================
 * 启动区2 点位表
 * ========================================================================== */
static const TargetPoint_t g_map_points_zone2[] = {
    MAP_POINT(0, "HOME", 0.0f, 0.0f, 0.0f),
    MAP_POINT(1, "QrCode", 24.0f, -100.0f + 15.0f, 0.0f),
    MAP_POINT(2, "MATL_GRAB", 101.0f + 10.0f, -192.0f, 0.0f),
    MAP_POINT(3, "MATL_PROCESSING", 110.0f, -15.0f, 180.0f),
    MAP_POINT(4, "MATL_STORAGE", 195.0f, -105.0f, 90.0f),
    MAP_POINT(5, "OA1", 190.0f, -15.0f, 90.0f),
    MAP_POINT(6, "OA2", 195.0f, -185.0f, 0.0f),
};

/**
 * @brief 获取指定区域的目标点数组
 * @param zone 区域编号（MAP_ZONE_1 / MAP_ZONE_2）
 */
const TargetPoint_t *Map_GetDataPoints(uint8_t zone) {
    return (zone == MAP_ZONE_2) ? g_map_points_zone2 : g_map_points_zone1;
}

/**
 * @brief 获取指定区域的目标点数量
 * @param zone 区域编号（MAP_ZONE_1 / MAP_ZONE_2）
 */
uint8_t Map_GetDataPointCount(uint8_t zone) {
    if (zone == MAP_ZONE_2) {
        return sizeof(g_map_points_zone2) / sizeof(TargetPoint_t);
    }
    return sizeof(g_map_points_zone1) / sizeof(TargetPoint_t);
}
