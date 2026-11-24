/**
 * @file nav_map.c
 * @author T-Car Team
 * @brief 导航地图源文件
 * 可调参数见 nav_config.h / Applications/nav/README.md。
 */

#include "nav_map.h"
#include "nav_config.h"
#include "chassis_config.h"
#include <string.h>
#include "FreeRTOS.h"

static TargetPoint_t *g_points = NULL;
static NavMapInfo_t *g_map_info = NULL;

/**
 * @brief 初始化地图
 * @param max_point_num 最大目标点数量
 * @param zone 目标点集区域编号（MAP_ZONE_1 / MAP_ZONE_2）
 * @return 初始化状态
 */
bool Map_Init(uint8_t max_point_num, uint8_t zone) {
    g_map_info = (NavMapInfo_t *)pvPortMalloc(sizeof(NavMapInfo_t));
    if (g_map_info == NULL) {
        return false;
    }
    memset(g_map_info, 0, sizeof(NavMapInfo_t));

    g_points = (TargetPoint_t *)pvPortMalloc(max_point_num * sizeof(TargetPoint_t));
    if (g_points == NULL) {
        vPortFree(g_map_info);
        return false;
    }
    memset(g_points, 0, max_point_num * sizeof(TargetPoint_t));
    g_map_info->max_points = max_point_num;

    if (!Map_LoadPoints(Map_GetDataPoints(zone), Map_GetDataPointCount(zone))) {
        vPortFree(g_points);
        vPortFree(g_map_info);
        return false;
    }

    return true;
}

/**
 * @brief 获取地图信息
 * @return 地图信息指针
 */
NavMapInfo_t *Map_GetInfo(void) {
    return g_map_info;
}

/**
 * @brief 添加目标点（支持插入）
 * @param point 目标点指针
 * @return 添加状态
 */
bool Map_AddPoint(const TargetPoint_t *point) {
    if (g_map_info->point_count >= g_map_info->max_points) {
        return false;
    }

    // 找到插入位置
    uint8_t insert_index = g_map_info->point_count;
    for (uint8_t i = 0; i < g_map_info->point_count; i++) {
        if (g_points[i].id >= point->id) {
            insert_index = i;
            break;
        }
    }

    for (uint8_t i = g_map_info->point_count; i > insert_index; i--) {
        memcpy(&g_points[i], &g_points[i - 1], sizeof(TargetPoint_t));
        g_points[i].id++;
    }

    memcpy(&g_points[insert_index], point, sizeof(TargetPoint_t));
    g_map_info->point_count++;

    return true;
}

/**
 * @brief 删除目标点（并重新编号）
 * @param id 目标点ID
 * @return 删除状态
 */
bool Map_RemovePoint(uint8_t id) {
    for (uint8_t i = 0; i < g_map_info->point_count; i++) {
        if (g_points[i].id == id) {
            for (uint8_t j = i; j < g_map_info->point_count - 1; j++) {
                memcpy(&g_points[j], &g_points[j + 1], sizeof(TargetPoint_t));
                g_points[j].id = j;
            }
            g_map_info->point_count--;

            return true;
        }
    }
    return false;
}

/**
 * @brief 根据ID获取目标点
 * @param id 目标点ID
 * @return 目标点指针
 */
TargetPoint_t *Map_GetPoint(uint8_t id) {
    if (id >= g_map_info->point_count) {
        return NULL;
    }
    return &g_points[id];
}

/**
 * @brief 根据名称获取目标点
 * @param name 目标点名称
 * @return 目标点指针
 */
TargetPoint_t *Map_GetPointByName(const char *name) {
    for (uint8_t i = 0; i < g_map_info->point_count; i++) {
        if (strcmp(g_points[i].name, name) == 0) {
            return &g_points[i];
        }
    }
    return NULL;
}

/**
 * @brief 修改目标点
 * @param id 目标点ID
 * @param point 新的目标点数据
 * @return 修改状态
 */
bool Map_UpdatePoint(uint8_t id, TargetPoint_t *point) {
    if (id >= g_map_info->point_count) {
        return false;
    }

    point->id = id;
    memcpy(&g_points[id], point, sizeof(TargetPoint_t));

    return true;
}

/**
 * @brief 加载预定义目标点
 * @param points 目标点数组指针（编译期任务点坐标，按 OPS 位姿点定义）
 * @param count 目标点数量
 * @return 加载状态
 * @note 若配置了 CHASSIS_BODY_REF_OFFSET_X/Y（导航位姿点=车体参考点），
 *       这里把每个目标点按 G = P + R(yaw)*offset 换算成车体参考点坐标，
 *       保证同一任务点的物理位置不变。默认偏移 0 时行为与旧版完全一致。
 */
bool Map_LoadPoints(const TargetPoint_t *points, uint8_t count) {
    for (uint8_t i = 0; i < count; i++) {
        TargetPoint_t point = points[i];
        pose_apply_body_ref_offset(
            &point.pose.x, &point.pose.y, point.pose.yaw, CHASSIS_BODY_REF_OFFSET_X, CHASSIS_BODY_REF_OFFSET_Y);
        if (!Map_AddPoint(&point)) {
            return false;
        }
    }

    return true;
}
