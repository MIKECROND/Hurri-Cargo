/**
 * @file Cstar_map_core.h
 * @author T-Car Team
 * @brief 地图核心：栅格地图、清障距离场与"方形车体"连续几何碰撞判定
 * @details
 *  地图由两层组成：
 *   1) 栅格位图 g_map.grid（2bit/格）：0=空闲、1=静态障碍（预定义场地障碍物）、
 *      2=动态障碍（树莓派雷达扫描写入、shell 手工添加）。位图只用于显示/调试与
 *      动态障碍记录，不直接参与精确定位判定；
 *   2) 精确几何：静态障碍物在加载时由"世界坐标轴对齐矩形"构造，动态障碍格按其
 *      10cm 方格处理。碰撞/余量判定基于这两组精确几何。
 *
 *  两种距离场（每次规划前重建）：
 *   - g_map.clearance[MAX_CELLS]（uint8）：格中心到最近障碍/地图边界的欧氏距离 (cm)，
 *     用于原地旋转安全性判断与诊断；
 *   - g_map.square_clearance[MAX_CELLS]（int8，带符号）：格中心到"障碍按方形车体
 *     半长 H0=PLANNER_BODY_HALF_CM 做 Minkowski 扩张后的中心禁区"的距离：
 *     >0 安全余量，0 相切，<0 侵入深度。任意朝向 psi 的方形可用保守投影半长
 *     H(psi)=half*(|cos|+|sin|)；由于扩张量在 x/y 两轴相同，
 *     clearance(H) = clearance(H0) - (H-H0)，A* 可 O(1) 查任意朝向可行性。
 */

#ifndef __CSTAR_MAP_CORE_H__
#define __CSTAR_MAP_CORE_H__

#include <stdbool.h>
#include <stdint.h>
#include "planner_config.h"

#if PLANNER_ENABLED && (PLANNER_PLATFORM == PLATFORM_MCU)

/* 栅格取值（2bit/格） */
#define MAP_CELL_FREE 0 /* 空闲 */
#define MAP_CELL_STATIC_OBSTACLE 1 /* 预定义静态障碍 */
#define MAP_CELL_DYNAMIC_OBSTACLE 2 /* 雷达/手工添加的动态障碍 */

/* 地图对象：存储由移植层提供（planner_port.c 放在 SRAM2），地图数据由本模块加载 */
typedef struct {
    uint8_t *grid; /* 栅格位图（2bit/格），长度 MAX_MAP_BYTES */
    uint8_t *clearance; /* 欧氏清障距离场 (cm)，长度 MAX_CELLS */
    int8_t *square_clearance; /* 方形（H0 扩张）带符号清障场 (cm)，长度 MAX_CELLS */
    float res; /* 栅格分辨率 (m/格) */
    int grid_size; /* 地图边长（格） */
} ServerMap;

extern ServerMap g_map;

/* ==========================================================================
 * 生命周期与统计
 * ========================================================================== */

/**
 * @brief 初始化地图：清空栅格、按启动区载入预定义障碍物、重建两个距离场
 * @param zone             启动区（1 / 2）
 * @param grid             栅格位图缓冲区，长度 MAX_MAP_BYTES
 * @param clearance        欧氏清障距离场缓冲区，长度 MAX_CELLS
 * @param square_clearance 方形带符号清障场缓冲区，长度 MAX_CELLS
 * @return true 初始化成功
 */
bool MapCore_Init(uint8_t zone, uint8_t *grid, uint8_t *clearance, int8_t *square_clearance);

/**
 * @brief 重建距离场（动态障碍变更后调用，规划前也会自动重建）
 */
void MapCore_RebuildClearance(void);

/**
 * @brief 统计当前障碍格数量（静态 + 动态）
 */
uint16_t MapCore_ObstacleCells(void);

/**
 * @brief 当前启动区加载的静态障碍矩形数量（调试用）
 */
uint8_t MapCore_StaticRectCount(void);

/* ==========================================================================
 * 栅格位图访问（2bit/格打包；树莓派雷达通过 set_grid_val 写入动态障碍）
 * ========================================================================== */

static inline bool map_cell_in_bounds(const ServerMap *map, int x, int y) {
    return (x >= 0) && (x < map->grid_size) && (y >= 0) && (y < map->grid_size);
}

static inline uint8_t get_grid_val(ServerMap *map, int x, int y) {
    int idx = x * map->grid_size + y;
    uint8_t shift = (uint8_t)((idx & 3) << 1); /* 每字节 4 格，每格 2bit */
    return (uint8_t)((map->grid[idx >> 2] >> shift) & 0x03);
}

static inline void set_grid_val(ServerMap *map, int x, int y, uint8_t val) {
    int idx = x * map->grid_size + y;
    int byte_idx = idx >> 2;
    uint8_t shift = (uint8_t)((idx & 3) << 1);
    uint8_t mask = (uint8_t) ~(0x03u << shift);
    map->grid[byte_idx] = (uint8_t)((map->grid[byte_idx] & mask) | ((val & 0x03u) << shift));
}

static inline bool map_cell_is_obstacle(uint8_t val) {
    return val != MAP_CELL_FREE;
}

/* ==========================================================================
 * 欧氏清障查询（旋转安全 / 诊断用；单位 cm，地图坐标 m）
 * ========================================================================== */

/**
 * @brief 查询格中心欧氏清障距离（缓存值，cm）
 */
uint8_t MapCore_CellClearanceCm(int gx, int gy);

/**
 * @brief 精确查询任意点到最近障碍/地图边界的欧氏距离 (cm)
 */
float MapCore_PointClearanceCm(Point2D p);

/**
 * @brief 线段最小欧氏清障 (cm)：按 PLANNER_PATH_SAMPLE_STEP_CM 采样
 */
float MapCore_SegmentMinClearanceCm(Point2D a, Point2D b);

/**
 * @brief 点/线段是否满足最小欧氏清障要求
 */
bool MapCore_PointFree(Point2D p, float required_cm);
bool MapCore_SegmentFree(Point2D a, Point2D b, float required_cm);

/* ==========================================================================
 * 方形车体几何查询（带符号净空，单位 cm；地图坐标 m）
 * 方形半长 = half_cm，轴对齐近似：任意朝向按投影半长 best 保守处理。
 * ========================================================================== */

/**
 * @brief 查询方形车体中心的带符号净空 (cm)
 * @param p       中心点（地图坐标，米）
 * @param half_cm 方形半长 (cm)（按朝向的投影半长 H(psi)）
 * @return >0 到中心禁区的余量；0 相切；<0 已侵入（负值为侵入深度）
 */
float MapCore_PointSquareClearanceCm(Point2D p, float half_cm);

/**
 * @brief 线段上的最小方形净空 (cm)：按 2cm 采样（含端点）
 */
float MapCore_SquareSegmentMinClearanceCm(Point2D a, Point2D b, float half_cm);

/**
 * @brief 点/线段是否满足方形净空要求（>= required_cm - 1e-3）
 */
bool MapCore_SquareFree(Point2D p, float half_cm, float required_cm);
bool MapCore_SquareSegmentFree(Point2D a, Point2D b, float half_cm, float required_cm);

/**
 * @brief 查询格中心的方形净空基值（H0=PLANNER_BODY_HALF_CM）
 * @note  任意朝向 psi 的半长 H 的净空 = 返回值 - (H - PLANNER_BODY_HALF_CM)
 */
int16_t MapCore_CellSquareClearanceCm(int gx, int gy);

/* ==========================================================================
 * 原地旋转安全性
 * ========================================================================== */

/**
 * @brief 该点位能否安全完成车体原地旋转
 * @details 旋转扫过所有朝向，需要欧氏清障 >= PLANNER_ROTATION_CLEARANCE_CM
 *          （默认旋转扫掠半径 21.22cm + 2cm 余量；扫掠半径由
 *          PLANNER_ROTATION_SWEEP_RADIUS_CM 配置，车体有凸出时按实测值）。
 */
bool MapCore_RotationSafe(Point2D p);

/**
 * @brief 在整张地图中找离 near 最近的旋转安全格中心（5cm/格中心精度）
 * @param near 参考点（地图坐标，米）
 * @param out  输出：旋转安全点（地图坐标，米）
 * @return true 找到
 */
bool MapCore_FindRotationSafePoint(Point2D near, Point2D *out);

#endif /* PLANNER_ENABLED && PLANNER_PLATFORM == PLATFORM_MCU */

#endif /* __CSTAR_MAP_CORE_H__ */
