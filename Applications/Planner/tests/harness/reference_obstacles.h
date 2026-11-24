/**
 * @file reference_obstacles.h
 * @brief 独立几何真值用的预定义障碍物数据（世界坐标，单位 cm）
 *
 * @details 数据来源：重构版 `Applications/Planner/utils/Cstar_map_core.c` 里迁移的预定义障碍物
 *          顶点（与世界坐标一致的一手场地数据），本文件只**复制数据**，不包含任何碰撞判定实现；
 *          所有距离计算仍由 `reference_metrics.c` 独立完成，不使用 Planner 的网格/清障距离场。
 *
 *          为避免“数据复制后与实现脱节”，`new_runner.c` 启动时会做一致性校验：
 *          逐格比较 MapCore_PointClearanceCm(格中心) 与本文件矩形距离的最小值（含地图边界项），
 *          偏差超过 0.1cm 即判失败并退出。若重构版修改了预定义障碍数据，这里必须同步更新。
 *
 *          分组规则与实现一致：每 4 个点 = 一个轴对齐矩形（取 min/max），数组按 x0,y0,x1,y1...
 *          交替存放。
 */

#ifndef PLANNER_TESTS_REFERENCE_OBSTACLES_H
#define PLANNER_TESTS_REFERENCE_OBSTACLES_H

/* ===== 启动区 1：4 个方框 + 2 面墙 + 4 条场边条 = 10 个矩形（40 个顶点） ===== */
static const float s_ref_obstacle_quad_zone1[] = {
    /* 方框：左上 */
    35.0f, 175.0f, 35.0f, 120.0f, 90.0f, 175.0f, 90.0f, 120.0f,
    /* 方框：右上 */
    120.0f, 175.0f, 175.0f, 175.0f, 120.0f, 120.0f, 175.0f, 120.0f,
    /* 方框：左下 */
    35.0f, 90.0f, 90.0f, 90.0f, 35.0f, 35.0f, 90.0f, 35.0f,
    /* 方框：右下 */
    120.0f, 90.0f, 175.0f, 90.0f, 120.0f, 35.0f, 175.0f, 35.0f,
    /* 横墙：顶部中间 */
    76.0f, 210.0f, 134.0f, 210.0f, 76.0f, 225.0f, 134.0f, 225.0f,
    /* 竖墙：右侧中间 */
    210.0f, 134.0f, 225.0f, 134.0f, 210.0f, 76.0f, 225.0f, 76.0f,
    /* 场边条：左 */
    -25.0f, 235.0f, -25.0f, -35.0f, -15.0f, 235.0f, -15.0f, -35.0f,
    /* 场边条：右 */
    225.0f, 235.0f, 225.0f, -35.0f, 235.0f, 235.0f, 235.0f, -35.0f,
    /* 场边条：上 */
    -15.0f, 235.0f, -15.0f, 225.0f, 225.0f, 235.0f, 225.0f, 225.0f,
    /* 场边条：下 */
    -15.0f, -25.0f, -15.0f, -35.0f, 225.0f, -25.0f, 225.0f, -35.0f,
};
#define REF_OBSTACLE_POINTS_ZONE1 (sizeof(s_ref_obstacle_quad_zone1) / sizeof(s_ref_obstacle_quad_zone1[0]) / 2)

/* ===== 启动区 2：4 个方框 + 2 面墙 + 4 条场边条 = 10 个矩形（40 个顶点） ===== */
static const float s_ref_obstacle_quad_zone2[] = {
    /* 方框：左下 */
    35.0f, 35.0f, 90.0f, 35.0f, 35.0f, 90.0f, 90.0f, 90.0f,
    /* 方框：左上 */
    35.0f, 120.0f, 35.0f, 175.0f, 90.0f, 120.0f, 90.0f, 175.0f,
    /* 方框：右下 */
    120.0f, 35.0f, 120.0f, 90.0f, 175.0f, 35.0f, 175.0f, 90.0f,
    /* 方框：右上 */
    120.0f, 120.0f, 120.0f, 175.0f, 175.0f, 120.0f, 175.0f, 175.0f,
    /* 竖墙：左侧中间 */
    0.0f, 76.0f, 0.0f, 134.0f, -15.0f, 76.0f, -15.0f, 134.0f,
    /* 横墙：顶部中间 */
    76.0f, 210.0f, 76.0f, 225.0f, 134.0f, 210.0f, 134.0f, 225.0f,
    /* 场边条：下 */
    -25.0f, -25.0f, 235.0f, -25.0f, -25.0f, -15.0f, 235.0f, -15.0f,
    /* 场边条：上 */
    -25.0f, 225.0f, 235.0f, 225.0f, -25.0f, 235.0f, 235.0f, 235.0f,
    /* 场边条：左 */
    -25.0f, -15.0f, -15.0f, -15.0f, -25.0f, 225.0f, -15.0f, 225.0f,
    /* 场边条：右 */
    225.0f, -15.0f, 235.0f, -15.0f, 225.0f, 225.0f, 235.0f, 225.0f,
};
#define REF_OBSTACLE_POINTS_ZONE2 (sizeof(s_ref_obstacle_quad_zone2) / sizeof(s_ref_obstacle_quad_zone2[0]) / 2)

/* 地图范围（地图坐标：世界坐标 + 240cm，边长 480cm），仅用于一致性校验里的地图边界项 */
#define REF_MAP_SIZE_CM 480.0f

/* ==========================================================================================
 * 任务命名点位（世界坐标 cm + 到达朝向 deg）
 * 与 Applications/nav/av_map_data.c 的“规划器版”点位一致：
 *   zone1: HOME 0 / QrCode 0 / MATL_GRAB 0 / MATL_STORAGE 90 / MATL_PROCESSING 180
 *   zone2: 全部 0
 * ========================================================================================== */
typedef struct {
    const char *name;
    float x_cm;
    float y_cm;
    float yaw_deg;
} RefNamedPoint;

#define REF_POINTS_PER_ZONE 5

static const RefNamedPoint s_ref_points_zone1[REF_POINTS_PER_ZONE] = {
    {"HOME", 0.0f, 0.0f, 0.0f},
    {"QrCode", 10.0f, 105.0f, 0.0f},
    {"MATL_GRAB", 105.0f, 14.0f, 0.0f},
    {"MATL_STORAGE", 190.0f, 105.0f, 90.0f},
    {"MATL_PROCESSING", 105.0f, 190.0f, 180.0f},
};

static const RefNamedPoint s_ref_points_zone2[REF_POINTS_PER_ZONE] = {
    {"HOME", 0.0f, 0.0f, 0.0f},
    {"QrCode", 105.0f, 10.0f, 0.0f},
    {"MATL_GRAB", 196.0f, 105.0f, 0.0f},
    {"MATL_STORAGE", 105.0f, 190.0f, 0.0f},
    {"MATL_PROCESSING", 20.0f, 105.0f, 0.0f},
};

#endif /* PLANNER_TESTS_REFERENCE_OBSTACLES_H */
