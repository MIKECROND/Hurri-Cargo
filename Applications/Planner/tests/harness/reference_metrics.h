/**
 * @file reference_metrics.h
 * @brief 规划器评测用的独立几何/指标库（参考真值，不依赖 Planner 内部网格碰撞实现）
 *
 * @details 设计原则：
 *          1. 本模块只依赖障碍物的真实世界坐标（旧版 planner_config.h 里每 4 点一组的轴对齐矩形），
 *             不使用地图网格的 0/1 占用状态，因此可以独立验证“网格判据”与“真实几何”的差异，
 *             避免用被测实现自己的碰撞判断来自证。
 *          2. 评测全部在【世界坐标（场地坐标）】下进行，单位统一为 cm；
 *             地图坐标与场地坐标的换算是 map = world + MAP_OFFSET，世界恢复时 world = map - MAP_OFFSET。
 *          3. 该模块与规划器版本无关：重构前后的路径都可以用同一套指标评测，便于做 A/B 对比。
 */

#ifndef PLANNER_TESTS_REFERENCE_METRICS_H
#define PLANNER_TESTS_REFERENCE_METRICS_H

#include <stdbool.h>

/* ===== 物理/采样常量（与 HEAD 版 planner_config.h 的物理含义保持一致，独立抄写而非 include） ===== */
#define REF_MAP_OFFSET_CM 240.0f /* 地图坐标 = 世界坐标 + 240cm（HEAD: MAP_OFFSET_X/Y = 2.4m） */
#define REF_ROBOT_RADIUS_CM 15.0f /* 机器人物理半径 15cm（HEAD: PHYSICAL_RADIUS = 0.15m） */
#define REF_SAMPLE_STEP_CM 2.0f /* 路径折线采样步长上限 2cm，用于找全程最小清障距离 */

/* ===== 静态容量（主机侧够用即可，超出会被截断并返回错误） ===== */
#define REF_MAX_RECTS 64 /* 障碍矩形上限（每 4 个顶点构成 1 个矩形） */
#define REF_MAX_POINTS 1024 /* 单条路径折线点数上限 */
#define REF_MAX_SEGMENTS 1024 /* 单条路径段数上限 */

/** @brief 轴对齐障碍矩形（世界坐标，cm；约定 x0<=x1, y0<=y1） */
typedef struct {
    float x0;
    float y0;
    float x1;
    float y1;
} RefRect;

/** @brief 单段路径指标（世界坐标，cm） */
typedef struct {
    float len_cm; /* 段长 cm */
    float min_clearance_cm; /* 该段采样点上的最小清障距离 cm */
    bool over_radius; /* 该段是否有点落入机器人半径内（< REF_ROBOT_RADIUS_CM） */
} RefSegMetrics;

/** @brief 单条路径的汇总指标（世界坐标，cm） */
typedef struct {
    int point_count; /* 折线点数（= simplify 后航点数） */
    int seg_count; /* 折线段数 = point_count - 1 */
    float length_cm; /* 折线总长 cm */
    float min_clearance_cm; /* 全程最小清障距离 cm（严格真值：点到障碍矩形） */
    float min_clearance_x_cm; /* 最小清障发生点的世界坐标 x（cm） */
    float min_clearance_y_cm; /* 最小清障发生点的世界坐标 y（cm） */
    int min_clearance_rect; /* 最小清障处最近的障碍矩形下标（0 基；无矩形为 -1） */
    int min_clearance_seg; /* 最小清障发生在第几段（0 基；单点路径为 -1） */
    bool collision; /* 是否物理碰撞（min_clearance_cm < REF_ROBOT_RADIUS_CM） */
    int segs_over_radius; /* 含“清障 < 半径”采样点的段数 */
    int waypoints_over_radius; /* 清障 < 半径的航点（折线顶点）数 */
    int min_clearance_wp; /* 清障距离最小的航点索引（0 基；仅作辅助） */
    float goal_error_cm; /* 路径末点与目标点的距离 cm（正常应为 0，末点被精确替换） */
    float max_turn_deg; /* 内部航点最大转角度数（0 = 直行） */
    float sum_turn_deg; /* 内部航点转角绝对值之和（度），越小路径越直 */
    int sample_count; /* 全程实际采样点总数（调试用） */
} RefPathMetrics;

/**
 * @brief 由障碍顶点构造轴对齐矩形（世界坐标 cm）
 * @details 与 HEAD Planner_Init 一致：每 4 个顶点一组，取这一组的 min/max 形成一个轴对齐矩形。
 * @param quad_xy 顶点数组，按 x0,y0,x1,y1,... 交替存放（世界坐标 cm）
 * @param point_count 顶点个数（必须为 4 的整数倍，不足 4 个的尾部忽略）
 * @param out_rects 输出矩形数组
 * @param max_rects 输出数组容量
 * @return 构造出的矩形个数；参数非法/容量不足返回 -1
 */
int ref_build_rects(const float *quad_xy, int point_count, RefRect *out_rects, int max_rects);

/**
 * @brief 点到单个轴对齐矩形的精确欧氏距离（cm）
 * @details 点在矩形内部时距离为 0，否则为到最近边/角的欧氏距离。
 */
float ref_point_to_rect_distance(float px, float py, const RefRect *rect);

/**
 * @brief 点到一组矩形的最小欧氏距离（cm），即该点的“清障距离”
 * @return 最小距离；rect_count <= 0 时返回一个很大值（1e9）
 */
float ref_min_rect_distance(float px, float py, const RefRect *rects, int rect_count);

/**
 * @brief 同 ref_min_rect_distance，但额外返回最近矩形的下标（用于报告归因）
 * @param out_index 输出最近矩形下标（可为 NULL；无矩形时写 -1）
 */
float ref_min_rect_distance_ex(float px, float py, const RefRect *rects, int rect_count, int *out_index);

/**
 * @brief 评测一条折线路径（世界坐标 cm）
 * @details 对每一段按 REF_SAMPLE_STEP_CM 均匀采样（含两端点），逐点计算到所有障碍矩形的
 *          真实最小距离，得到全程最小清障距离、碰撞判定、每段长度/最小清障、转角和目标误差。
 * @param rects 障碍矩形数组（世界坐标 cm）
 * @param rect_count 矩形个数
 * @param path_xy 路径折线，按 x0,y0,x1,y1,... 交替存放（世界坐标 cm）
 * @param point_count 折线点数
 * @param goal_x 目标点世界坐标 x（cm），用于计算终点误差
 * @param goal_y 目标点世界坐标 y（cm）
 * @param out 输出汇总指标（不可为 NULL）
 * @param segs 输出每段指标（可为 NULL；非 NULL 时最多写 seg_capacity 段）
 * @param seg_capacity segs 数组容量
 * @return 评测成功返回 true；参数非法或点数非法返回 false
 */
bool ref_measure_path(const RefRect *rects,
                      int rect_count,
                      const float *path_xy,
                      int point_count,
                      float goal_x,
                      float goal_y,
                      RefPathMetrics *out,
                      RefSegMetrics *segs,
                      int seg_capacity);

/**
 * @brief 计算每个航点（折线顶点）的清障距离
 * @return 实际写入的点数；容量不足返回 -1
 */
int ref_waypoint_clearances(
    const RefRect *rects, int rect_count, const float *path_xy, int point_count, float *out_cm, int capacity);

/**
 * @brief 计算每个内部航点的转角（度，带符号：正 = 逆时针/左转，负 = 顺时针/右转，0 = 直行）
 * @return 实际写入的转角个数（point_count - 2）；参数非法返回 -1
 */
int ref_turn_angles(const float *path_xy, int point_count, float *out_deg, int capacity);

/* ==========================================================================================
 * v2（方形车体 30×30）独立几何真值：精确 SAT
 *
 *  方形车体 = 边长 2*half_cm 的正方形，中心在世界坐标 (px,py)，航向 heading_deg。
 *  与轴对齐障碍矩形做精确分离轴（SAT）判定：
 *    - 相交时 signed clearance = -MTV（最小平移距离，>0 的侵入深度）；
 *    - 分离时 signed clearance = 两个凸多边形之间的精确欧氏距离（顶点-边距离取最小）；
 *  地图边界按世界坐标 [-240,240]×[-240,240] 的 4 条边处理（方形完整在地图内）。
 *  任务点位对均为 0/90/180° 朝向；45° 等任意朝向用于旋转/任意 yaw 回归。
 * ========================================================================================== */

/** @brief v2 方形路径指标（世界坐标 cm；signed clearance：>0 余量，0 相切，<0 侵入） */
typedef struct {
    int point_count; /* 路径点数 */
    int seg_count; /* 路径段数 */
    int sample_count; /* 1cm 采样的总采样点数 */
    float length_cm; /* 路径折线总长 */
    float min_general_signed_cm; /* 一般路段（到起终点欧氏 >端点区半径）最小带符号净空 */
    float min_endpoint_signed_cm; /* 端点豁免区（到起终点欧氏 <=端点区半径）最小带符号净空 */
    float min_signed_cm; /* 全程最小带符号净空 */
    float min_signed_x_cm; /* 最差点世界坐标 */
    float min_signed_y_cm;
    float max_penetration_cm; /* 最大侵入深度 = max(0, -最小 signed) */
    float max_penetration_x_cm;
    float max_penetration_y_cm;
    int general_collision_samples; /* 一般路段 signed < -阈值 的采样点数（正方形与障碍相交） */
    int endpoint_penetration_violations; /* 端点区 signed < -(允许侵入) 的采样点数 */
} RefSquarePathMetrics;

/**
 * @brief 计算"单个采样点 + 给定航向"的方形车体带符号净空（精确 SAT，含地图边界）
 * @param px,py        方形中心（世界坐标，cm）
 * @param heading_deg  车体航向（度）
 * @param half_cm      方形半边长（cm，通常 15）
 * @param rects        静态/动态障碍矩形
 * @param rect_count   矩形数量
 * @return 带符号净空 (cm)：>0 余量；0 相切；<0 侵入深度
 */
float ref_square_scene_clearance_cm(
    float px, float py, float heading_deg, float half_cm, const RefRect *rects, int rect_count);

/**
 * @brief 点到最近障碍矩形/地图边界的精确欧氏距离（cm）：旋转安全判定用
 * @details 与 MapCore_PointClearanceCm 同口径（静态矩形 + 地图边界；动态障碍格由调用方
 *          加入 rects），但本函数为独立实现。
 */
float ref_point_euclid_clearance_cm(float px, float py, const RefRect *rects, int rect_count);

/**
 * @brief 按"逐段航向"对折线路径做 1cm 采样，计算方形车体 SAT 真值指标
 * @param rects              障碍矩形（世界坐标 cm）
 * @param rect_count         矩形数量
 * @param path_xy            路径折线（世界坐标 cm，x0,y0,x1,y1,...）
 * @param point_count        点数
 * @param seg_heading_deg    每段航向：数组长度 heading_count（可为 point_count-1，或 1 表示所有段同航向）
 * @param heading_count      航向数组长度
 * @param start_x,start_y    精确起点世界坐标（用于端点豁免区判定）
 * @param goal_x,goal_y      精确终点世界坐标
 * @param half_cm            方形半边长（cm，通常 PLANNER_BODY_HALF_CM=15）
 * @param endpoint_zone_cm   端点豁免区半径（cm，通常 25）
 * @param endpoint_pen_tol_cm 端点区允许的模型侵入深度（cm，通常 1.0）
 * @param out                输出指标（不可为 NULL）
 * @return true 成功
 */
bool ref_measure_square_path(const RefRect *rects,
                             int rect_count,
                             const float *path_xy,
                             int point_count,
                             const float *seg_heading_deg,
                             int heading_count,
                             float start_x,
                             float start_y,
                             float goal_x,
                             float goal_y,
                             float half_cm,
                             float endpoint_zone_cm,
                             float endpoint_pen_tol_cm,
                             RefSquarePathMetrics *out);

#endif /* PLANNER_TESTS_REFERENCE_METRICS_H */
