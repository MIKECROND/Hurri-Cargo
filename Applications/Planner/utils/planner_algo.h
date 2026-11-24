/**
 * @file planner_algo.h
 * @author T-Car Team
 * @brief 路径规划算法层（方形车体 30×30）：A* + 路径后处理 + 朝向/旋转方案
 * @details
 *  对外入口：
 *   - Planner_PlanPathWithYaw(start, goal, start_yaw, goal_yaw, ...)：
 *     规划"平移路径 + 原地旋转方案"：
 *       1) 按 start/goal 的朝向计算方形车体保守投影半长 H(psi)，
 *          A* 节点/边用地图层的"方形净空"判定（障碍按 H 做 Minkowski 扩张）；
 *       2) 分级尝试软目标（方形净空 >=4cm）/硬约束（>=1cm，采样门限）；
 *       3) 路径直线化 + 重采样 + 逐段方形净空校验；
 *       4) 朝向方案：需要改变 yaw 时，只在"旋转安全点"原地旋转
 *          （中心欧氏清障 >= 旋转扫掠半径 PLANNER_ROTATION_SWEEP_RADIUS_CM + 2cm，
 *          默认扫掠半径 = 30×30 半对角线 21.22cm，车体有凸出时按实测配置）；
 *          若路径上没有安全点，则在附近找安全点作为 via 点绕行；
 *          平移段保持固定朝向（麦轮蟹行/直行），绝不在 40cm/35cm 通道内旋转。
 *   - Planner_PlanPath(start, goal, ...)：start/goal yaw 均取 0 的兼容入口（无旋转）。
 *
 *  纯算法层：只依赖 Cstar_map_core 的几何查询，不依赖 FreeRTOS/导航层，
 *  可在 PC 上用 gcc 做单元测试（定义 PLANNER_HOST_TEST）。
 */

#ifndef __PLANNER_ALGO_H__
#define __PLANNER_ALGO_H__

#include <stdbool.h>
#include "Cstar_map_core.h"
#include "planner_config.h"

#if PLANNER_ENABLED && (PLANNER_PLATFORM == PLATFORM_MCU)

/* 规划结果等级 */
#define PLANNER_TIER_SOFT 0 /* 满足方形安全余量（推荐） */
#define PLANNER_TIER_HARD 1 /* 退化为仅满足采样硬门限（tight，执行时限速） */

/* rotation_index 的“不旋转”取值 */
#define PLANNER_ROTATE_NONE 0xFFFFu

/* 规划失败原因（供移植层日志/诊断） */
typedef enum {
    PLANNER_FAIL_NONE = 0,
    PLANNER_FAIL_ENDPOINT = 1, /* 起点/终点落在方形禁区，无法规划 */
    PLANNER_FAIL_NO_SOLUTION = 2, /* A* 找不到可行路径 */
    PLANNER_FAIL_NO_ROTATION_POINT = 3, /* 需要改变朝向但找不到安全旋转点 */
    PLANNER_FAIL_BAD_PARAM = 4,
} PlannerFailReason_t;

/**
 * @brief 一次规划的统计信息（用于日志/调试显示/工具命令）
 */
typedef struct {
    uint16_t waypoints; /* 路径点数（含起点与精确终点） */
    uint16_t expanded_nodes; /* A* 展开节点数（搜索规模，诊断用） */
    uint16_t rotation_index; /* 原地旋转发生在 path[rotation_index]；PLANNER_ROTATE_NONE=不旋转 */
    float heading_before_deg; /* 旋转前平移段的朝向（目标 yaw） */
    float heading_after_deg; /* 旋转后平移段的朝向（= 目标 yaw） */
    float length_cm; /* 路径折线总长 (cm) */
    float min_square_clearance_cm; /* 最终路径最小方形净空（带符号，cm） */
    float rotation_clearance_cm; /* 旋转点欧氏清障 (cm)，不旋转时为 -1 */
    float min_clearance_cm; /* 路径最小欧氏清障 (cm)，诊断/显示用 */
    uint8_t tier; /* PLANNER_TIER_SOFT / PLANNER_TIER_HARD */
    uint8_t fail_reason; /* PlannerFailReason_t */
} PlannerPathStats_t;

/**
 * @brief 完整规划（带起终点朝向）：输出路径 + 旋转方案
 * @param start         起点（地图坐标，米）
 * @param goal          终点（地图坐标，米）
 * @param start_yaw_deg 起点实际朝向（度）；规划出的旋转前平移段会保持该朝向
 * @param goal_yaw_deg  终点要求朝向（度）
 * @param out_path      输出路径（地图坐标，米；含精确起点/终点）
 * @param out_cap       输出容量
 * @param stats         输出统计（不可为 NULL 更利于诊断）
 * @return 路径点数；0 表示失败（原因见 stats->fail_reason）
 */
int Planner_PlanPathWithYaw(Point2D start,
                            Point2D goal,
                            float start_yaw_deg,
                            float goal_yaw_deg,
                            Point2D *out_path,
                            int out_cap,
                            PlannerPathStats_t *stats);

/**
 * @brief 兼容入口：起终点朝向都按 0° 处理（只规划平移路径，不生成旋转方案）
 */
int Planner_PlanPath(Point2D start, Point2D goal, Point2D *out_path, int out_cap, PlannerPathStats_t *stats);

/* ==========================================================================
 * 几何/朝向工具
 * ========================================================================== */

/**
 * @brief 方形车体在给定朝向下的保守投影半长 H(psi) (cm)
 * @details H(psi) = half * (|cos(psi mod 90)| + |sin(psi mod 90)|)，范围 [15, 21.22]；
 *          轴对齐朝向（0/90/180/270）时等于半边长 15cm。
 */
float Planner_HeadingFootprintHalfCm(float yaw_deg);

/** @brief 两点欧氏距离 (cm) */
float Planner_PointDistanceCm(Point2D a, Point2D b);

/** @brief 点到线段的最短距离 (cm)：执行期轨迹偏离监测用 */
float Planner_PointSegmentDistanceCm(Point2D p, Point2D a, Point2D b);

/** @brief 路径折线总长 (cm) */
float Planner_PathLengthCm(const Point2D *path, int len);

/* ==========================================================================
 * 路径后处理（方形车体版）
 * ========================================================================== */

/**
 * @brief 路径直线化（原地压缩）：每段用方形净空校验
 * @param path             路径点数组（原地修改）
 * @param len              压缩前点数
 * @param half_cm          方形车体在平移朝向下的投影半长 (cm)
 * @param min_clearance_cm 每段要求的最小方形净空 (cm，带符号)
 * @return 压缩后点数
 */
int Planner_PathSimplify(Point2D *path, int len, float half_cm, float min_clearance_cm);

/**
 * @brief 长段重采样（默认配置关闭）
 */
int Planner_PathResample(const Point2D *in, int len, Point2D *out, int out_cap, float max_segment_cm);

/**
 * @brief 方形路径安全校验：逐段采样，检查方形净空是否满足要求
 * @param path                 路径点数组
 * @param len                  点数
 * @param half_cm              方形车体投影半长 (cm)
 * @param required_cm          一般路段要求的方形净空（cm，采样门限）
 * @param endpoint_allowed_cm  端点豁免区内允许的方形净空（cm）
 * @param out_min_square_cm    输出整条路径最小方形净空（cm，带符号）
 * @param out_min_circle_cm    输出整条路径最小欧氏清障（cm，诊断用，可为 NULL）
 * @return true 通过校验
 */
bool Planner_SquarePathValidate(const Point2D *path,
                                int len,
                                float half_cm,
                                float required_cm,
                                float endpoint_allowed_cm,
                                float *out_min_square_cm,
                                float *out_min_circle_cm);

/* ==========================================================================
 * 航点执行剖面（纯函数：不依赖 RTOS/导航层，可在 PC 上单元测试）
 * ========================================================================== */

/**
 * @brief 预计算每个路径点之后的剩余路程 (cm)
 */
bool Planner_BuildRemainingCm(const Point2D *path, int len, float *out_remaining_cm, int out_cap);

/**
 * @brief 计算某个航点的目标速度：剩余路程进入减速区后线性降到终点速度
 */
float Planner_WaypointSpeedCmS(float remaining_cm, bool tight);

/**
 * @brief 按单段长度限制入段速度（保证导航层在该段内能减速停稳）
 */
float Planner_SegmentSpeedLimitCmS(float seg_cm);

/**
 * @brief 按段长与实际速度估算单段超时 (ms)
 */
uint32_t Planner_WaypointTimeoutMs(float seg_cm, float speed_cm_s);

/**
 * @brief 原地旋转超时估算 (ms)
 * @param delta_deg 需要旋转的角度（绝对值，度）
 */
uint32_t Planner_RotationTimeoutMs(float delta_deg);

#endif /* PLANNER_ENABLED && PLANNER_PLATFORM == PLATFORM_MCU */

#endif /* __PLANNER_ALGO_H__ */
