/**
 * @file planner_algo.c
 * @author T-Car Team
 * @brief 路径规划算法层（方形车体 30×30）：方形净空 A* + 路径后处理 + 朝向/旋转方案
 * @details
 *  相对"圆形半径15cm"旧模型的关键变化：
 *  1) 碰撞判定用方形车体 Minkowski 扩张：车体半长 H 的轴对齐方形 → 中心禁区 =
 *     障碍矩形在 x/y 各向外扩 H；任意朝向 psi 用保守投影半长
 *     H(psi)=15*(|cos|+|sin|)（≤21.22cm），轴对齐朝向时 H=15cm 精确。
 *  2) 原地旋转会扫过所有朝向，因此旋转只在"旋转安全点"进行：中心欧氏清障
 *     ≥ 旋转扫掠半径 PLANNER_ROTATION_SWEEP_RADIUS_CM + 2cm 余量（默认 21.22+2cm，
 *     车体有凸出时按实测最大外缘半径配置）；40cm/35cm 通道内禁止旋转。
 *  3) 朝向方案与路径一起规划：需要改变 yaw 时先规划平移路径，再在路径上（或通过
 *     附近 via 点绕行）选一个旋转安全点插入，旋转点前后分别保持固定朝向平移。
 *
 *  纯算法层：只依赖 Cstar_map_core 的几何查询，不依赖 FreeRTOS/导航层，
 *  可在 PC 上用 gcc 直接编译测试（PLANNER_HOST_TEST）。
 */

#include "planner_algo.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "priority_queue.h"

#if PLANNER_ENABLED && (PLANNER_PLATFORM == PLATFORM_MCU)

/* ==========================================================================
 * 静态内存（BSS，不占任务栈）
 * ========================================================================== */
PLACE_IN_SRAM2 static uint16_t s_g_score[MAX_CELLS]; /* A* 累计代价 */
PLACE_IN_SRAM2 static uint8_t s_came_from_dir[MAX_CELLS]; /* A* 父节点方向 */
PLACE_IN_SRAM2 static Point2D s_raw_path[MAX_PATH_LEN]; /* A* 原始逐格路径 */
PLACE_IN_SRAM2 static Point2D s_via_a[MAX_PATH_LEN]; /* via 旋转点方案：起点→旋转点（兼作连续化输出的临时缓冲） */

/* 前向声明（定义在后文，供 plan_translation_path 使用） */
static int copy_path_to_out(const Point2D *src, int len, Point2D *out_path, int out_cap);
static int finalize_resample(Point2D *path, int len, Point2D *out_path, int out_cap);

/* 8 邻域方向与移动代价 */
static const int8_t s_dir_x[8] = {0, 0, 1, -1, 1, 1, -1, -1};
static const int8_t s_dir_y[8] = {1, -1, 0, 0, 1, -1, 1, -1};
static const uint16_t s_move_cost[8] = {
    STEP_STRAIGHT,
    STEP_STRAIGHT,
    STEP_STRAIGHT,
    STEP_STRAIGHT,
    STEP_DIAG,
    STEP_DIAG,
    STEP_DIAG,
    STEP_DIAG};

/* ==========================================================================
 * 基础工具
 * ========================================================================== */

/** @brief 两点欧氏距离 (cm) */
static inline float dist_cm(Point2D a, Point2D b) {
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    return sqrtf(dx * dx + dy * dy) * 100.0f;
}

/** @brief 两点距离平方 (m²)：仅做阈值比较时免开方 */
static inline float dist_sq_m(Point2D a, Point2D b) {
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    return dx * dx + dy * dy;
}

/** @brief 格中心坐标（地图坐标，米） */
static inline Point2D cell_center(int gx, int gy) {
    Point2D p;
    p.x = ((float)gx + 0.5f) * PLANNER_MAP_RES;
    p.y = ((float)gy + 0.5f) * PLANNER_MAP_RES;
    return p;
}

/** @brief 角度归一化到 (-180, 180] 度 */
static float yaw_wrap_deg(float yaw_deg) {
    float y = fmodf(yaw_deg, 360.0f);
    if (y > 180.0f) y -= 360.0f;
    if (y <= -180.0f) y += 360.0f;
    return y;
}

float Planner_HeadingFootprintHalfCm(float yaw_deg) {
    /* 方形对 90° 对称：取 mod 90 的相对角度 */
    float a = fmodf(yaw_deg, 90.0f);
    if (a < 0.0f) a += 90.0f;
    float rad = a * (3.14159265358979f / 180.0f);
    return PLANNER_BODY_HALF_CM * (fabsf(cosf(rad)) + fabsf(sinf(rad)));
}

/**
 * @brief 方形净空余量惩罚（固定点代价，×10）：越贴中心禁区代价越高
 */
static inline uint16_t square_penalty(float square_clearance_cm) {
    const float soft_cm = PLANNER_SAFE_MARGIN_CM;
    const float hard_cm = PLANNER_SQUARE_HARD_MARGIN_CM;
    const float span_cm = soft_cm - hard_cm;

    if (square_clearance_cm >= soft_cm || span_cm <= 0.0f) {
        return 0;
    }

    float deficit = (soft_cm - square_clearance_cm) / span_cm;
    if (deficit < 0.0f) deficit = 0.0f;
    if (deficit > 1.0f) deficit = 1.0f;

    float penalty = (float)PLANNER_CLEARANCE_PENALTY_MAX * deficit * deficit;
    return (uint16_t)(penalty + 0.5f);
}

/**
 * @brief Octile 启发函数（8 邻域最短代价的精确下界，A* 可采纳且一致）
 */
static inline uint16_t octile_heuristic(int ax, int ay, int bx, int by) {
    int dx = abs(ax - bx);
    int dy = abs(ay - by);
    int min_d = (dx < dy) ? dx : dy;
    int max_d = (dx > dy) ? dx : dy;
    return (uint16_t)(STEP_DIAG * min_d + STEP_STRAIGHT * (max_d - min_d));
}

/* ==========================================================================
 * 单段 A* 规划（方形净空；反向：从终点搜到起点附近）
 * ========================================================================== */

/** @brief 格中心方形净空（按朝向投影半长 half_cm，含障碍格负值处理） */
static inline float cell_square_clearance_cm(int gx, int gy, float half_cm) {
    return (float)MapCore_CellSquareClearanceCm(gx, gy) - (half_cm - PLANNER_BODY_HALF_CM);
}

/**
 * @brief 规划两点之间的平移路径（方形净空约束）
 * @param start               精确起点（地图坐标，米）
 * @param goal                精确终点（地图坐标，米）
 * @param half_cm             方形车体投影半长 (cm)
 * @param required_cm         一般路段要求的方形净空（采样门限，cm）
 * @param endpoint_allowed_cm 起终点豁免区允许的方形净空（cm）
 * @param out_path            输出路径（[0]=精确起点，中间格中心，末尾=精确终点）
 * @param out_cap             容量
 * @param expanded_nodes      输出 A* 展开节点数（可为 NULL）
 * @param fail_reason         输出失败原因（可为 NULL）
 * @return 路径点数；0 失败
 */
static int plan_segment(Point2D start,
                        Point2D goal,
                        float half_cm,
                        float required_cm,
                        float endpoint_allowed_cm,
                        Point2D *out_path,
                        int out_cap,
                        uint16_t *expanded_nodes,
                        uint8_t *fail_reason) {
    if (fail_reason != NULL) *fail_reason = PLANNER_FAIL_NO_SOLUTION;
    if (out_path == NULL || out_cap < 2) {
        if (fail_reason != NULL) *fail_reason = PLANNER_FAIL_BAD_PARAM;
        return 0;
    }

    const float res = g_map.res;
    const int grid = g_map.grid_size;
    const int total_cells = grid * grid;

    int start_gx = (int)(start.x / res);
    int start_gy = (int)(start.y / res);
    int goal_gx = (int)(goal.x / res);
    int goal_gy = (int)(goal.y / res);

    if (!map_cell_in_bounds(&g_map, start_gx, start_gy) || !map_cell_in_bounds(&g_map, goal_gx, goal_gy)) {
        if (fail_reason != NULL) *fail_reason = PLANNER_FAIL_BAD_PARAM;
        return 0;
    }

    /* 起终点自身必须满足方形净空（允许端点容差，例如 HOME/加工点方形净空=0 恰好贴边） */
    const float start_clear_cm = MapCore_PointSquareClearanceCm(start, half_cm);
    const float goal_clear_cm = MapCore_PointSquareClearanceCm(goal, half_cm);
    if (start_clear_cm < endpoint_allowed_cm - 1e-3f || goal_clear_cm < endpoint_allowed_cm - 1e-3f) {
        if (fail_reason != NULL) *fail_reason = PLANNER_FAIL_ENDPOINT;
        return 0;
    }

    /* 起终点本身清障充足时，首末段也必须满足完整要求，不能借端点容差放松 */
    const float start_need_cm = (start_clear_cm < required_cm) ? endpoint_allowed_cm : required_cm;
    const float short_need_cm =
        (start_clear_cm < required_cm || goal_clear_cm < required_cm) ? endpoint_allowed_cm : required_cm;

    /* 短距离直达（标定微调/原地小位移）：按完整 required 校验 */
    if (dist_cm(start, goal) <= PLANNER_DIRECT_PATH_MAX_CM &&
        MapCore_SquareSegmentFree(start, goal, half_cm, required_cm)) {
        out_path[0] = start;
        out_path[1] = goal;
        if (expanded_nodes != NULL) *expanded_nodes = 0;
        return 2;
    }

    /* 初始化 g 值与父方向 */
    for (int i = 0; i < total_cells; i++) {
        s_g_score[i] = MAX_COST_INF;
        s_came_from_dir[i] = 0xFF;
    }
    if (expanded_nodes != NULL) *expanded_nodes = 0;

    PriorityQueue pq;
    pq_init(&pq);

    int goal_idx = goal_gx * grid + goal_gy;
    s_g_score[goal_idx] = 0;
    if (!pq_push(&pq, (uint16_t)goal_idx, 0)) {
        return 0;
    }

    /* 端点豁免区（欧氏距离，米） */
    const float endpoint_zone_m = PLANNER_ENDPOINT_ZONE_CM * 0.01f;
    const float zone_sq_m2 = endpoint_zone_m * endpoint_zone_m;

    uint16_t curr_idx = 0;
    uint16_t curr_f = 0;
    int found_x = -1;
    int found_y = -1;

    while (pq_pop(&pq, &curr_idx, &curr_f)) {
        if (expanded_nodes != NULL) (*expanded_nodes)++;

        int cx = (int)curr_idx / grid;
        int cy = (int)curr_idx % grid;

        /* 收尾格必须与精确起点直线可通行；若收尾格就是终点格，还需校验两点直达 */
        if (abs(cx - start_gx) <= ASTAR_START_TOL_GRIDS && abs(cy - start_gy) <= ASTAR_START_TOL_GRIDS) {
            bool at_goal_cell = (cx == goal_gx && cy == goal_gy);
            bool accept = MapCore_SquareSegmentFree(start, cell_center(cx, cy), half_cm, start_need_cm);
            if (accept && at_goal_cell) {
                accept = MapCore_SquareSegmentFree(start, goal, half_cm, short_need_cm);
            }
            if (accept) {
                found_x = cx;
                found_y = cy;
                break;
            }
        }

        /* 反向种子点是终点格：第一条边从精确终点出发校验；起终点邻域按端点容差放宽 */
        bool curr_is_seed = (curr_idx == (uint16_t)goal_idx);
        Point2D curr_pt = curr_is_seed ? goal : cell_center(cx, cy);
        bool curr_near_start = (dist_sq_m(curr_pt, start) <= zone_sq_m2);
        bool curr_near_goal = (dist_sq_m(curr_pt, goal) <= zone_sq_m2);

        for (int i = 0; i < 8; i++) {
            int nx = cx + s_dir_x[i];
            int ny = cy + s_dir_y[i];

            if (!map_cell_in_bounds(&g_map, nx, ny)) {
                continue;
            }

            Point2D nb_center = cell_center(nx, ny);
            bool nb_near_start = (dist_sq_m(nb_center, start) <= zone_sq_m2);
            bool nb_near_goal = (dist_sq_m(nb_center, goal) <= zone_sq_m2);
            bool edge_relaxed = (curr_near_start && nb_near_start) || (curr_near_goal && nb_near_goal);
            float node_need_cm = (nb_near_start || nb_near_goal) ? endpoint_allowed_cm : required_cm;
            float edge_need_cm = edge_relaxed ? endpoint_allowed_cm : required_cm;

            float cell_sq_cm = cell_square_clearance_cm(nx, ny, half_cm);
            if (cell_sq_cm < node_need_cm - 1e-3f) {
                continue;
            }

            if (!MapCore_SquareSegmentFree(curr_pt, nb_center, half_cm, edge_need_cm)) {
                continue;
            }

            int32_t step_cost = (int32_t)s_move_cost[i] + (int32_t)square_penalty(cell_sq_cm);
            if (step_cost < 1) {
                step_cost = 1;
            }

            int n_idx = nx * grid + ny;
            uint32_t next_g = (uint32_t)s_g_score[curr_idx] + (uint32_t)step_cost;
            if (next_g > MAX_COST_INF) {
                next_g = MAX_COST_INF;
            }

            if ((uint16_t)next_g < s_g_score[n_idx]) {
                s_came_from_dir[n_idx] = (uint8_t)i;
                s_g_score[n_idx] = (uint16_t)next_g;

                uint16_t h = octile_heuristic(nx, ny, start_gx, start_gy);
                uint32_t f = (uint32_t)next_g + (uint32_t)h + 1U;
                if (f > MAX_COST_INF) {
                    f = MAX_COST_INF;
                }
                if (!pq_push(&pq, (uint16_t)n_idx, (uint16_t)f)) {
                    continue;
                }
            }
        }
    }

    if (found_x < 0) {
        return 0;
    }

    /* 重建（从靠近起点的格开始，向终点回退 = 行进顺序） */
    int count = 0;
    int cx = found_x;
    int cy = found_y;

    while (s_came_from_dir[cx * grid + cy] != 0xFF) {
        if (count + 2 >= out_cap) {
            return 0;
        }
        out_path[1 + count] = cell_center(cx, cy);
        count++;
        uint8_t dir = s_came_from_dir[cx * grid + cy];
        cx -= s_dir_x[dir];
        cy -= s_dir_y[dir];
    }

    if (count <= 0) {
        if (!MapCore_SquareSegmentFree(start, goal, half_cm, short_need_cm)) {
            return 0;
        }
        out_path[0] = start;
        out_path[1] = goal;
        return 2;
    }

    /* 收尾格与精确起点视线不满足时，逐个丢弃（通常已被收尾条件过滤，兜底用） */
    int keep_from = 1;
    while (keep_from <= count && !MapCore_SquareSegmentFree(start, out_path[keep_from], half_cm, start_need_cm)) {
        keep_from++;
    }
    if (keep_from > count) {
        return 0;
    }
    if (keep_from > 1) {
        memmove(&out_path[1], &out_path[keep_from], sizeof(Point2D) * (size_t)(count - keep_from + 1));
        count -= (keep_from - 1);
    }

    out_path[0] = start;
    out_path[1 + count] = goal;
    return count + 2;
}

/* ==========================================================================
 * 路径后处理
 * ========================================================================== */

float Planner_PathLengthCm(const Point2D *path, int len) {
    if (path == NULL || len < 2) {
        return 0.0f;
    }
    float total = 0.0f;
    for (int i = 0; i < len - 1; i++) {
        total += dist_cm(path[i], path[i + 1]);
    }
    return total;
}

int Planner_PathSimplify(Point2D *path, int len, float half_cm, float min_clearance_cm) {
    if (path == NULL || len <= 2) {
        return (len < 0) ? 0 : len;
    }

    int out = 0;
    int i = 0;

    path[out++] = path[0];
    while (i < len - 1) {
        int j = len - 1;
        while (j > i + 1 && !MapCore_SquareSegmentFree(path[i], path[j], half_cm, min_clearance_cm)) {
            j--;
        }
        path[out++] = path[j];
        i = j;
        if (out >= len) {
            break;
        }
    }
    return out;
}

int Planner_PathResample(const Point2D *in, int in_len, Point2D *out, int out_cap, float max_segment_cm) {
    if (in == NULL || out == NULL || in_len <= 0 || out_cap <= 0) {
        return 0;
    }
    if (in_len > out_cap) {
        return 0;
    }
    if (max_segment_cm <= 0.0f || in_len == 1) {
        memcpy(out, in, sizeof(Point2D) * (size_t)in_len);
        return in_len;
    }

    int out_len = 0;
    out[out_len++] = in[0];
    for (int i = 0; i < in_len - 1; i++) {
        float seg_cm = dist_cm(in[i], in[i + 1]);
        int parts = (int)ceilf(seg_cm / max_segment_cm);
        if (parts < 1) parts = 1;
        for (int k = 1; k <= parts; k++) {
            if (out_len >= out_cap) {
                return 0;
            }
            if (k == parts) {
                out[out_len++] = in[i + 1];
            } else {
                float t = (float)k / (float)parts;
                out[out_len].x = in[i].x + (in[i + 1].x - in[i].x) * t;
                out[out_len].y = in[i].y + (in[i + 1].y - in[i].y) * t;
                out_len++;
            }
        }
    }
    return out_len;
}

bool Planner_SquarePathValidate(const Point2D *path,
                                int len,
                                float half_cm,
                                float required_cm,
                                float endpoint_allowed_cm,
                                float *out_min_square_cm,
                                float *out_min_circle_cm) {
    if (path == NULL || len < 2) {
        return false;
    }

    const float base_allowed_cm = required_cm - PLANNER_VALIDATION_TOL_CM;
    const float zone_cm = PLANNER_ENDPOINT_ZONE_CM;
    const float zone_sq_cm2 = zone_cm * zone_cm;
    const Point2D first = path[0];
    const Point2D last = path[len - 1];

    float min_square_cm = 1.0e9f;
    float min_circle_cm = 1.0e9f;
    bool ok = true;

    for (int i = 0; i < len - 1; i++) {
        float seg_cm = dist_cm(path[i], path[i + 1]);
        int steps = (int)ceilf(seg_cm / PLANNER_PATH_SAMPLE_STEP_CM);
        if (steps < 1) steps = 1;

        for (int k = 0; k <= steps; k++) {
            if (k == 0 && i > 0) {
                continue;
            }

            float t = (float)k / (float)steps;
            Point2D p;
            p.x = path[i].x + (path[i + 1].x - path[i].x) * t;
            p.y = path[i].y + (path[i + 1].y - path[i].y) * t;

            float square_cm = MapCore_PointSquareClearanceCm(p, half_cm);
            float circle_cm = MapCore_PointClearanceCm(p);
            if (square_cm < min_square_cm) min_square_cm = square_cm;
            if (circle_cm < min_circle_cm) min_circle_cm = circle_cm;

            float d_start_cm = dist_cm(p, first);
            float d_goal_cm = dist_cm(p, last);
            float allowed_cm = base_allowed_cm;
            if (d_start_cm * d_start_cm <= zone_sq_cm2 || d_goal_cm * d_goal_cm <= zone_sq_cm2) {
                allowed_cm = endpoint_allowed_cm;
            }
            if (square_cm < allowed_cm - 1e-3f) {
                ok = false;
            }
        }
    }

    if (out_min_square_cm != NULL) *out_min_square_cm = min_square_cm;
    if (out_min_circle_cm != NULL) *out_min_circle_cm = min_circle_cm;
    return ok;
}

/* ==========================================================================
 * 航点执行剖面（纯函数）
 * ========================================================================== */

bool Planner_BuildRemainingCm(const Point2D *path, int len, float *out_remaining_cm, int out_cap) {
    if (path == NULL || out_remaining_cm == NULL || len <= 0 || out_cap < len) {
        return false;
    }
    out_remaining_cm[len - 1] = 0.0f;
    for (int i = len - 2; i >= 0; i--) {
        out_remaining_cm[i] = out_remaining_cm[i + 1] + dist_cm(path[i], path[i + 1]);
    }
    return true;
}

float Planner_PointDistanceCm(Point2D a, Point2D b) {
    return dist_cm(a, b);
}

float Planner_PointSegmentDistanceCm(Point2D p, Point2D a, Point2D b) {
    float abx = b.x - a.x;
    float aby = b.y - a.y;
    float apx = p.x - a.x;
    float apy = p.y - a.y;
    float ab2 = abx * abx + aby * aby;
    float t = 0.0f;
    if (ab2 > 1e-9f) {
        t = (apx * abx + apy * aby) / ab2;
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
    }
    float dx = apx - abx * t;
    float dy = apy - aby * t;
    return sqrtf(dx * dx + dy * dy) * 100.0f;
}

float Planner_WaypointSpeedCmS(float remaining_cm, bool tight) {
    float speed = PLANNER_WAYPOINT_SPEED_CM_S;

    if (PLANNER_SLOWDOWN_DIST_CM > 0.0f && remaining_cm < PLANNER_SLOWDOWN_DIST_CM) {
        float ratio = remaining_cm / PLANNER_SLOWDOWN_DIST_CM;
        if (ratio < 0.0f) ratio = 0.0f;
        if (ratio > 1.0f) ratio = 1.0f;
        speed = PLANNER_FINAL_SPEED_CM_S + (PLANNER_WAYPOINT_SPEED_CM_S - PLANNER_FINAL_SPEED_CM_S) * ratio;
    }

    if (tight && speed > PLANNER_TIGHT_SPEED_CM_S) {
        speed = PLANNER_TIGHT_SPEED_CM_S;
    }
    if (speed < PLANNER_FINAL_SPEED_CM_S) {
        speed = PLANNER_FINAL_SPEED_CM_S;
    }
    return speed;
}

float Planner_SegmentSpeedLimitCmS(float seg_cm) {
    float usable_cm = seg_cm - PLANNER_FINAL_ARRIVE_DIST_CM;
    if (usable_cm < 5.0f) {
        usable_cm = 5.0f;
    }

    float v = sqrtf(2.0f * PLANNER_WAYPOINT_DECEL_CM_S2 * usable_cm);
    if (v > PLANNER_WAYPOINT_SPEED_CM_S) {
        v = PLANNER_WAYPOINT_SPEED_CM_S;
    }
    if (v < PLANNER_FINAL_SPEED_CM_S) {
        v = PLANNER_FINAL_SPEED_CM_S;
    }
    return v;
}

uint32_t Planner_WaypointTimeoutMs(float seg_cm, float speed_cm_s) {
    float speed = (speed_cm_s > 1.0f) ? speed_cm_s : 1.0f;
    float estimate_ms = seg_cm / speed * 1000.0f;
    float timeout = (float)PLANNER_WAYPOINT_TIMEOUT_BASE_MS + estimate_ms * PLANNER_WAYPOINT_TIMEOUT_SCALE;
    if (timeout > 60000.0f) timeout = 60000.0f;
    if (timeout < (float)PLANNER_WAYPOINT_TIMEOUT_BASE_MS) timeout = (float)PLANNER_WAYPOINT_TIMEOUT_BASE_MS;
    return (uint32_t)timeout;
}

uint32_t Planner_RotationTimeoutMs(float delta_deg) {
    float deg = fabsf(delta_deg);
    float timeout = (float)PLANNER_ROTATION_TIMEOUT_BASE_MS + deg * (float)PLANNER_ROTATION_TIMEOUT_PER_DEG_MS;
    if (timeout > 60000.0f) timeout = 60000.0f;
    return (uint32_t)timeout;
}

/* ==========================================================================
 * 路径连续化 + 每点方形净空优化
 * ========================================================================== */

/**
 * @brief 线段"内部采样"最小方形净空（不含两端点）
 * @details 端点本身贴障碍（净空=0）时，若把端点也算入最小值，单点移动无法改善
 *          "相邻段最小值"，优化会被钉死在原折线上；排除端点后可逐点把整条直线
 *          逐步推到通道中心（多轮迭代收敛）。
 */
static float segment_interior_square_cm(Point2D a, Point2D b, float half_cm) {
    float seg_cm = dist_cm(a, b);
    int steps = (int)ceilf(seg_cm / PLANNER_PATH_SAMPLE_STEP_CM);
    if (steps < 4) steps = 4;

    float best = 1.0e9f;
    for (int k = 1; k < steps; k++) {
        float t = (float)k / (float)steps;
        Point2D p;
        p.x = a.x + (b.x - a.x) * t;
        p.y = a.y + (b.y - a.y) * t;
        float sq = MapCore_PointSquareClearanceCm(p, half_cm);
        if (sq < best) best = sq;
    }
    if (best > 1.0e8f) {
        best = MapCore_PointSquareClearanceCm(a, half_cm);
    }
    return best;
}

/**
 * @brief 把长段细分 + 逐点局部优化，提升窄通道内的实际方形净空
 * @details A* 在 10cm 栅格上可能只能给出"贴近障碍"的格中心折线（例如 35cm 通道
 *          内可通行中心带仅 5cm，格中心落在带边缘），本函数把路径细分到
 *          <= PLANNER_REFINE_SEG_CM，并对每个中间点在其邻域内搜索使
 *          "相邻两段方形净空最小值"最大的位置（目标达到 required_cm）。
 * @param in         输入路径
 * @param in_len     输入点数
 * @param out        输出路径（可与 in 不同）
 * @param out_cap    输出容量
 * @param half_cm    方形车体投影半长
 * @param target_cm  期望达到的方形净空（cm）；已满足的点跳过优化
 * @return 输出点数；0 表示容量不足
 */
static int refine_path_clearance(const Point2D *in,
                                 int in_len,
                                 Point2D *out,
                                 int out_cap,
                                 float half_cm,
                                 float target_cm) {
    if (in == NULL || out == NULL || in_len < 2 || out_cap < 2) {
        return 0;
    }

    /* 1) 细分长段到 <= PLANNER_REFINE_SEG_CM，保证优化时有足够的中间点 */
    int len = 0;
    out[len++] = in[0];
    for (int i = 0; i < in_len - 1; i++) {
        float seg_cm = dist_cm(in[i], in[i + 1]);
        int parts = (int)ceilf(seg_cm / PLANNER_REFINE_SEG_CM);
        if (parts < 1) parts = 1;
        for (int k = 1; k <= parts; k++) {
            if (len >= out_cap) {
                return 0;
            }
            if (k == parts) {
                out[len++] = in[i + 1];
            } else {
                float t = (float)k / (float)parts;
                out[len].x = in[i].x + (in[i + 1].x - in[i].x) * t;
                out[len].y = in[i].y + (in[i + 1].y - in[i].y) * t;
                len++;
            }
        }
    }

    /* 2) 逐点局部优化（只优化当前净空不足的点，避免无谓开销） */
    const float max_off_m = PLANNER_REFINE_MAX_OFFSET_CM * 0.01f;
    const float step_m = PLANNER_REFINE_STEP_CM * 0.01f;

    for (int pass = 0; pass < PLANNER_REFINE_MAX_PASSES; pass++) {
        for (int i = 1; i < len - 1; i++) {
            float cur = segment_interior_square_cm(out[i - 1], out[i], half_cm);
            float cur2 = segment_interior_square_cm(out[i], out[i + 1], half_cm);
            if (cur2 < cur) cur = cur2;
            if (cur >= target_cm) {
                continue; /* 已达标 */
            }

            Point2D base = out[i];
            Point2D best = base;
            float best_score = cur;
            float best_extra_len = 1.0e9f;

            for (float dx = -max_off_m; dx <= max_off_m + 1e-6f; dx += step_m) {
                for (float dy = -max_off_m; dy <= max_off_m + 1e-6f; dy += step_m) {
                    Point2D cand = {base.x + dx, base.y + dy};
                    float cand_pt_cm = MapCore_PointSquareClearanceCm(cand, half_cm);
                    if (cand_pt_cm < 0.0f) {
                        continue; /* 候选点本身已碰撞 */
                    }
                    float score = segment_interior_square_cm(out[i - 1], cand, half_cm);
                    float score2 = segment_interior_square_cm(cand, out[i + 1], half_cm);
                    if (score2 < score) score = score2;
                    if (cand_pt_cm < score) score = cand_pt_cm; /* 新点自身净空也要计入 */
                    float extra_len = dist_cm(out[i - 1], cand) + dist_cm(cand, out[i + 1]) -
                                      dist_cm(out[i - 1], base) - dist_cm(base, out[i + 1]);
                    if (extra_len < 0.0f) extra_len = 0.0f;
                    if (score > best_score + 1e-3f ||
                        (fabsf(score - best_score) <= 1e-3f && extra_len < best_extra_len)) {
                        best_score = score;
                        best_extra_len = extra_len;
                        best = cand;
                    }
                }
            }
            out[i] = best;
        }
    }

    return len;
}

/**
 * @brief 去掉相邻重复点/零长段（严格保留首点与末点之外的重复去除）
 * @return 去重后点数（可能为 1，表示整条路径退化为一个点）
 */
static int dedupe_path(Point2D *path, int len) {
    if (path == NULL || len < 2) {
        return (len < 0) ? 0 : len;
    }
    int out = 1;
    for (int i = 1; i < len; i++) {
        if (dist_cm(path[out - 1], path[i]) > 0.01f) {
            path[out++] = path[i];
        }
    }
    return out;
}

/**
 * @brief 规划一条"平移路径"：A*（0 净空连通）→ 连续化/净空优化 → 直线化 → 校验
 * @param start               精确起点（地图坐标，米）
 * @param goal                精确终点（地图坐标，米）
 * @param half_cm             方形车体投影半长 (cm)
 * @param required_cm         本 tier 的方形净空要求（cm）
 * @param endpoint_allowed_cm 端点豁免区允许的方形净空（cm）
 * @param out_path            输出路径
 * @param out_cap             输出容量
 * @param expanded_nodes      A* 展开节点数（输出）
 * @param fail_reason         失败原因（输出）
 * @return 路径点数；0 失败
 */
static int plan_translation_path(Point2D start,
                                 Point2D goal,
                                 float half_cm,
                                 float required_cm,
                                 float endpoint_allowed_cm,
                                 Point2D *out_path,
                                 int out_cap,
                                 uint16_t *expanded_nodes,
                                 uint8_t *fail_reason) {
    if (fail_reason != NULL) *fail_reason = PLANNER_FAIL_NO_SOLUTION;

    /* 起终点必须满足端点方形净空要求 */
    if (MapCore_PointSquareClearanceCm(start, half_cm) < endpoint_allowed_cm - 1e-3f ||
        MapCore_PointSquareClearanceCm(goal, half_cm) < endpoint_allowed_cm - 1e-3f) {
        if (fail_reason != NULL) *fail_reason = PLANNER_FAIL_ENDPOINT;
        return 0;
    }

    /* A* 用 0 净空（允许相切）保证栅格连通性；真正的余量由后续连续化+校验保证。
     * 端点容差（可能为负，见 PLANNER_ENDPOINT_PENETRATION_TOL_CM）必须传进搜索，
     * 否则 A* 起终点/收尾检查会按 0 判失败，实际摆放偏差 0~1cm 的任务点无法规划。 */
    int raw_len = plan_segment(
        start, goal, half_cm, 0.0f, endpoint_allowed_cm, s_raw_path, MAX_PATH_LEN, expanded_nodes, fail_reason);
    if (raw_len < 2) {
        return 0;
    }

    /* 连续化/净空优化：使用 s_via_a 作临时缓冲（via 方案在输出到 s_via_a 后才顺序使用） */
    int refined_len = refine_path_clearance(s_raw_path,
                                            raw_len,
                                            s_via_a,
                                            MAX_PATH_LEN,
                                            half_cm,
                                            required_cm + PLANNER_REFINE_TARGET_EXTRA_CM);
    if (refined_len < 2) {
        return 0;
    }

    int simp_len = Planner_PathSimplify(s_via_a, refined_len, half_cm, required_cm);
    if (simp_len < 2) {
        return 0;
    }

    int out_len = finalize_resample(s_via_a, simp_len, out_path, out_cap);
    if (out_len < 2) {
        return 0;
    }

    /* 去掉连续化/直线化可能残留的零长段（相邻重复点会让执行层速度/超时剖面退化） */
    out_len = dedupe_path(out_path, out_len);
    if (out_len < 2) {
        return 0;
    }

    float min_sq = 0.0f;
    float min_circle = 0.0f;
    if (!Planner_SquarePathValidate(
            out_path, out_len, half_cm, required_cm, endpoint_allowed_cm, &min_sq, &min_circle)) {
        return 0;
    }
    return out_len;
}

/* ==========================================================================
 * 朝向方案：旋转安全点查找 / via 绕行
 * ========================================================================== */

/**
 * @brief 在已规划路径上插入第一个旋转安全点
 * @param path      路径数组（会被原地插入一个点；容量必须 > len）
 * @param len       原路径点数
 * @param out_cap   容量
 * @param rot_idx   输出旋转点索引
 * @param rot_clear 输出旋转点欧氏清障
 * @return 新路径点数；0 表示没找到安全点或容量不足
 */
static int insert_rotation_point(Point2D *path, int len, int out_cap, int *rot_idx, float *rot_clear) {
    if (path == NULL || len < 2 || rot_idx == NULL) {
        return 0;
    }
    const float step_cm = PLANNER_ROTATION_SCAN_STEP_CM;

    for (int i = 0; i < len - 1; i++) {
        float seg_cm = dist_cm(path[i], path[i + 1]);
        int steps = (int)ceilf(seg_cm / step_cm);
        if (steps < 1) steps = 1;
        for (int k = 0; k <= steps; k++) {
            if (k == 0) continue; /* 段起点是已有顶点，前一段已覆盖 */
            float t = (float)k / (float)steps;
            Point2D p;
            p.x = path[i].x + (path[i + 1].x - path[i].x) * t;
            p.y = path[i].y + (path[i + 1].y - path[i].y) * t;
            if (!MapCore_RotationSafe(p)) {
                continue;
            }
            if (k == steps) {
                /* t=1 就是已存在的下一个顶点：直接把它设为旋转点，不能再插入（避免重复航点） */
                *rot_idx = i + 1;
                if (rot_clear != NULL) *rot_clear = MapCore_PointClearanceCm(path[i + 1]);
                return len;
            }
            if (len + 1 > out_cap) {
                return 0;
            }
            memmove(&path[i + 2], &path[i + 1], sizeof(Point2D) * (size_t)(len - i - 1));
            path[i + 1] = p;
            *rot_idx = i + 1;
            if (rot_clear != NULL) *rot_clear = MapCore_PointClearanceCm(p);
            return len + 1;
        }
    }
    /* 兜底：终点本身是旋转安全点（路径终点旋转 = 到点后原地转，安全） */
    if (MapCore_RotationSafe(path[len - 1])) {
        *rot_idx = len - 1;
        if (rot_clear != NULL) *rot_clear = MapCore_PointClearanceCm(path[len - 1]);
        return len;
    }
    return 0;
}

/** @brief via 旋转点候选 */
typedef struct {
    Point2D p;
    float detour_cm;
} RotationCandidate_t;

/**
 * @brief 路径上没有安全旋转点时：找附近安全点作为 via 点绕行完成朝向切换
 */
static int plan_via_rotation(Point2D start,
                             Point2D goal,
                             float half_cm,
                             float required_cm,
                             float endpoint_allowed_cm,
                             Point2D *out_path,
                             int out_cap,
                             uint16_t *expanded_nodes,
                             int *rot_idx,
                             float *rot_clear) {
    const int max_cand = PLANNER_ROTATION_VIA_MAX_CANDIDATES;
    RotationCandidate_t cands[PLANNER_ROTATION_VIA_MAX_CANDIDATES];
    int cand_count = 0;
    const float direct_cm = dist_cm(start, goal);

    for (int gx = 0; gx < g_map.grid_size; gx++) {
        for (int gy = 0; gy < g_map.grid_size; gy++) {
            Point2D p = cell_center(gx, gy);
            if (!MapCore_RotationSafe(p)) {
                continue;
            }
            float detour = dist_cm(start, p) + dist_cm(p, goal) - direct_cm;
            if (detour < 0.0f) detour = 0.0f;

            if (cand_count < max_cand) {
                int pos = cand_count;
                while (pos > 0 && cands[pos - 1].detour_cm > detour) {
                    cands[pos] = cands[pos - 1];
                    pos--;
                }
                cands[pos].p = p;
                cands[pos].detour_cm = detour;
                cand_count++;
            } else if (detour < cands[max_cand - 1].detour_cm) {
                int pos = max_cand - 1;
                while (pos > 0 && cands[pos - 1].detour_cm > detour) {
                    cands[pos] = cands[pos - 1];
                    pos--;
                }
                cands[pos].p = p;
                cands[pos].detour_cm = detour;
            }
        }
    }

    uint16_t nodes_a = 0;
    uint16_t nodes_b = 0;

    for (int c = 0; c < cand_count; c++) {
        const Point2D p = cands[c].p;
        uint8_t seg_fail = PLANNER_FAIL_NO_SOLUTION;
        int len_a = plan_translation_path(
            start, p, half_cm, required_cm, endpoint_allowed_cm, s_via_a, MAX_PATH_LEN, &nodes_a, &seg_fail);
        if (len_a < 2 || len_a > out_cap) {
            continue;
        }
        /* 先落盘第一段：后续 plan_translation_path 会复用 s_via_a 作临时缓冲；
         * 第二段直接写入 out_path 尾部（起点 p 与第一段末点重合，从 len_a-1 开始写） */
        memcpy(out_path, s_via_a, sizeof(Point2D) * (size_t)len_a);

        int len_b = plan_translation_path(p,
                                          goal,
                                          half_cm,
                                          required_cm,
                                          endpoint_allowed_cm,
                                          &out_path[len_a - 1],
                                          out_cap - (len_a - 1),
                                          &nodes_b,
                                          &seg_fail);
        if (len_b < 2) {
            continue;
        }

        int total = len_a + len_b - 1;
        if (total > out_cap) {
            continue;
        }

        float min_sq = 0.0f;
        float min_circle = 0.0f;
        if (!Planner_SquarePathValidate(
                out_path, total, half_cm, required_cm, endpoint_allowed_cm, &min_sq, &min_circle)) {
            continue;
        }

        *rot_idx = len_a - 1;
        if (rot_clear != NULL) *rot_clear = MapCore_PointClearanceCm(p);
        if (expanded_nodes != NULL) *expanded_nodes = (uint16_t)(nodes_a + nodes_b);
        return total;
    }

    return 0;
}

/* ==========================================================================
 * 单个安全级别的完整规划（含朝向/旋转方案）
 * ========================================================================== */

/** @brief 填充统计中的路径通用字段（length 由调用方按输出路径计算） */
static void fill_path_stats(PlannerPathStats_t *stats,
                            int len,
                            uint16_t expanded_nodes,
                            float min_square_cm,
                            float min_circle_cm) {
    if (stats == NULL) {
        return;
    }
    stats->waypoints = (uint16_t)len;
    stats->expanded_nodes = expanded_nodes;
    stats->min_square_clearance_cm = min_square_cm;
    stats->min_clearance_cm = min_circle_cm;
}

/** @brief 拷贝路径到输出（容量检查） */
static int copy_path_to_out(const Point2D *src, int len, Point2D *out_path, int out_cap) {
    if (len <= 0 || out_path == NULL || out_cap < len) {
        return 0;
    }
    memcpy(out_path, src, sizeof(Point2D) * (size_t)len);
    return len;
}

/** @brief 重采样（若配置关闭则原地保留，不做拷贝） */
static int finalize_resample(Point2D *path, int len, Point2D *out_path, int out_cap) {
    if (PLANNER_MAX_SEGMENT_CM > 0.0f) {
        return Planner_PathResample(path, len, out_path, out_cap, PLANNER_MAX_SEGMENT_CM);
    }
    if (path != out_path) {
        return copy_path_to_out(path, len, out_path, out_cap);
    }
    return len;
}

/**
 * @brief 在一个安全级别内规划完整方案（路径 + 旋转点）
 * @return 路径点数；0 失败（fail_reason 输出原因）
 */
static int plan_one_tier(Point2D start,
                         Point2D goal,
                         float start_yaw_deg,
                         float goal_yaw_deg,
                         float required_cm,
                         float endpoint_allowed_cm,
                         Point2D *out_path,
                         int out_cap,
                         PlannerPathStats_t *stats,
                         uint8_t *fail_reason) {
    const float half_start_cm = Planner_HeadingFootprintHalfCm(start_yaw_deg);
    const float half_goal_cm = Planner_HeadingFootprintHalfCm(goal_yaw_deg);
    const float half_max_cm =
        (half_start_cm > half_goal_cm) ? half_start_cm : half_goal_cm; /* >= 15 */

    const float yaw_delta = yaw_wrap_deg(goal_yaw_deg - start_yaw_deg);
    const bool need_rotation = fabsf(yaw_delta) > PLANNER_HEADING_DEADBAND_DEG;

    uint16_t expanded_nodes = 0;
    uint8_t seg_fail = PLANNER_FAIL_NO_SOLUTION;
    float min_sq = 0.0f;
    float min_circle = 0.0f;
    int rot_idx = (int)PLANNER_ROTATE_NONE;
    float rot_clear = -1.0f;

    if (!need_rotation) {
        /* 不需要旋转：整条路径按 max(H(start), H(goal)) 保守规划，平移朝向统一为目标 yaw */
        int len = plan_translation_path(
            start, goal, half_max_cm, required_cm, endpoint_allowed_cm, out_path, out_cap, &expanded_nodes, &seg_fail);
        if (len < 2) {
            if (fail_reason != NULL) *fail_reason = seg_fail;
            return 0;
        }
        (void)Planner_SquarePathValidate(
            out_path, len, half_max_cm, required_cm, endpoint_allowed_cm, &min_sq, &min_circle);
        if (stats != NULL) {
            fill_path_stats(stats, len, expanded_nodes, min_sq, min_circle);
            stats->length_cm = Planner_PathLengthCm(out_path, len);
            stats->rotation_index = PLANNER_ROTATE_NONE;
            stats->heading_before_deg = goal_yaw_deg;
            stats->heading_after_deg = goal_yaw_deg;
            stats->rotation_clearance_cm = -1.0f;
        }
        return len;
    }

    /* ---- 需要改变朝向 ---- */

    /* 情况 1：起点本身就是旋转安全点 → 先在起点原地转好，再按目标朝向平移 */
    if (MapCore_RotationSafe(start)) {
        int len = plan_translation_path(
            start, goal, half_goal_cm, required_cm, endpoint_allowed_cm, out_path, out_cap, &expanded_nodes, &seg_fail);
        if (len >= 2) {
            (void)Planner_SquarePathValidate(
                out_path, len, half_goal_cm, required_cm, endpoint_allowed_cm, &min_sq, &min_circle);
            if (stats != NULL) {
                fill_path_stats(stats, len, expanded_nodes, min_sq, min_circle);
                stats->length_cm = Planner_PathLengthCm(out_path, len);
                stats->rotation_index = 0;
                stats->heading_before_deg = start_yaw_deg;
                stats->heading_after_deg = goal_yaw_deg;
                stats->rotation_clearance_cm = MapCore_PointClearanceCm(start);
            }
            return len;
        }
        /* 起点可旋转但按目标朝向规划失败：继续尝试"到终点再转"/保守半长 */
    }

    /* 情况 1b：终点本身旋转安全 → 整条路径按起点朝向规划，到终点后再原地转到目标朝向
     * （适合目标 yaw 非 90°倍数、但路径要经过窄通道的情况：行驶中保持起点朝向，
     *   终点再转，避免用目标朝向的方形投影把窄通道堵死） */
    if (MapCore_RotationSafe(goal)) {
        float half_pre_cm = half_start_cm;
        if (half_pre_cm < PLANNER_BODY_HALF_CM) half_pre_cm = PLANNER_BODY_HALF_CM;
        int len = plan_translation_path(
            start, goal, half_pre_cm, required_cm, endpoint_allowed_cm, out_path, out_cap, &expanded_nodes, &seg_fail);
        if (len >= 2) {
            (void)Planner_SquarePathValidate(
                out_path, len, half_pre_cm, required_cm, endpoint_allowed_cm, &min_sq, &min_circle);
            if (stats != NULL) {
                fill_path_stats(stats, len, expanded_nodes, min_sq, min_circle);
                stats->length_cm = Planner_PathLengthCm(out_path, len);
                stats->rotation_index = (uint16_t)(len - 1);
                stats->heading_before_deg = start_yaw_deg;
                stats->heading_after_deg = goal_yaw_deg;
                stats->rotation_clearance_cm = MapCore_PointClearanceCm(goal);
            }
            return len;
        }
    }

    /* 情况 2：按保守半长规划路径，再在路径上找旋转安全点插入 */
    {
        int len = plan_translation_path(
            start, goal, half_max_cm, required_cm, endpoint_allowed_cm, out_path, out_cap, &expanded_nodes, &seg_fail);
        if (len >= 2) {
            int inserted = insert_rotation_point(out_path, len, out_cap, &rot_idx, &rot_clear);
            if (inserted > 0) {
                len = inserted;
                if (Planner_SquarePathValidate(
                        out_path, len, half_max_cm, required_cm, endpoint_allowed_cm, &min_sq, &min_circle)) {
                    if (stats != NULL) {
                        fill_path_stats(stats, len, expanded_nodes, min_sq, min_circle);
                        stats->length_cm = Planner_PathLengthCm(out_path, len);
                        stats->rotation_index = (uint16_t)rot_idx;
                        stats->heading_before_deg = start_yaw_deg;
                        stats->heading_after_deg = goal_yaw_deg;
                        stats->rotation_clearance_cm = rot_clear;
                    }
                    return len;
                }
            }
        } else if (fail_reason != NULL) {
            *fail_reason = seg_fail;
        }
    }

    /* 情况 3：路径上没有安全旋转点 → 找附近 via 安全点绕行 */
    {
        uint16_t via_nodes = 0;
        int out_len = plan_via_rotation(start,
                                        goal,
                                        half_max_cm,
                                        required_cm,
                                        endpoint_allowed_cm,
                                        out_path,
                                        out_cap,
                                        &via_nodes,
                                        &rot_idx,
                                        &rot_clear);
        if (out_len > 0) {
            if (stats != NULL) {
                float min_sq_via = 0.0f;
                float min_circle_via = 0.0f;
                (void)Planner_SquarePathValidate(
                    out_path, out_len, half_max_cm, required_cm, endpoint_allowed_cm, &min_sq_via, &min_circle_via);
                fill_path_stats(stats, out_len, via_nodes, min_sq_via, min_circle_via);
                stats->length_cm = Planner_PathLengthCm(out_path, out_len);
                stats->rotation_index = (uint16_t)rot_idx;
                stats->heading_before_deg = start_yaw_deg;
                stats->heading_after_deg = goal_yaw_deg;
                stats->rotation_clearance_cm = rot_clear;
            }
            return out_len;
        }
        if (fail_reason != NULL) *fail_reason = PLANNER_FAIL_NO_ROTATION_POINT;
        return 0;
    }
}

int Planner_PlanPathWithYaw(Point2D start,
                            Point2D goal,
                            float start_yaw_deg,
                            float goal_yaw_deg,
                            Point2D *out_path,
                            int out_cap,
                            PlannerPathStats_t *stats) {
    if (stats != NULL) {
        memset(stats, 0, sizeof(*stats));
        stats->rotation_index = PLANNER_ROTATE_NONE;
        stats->rotation_clearance_cm = -1.0f;
    }
    if (out_path == NULL || out_cap < 2 || g_map.grid == NULL || g_map.square_clearance == NULL) {
        if (stats != NULL) stats->fail_reason = PLANNER_FAIL_BAD_PARAM;
        return 0;
    }

    /* 雷达可能刚更新过地图：每次规划前重建距离场 */
    MapCore_RebuildClearance();

    /* 起终点几乎重合：无需路径（旋转方案：起点可转则原地转，否则交给 via 绕行） */
    {
        const float h_start_cm0 = Planner_HeadingFootprintHalfCm(start_yaw_deg);
        const float h_goal_cm0 = Planner_HeadingFootprintHalfCm(goal_yaw_deg);
        const float half_max_cm0 = (h_start_cm0 > h_goal_cm0) ? h_start_cm0 : h_goal_cm0;
        const float yaw_delta0 = yaw_wrap_deg(goal_yaw_deg - start_yaw_deg);
        const bool need_rot0 = fabsf(yaw_delta0) > PLANNER_HEADING_DEADBAND_DEG;
        if (dist_cm(start, goal) < 1.0f && (!need_rot0 || MapCore_RotationSafe(start))) {
            out_path[0] = start;
            out_path[1] = goal;
            const float min_sq0 = MapCore_PointSquareClearanceCm(start, half_max_cm0);
            const float min_circle0 = MapCore_PointClearanceCm(start);
            if (stats != NULL) {
                stats->waypoints = 2;
                stats->expanded_nodes = 0;
                stats->length_cm = 0.0f;
                stats->min_square_clearance_cm = min_sq0;
                stats->min_clearance_cm = min_circle0;
                stats->rotation_index = need_rot0 ? 0 : PLANNER_ROTATE_NONE;
                stats->heading_before_deg = need_rot0 ? start_yaw_deg : goal_yaw_deg;
                stats->heading_after_deg = goal_yaw_deg;
                stats->rotation_clearance_cm = need_rot0 ? min_circle0 : -1.0f;
                stats->tier = (min_sq0 >= PLANNER_SAFE_MARGIN_CM) ? PLANNER_TIER_SOFT : PLANNER_TIER_HARD;
                stats->fail_reason = PLANNER_FAIL_NONE;
            }
            return 2;
        }
    }

    uint8_t last_fail = PLANNER_FAIL_NO_SOLUTION;
    for (int tier = 0; tier < 2; tier++) {
        float required_cm = (tier == PLANNER_TIER_SOFT) ? PLANNER_SAFE_MARGIN_CM : PLANNER_SQUARE_HARD_MARGIN_CM;
        float endpoint_allowed_cm = required_cm - PLANNER_ENDPOINT_SQUARE_TOL_CM;
        if (endpoint_allowed_cm < -PLANNER_ENDPOINT_PENETRATION_TOL_CM) {
            endpoint_allowed_cm = -PLANNER_ENDPOINT_PENETRATION_TOL_CM;
        }
        if (endpoint_allowed_cm > required_cm) endpoint_allowed_cm = required_cm;

        uint8_t fail_reason = PLANNER_FAIL_NO_SOLUTION;
        int len = plan_one_tier(start,
                                goal,
                                start_yaw_deg,
                                goal_yaw_deg,
                                required_cm,
                                endpoint_allowed_cm,
                                out_path,
                                out_cap,
                                stats,
                                &fail_reason);
        if (len > 0) {
            if (stats != NULL) {
                stats->tier = (uint8_t)tier;
                stats->fail_reason = PLANNER_FAIL_NONE;
            }
            return len;
        }
        last_fail = fail_reason;
    }

    if (stats != NULL) stats->fail_reason = last_fail;
    return 0;
}

int Planner_PlanPath(Point2D start, Point2D goal, Point2D *out_path, int out_cap, PlannerPathStats_t *stats) {
    PlannerPathStats_t local;
    return Planner_PlanPathWithYaw(start, goal, 0.0f, 0.0f, out_path, out_cap, (stats != NULL) ? stats : &local);
}

#endif /* PLANNER_ENABLED && PLANNER_PLATFORM == PLATFORM_MCU */
