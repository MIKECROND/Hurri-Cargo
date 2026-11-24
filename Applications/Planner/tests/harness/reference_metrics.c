/**
 * @file reference_metrics.c
 * @brief 规划器评测用的独立几何/指标库实现
 *
 * @details 本文件是“参考真值”实现：只用障碍物的真实世界坐标做欧氏距离计算，
 *          不读取 Planner 的地图网格、不调用 Planner 的碰撞判断，避免自证。
 *          坐标单位统一为 cm，世界坐标（场地坐标），与 HEAD planner_config.h 的米制数据
 *          之间只差 ×100，避免浮点单位混用。
 */

#include "reference_metrics.h"

#include <math.h>
#include <stddef.h>

/* 无有效障碍时的清障距离（足够大且有限，便于打印/比较） */
#define REF_DIST_INF 1.0e9f

/* 弧度转角度 */
#define REF_RAD_TO_DEG 57.29577951308232f

/* 向量长度视为 0 的阈值（cm），用于跳过重合点导致的转角无定义 */
#define REF_EPS_LEN_CM 1.0e-6f

/**
 * @brief 由障碍顶点构造轴对齐矩形（世界坐标 cm）
 * @details 与 HEAD Planner_Init 的矩形填充分组方式一致：每 4 个顶点一组取 min/max。
 */
int ref_build_rects(const float *quad_xy, int point_count, RefRect *out_rects, int max_rects) {
    if (quad_xy == NULL || out_rects == NULL || point_count < 4 || max_rects <= 0) return -1;

    int rect_count = 0;
    for (int i = 0; i + 3 < point_count; i += 4) {
        if (rect_count >= max_rects) return -1;

        float min_x = quad_xy[(i + 0) * 2 + 0];
        float max_x = min_x;
        float min_y = quad_xy[(i + 0) * 2 + 1];
        float max_y = min_y;

        for (int k = 1; k < 4; k++) {
            float px = quad_xy[(i + k) * 2 + 0];
            float py = quad_xy[(i + k) * 2 + 1];
            if (px < min_x) min_x = px;
            if (px > max_x) max_x = px;
            if (py < min_y) min_y = py;
            if (py > max_y) max_y = py;
        }

        out_rects[rect_count].x0 = min_x;
        out_rects[rect_count].y0 = min_y;
        out_rects[rect_count].x1 = max_x;
        out_rects[rect_count].y1 = max_y;
        rect_count++;
    }
    return rect_count;
}

/**
 * @brief 点到单个轴对齐矩形的精确欧氏距离（cm）
 * @details 点到矩形 = 分别对 x、y 方向做“夹到区间外”的差，再取欧氏长度；
 *          点在矩形内部（含边界）时距离为 0。
 */
float ref_point_to_rect_distance(float px, float py, const RefRect *rect) {
    float dx = 0.0f;
    float dy = 0.0f;

    if (px < rect->x0) {
        dx = rect->x0 - px;
    } else if (px > rect->x1) {
        dx = px - rect->x1;
    }
    if (py < rect->y0) {
        dy = rect->y0 - py;
    } else if (py > rect->y1) {
        dy = py - rect->y1;
    }
    return sqrtf(dx * dx + dy * dy);
}

/**
 * @brief 点到一组矩形的最小欧氏距离（cm）
 */
float ref_min_rect_distance_ex(float px, float py, const RefRect *rects, int rect_count, int *out_index) {
    if (out_index != NULL) *out_index = -1;
    if (rects == NULL || rect_count <= 0) return REF_DIST_INF;

    float best = REF_DIST_INF;
    int best_idx = -1;
    for (int i = 0; i < rect_count; i++) {
        float d = ref_point_to_rect_distance(px, py, &rects[i]);
        if (d < best) {
            best = d;
            best_idx = i;
        }
    }
    if (out_index != NULL) *out_index = best_idx;
    return best;
}

/**
 * @brief 点到一组矩形的最小欧氏距离（cm）
 */
float ref_min_rect_distance(float px, float py, const RefRect *rects, int rect_count) {
    return ref_min_rect_distance_ex(px, py, rects, rect_count, NULL);
}

/**
 * @brief 计算每个航点（折线顶点）的清障距离（cm）
 */
int ref_waypoint_clearances(
    const RefRect *rects, int rect_count, const float *path_xy, int point_count, float *out_cm, int capacity) {
    if (path_xy == NULL || out_cm == NULL || point_count <= 0 || capacity < point_count) return -1;

    for (int i = 0; i < point_count; i++) {
        out_cm[i] = ref_min_rect_distance(path_xy[i * 2 + 0], path_xy[i * 2 + 1], rects, rect_count);
    }
    return point_count;
}

/**
 * @brief 计算每个内部航点的转角（度，带符号：正 = 逆时针/左转）
 */
int ref_turn_angles(const float *path_xy, int point_count, float *out_deg, int capacity) {
    if (path_xy == NULL || out_deg == NULL || point_count < 3) return (point_count < 3) ? 0 : -1;

    int turn_count = point_count - 2;
    if (capacity < turn_count) return -1;

    for (int i = 1; i <= point_count - 2; i++) {
        float v1x = path_xy[i * 2 + 0] - path_xy[(i - 1) * 2 + 0];
        float v1y = path_xy[i * 2 + 1] - path_xy[(i - 1) * 2 + 1];
        float v2x = path_xy[(i + 1) * 2 + 0] - path_xy[i * 2 + 0];
        float v2y = path_xy[(i + 1) * 2 + 1] - path_xy[i * 2 + 1];

        float len1 = sqrtf(v1x * v1x + v1y * v1y);
        float len2 = sqrtf(v2x * v2x + v2y * v2y);
        if (len1 <= REF_EPS_LEN_CM || len2 <= REF_EPS_LEN_CM) {
            out_deg[i - 1] = 0.0f;
            continue;
        }

        float cross = v1x * v2y - v1y * v2x;
        float dot = v1x * v2x + v1y * v2y;
        out_deg[i - 1] = atan2f(cross, dot) * REF_RAD_TO_DEG; /* (-180, 180]，正 = 左转 */
    }
    return turn_count;
}

/**
 * @brief 评测一条折线路径（世界坐标 cm）
 * @details 采样策略：每段按 REF_SAMPLE_STEP_CM 分成 steps 份，取 k = 0..steps 共 steps+1 个点
 *          （相邻采样点间距 <= REF_SAMPLE_STEP_CM，段两端点一定被采到），
 *          这样既不会漏掉“采样间隙里的最近点”，也不会因为只测航点而高估清障距离。
 */
bool ref_measure_path(const RefRect *rects,
                      int rect_count,
                      const float *path_xy,
                      int point_count,
                      float goal_x,
                      float goal_y,
                      RefPathMetrics *out,
                      RefSegMetrics *segs,
                      int seg_capacity) {
    if (path_xy == NULL || out == NULL || point_count <= 0 || point_count > REF_MAX_POINTS) return false;

    /* 汇总清零 */
    for (size_t i = 0; i < sizeof(*out); i++) ((unsigned char *)out)[i] = 0;

    out->point_count = point_count;
    out->seg_count = point_count - 1;
    out->min_clearance_cm = REF_DIST_INF;
    out->min_clearance_x_cm = 0.0f;
    out->min_clearance_y_cm = 0.0f;
    out->min_clearance_rect = -1;
    out->min_clearance_seg = -1;
    out->min_clearance_wp = -1;
    out->goal_error_cm = 0.0f;
    out->sample_count = 0;

    /* ===== 航点清障距离：用于按航点统计/找最差航点 ===== */
    float min_wp_clearance = REF_DIST_INF;
    for (int i = 0; i < point_count; i++) {
        float px = path_xy[i * 2 + 0];
        float py = path_xy[i * 2 + 1];
        float d = ref_min_rect_distance(px, py, rects, rect_count);

        if (d < REF_ROBOT_RADIUS_CM) out->waypoints_over_radius++;
        if (d < min_wp_clearance) {
            min_wp_clearance = d;
            out->min_clearance_wp = i;
        }
    }

    /* ===== 逐段采样：段长、段内最小清障、全程最小清障 ===== */
    for (int s = 0; s + 1 < point_count; s++) {
        float ax = path_xy[s * 2 + 0];
        float ay = path_xy[s * 2 + 1];
        float bx = path_xy[(s + 1) * 2 + 0];
        float by = path_xy[(s + 1) * 2 + 1];
        float dx = bx - ax;
        float dy = by - ay;
        float len = sqrtf(dx * dx + dy * dy);

        int steps = (int)(len / REF_SAMPLE_STEP_CM) + 1; /* ceil(len/step)，至少 1 份 */
        if (steps < 1) steps = 1;

        float seg_min = REF_DIST_INF;
        bool seg_over = false;
        for (int k = 0; k <= steps; k++) {
            float t = (float)k / (float)steps;
            float px = ax + dx * t;
            float py = ay + dy * t;
            int nearest_rect = -1;
            float d = ref_min_rect_distance_ex(px, py, rects, rect_count, &nearest_rect);

            out->sample_count++;
            if (d < seg_min) seg_min = d;
            if (d < REF_ROBOT_RADIUS_CM) seg_over = true;
            if (d < out->min_clearance_cm) {
                out->min_clearance_cm = d;
                out->min_clearance_x_cm = px;
                out->min_clearance_y_cm = py;
                out->min_clearance_rect = nearest_rect;
                out->min_clearance_seg = s;
            }
        }

        out->length_cm += len;
        if (seg_over) out->segs_over_radius++;
        if (segs != NULL && s < seg_capacity) {
            segs[s].len_cm = len;
            segs[s].min_clearance_cm = seg_min;
            segs[s].over_radius = seg_over;
        }
    }

    /* 单点路径（point_count == 1）没有段，全程最小清障直接取该航点 */
    if (point_count == 1) {
        out->min_clearance_cm = min_wp_clearance;
        out->min_clearance_x_cm = path_xy[0];
        out->min_clearance_y_cm = path_xy[1];
        out->min_clearance_seg = -1;
    }

    out->collision = (out->min_clearance_cm < REF_ROBOT_RADIUS_CM);

    /* ===== 转角统计 ===== */
    if (point_count >= 3) {
        float turn[REF_MAX_POINTS];
        int turn_count = ref_turn_angles(path_xy, point_count, turn, REF_MAX_POINTS);
        float sum_abs = 0.0f;
        float max_abs = 0.0f;
        for (int i = 0; i < turn_count; i++) {
            float a = fabsf(turn[i]);
            sum_abs += a;
            if (a > max_abs) max_abs = a;
        }
        out->max_turn_deg = max_abs;
        out->sum_turn_deg = sum_abs;
    }

    /* ===== 终点误差：末点应被精确替换成目标点 ===== */
    float ex = path_xy[(point_count - 1) * 2 + 0] - goal_x;
    float ey = path_xy[(point_count - 1) * 2 + 1] - goal_y;
    out->goal_error_cm = sqrtf(ex * ex + ey * ey);

    return true;
}

/* ==========================================================================================
 * v2：方形车体（30×30）独立几何真值——精确 SAT
 * ========================================================================================== */

#define REF_WORLD_HALF_CM 240.0f /* 地图世界坐标范围 [-240,240]cm（48 格 ×10cm） */
#define REF_SAT_COLLISION_EPS_CM 0.02f /* 判定"相交"的 signed 噪声容差 (cm) */
#define REF_DEG_TO_RAD 0.017453292519943295f

typedef struct {
    float x;
    float y;
} RefVec2;

/** @brief 方形四角顶点（世界坐标 cm） */
static void ref_square_corners(float cx, float cy, float heading_deg, float half_cm, RefVec2 out[4]) {
    float rad = heading_deg * REF_DEG_TO_RAD;
    float c = cosf(rad);
    float s = sinf(rad);
    const float lx[4] = {half_cm, -half_cm, -half_cm, half_cm};
    const float ly[4] = {half_cm, half_cm, -half_cm, -half_cm};
    for (int i = 0; i < 4; i++) {
        out[i].x = cx + lx[i] * c - ly[i] * s;
        out[i].y = cy + lx[i] * s + ly[i] * c;
    }
}

/** @brief 轴对齐矩形四角顶点（世界坐标 cm） */
static void ref_rect_corners(const RefRect *r, RefVec2 out[4]) {
    out[0].x = r->x0;
    out[0].y = r->y0;
    out[1].x = r->x1;
    out[1].y = r->y0;
    out[2].x = r->x1;
    out[2].y = r->y1;
    out[3].x = r->x0;
    out[3].y = r->y1;
}

/** @brief 顶点集合在轴 (ax,ay) 上的投影区间 */
static void ref_project(const RefVec2 *pts, int n, float ax, float ay, float *out_min, float *out_max) {
    float mn = pts[0].x * ax + pts[0].y * ay;
    float mx = mn;
    for (int i = 1; i < n; i++) {
        float v = pts[i].x * ax + pts[i].y * ay;
        if (v < mn) mn = v;
        if (v > mx) mx = v;
    }
    *out_min = mn;
    *out_max = mx;
}

/** @brief 点到轴对齐矩形的精确距离（cm，点在矩形内为 0） */
static float ref_point_rect_dist(float px, float py, const RefRect *r) {
    RefRect tmp = *r;
    return ref_point_to_rect_distance(px, py, &tmp);
}

/** @brief 点到线段距离（cm） */
static float ref_point_seg_dist(float px, float py, float ax, float ay, float bx, float by) {
    float abx = bx - ax;
    float aby = by - ay;
    float apx = px - ax;
    float apy = py - ay;
    float ab2 = abx * abx + aby * aby;
    float t = 0.0f;
    if (ab2 > 1e-9f) {
        t = (apx * abx + apy * aby) / ab2;
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
    }
    float dx = apx - abx * t;
    float dy = apy - aby * t;
    return sqrtf(dx * dx + dy * dy);
}

/**
 * @brief 方形车体与单个轴对齐矩形的带符号净空（精确 SAT）
 * @return >0 分离（欧氏距离）；0 相切；<0 相交（-MTV 侵入深度）
 */
static float ref_square_rect_signed(float cx, float cy, float heading_deg, float half_cm, const RefRect *r) {
    RefVec2 sq[4];
    RefVec2 rc[4];
    ref_square_corners(cx, cy, heading_deg, half_cm, sq);
    ref_rect_corners(r, rc);

    float rad = heading_deg * REF_DEG_TO_RAD;
    float c = cosf(rad);
    float s = sinf(rad);
    const float axes[4][2] = {{1.0f, 0.0f}, {0.0f, 1.0f}, {c, s}, {-s, c}};

    float min_overlap = 1.0e9f;
    for (int a = 0; a < 4; a++) {
        float ax = axes[a][0];
        float ay = axes[a][1];
        float s_min, s_max, r_min, r_max;
        ref_project(sq, 4, ax, ay, &s_min, &s_max);
        ref_project(rc, 4, ax, ay, &r_min, &r_max);
        float overlap = (s_max < r_max ? s_max : r_max) - (s_min > r_min ? s_min : r_min);
        if (overlap <= 0.0f) {
            /* 找到分离轴：两个凸多边形不相交，下面计算精确欧氏距离 */
            float best = 1.0e9f;
            for (int i = 0; i < 4; i++) {
                float d = ref_point_rect_dist(sq[i].x, sq[i].y, r);
                if (d < best) best = d;
            }
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    const RefVec2 *p0 = &sq[j];
                    const RefVec2 *p1 = &sq[(j + 1) & 3];
                    float d = ref_point_seg_dist(rc[i].x, rc[i].y, p0->x, p0->y, p1->x, p1->y);
                    if (d < best) best = d;
                }
            }
            return best;
        }
        if (overlap < min_overlap) min_overlap = overlap;
    }
    return -min_overlap;
}

/** @brief 方形车体相对地图边界（世界 [-240,240]²）的带符号净空 */
static float ref_square_boundary_signed(float cx, float cy, float heading_deg, float half_cm) {
    float rad = heading_deg * REF_DEG_TO_RAD;
    float c = fabsf(cosf(rad));
    float s = fabsf(sinf(rad));
    float ext = half_cm * (c + s); /* 旋转方形的单轴半宽（x/y 相同） */
    float left = cx + REF_WORLD_HALF_CM - ext;
    float right = REF_WORLD_HALF_CM - cx - ext;
    float bottom = cy + REF_WORLD_HALF_CM - ext;
    float top = REF_WORLD_HALF_CM - cy - ext;
    float best = left;
    if (right < best) best = right;
    if (bottom < best) best = bottom;
    if (top < best) best = top;
    return best;
}

float ref_square_scene_clearance_cm(
    float px, float py, float heading_deg, float half_cm, const RefRect *rects, int rect_count) {
    float best = ref_square_boundary_signed(px, py, heading_deg, half_cm);
    if (rects != NULL) {
        for (int i = 0; i < rect_count; i++) {
            float d = ref_square_rect_signed(px, py, heading_deg, half_cm, &rects[i]);
            if (d < best) best = d;
        }
    }
    return best;
}

float ref_point_euclid_clearance_cm(float px, float py, const RefRect *rects, int rect_count) {
    float best = ref_min_rect_distance(px, py, rects, rect_count);
    float dx_l = px + REF_WORLD_HALF_CM;
    float dx_r = REF_WORLD_HALF_CM - px;
    float dy_b = py + REF_WORLD_HALF_CM;
    float dy_t = REF_WORLD_HALF_CM - py;
    float dx = (dx_l < dx_r) ? dx_l : dx_r;
    float dy = (dy_b < dy_t) ? dy_b : dy_t;
    float boundary = (dx < dy) ? dx : dy;
    return (boundary < best) ? boundary : best;
}

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
                             RefSquarePathMetrics *out) {
    if (path_xy == NULL || seg_heading_deg == NULL || out == NULL || point_count < 2 || heading_count < 1) {
        return false;
    }
    if (heading_count != 1 && heading_count < point_count - 1) return false;

    for (size_t i = 0; i < sizeof(*out); i++) ((unsigned char *)out)[i] = 0;
    out->point_count = point_count;
    out->seg_count = point_count - 1;
    out->min_general_signed_cm = 1.0e9f;
    out->min_endpoint_signed_cm = 1.0e9f;
    out->min_signed_cm = 1.0e9f;
    out->max_penetration_cm = 0.0f;

    const float zone_sq = endpoint_zone_cm * endpoint_zone_cm;
    const float pen_limit = endpoint_pen_tol_cm + REF_SAT_COLLISION_EPS_CM;

    for (int i = 0; i + 1 < point_count; i++) {
        float ax = path_xy[i * 2 + 0];
        float ay = path_xy[i * 2 + 1];
        float bx = path_xy[(i + 1) * 2 + 0];
        float by = path_xy[(i + 1) * 2 + 1];
        float heading = (heading_count == 1) ? seg_heading_deg[0] : seg_heading_deg[i];
        float len = sqrtf((bx - ax) * (bx - ax) + (by - ay) * (by - ay));
        int steps = (int)ceilf(len / 1.0f); /* 1cm 采样 */
        if (steps < 1) steps = 1;

        for (int k = 0; k <= steps; k++) {
            if (k == 0 && i > 0) continue; /* 段端点已在前一段采过 */
            float t = (float)k / (float)steps;
            float px = ax + (bx - ax) * t;
            float py = ay + (by - ay) * t;
            float signed_cm = ref_square_scene_clearance_cm(px, py, heading, half_cm, rects, rect_count);

            out->sample_count++;
            if (signed_cm < out->min_signed_cm) {
                out->min_signed_cm = signed_cm;
                out->min_signed_x_cm = px;
                out->min_signed_y_cm = py;
            }
            float penetration = (signed_cm < 0.0f) ? -signed_cm : 0.0f;
            if (penetration > out->max_penetration_cm) {
                out->max_penetration_cm = penetration;
                out->max_penetration_x_cm = px;
                out->max_penetration_y_cm = py;
            }

            float dxs = px - start_x;
            float dys = py - start_y;
            float dxg = px - goal_x;
            float dyg = py - goal_y;
            bool endpoint_zone = (dxs * dxs + dys * dys <= zone_sq) || (dxg * dxg + dyg * dyg <= zone_sq);

            if (endpoint_zone) {
                if (signed_cm < out->min_endpoint_signed_cm) out->min_endpoint_signed_cm = signed_cm;
                if (signed_cm < -pen_limit) out->endpoint_penetration_violations++;
            } else {
                if (signed_cm < out->min_general_signed_cm) out->min_general_signed_cm = signed_cm;
                if (signed_cm < -REF_SAT_COLLISION_EPS_CM) out->general_collision_samples++;
            }
        }
        out->length_cm += len;
    }

    return true;
}
