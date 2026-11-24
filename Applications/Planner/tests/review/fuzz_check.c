/**
 * @file fuzz_check.c
 * @brief [v2 方形车体] 独立对抗性评审：模糊测试 + 边界/专项 + 地图一致性
 *
 * @details 独立几何原则：
 *          - 仅从 HEAD 6413fac planner_config.h 复制障碍物世界坐标（cm）/动态格，自己实现：
 *              * 点到 AABB / 地图边界欧氏距离；
 *              * 方形车体（30x30，half=15cm）Minkowski/带符号净空（模型口径独立实现）；
 *              * 30x30 旋转正方形 vs AABB 矩形/地图边界的 SAT 精确碰撞（含穿透深度）；
 *              * 旋转安全点：中心欧氏清障 >= 23.22cm。
 *          - 只通过公开 API 接触被测实现：MapCore_Init/RebuildClearance/查询，
 *            Planner_PlanPathWithYaw/Planner_PlanPath。
 *          - 按“执行语义”校验规划结果：按 rotation_index/heading_before/after 取每段 yaw，
 *            逐 1cm 采样 SAT + 独立方形净空，一般路段不允许碰撞，端点 25cm 豁免区允许 ≤1cm 侵入。
 *
 * 编译（%TEMP%）：
 *   gcc -std=c99 -O2 -Wall -Wextra -DPLANNER_HOST_TEST -msse2 -mfpmath=sse \
 *       -I Applications/Planner -I Applications/Planner/utils \
 *       Applications/Planner/tests/review/fuzz_check.c \
 *       Applications/Planner/utils/Cstar_map_core.c \
 *       Applications/Planner/utils/planner_algo.c \
 *       Applications/Planner/utils/priority_queue.c -o %TEMP%/fuzz_check.exe -lm
 *
 * 用法：fuzz_check.exe [module]
 *   module = all（默认）| map | fuzz | dynamic | targeted | timing
 */

#define PLANNER_HOST_TEST 1

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "Cstar_map_core.h"
#include "planner_algo.h"
#include "planner_config.h"

/* ==========================================================================================
 * 1. 独立几何真值（世界坐标 cm；地图坐标 = 世界 + 240cm）
 * ========================================================================================== */

#define REV_MAP_OFF_CM 240.0f
#define REV_MAP_SIDE_CM 480.0f
#define REV_HALF_CM PLANNER_BODY_HALF_CM
#define REV_HALF_DIAG_CM PLANNER_BODY_HALF_DIAGONAL_CM
#define REV_ROT_CLEAR_CM PLANNER_ROTATION_CLEARANCE_CM
#define REV_SAT_STEP_CM 1.0f
#define REV_ENDPOINT_ZONE_CM PLANNER_ENDPOINT_ZONE_CM

#define REV_MAX_STATIC_RECTS 32
#define REV_MAX_DYN_RECTS 1024

typedef struct {
    float x0, y0, x1, y1;
} RevRect;

/* 与 HEAD 6413fac planner_config.h 完全一致的障碍物世界坐标（cm），每 4 点一组 */
static const float s_zone1_quads[] = {
    35.0f, 175.0f, 35.0f, 120.0f, 90.0f, 175.0f, 90.0f, 120.0f,
    120.0f, 175.0f, 175.0f, 175.0f, 120.0f, 120.0f, 175.0f, 120.0f,
    35.0f, 90.0f, 90.0f, 90.0f, 35.0f, 35.0f, 90.0f, 35.0f,
    120.0f, 90.0f, 175.0f, 90.0f, 120.0f, 35.0f, 175.0f, 35.0f,
    76.0f, 210.0f, 134.0f, 210.0f, 76.0f, 225.0f, 134.0f, 225.0f,
    210.0f, 134.0f, 225.0f, 134.0f, 210.0f, 76.0f, 225.0f, 76.0f,
    -25.0f, 235.0f, -25.0f, -35.0f, -15.0f, 235.0f, -15.0f, -35.0f,
    225.0f, 235.0f, 225.0f, -35.0f, 235.0f, 235.0f, 235.0f, -35.0f,
    -15.0f, 235.0f, -15.0f, 225.0f, 225.0f, 235.0f, 225.0f, 225.0f,
    -15.0f, -25.0f, -15.0f, -35.0f, 225.0f, -25.0f, 225.0f, -35.0f,
};

static const float s_zone2_quads[] = {
    35.0f, 35.0f, 90.0f, 35.0f, 35.0f, 90.0f, 90.0f, 90.0f,
    35.0f, 120.0f, 35.0f, 175.0f, 90.0f, 120.0f, 90.0f, 175.0f,
    120.0f, 35.0f, 120.0f, 90.0f, 175.0f, 35.0f, 175.0f, 90.0f,
    120.0f, 120.0f, 120.0f, 175.0f, 175.0f, 120.0f, 175.0f, 175.0f,
    0.0f, 76.0f, 0.0f, 134.0f, -15.0f, 76.0f, -15.0f, 134.0f,
    76.0f, 210.0f, 76.0f, 225.0f, 134.0f, 210.0f, 134.0f, 225.0f,
    -25.0f, -25.0f, 235.0f, -25.0f, -25.0f, -15.0f, 235.0f, -15.0f,
    -25.0f, 225.0f, 235.0f, 225.0f, -25.0f, 235.0f, 235.0f, 235.0f,
    -25.0f, -15.0f, -15.0f, -15.0f, -25.0f, 225.0f, -15.0f, 225.0f,
    225.0f, -15.0f, 235.0f, -15.0f, 225.0f, 225.0f, 235.0f, 225.0f,
};

static RevRect s_static_rects[REV_MAX_STATIC_RECTS];
static int s_static_rect_count = 0;
static RevRect s_dyn_rects[REV_MAX_DYN_RECTS];
static int s_dyn_rect_count = 0;

/* 被测地图缓冲区（额外 canary 防越界写） */
#define REV_GUARD 64
static uint8_t s_grid_storage[MAX_MAP_BYTES + REV_GUARD];
static uint8_t s_clear_storage[MAX_CELLS + REV_GUARD];
static int8_t s_square_storage[MAX_CELLS + REV_GUARD];

static int g_problems = 0;

/* ==========================================================================================
 * 2. 独立几何工具
 * ========================================================================================== */

static void rev_build_static_rects(const float *quads, int float_count) {
    s_static_rect_count = 0;
    for (int i = 0; i + 7 < float_count; i += 8) {
        if (s_static_rect_count >= REV_MAX_STATIC_RECTS) break;
        float min_x = quads[i + 0], max_x = quads[i + 0];
        float min_y = quads[i + 1], max_y = quads[i + 1];
        for (int k = 2; k < 8; k += 2) {
            if (quads[i + k] < min_x) min_x = quads[i + k];
            if (quads[i + k] > max_x) max_x = quads[i + k];
            if (quads[i + k + 1] < min_y) min_y = quads[i + k + 1];
            if (quads[i + k + 1] > max_y) max_y = quads[i + k + 1];
        }
        s_static_rects[s_static_rect_count].x0 = min_x;
        s_static_rects[s_static_rect_count].y0 = min_y;
        s_static_rects[s_static_rect_count].x1 = max_x;
        s_static_rects[s_static_rect_count].y1 = max_y;
        s_static_rect_count++;
    }
}

static float rev_point_rect_dist(float px, float py, const RevRect *r) {
    float dx = 0.0f, dy = 0.0f;
    if (px < r->x0) dx = r->x0 - px;
    else if (px > r->x1) dx = px - r->x1;
    if (py < r->y0) dy = r->y0 - py;
    else if (py > r->y1) dy = py - r->y1;
    return sqrtf(dx * dx + dy * dy);
}

/* 点到扩张矩形的带符号净空 cm：内部为 -min穿透，外部为 +欧氏距离 */
static float rev_rect_signed(float px, float py, const RevRect *r, float half) {
    float ex0 = r->x0 - half, ey0 = r->y0 - half;
    float ex1 = r->x1 + half, ey1 = r->y1 + half;
    if (px >= ex0 && px <= ex1 && py >= ey0 && py <= ey1) {
        float d = px - ex0;
        if (ex1 - px < d) d = ex1 - px;
        if (py - ey0 < d) d = py - ey0;
        if (ey1 - py < d) d = ey1 - py;
        return -d;
    }
    float dx = 0.0f, dy = 0.0f;
    if (px < ex0) dx = ex0 - px;
    else if (px > ex1) dx = px - ex1;
    if (py < ey0) dy = ey0 - py;
    else if (py > ey1) dy = py - ey1;
    return sqrtf(dx * dx + dy * dy);
}

/* 独立方形带符号净空（世界 cm）：静态矩形 + 动态格 + 地图边界 */
static float rev_square_clearance(float px, float py, float half) {
    float inside = -1.0e9f;
    float outside = 1.0e9f;
    for (int i = 0; i < s_static_rect_count; i++) {
        float v = rev_rect_signed(px, py, &s_static_rects[i], half);
        if (v < 0.0f) {
            if (-v > inside) inside = -v;
        } else if (v < outside) {
            outside = v;
        }
    }
    for (int i = 0; i < s_dyn_rect_count; i++) {
        float v = rev_rect_signed(px, py, &s_dyn_rects[i], half);
        if (v < 0.0f) {
            if (-v > inside) inside = -v;
        } else if (v < outside) {
            outside = v;
        }
    }
    float val = (inside >= 0.0f) ? -inside : outside;
    float mx = px + REV_MAP_OFF_CM;
    float my = py + REV_MAP_OFF_CM;
    float b = mx;
    if (my < b) b = my;
    if (REV_MAP_SIDE_CM - mx < b) b = REV_MAP_SIDE_CM - mx;
    if (REV_MAP_SIDE_CM - my < b) b = REV_MAP_SIDE_CM - my;
    float bc = b - half;
    return (bc < val) ? bc : val;
}

/* 独立欧氏清障（世界 cm）：静态矩形 + 动态格 + 地图边界（中心点） */
static float rev_point_clearance(float px, float py) {
    float best = 1.0e9f;
    for (int i = 0; i < s_static_rect_count; i++) {
        float d = rev_point_rect_dist(px, py, &s_static_rects[i]);
        if (d < best) best = d;
    }
    for (int i = 0; i < s_dyn_rect_count; i++) {
        float d = rev_point_rect_dist(px, py, &s_dyn_rects[i]);
        if (d < best) best = d;
    }
    float mx = px + REV_MAP_OFF_CM, my = py + REV_MAP_OFF_CM;
    float b = mx;
    if (my < b) b = my;
    if (REV_MAP_SIDE_CM - mx < b) b = REV_MAP_SIDE_CM - mx;
    if (REV_MAP_SIDE_CM - my < b) b = REV_MAP_SIDE_CM - my;
    if (b < best) best = b;
    return best;
}

/* SAT：30x30 正方形（yaw） vs AABB 矩形，返回穿透深度 (>0 碰撞) */
static float rev_sat_square_rect(float px, float py, float yaw_deg, float half, const RevRect *r) {
    float c = cosf(yaw_deg * 3.14159265358979f / 180.0f);
    float s = sinf(yaw_deg * 3.14159265358979f / 180.0f);
    float sq[4][2];
    const float sx[4] = {-1.0f, 1.0f, 1.0f, -1.0f};
    const float sy[4] = {-1.0f, -1.0f, 1.0f, 1.0f};
    for (int i = 0; i < 4; i++) {
        float lx = sx[i] * half, ly = sy[i] * half;
        sq[i][0] = px + lx * c - ly * s;
        sq[i][1] = py + lx * s + ly * c;
    }
    const float rx[4] = {r->x0, r->x1, r->x1, r->x0};
    const float ry[4] = {r->y0, r->y0, r->y1, r->y1};
    const float axes[4][2] = {{1.0f, 0.0f}, {0.0f, 1.0f}, {c, s}, {-s, c}};
    float min_overlap = 1.0e9f;
    for (int a = 0; a < 4; a++) {
        float ax = axes[a][0], ay = axes[a][1];
        float smin = 1.0e9f, smax = -1.0e9f, rmin = 1.0e9f, rmax = -1.0e9f;
        for (int i = 0; i < 4; i++) {
            float ps = sq[i][0] * ax + sq[i][1] * ay;
            if (ps < smin) smin = ps;
            if (ps > smax) smax = ps;
            float pr = rx[i] * ax + ry[i] * ay;
            if (pr < rmin) rmin = pr;
            if (pr > rmax) rmax = pr;
        }
        float overlap = (smax < rmax ? smax : rmax) - (smin > rmin ? smin : rmin);
        if (overlap <= 0.0f) return 0.0f; /* 分离轴 */
        if (overlap < min_overlap) min_overlap = overlap;
    }
    return min_overlap;
}

/* 位姿穿透深度（世界 cm）：方形 vs 静态/动态矩形/地图边界，0 表示无碰撞 */
static float rev_pose_penetration(float px, float py, float yaw_deg) {
    float pen = 0.0f;
    for (int i = 0; i < s_static_rect_count; i++) {
        float p = rev_sat_square_rect(px, py, yaw_deg, REV_HALF_CM, &s_static_rects[i]);
        if (p > pen) pen = p;
    }
    for (int i = 0; i < s_dyn_rect_count; i++) {
        float p = rev_sat_square_rect(px, py, yaw_deg, REV_HALF_CM, &s_dyn_rects[i]);
        if (p > pen) pen = p;
    }
    float mx = px + REV_MAP_OFF_CM, my = py + REV_MAP_OFF_CM;
    float dmin = mx;
    if (my < dmin) dmin = my;
    if (REV_MAP_SIDE_CM - mx < dmin) dmin = REV_MAP_SIDE_CM - mx;
    if (REV_MAP_SIDE_CM - my < dmin) dmin = REV_MAP_SIDE_CM - my;
    float bp = REV_HALF_CM - dmin;
    if (bp > pen) pen = bp;
    return pen;
}

static Point2D rev_map_from_world(float wx, float wy) {
    Point2D p;
    p.x = wx * 0.01f + MAP_OFFSET_X;
    p.y = wy * 0.01f + MAP_OFFSET_Y;
    return p;
}
static void rev_world_from_map(Point2D p, float *wx, float *wy) {
    *wx = (p.x - MAP_OFFSET_X) * 100.0f;
    *wy = (p.y - MAP_OFFSET_Y) * 100.0f;
}

static float rev_wrap_deg(float a) {
    float r = fmodf(a, 360.0f);
    if (r > 180.0f) r -= 360.0f;
    if (r < -180.0f) r += 360.0f;
    return r;
}
static float rev_dist_cm(float ax, float ay, float bx, float by) {
    float dx = ax - bx, dy = ay - by;
    return sqrtf(dx * dx + dy * dy);
}

/* ==========================================================================================
 * 3. 统计
 * ========================================================================================== */

typedef struct {
    long plans, ok, fail;
    long fail_endpoint, fail_no_solution, fail_no_rotation, fail_bad_param, fail_other;
    long endpoint_mismatch, heading_mismatch, rotation_unsafe, rotation_index_bad, waypoints_mismatch;
    long nan_bad, dup_bad, stats_min_mismatch;
    long sat_violation_general, sat_violation_endpoint;
    long square_violation_general, square_violation_endpoint;
    long suspicious_false_negative_hint;
    double min_square_general, min_square_endpoint, min_rot_clearance;
    double gen_required, gen_allowed, ep_allowed;
} RevStats;

static RevStats g_st;

static void rev_stats_reset(RevStats *st) {
    memset(st, 0, sizeof(*st));
    st->min_square_general = 1.0e9;
    st->min_square_endpoint = 1.0e9;
    st->min_rot_clearance = 1.0e9;
}

#define REV_WORST_MAX 12
typedef struct {
    int zone;
    float sx, sy, syaw, gx, gy, gyaw;
    int n, tier, rot_idx;
    float meas_min_sq;
    Point2D path[MAX_PATH_LEN];
} RevWorst;

static RevWorst g_worst[REV_WORST_MAX];
static int g_worst_count = 0;
static int g_printed_viol = 0;

static void rev_worst_insert(int zone, float sx, float sy, float syaw, float gx, float gy, float gyaw, int n, int tier,
                             int rot_idx, float min_sq, const Point2D *path) {
    int pos = -1;
    if (g_worst_count < REV_WORST_MAX) pos = g_worst_count++;
    else if (min_sq < g_worst[REV_WORST_MAX - 1].meas_min_sq) pos = REV_WORST_MAX - 1;
    else return;
    RevWorst *w = &g_worst[pos];
    w->zone = zone;
    w->sx = sx; w->sy = sy; w->syaw = syaw;
    w->gx = gx; w->gy = gy; w->gyaw = gyaw;
    w->n = n;
    w->tier = tier;
    w->rot_idx = rot_idx;
    w->meas_min_sq = min_sq;
    if (n > MAX_PATH_LEN) n = MAX_PATH_LEN;
    memcpy(w->path, path, sizeof(Point2D) * (size_t)n);
    for (int i = g_worst_count - 1; i > 0; i--) {
        if (g_worst[i].meas_min_sq >= g_worst[i - 1].meas_min_sq) break;
        RevWorst t = g_worst[i];
        g_worst[i] = g_worst[i - 1];
        g_worst[i - 1] = t;
    }
}

static void rev_print_path_world(const Point2D *path, int n) {
    printf("        path_world_cm:");
    for (int i = 0; i < n; i++) {
        float wx, wy;
        rev_world_from_map(path[i], &wx, &wy);
        printf(" (%.1f,%.1f)", (double)wx, (double)wy);
        if ((i % 6) == 5 && i + 1 < n) printf("\n                     ");
    }
    printf("\n");
}

/* ==========================================================================================
 * 4. 随机数 / 初始化
 * ========================================================================================== */

static uint64_t s_rng = 0x9E3779B97F4A7C15ULL;
static uint32_t rev_rand(void) {
    s_rng ^= s_rng << 13;
    s_rng ^= s_rng >> 7;
    s_rng ^= s_rng << 17;
    return (uint32_t)(s_rng >> 32);
}
static uint32_t rev_rand_range(uint32_t n) { return (n == 0) ? 0 : (rev_rand() % n); }
static float rev_rand_float(float lo, float hi) {
    return lo + (hi - lo) * ((float)(rev_rand() >> 8) / 16777216.0f);
}

static int rev_check_canary(void) {
    int bad = 0;
    for (int i = MAX_MAP_BYTES; i < (int)sizeof(s_grid_storage); i++)
        if (s_grid_storage[i] != 0x5A) bad++;
    for (int i = MAX_CELLS; i < (int)sizeof(s_clear_storage); i++)
        if (s_clear_storage[i] != 0xA5) bad++;
    for (int i = MAX_CELLS; i < (int)sizeof(s_square_storage); i++)
        if ((uint8_t)s_square_storage[i] != 0x3C) bad++;
    return bad;
}

static void rev_init_zone(int zone) {
    s_dyn_rect_count = 0;
    rev_build_static_rects((zone == 2) ? s_zone2_quads : s_zone1_quads,
                           (int)((zone == 2) ? (sizeof(s_zone2_quads) / sizeof(float))
                                             : (sizeof(s_zone1_quads) / sizeof(float))));
    memset(s_grid_storage, 0x5A, sizeof(s_grid_storage));
    memset(s_clear_storage, 0xA5, sizeof(s_clear_storage));
    memset(s_square_storage, 0x3C, sizeof(s_square_storage));
    if (!MapCore_Init((uint8_t)zone, s_grid_storage, s_clear_storage, s_square_storage)) {
        printf("[PROBLEM] MapCore_Init(zone=%d) failed\n", zone);
        g_problems++;
    }
    MapCore_RebuildClearance();
}

/* ==========================================================================================
 * 5. 独立可达性（仅“不需要旋转”的失败用例；5cm 网格 + 同一保守模型口径）
 * ========================================================================================== */

#define REV_FINE_RES 5.0f
#define REV_FINE_N 96 /* 480cm / 5cm */

static float s_fine_clr[REV_FINE_N][REV_FINE_N];
static uint8_t s_fine_state[REV_FINE_N][REV_FINE_N];

static int rev_fine_reachable(float sx, float sy, float gx, float gy, float half_cm, float endpoint_allowed_cm) {
    const float res = REV_FINE_RES;
    for (int ix = 0; ix < REV_FINE_N; ix++)
        for (int iy = 0; iy < REV_FINE_N; iy++) {
            float wx = res * ((float)ix + 0.5f) - REV_MAP_OFF_CM;
            float wy = res * ((float)iy + 0.5f) - REV_MAP_OFF_CM;
            s_fine_clr[ix][iy] = rev_square_clearance(wx, wy, half_cm);
            s_fine_state[ix][iy] = 0;
        }
    int sgx = (int)((sx + REV_MAP_OFF_CM) / res);
    int sgy = (int)((sy + REV_MAP_OFF_CM) / res);
    int ggx = (int)((gx + REV_MAP_OFF_CM) / res);
    int ggy = (int)((gy + REV_MAP_OFF_CM) / res);
    if (sgx < 0 || sgy < 0 || sgx >= REV_FINE_N || sgy >= REV_FINE_N) return 0;
    if (ggx < 0 || ggy < 0 || ggx >= REV_FINE_N || ggy >= REV_FINE_N) return 0;

    static int qx[REV_FINE_N * REV_FINE_N], qy[REV_FINE_N * REV_FINE_N];
    static float qf[REV_FINE_N * REV_FINE_N];
    int qn = 0;
    s_fine_state[sgx][sgy] = 1;
    qx[qn] = sgx; qy[qn] = sgy; qf[qn] = 0.0f; qn++;
    while (qn > 0) {
        int bi = 0;
        for (int i = 1; i < qn; i++)
            if (qf[i] < qf[bi]) bi = i;
        int cx = qx[bi], cy = qy[bi];
        float cg = qf[bi];
        qn--;
        qx[bi] = qx[qn]; qy[bi] = qy[qn]; qf[bi] = qf[qn];
        if (cx == ggx && cy == ggy) return 1;
        if (s_fine_state[cx][cy] == 2) continue;
        s_fine_state[cx][cy] = 2;
        static const int ddx[8] = {0, 0, 1, -1, 1, 1, -1, -1};
        static const int ddy[8] = {1, -1, 0, 0, 1, -1, 1, -1};
        for (int k = 0; k < 8; k++) {
            int nx = cx + ddx[k], ny = cy + ddy[k];
            if (nx < 0 || ny < 0 || nx >= REV_FINE_N || ny >= REV_FINE_N) continue;
            if (s_fine_state[nx][ny] == 2) continue;
            float wxp = res * ((float)nx + 0.5f) - REV_MAP_OFF_CM;
            float wyp = res * ((float)ny + 0.5f) - REV_MAP_OFF_CM;
            float need = 0.0f;
            if (rev_dist_cm(wxp, wyp, sx, sy) <= REV_ENDPOINT_ZONE_CM ||
                rev_dist_cm(wxp, wyp, gx, gy) <= REV_ENDPOINT_ZONE_CM)
                need = endpoint_allowed_cm;
            if (s_fine_clr[nx][ny] < need - 1e-3f) continue;
            float ax = res * ((float)cx + 0.5f) - REV_MAP_OFF_CM;
            float ay = res * ((float)cy + 0.5f) - REV_MAP_OFF_CM;
            int steps = (int)ceilf(rev_dist_cm(ax, ay, wxp, wyp) / 1.0f);
            if (steps < 1) steps = 1;
            int ok_edge = 1;
            for (int t = 0; t <= steps; t++) {
                float u = (float)t / (float)steps;
                float px = ax + (wxp - ax) * u;
                float py = ay + (wyp - ay) * u;
                float ne = 0.0f;
                if (rev_dist_cm(px, py, sx, sy) <= REV_ENDPOINT_ZONE_CM ||
                    rev_dist_cm(px, py, gx, gy) <= REV_ENDPOINT_ZONE_CM)
                    ne = endpoint_allowed_cm;
                if (rev_square_clearance(px, py, half_cm) < ne - 1e-3f) {
                    ok_edge = 0;
                    break;
                }
            }
            if (!ok_edge) continue;
            float ng = cg + rev_dist_cm(ax, ay, wxp, wyp);
            if (s_fine_state[nx][ny] != 1) {
                s_fine_state[nx][ny] = 1;
                qx[qn] = nx; qy[qn] = ny;
                qf[qn] = ng + rev_dist_cm(wxp, wyp, gx, gy);
                qn++;
            }
        }
    }
    return 0;
}

/* ==========================================================================================
 * 6. 单次规划 + 独立校验
 * ========================================================================================== */

static Point2D s_out_path[MAX_PATH_LEN];

static int rev_validate_plan(int zone, float sx, float sy, float syaw, float gx, float gy, float gyaw,
                             const Point2D *path, int n, const PlannerPathStats_t *stats, int quiet) {
    int bad = 0;
    float wx0, wy0, wxn, wyn;
    rev_world_from_map(path[0], &wx0, &wy0);
    rev_world_from_map(path[n - 1], &wxn, &wyn);
    if (rev_dist_cm(wx0, wy0, sx, sy) > 1.0e-2f) {
        g_st.endpoint_mismatch++;
        bad = 1;
    }
    if (rev_dist_cm(wxn, wyn, gx, gy) > 1.0e-2f) {
        g_st.endpoint_mismatch++;
        bad = 1;
    }
    if (stats->waypoints != (uint16_t)n) {
        g_st.waypoints_mismatch++;
        bad = 1;
    }
    if (stats->fail_reason != PLANNER_FAIL_NONE) bad = 1;

    int rot = (stats->rotation_index == PLANNER_ROTATE_NONE) ? -1 : (int)stats->rotation_index;
    float hb = stats->heading_before_deg;
    float ha = stats->heading_after_deg;
    float need_general = (stats->tier == PLANNER_TIER_SOFT) ? PLANNER_SAFE_MARGIN_CM : PLANNER_SQUARE_HARD_MARGIN_CM;
    /* v2.1：hard tier endpoint_allowed = required - 2 = -1cm 真正生效；端点区允许 ≤1cm 模型侵入 */
    float ep_allowed = need_general - PLANNER_ENDPOINT_SQUARE_TOL_CM;
    if (ep_allowed < -PLANNER_ENDPOINT_PENETRATION_TOL_CM) ep_allowed = -PLANNER_ENDPOINT_PENETRATION_TOL_CM;
    if (ep_allowed > need_general) ep_allowed = need_general;
    g_st.gen_required = need_general;
    g_st.gen_allowed = need_general - PLANNER_VALIDATION_TOL_CM;
    g_st.ep_allowed = ep_allowed;

    if (rot < 0) {
        if (fabsf(rev_wrap_deg(hb - gyaw)) > 1.0e-2f || fabsf(rev_wrap_deg(ha - gyaw)) > 1.0e-2f) {
            g_st.heading_mismatch++;
            if (!quiet)
                printf("[PROBLEM] heading fields wrong(no-rot): hb=%.2f ha=%.2f goal=%.2f\n", (double)hb,
                       (double)ha, (double)gyaw);
            bad = 1;
        }
        if (fabsf(rev_wrap_deg(hb - syaw)) > PLANNER_HEADING_DEADBAND_DEG + 1.0e-2f) {
            g_st.heading_mismatch++;
            bad = 1;
        }
    } else {
        if (rot >= n) {
            g_st.rotation_index_bad++;
            bad = 1;
        }
        if (fabsf(rev_wrap_deg(hb - syaw)) > 1.0e-2f || fabsf(rev_wrap_deg(ha - gyaw)) > 1.0e-2f) {
            g_st.heading_mismatch++;
            if (!quiet)
                printf("[PROBLEM] heading fields wrong: hb=%.2f syaw=%.2f ha=%.2f gyaw=%.2f\n", (double)hb,
                       (double)syaw, (double)ha, (double)gyaw);
            bad = 1;
        }
        if (rot < n) {
            float rx, ry;
            rev_world_from_map(path[rot], &rx, &ry);
            float clr = rev_point_clearance(rx, ry);
            if (clr < REV_ROT_CLEAR_CM - 0.5f) {
                g_st.rotation_unsafe++;
                bad = 1;
            }
            if (clr < g_st.min_rot_clearance) g_st.min_rot_clearance = clr;
            if (fabsf(stats->rotation_clearance_cm - clr) > 0.2f) g_st.stats_min_mismatch++;
        }
    }

    float min_sq = 1.0e9f, min_circle = 1.0e9f;
    for (int i = 1; i < n; i++) {
        float heading;
        if (rot < 0) heading = hb;
        else if (rot == 0) heading = ha;
        else heading = (i <= rot) ? hb : ha;
        float half_cm = Planner_HeadingFootprintHalfCm(heading);
        float ax, ay, bx, by;
        rev_world_from_map(path[i - 1], &ax, &ay);
        rev_world_from_map(path[i], &bx, &by);
        float len = rev_dist_cm(ax, ay, bx, by);
        int steps = (int)ceilf(len / REV_SAT_STEP_CM);
        if (steps < 1) steps = 1;
        for (int k = 0; k <= steps; k++) {
            float t = (float)k / (float)steps;
            float px = ax + (bx - ax) * t;
            float py = ay + (by - ay) * t;
            int in_ep = (rev_dist_cm(px, py, sx, sy) <= REV_ENDPOINT_ZONE_CM ||
                         rev_dist_cm(px, py, gx, gy) <= REV_ENDPOINT_ZONE_CM);
            float sq = rev_square_clearance(px, py, half_cm);
            float clr = rev_point_clearance(px, py);
            if (sq < min_sq) min_sq = sq;
            if (clr < min_circle) min_circle = clr;
            if (in_ep) {
                if (sq < g_st.min_square_endpoint) g_st.min_square_endpoint = sq;
                if (sq < ep_allowed - 1.0e-3f) {
                    g_st.square_violation_endpoint++;
                    bad = 1;
                }
            } else {
                if (sq < g_st.min_square_general) g_st.min_square_general = sq;
                if (sq < g_st.gen_allowed - 1.0e-3f) {
                    g_st.square_violation_general++;
                    bad = 1;
                }
            }
            float pen = rev_pose_penetration(px, py, heading);
            if (in_ep) {
                if (pen > 1.0f + 1.0e-3f) {
                    g_st.sat_violation_endpoint++;
                    bad = 1;
                }
            } else if (pen > 1.0e-3f) {
                g_st.sat_violation_general++;
                bad = 1;
            }
            if (bad && g_printed_viol < 12 && !quiet) {
                g_printed_viol++;
                printf("[VIOLATION] zone=%d seg=%d t=%.2f world=(%.2f,%.2f) heading=%.2f sq=%.3f euclid=%.3f "
                       "satPen=%.3f epZone=%d tier=%d\n",
                       zone, i, (double)t, (double)px, (double)py, (double)heading, (double)sq, (double)clr,
                       (double)pen, in_ep, (int)stats->tier);
            }
        }
    }
    if (fabsf(stats->min_square_clearance_cm - min_sq) > 0.3f) {
        g_st.stats_min_mismatch++;
        if (g_st.stats_min_mismatch <= 8)
            printf("[STATS-MISMATCH] zone=%d sq: stats=%.3f meas=%.3f start=(%.1f,%.1f,%.1f) goal=(%.1f,%.1f,%.1f)\n",
                   zone, (double)stats->min_square_clearance_cm, (double)min_sq, (double)sx, (double)sy,
                   (double)syaw, (double)gx, (double)gy, (double)gyaw);
    }
    if (fabsf(stats->min_clearance_cm - min_circle) > 0.5f) {
        g_st.stats_min_mismatch++;
        if (g_st.stats_min_mismatch <= 8)
            printf("[STATS-MISMATCH] zone=%d circle: stats=%.3f meas=%.3f start=(%.1f,%.1f,%.1f) goal=(%.1f,%.1f,%.1f)\n",
                   zone, (double)stats->min_clearance_cm, (double)min_circle, (double)sx, (double)sy, (double)syaw,
                   (double)gx, (double)gy, (double)gyaw);
    }

    if (n < 2) g_st.nan_bad++;
    {
        /* v2.1 特例：start==goal（dist<1cm，原地不动/原地转）返回 2 个相同坐标点是合法零长路径 */
        float total_len = Planner_PathLengthCm(path, n);
        int legal_zero_len = (n == 2 && total_len < 1.0f);
        for (int i = 0; i < n; i++) {
            if (!(isfinite(path[i].x) && isfinite(path[i].y))) g_st.nan_bad++;
            if (i > 0 && !legal_zero_len) {
                float ax, ay, bx, by;
                rev_world_from_map(path[i - 1], &ax, &ay);
                rev_world_from_map(path[i], &bx, &by);
                if (rev_dist_cm(ax, ay, bx, by) < 1.0e-3f) g_st.dup_bad++;
            }
        }
    }
    rev_worst_insert(zone, sx, sy, syaw, gx, gy, gyaw, n, (int)stats->tier, rot, min_sq, path);
    return bad ? 0 : 1;
}

static void rev_plan_once(int zone, float sx, float sy, float syaw, float gx, float gy, float gyaw, int check_reach) {
    PlannerPathStats_t stats;
    memset(&stats, 0, sizeof(stats));
    int n = Planner_PlanPathWithYaw(rev_map_from_world(sx, sy), rev_map_from_world(gx, gy), syaw, gyaw, s_out_path,
                                    MAX_PATH_LEN, &stats);
    g_st.plans++;
    if (n <= 0) {
        g_st.fail++;
        switch (stats.fail_reason) {
        case PLANNER_FAIL_ENDPOINT: g_st.fail_endpoint++; break;
        case PLANNER_FAIL_NO_SOLUTION: g_st.fail_no_solution++; break;
        case PLANNER_FAIL_NO_ROTATION_POINT: g_st.fail_no_rotation++; break;
        case PLANNER_FAIL_BAD_PARAM: g_st.fail_bad_param++; break;
        default: g_st.fail_other++; break;
        }
        if (check_reach && stats.fail_reason == PLANNER_FAIL_NO_SOLUTION &&
            fabsf(rev_wrap_deg(gyaw - syaw)) <= PLANNER_HEADING_DEADBAND_DEG) {
            float half_cm = Planner_HeadingFootprintHalfCm(syaw);
            float ep_allowed = PLANNER_SQUARE_HARD_MARGIN_CM - PLANNER_ENDPOINT_SQUARE_TOL_CM;
            if (ep_allowed < -PLANNER_ENDPOINT_PENETRATION_TOL_CM) ep_allowed = -PLANNER_ENDPOINT_PENETRATION_TOL_CM;
            if (rev_fine_reachable(sx, sy, gx, gy, half_cm, ep_allowed)) {
                g_st.suspicious_false_negative_hint++;
                if (g_st.suspicious_false_negative_hint <= 12)
                    printf("[SUSPICIOUS-FN] zone=%d start=(%.1f,%.1f,%.1f) goal=(%.1f,%.1f,%.1f) "
                           "5cm 独立网格 A* 可达但规划失败(no-solution)\n",
                           zone, (double)sx, (double)sy, (double)syaw, (double)gx, (double)gy, (double)gyaw);
            }
        }
        return;
    }
    g_st.ok++;
    (void)rev_validate_plan(zone, sx, sy, syaw, gx, gy, gyaw, s_out_path, n, &stats, 1);
}

/* ==========================================================================================
 * 7. 地图核心一致性
 * ========================================================================================== */

static void rev_test_map(int zone) {
    rev_init_zone(zone);
    printf("[MAP] zone=%d rects=%u obstacles=%u grid_bytes=%d cells=%d canary=%d\n", zone,
           (unsigned)MapCore_StaticRectCount(), (unsigned)MapCore_ObstacleCells(), (int)MAX_MAP_BYTES,
           (int)MAX_CELLS, rev_check_canary());
    if (rev_check_canary() != 0 || (int)MapCore_StaticRectCount() != s_static_rect_count) {
        printf("[PROBLEM] map init/canary/rect-count\n");
        g_problems++;
    }

    float max_pt = 0.0f, max_sq = 0.0f;
    int bad_pt = 0, bad_sq = 0;
    for (int gx2 = 0; gx2 < g_map.grid_size; gx2++) {
        for (int gy2 = 0; gy2 < g_map.grid_size; gy2++) {
            float wx = (float)gx2 * 10.0f + 5.0f - REV_MAP_OFF_CM;
            float wy = (float)gy2 * 10.0f + 5.0f - REV_MAP_OFF_CM;
            Point2D mp = rev_map_from_world(wx, wy);
            float d_pt = fabsf(MapCore_PointClearanceCm(mp) - rev_point_clearance(wx, wy));
            float d_sq = fabsf(MapCore_PointSquareClearanceCm(mp, REV_HALF_CM) - rev_square_clearance(wx, wy, REV_HALF_CM));
            if (d_pt > max_pt) max_pt = d_pt;
            if (d_sq > max_sq) max_sq = d_sq;
            if (d_pt > 0.01f) bad_pt++;
            if (d_sq > 0.01f) bad_sq++;
        }
    }
    printf("[MAP] zone=%d point_clr maxdiff=%.4fcm square_clr maxdiff=%.4fcm bad_pt=%d bad_sq=%d\n", zone,
           (double)max_pt, (double)max_sq, bad_pt, bad_sq);
    if (bad_pt || bad_sq) {
        printf("[PROBLEM] zone=%d 距离场与独立几何不一致\n", zone);
        g_problems++;
    }

    int bad_field = 0;
    for (int gx2 = 0; gx2 < g_map.grid_size; gx2++) {
        for (int gy2 = 0; gy2 < g_map.grid_size; gy2++) {
            if (get_grid_val(&g_map, gx2, gy2) != MAP_CELL_FREE) continue; /* 障碍格按设计存 -100 */
            float wx = (float)gx2 * 10.0f + 5.0f - REV_MAP_OFF_CM;
            float wy = (float)gy2 * 10.0f + 5.0f - REV_MAP_OFF_CM;
            float ref = rev_square_clearance(wx, wy, REV_HALF_CM);
            int expect = (int)floorf(ref + 1e-3f);
            if (expect < -100) expect = -100;
            if (expect > 100) expect = 100;
            int got = (int)MapCore_CellSquareClearanceCm(gx2, gy2);
            if (abs(expect - got) > 1) bad_field++;
        }
    }
    printf("[MAP] zone=%d cell square field diff>1cm=%d\n", zone, bad_field);
    if (bad_field) {
        printf("[PROBLEM] zone=%d 格方形净空场偏差过大\n", zone);
        g_problems++;
    }

    for (int i = 0; i < MAX_CELLS; i++)
        set_grid_val(&g_map, i / MAX_GRID_SIZE, i % MAX_GRID_SIZE, (uint8_t)(i % 3));
    int pack_bad = 0;
    for (int i = 0; i < MAX_CELLS; i++)
        if (get_grid_val(&g_map, i / MAX_GRID_SIZE, i % MAX_GRID_SIZE) != (uint8_t)(i % 3)) pack_bad++;
    printf("[MAP] zone=%d 2bit packing errors=%d, canary_after=%d\n", zone, pack_bad, rev_check_canary());
    if (pack_bad || rev_check_canary()) g_problems++;
}

/* ==========================================================================================
 * 8. 随机静态/动态模糊
 * ========================================================================================== */

#define REV_MAX_FREE 4096
static float s_free_x[REV_MAX_FREE], s_free_y[REV_MAX_FREE];
static int s_free_n = 0;

static void rev_build_free_points(void) {
    s_free_n = 0;
    for (int ix = -10; ix <= 210; ix += 5) {
        for (int iy = -10; iy <= 210; iy += 5) {
            if (rev_point_clearance((float)ix, (float)iy) >= 15.0f) {
                if (s_free_n < REV_MAX_FREE) {
                    s_free_x[s_free_n] = (float)ix;
                    s_free_y[s_free_n] = (float)iy;
                    s_free_n++;
                }
            }
        }
    }
}

static int rev_pose_plausible(float x, float y, float yaw) {
    /* 物理上可摆放的位姿：车体在自身朝向下不得与障碍重建的方形禁区相交（端点容差 hard=0） */
    return rev_square_clearance(x, y, Planner_HeadingFootprintHalfCm(yaw)) >= 0.0f;
}

static void rev_fuzz_static(int zone, int count) {
    rev_init_zone(zone);
    rev_build_free_points();
    printf("[FUZZ] zone=%d static free_points=%d\n", zone, s_free_n);
    RevStats before = g_st;
    int tries = 0, done = 0;
    while (done < count && tries < count * 80) {
        tries++;
        int i = (int)rev_rand_range((uint32_t)s_free_n);
        int j = (int)rev_rand_range((uint32_t)s_free_n);
        if (i == j) continue;
        float syaw = rev_rand_float(-180, 180);
        float gyaw = rev_rand_float(-180, 180);
        /* 只取“车体可摆放”的位姿（否则规划失败只是位姿本身已碰撞，没有评审意义） */
        if (!rev_pose_plausible(s_free_x[i], s_free_y[i], syaw) ||
            !rev_pose_plausible(s_free_x[j], s_free_y[j], gyaw))
            continue;
        rev_plan_once(zone, s_free_x[i], s_free_y[i], syaw, s_free_x[j], s_free_y[j], gyaw, done < 800);
        done++;
    }
    printf("[FUZZ] zone=%d static: plans=%ld ok=%ld fail=%ld (ep=%ld nosol=%ld norot=%ld badparam=%ld) fn_hint=%ld\n",
           zone, g_st.plans - before.plans, g_st.ok - before.ok, g_st.fail - before.fail,
           g_st.fail_endpoint - before.fail_endpoint, g_st.fail_no_solution - before.fail_no_solution,
           g_st.fail_no_rotation - before.fail_no_rotation, g_st.fail_bad_param - before.fail_bad_param,
           g_st.suspicious_false_negative_hint - before.suspicious_false_negative_hint);
}

static void rev_add_dynamic_cell(int gx, int gy) {
    set_grid_val(&g_map, gx, gy, MAP_CELL_DYNAMIC_OBSTACLE);
    if (s_dyn_rect_count < REV_MAX_DYN_RECTS) {
        RevRect *r = &s_dyn_rects[s_dyn_rect_count++];
        r->x0 = (float)gx * 10.0f - REV_MAP_OFF_CM;
        r->y0 = (float)gy * 10.0f - REV_MAP_OFF_CM;
        r->x1 = r->x0 + 10.0f;
        r->y1 = r->y0 + 10.0f;
    }
}

static void rev_fuzz_dynamic(int zone, int count) {
    RevStats before = g_st;
    int field_bad = 0;
    for (int round = 0; round < 10; round++) {
        rev_init_zone(zone);
        int target = 2 + (int)rev_rand_range(19);
        int added = 0, guard = 0;
        while (added < target && guard < 3000) {
            guard++;
            int gx = 4 + (int)rev_rand_range(MAX_GRID_SIZE - 8);
            int gy = 4 + (int)rev_rand_range(MAX_GRID_SIZE - 8);
            float wx = (float)gx * 10.0f + 5.0f - REV_MAP_OFF_CM;
            float wy = (float)gy * 10.0f + 5.0f - REV_MAP_OFF_CM;
            if (rev_point_clearance(wx, wy) < 20.0f) continue;
            if (get_grid_val(&g_map, gx, gy) != MAP_CELL_FREE) continue;
            rev_add_dynamic_cell(gx, gy);
            added++;
        }
        MapCore_RebuildClearance();
        rev_build_free_points();
        for (int k = 0; k < 40; k++) {
            float wx = rev_rand_float(-100.0f, 220.0f);
            float wy = rev_rand_float(-100.0f, 220.0f);
            Point2D mp = rev_map_from_world(wx, wy);
            if (fabsf(MapCore_PointSquareClearanceCm(mp, REV_HALF_CM) - rev_square_clearance(wx, wy, REV_HALF_CM)) >
                0.02f)
                field_bad++;
        }
        int per = (count + 9) / 10;
        for (int c = 0; c < per; c++) {
            int i = (int)rev_rand_range((uint32_t)s_free_n);
            int j = (int)rev_rand_range((uint32_t)s_free_n);
            if (i == j) continue;
            float syaw = rev_rand_float(-180, 180);
            float gyaw = rev_rand_float(-180, 180);
            if (!rev_pose_plausible(s_free_x[i], s_free_y[i], syaw) ||
                !rev_pose_plausible(s_free_x[j], s_free_y[j], gyaw))
                continue;
            rev_plan_once(zone, s_free_x[i], s_free_y[i], syaw, s_free_x[j], s_free_y[j], gyaw,
                          (round < 3) && (c < 20));
        }
    }
    printf("[FUZZ] zone=%d dynamic: plans=%ld ok=%ld fail=%ld ep=%ld nosol=%ld norot=%ld field_mismatch=%d\n", zone,
           g_st.plans - before.plans, g_st.ok - before.ok, g_st.fail - before.fail,
           g_st.fail_endpoint - before.fail_endpoint, g_st.fail_no_solution - before.fail_no_solution,
           g_st.fail_no_rotation - before.fail_no_rotation, field_bad);
    if (field_bad) {
        printf("[PROBLEM] dynamic 方形净空查询与独立几何不一致\n");
        g_problems++;
    }
}

/* ==========================================================================================
 * 9. 专项用例
 * ========================================================================================== */

typedef struct {
    const char *name;
    float x, y, yaw;
} RevPoint;

static void rev_test_task_points(void) {
    static const RevPoint z1[] = {{"HOME", 0, 0, 0},        {"QrCode", 10, 105, 0},
                                  {"MATL_GRAB", 105, 14, 0}, {"MATL_STORAGE", 190, 105, 90},
                                  {"MATL_PROCESSING", 105, 190, 180}};
    static const RevPoint z2[] = {{"HOME", 0, 0, 0},        {"QrCode", 105, 10, 0},
                                  {"MATL_GRAB", 196, 105, 0}, {"MATL_STORAGE", 105, 190, 0},
                                  {"MATL_PROCESSING", 20, 105, 0}};
    for (int zone = 1; zone <= 2; zone++) {
        rev_init_zone(zone);
        const RevPoint *pts = (zone == 1) ? z1 : z2;
        int ok = 0, total = 0;
        for (int i = 0; i < 5; i++) {
            for (int j = 0; j < 5; j++) {
                if (i == j) continue;
                total++;
                PlannerPathStats_t stats;
                memset(&stats, 0, sizeof(stats));
                int n = Planner_PlanPathWithYaw(rev_map_from_world(pts[i].x, pts[i].y),
                                                rev_map_from_world(pts[j].x, pts[j].y), pts[i].yaw, pts[j].yaw,
                                                s_out_path, MAX_PATH_LEN, &stats);
                if (n >= 2) {
                    if (rev_validate_plan(zone, pts[i].x, pts[i].y, pts[i].yaw, pts[j].x, pts[j].y, pts[j].yaw,
                                          s_out_path, n, &stats, 0))
                        ok++;
                } else {
                    printf("[TASK-FAIL] zone=%d %s->%s n=%d fail=%u\n", zone, pts[i].name, pts[j].name, n,
                           (unsigned)stats.fail_reason);
                }
            }
        }
        printf("[TASK] zone=%d task point pairs ok=%d/%d\n", zone, ok, total);
        if (ok != total) {
            printf("[PROBLEM] zone=%d 任务点位对未全部规划成功或校验失败\n", zone);
            g_problems++;
        }
    }
}

static void rev_test_one(const char *tag, int zone, float sx, float sy, float syaw, float gx, float gy, float gyaw,
                         int expect_ok) {
    rev_init_zone(zone);
    PlannerPathStats_t stats;
    memset(&stats, 0, sizeof(stats));
    int n = Planner_PlanPathWithYaw(rev_map_from_world(sx, sy), rev_map_from_world(gx, gy), syaw, gyaw, s_out_path,
                                    MAX_PATH_LEN, &stats);
    int valid = 0;
    if (n >= 2) valid = rev_validate_plan(zone, sx, sy, syaw, gx, gy, gyaw, s_out_path, n, &stats, 0);
    int pass = expect_ok ? (n >= 2 && valid) : (n <= 0);
    printf("[CASE-%s] %s: n=%d rot=%d sq=%.2f valid=%d fail=%u\n", pass ? "OK" : "FAIL", tag, n,
           (int)((stats.rotation_index == PLANNER_ROTATE_NONE) ? -1 : (int)stats.rotation_index),
           (double)stats.min_square_clearance_cm, valid, (unsigned)stats.fail_reason);
    if (!pass) g_problems++;
}

static void rev_test_targeted(void) {
    rev_test_one("zone1 35cm-channel straight", 1, 50.0f, 192.0f, 0.0f, 160.0f, 192.0f, 0.0f, 1);

    /* 30cm 十字通道：物理只能刚好塞满；记录行为 */
    {
        rev_init_zone(1);
        PlannerPathStats_t st;
        memset(&st, 0, sizeof(st));
        int n = Planner_PlanPathWithYaw(rev_map_from_world(105, 20), rev_map_from_world(105, 170), 0, 0, s_out_path,
                                        MAX_PATH_LEN, &st);
        int valid = 0;
        if (n >= 2) valid = rev_validate_plan(1, 105, 20, 0, 105, 170, 0, s_out_path, n, &st, 0);
        printf("[CASE-INFO] zone1 30cm-corridor crossing: n=%d sq=%.2f valid=%d fail=%u\n", n,
               (double)st.min_square_clearance_cm, valid, (unsigned)st.fail_reason);
        if (n >= 2 && !valid) {
            printf("[PROBLEM] 30cm 通道返回路径未通过独立安全校验\n");
            g_problems++;
        }
    }

    /* 通道内需要旋转：必须 via 到安全点或 fail-safe */
    {
        rev_init_zone(1);
        PlannerPathStats_t st;
        memset(&st, 0, sizeof(st));
        int n = Planner_PlanPathWithYaw(rev_map_from_world(105, 40), rev_map_from_world(105, 160), 0, 90, s_out_path,
                                        MAX_PATH_LEN, &st);
        if (n >= 2) {
            int rot = (st.rotation_index == PLANNER_ROTATE_NONE) ? -1 : (int)st.rotation_index;
            float rc = 1e9f;
            if (rot >= 0 && rot < n) {
                float rx, ry;
                rev_world_from_map(s_out_path[rot], &rx, &ry);
                rc = rev_point_clearance(rx, ry);
            }
            int valid = rev_validate_plan(1, 105, 40, 0, 105, 160, 90, s_out_path, n, &st, 0);
            printf("[CASE] zone1 rotate-in-30cm-corridor: n=%d rot=%d rot_clr=%.2f valid=%d\n", n, rot, (double)rc,
                   valid);
            if (valid && rot >= 0 && rc < REV_ROT_CLEAR_CM - 0.5f) g_problems++;
        } else {
            printf("[CASE] zone1 rotate-in-30cm-corridor: failed(reason=%u) —— 预期 fail-safe\n",
                   (unsigned)st.fail_reason);
            if (st.fail_reason != PLANNER_FAIL_NO_ROTATION_POINT && st.fail_reason != PLANNER_FAIL_NO_SOLUTION) {
                printf("[PROBLEM] 通道内旋转失败原因不合理\n");
                g_problems++;
            }
        }
    }

    rev_test_one("zone1 rotate-at-start(open)", 1, 10.0f, 195.0f, 10.0f, 10.0f, 30.0f, 100.0f, 1);
    rev_test_one("zone1 arbitrary yaw 37deg", 1, 20.0f, 195.0f, 0.0f, 200.0f, 195.0f, 37.0f, 1);
    rev_test_one("zone1 arbitrary goal yaw -123deg", 1, 50.0f, 192.5f, 0.0f, 200.0f, 195.0f, -123.0f, 1);
    rev_test_one("zone1 same-point rotate", 1, 105.0f, 195.0f, 0.0f, 105.0f, 195.0f, 90.0f, 1);
    /* v2.1 特例：旋转安全点上的同点零长路径（返回 2 点、rotation_index=0） */
    rev_test_one("zone1 same-point rotate@safe", 1, 20.0f, 195.0f, 0.0f, 20.0f, 195.0f, 90.0f, 1);
    /* v2.1 端点 -1cm 渗透容差：35cm 通道内目标点向墙偏移 3cm（方形净空约 -0.5cm） */
    rev_test_one("zone1 goal-endpoint -0.5cm penetration", 1, 50.0f, 192.5f, 0.0f, 105.0f, 195.5f, 0.0f, 1);
    rev_test_one("zone1 start-endpoint -0.5cm penetration", 1, 105.0f, 195.5f, 0.0f, 50.0f, 192.5f, 0.0f, 1);
    /* 超过 -1cm 渗透容差必须 clean fail（端点检查拒绝） */
    rev_test_one("zone1 goal-endpoint -1.5cm penetration(reject)", 1, 50.0f, 192.5f, 0.0f, 105.0f, 196.6f, 0.0f,
                 0);
    rev_test_one("zone1 goal-inside-obstacle", 1, 10.0f, 10.0f, 0.0f, 60.0f, 150.0f, 0.0f, 0);
}

/* 短距离直达灰区：plan_segment 用 0 净空直达后校验失败时不回退 A* 绕行 */
static void rev_test_short_direct(void) {
    rev_init_zone(1);
    rev_build_free_points();
    int tested = 0, suspicious = 0;
    for (int c = 0; c < 4000 && tested < 600; c++) {
        int i = (int)rev_rand_range((uint32_t)s_free_n);
        float sx = s_free_x[i], sy = s_free_y[i];
        float gx = sx + (float)((int)rev_rand_range(17) - 8) * 2.5f;
        float gy = sy + (float)((int)rev_rand_range(17) - 8) * 2.5f;
        float d = rev_dist_cm(sx, sy, gx, gy);
        if (d < 2.0f || d > 20.0f) continue;
        float dmin = 1e9f;
        int steps = (int)ceilf(d / 0.5f);
        for (int k = 0; k <= steps; k++) {
            float t = (float)k / (float)steps;
            float v = rev_square_clearance(sx + (gx - sx) * t, sy + (gy - sy) * t, REV_HALF_CM);
            if (v < dmin) dmin = v;
        }
        if (!(dmin >= 0.0f && dmin < 0.5f)) continue;
        tested++;
        PlannerPathStats_t st;
        memset(&st, 0, sizeof(st));
        int n = Planner_PlanPathWithYaw(rev_map_from_world(sx, sy), rev_map_from_world(gx, gy), 0, 0, s_out_path,
                                        MAX_PATH_LEN, &st);
        if (n >= 2) continue;
        int detour_ok = 0;
        for (int side = -1; side <= 1 && !detour_ok; side += 2) {
            for (int off = 4; off <= 14 && !detour_ok; off += 2) {
                float mx = (sx + gx) * 0.5f, my = (sy + gy) * 0.5f;
                float dx = gx - sx, dy = gy - sy;
                float len = sqrtf(dx * dx + dy * dy);
                if (len < 1e-3f) break;
                float px = mx + (-dy / len) * (float)off * (float)side;
                float py = my + (dx / len) * (float)off * (float)side;
                float m1 = 1e9f, m2 = 1e9f;
                int s1 = (int)ceilf(rev_dist_cm(sx, sy, px, py) / 0.5f);
                int s2 = (int)ceilf(rev_dist_cm(px, py, gx, gy) / 0.5f);
                for (int k = 0; k <= s1; k++) {
                    float t = (float)k / (float)s1;
                    float v = rev_square_clearance(sx + (px - sx) * t, sy + (py - sy) * t, REV_HALF_CM);
                    if (v < m1) m1 = v;
                }
                for (int k = 0; k <= s2; k++) {
                    float t = (float)k / (float)s2;
                    float v = rev_square_clearance(px + (gx - px) * t, py + (gy - py) * t, REV_HALF_CM);
                    if (v < m2) m2 = v;
                }
                if (m1 >= 0.6f && m2 >= 0.6f) detour_ok = 1;
            }
        }
        if (detour_ok) {
            suspicious++;
            if (suspicious <= 8)
                printf("[SUSPICIOUS-SHORT] zone1 start=(%.1f,%.1f) goal=(%.1f,%.1f) d=%.1f direct_min_sq=%.2f "
                       "fail=%u 但存在两段绕行(净空>=0.6)\n",
                       (double)sx, (double)sy, (double)gx, (double)gy, (double)d, (double)dmin,
                       (unsigned)st.fail_reason);
        }
    }
    printf("[SHORT-DIRECT] tested_gray=%d suspicious=%d\n", tested, suspicious);
    if (suspicious) {
        printf("[PROBLEM] 短距离直达灰区：存在可绕行路径但规划失败（直达后不回退 A*）\n");
        g_problems++;
    }
}

static void rev_test_capacity_and_stability(void) {
    rev_init_zone(1);
    PlannerPathStats_t st;
    memset(&st, 0, sizeof(st));
    Point2D start = rev_map_from_world(50, 195), goal = rev_map_from_world(160, 195);
    int n = Planner_PlanPathWithYaw(start, goal, 0, 0, s_out_path, MAX_PATH_LEN, &st);
    printf("[CAP] base plan n=%d\n", n);
    if (n >= 2) {
        for (int cap = 2; cap <= n + 1 && cap <= 12; cap++) {
            static Point2D buf[MAX_PATH_LEN + 8];
            memset(buf, 0x55, sizeof(buf));
            PlannerPathStats_t s2;
            memset(&s2, 0, sizeof(s2));
            int m = Planner_PlanPathWithYaw(start, goal, 0, 0, buf, cap, &s2);
            if (m < 0 || m > cap) {
                printf("[PROBLEM] out_cap=%d returned %d\n", cap, m);
                g_problems++;
            }
            for (int k = 0; k < 8; k++) {
                if (((uint8_t *)&buf[MAX_PATH_LEN])[k] != 0x55) {
                    printf("[PROBLEM] out_cap=%d 写越界\n", cap);
                    g_problems++;
                    break;
                }
            }
        }
        PlannerPathStats_t s3;
        memset(&s3, 0, sizeof(s3));
        int n2 = Planner_PlanPathWithYaw(start, goal, 0, 0, s_out_path, MAX_PATH_LEN, &s3);
        if (n2 != n || s3.rotation_index != st.rotation_index || s3.heading_after_deg != st.heading_after_deg) {
            printf("[PROBLEM] 重复规划不稳定 n=%d/%d\n", n, n2);
            g_problems++;
        } else {
            printf("[CAP] repeat stability OK (n=%d)\n", n);
        }
    }
}

static void rev_timing(void) {
    rev_init_zone(1);
    rev_build_free_points();
    enum { N = 200 };
    clock_t t0 = clock();
    int done = 0;
    while (done < N) {
        int i = (int)rev_rand_range((uint32_t)s_free_n);
        int j = (int)rev_rand_range((uint32_t)s_free_n);
        if (i == j) continue;
        PlannerPathStats_t st;
        (void)Planner_PlanPathWithYaw(rev_map_from_world(s_free_x[i], s_free_y[i]),
                                      rev_map_from_world(s_free_x[j], s_free_y[j]), rev_rand_float(-180, 180),
                                      rev_rand_float(-180, 180), s_out_path, MAX_PATH_LEN, &st);
        done++;
    }
    double ms = (double)(clock() - t0) * 1000.0 / (double)CLOCKS_PER_SEC;
    printf("[TIMING] host %d x PlanPathWithYaw = %.3fms/call（含 RebuildClearance）\n", N, ms / (double)N);
}

/* ==========================================================================================
 * 10. main
 * ========================================================================================== */

int main(int argc, char **argv) {
    const char *module = (argc > 1) ? argv[1] : "all";
    printf("=========================================================\n");
    printf(" Planner v2 方形车体（30x30, half=%.1f, diag=%.2f）独立评审\n", (double)REV_HALF_CM,
           (double)REV_HALF_DIAG_CM);
    printf(" 端点区=%.0fcm rotClear=%.2f soft=%.1f hard=%.1f\n", (double)REV_ENDPOINT_ZONE_CM,
           (double)REV_ROT_CLEAR_CM, (double)PLANNER_SAFE_MARGIN_CM, (double)PLANNER_SQUARE_HARD_MARGIN_CM);
    printf("=========================================================\n");
    rev_stats_reset(&g_st);

    if (!strcmp(module, "all") || !strcmp(module, "map")) {
        rev_test_map(1);
        rev_test_map(2);
    }
    if (!strcmp(module, "all") || !strcmp(module, "targeted")) {
        rev_test_task_points();
        rev_test_targeted();
        rev_test_short_direct();
        rev_test_capacity_and_stability();
    }
    if (!strcmp(module, "all") || !strcmp(module, "fuzz")) {
        rev_fuzz_static(1, 1500);
        rev_fuzz_static(2, 1500);
    }
    if (!strcmp(module, "all") || !strcmp(module, "dynamic")) {
        rev_fuzz_dynamic(1, 500);
        rev_fuzz_dynamic(2, 500);
    }
    if (!strcmp(module, "all") || !strcmp(module, "timing")) {
        rev_timing();
    }

    printf("\n===== 汇总 =====\n");
    printf("plans=%ld ok=%ld fail=%ld (endpoint=%ld no-solution=%ld no-rotation=%ld badparam=%ld other=%ld)\n",
           g_st.plans, g_st.ok, g_st.fail, g_st.fail_endpoint, g_st.fail_no_solution, g_st.fail_no_rotation,
           g_st.fail_bad_param, g_st.fail_other);
    printf("checks: endpoint_mismatch=%ld heading_mismatch=%ld rotation_unsafe=%ld rot_idx_bad=%ld "
           "waypoints_mismatch=%ld stats_min_mismatch=%ld nan=%ld dup=%ld\n",
           g_st.endpoint_mismatch, g_st.heading_mismatch, g_st.rotation_unsafe, g_st.rotation_index_bad,
           g_st.waypoints_mismatch, g_st.stats_min_mismatch, g_st.nan_bad, g_st.dup_bad);
    printf("safety: SAT_general=%ld SAT_endpoint(>1cm)=%ld square_general=%ld square_endpoint=%ld\n",
           g_st.sat_violation_general, g_st.sat_violation_endpoint, g_st.square_violation_general,
           g_st.square_violation_endpoint);
    printf("min: square_general=%.3f square_endpoint=%.3f rotation_clearance=%.3f (gen_allowed=%.2f ep_allowed=%.2f)\n",
           g_st.min_square_general, g_st.min_square_endpoint, g_st.min_rot_clearance, g_st.gen_allowed,
           g_st.ep_allowed);
    printf("suspicious_false_negative(5cm独立网格可达)=%ld\n", g_st.suspicious_false_negative_hint);
    printf("\n=== 最差路径（按最小方形净空）===\n");
    for (int i = 0; i < g_worst_count; i++) {
        RevWorst *w = &g_worst[i];
        printf("[WORST %d] zone=%d start=(%.1f,%.1f,%.1f) goal=(%.1f,%.1f,%.1f) n=%d tier=%d rot=%d min_sq=%.3f\n",
               i + 1, w->zone, (double)w->sx, (double)w->sy, (double)w->syaw, (double)w->gx, (double)w->gy,
               (double)w->gyaw, w->n, w->tier, w->rot_idx, (double)w->meas_min_sq);
        rev_print_path_world(w->path, w->n);
    }
    printf("\nproblems(断言失败)=%d\n", g_problems);
    return (g_problems == 0) ? 0 : 2;
}
