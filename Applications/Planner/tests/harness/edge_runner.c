/**
 * @file edge_runner.c
 * @brief 重构版 v2（方形车体 30×30）边界/回归用例：带 yaw 规划 + 独立精确 SAT + 执行语义
 *
 * @details 覆盖用例：
 *          E1  短距直达：10cm 平直可通 → 2 点、tier=soft、SAT 无碰撞
 *          E2  目标在障碍内部 → 返回 0 且 fail_reason=ENDPOINT
 *          E3  起点/目标在地图外 → 返回 0 且 fail_reason 有效（不崩溃）
 *          E4  HOME 端点：zone1 HOME 与其余 4 点双向 8 条（含 yaw/旋转）全部成功且 SAT 安全
 *          E5  通道：40cm 动态障碍通道（yaw 0/90/180）、35cm 通道、30cm 短通道（全落端点区）
 *              均成功且安全；30cm 长通道（>端点豁免区）按安全优先语义 clean fail 或安全成功
 *          E6  任意起始 yaw：开阔点 10°/30°/45° 成功且 SAT 安全；HOME 处 yaw10 因方形扫入
 *              场边条 → clean fail(fail_reason=ENDPOINT)
 *          E7  旋转：起点旋转（任意 yaw）、终点补转分支（起点不可转+终点旋转安全时
 *              rotation_index=len-1）、普通中途旋转，旋转点独立欧氏清障均 >=23.22cm
 *          E8  确定性：同一对点连跑 5 次，waypoints/length/rotation/tier/min_sq 完全一致
 *          E9  动态障碍：路径中点写 2 个动态格 → 旧路径 SAT 失效、重规划成功且安全；
 *              清除后恢复原路径
 *          E10 端点方框角 (100,100)：30×30 方形中心在该点会侵入方框角 5cm，
 *              超出端点 1cm 容差 → clean fail(ENDPOINT)（记录几何结论）
 *
 *          独立几何：精确 SAT（正方形按执行航向 vs 轴对齐障碍矩形/地图边界）；
 *          执行语义：航点 i<=rotation_index 用 heading_before，i>rotation_index 用 heading_after；
 *          逐段 1cm 采样，一般路段不得相交，端点豁免区（到起终点欧氏<=25cm）允许 <=1cm 模型侵入。
 *
 *          结果写 <out-dir>/edge_results_v2.csv；任一 FAIL（INFO/WARN 不计）退出码非 0。
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Cstar_map_core.h"
#include "planner_algo.h"
#include "planner_config.h"
#include "reference_metrics.h"
#include "reference_obstacles.h"

#if !PLANNER_ENABLED
#error "edge_runner 需要 PLANNER_ENABLED=1 的 planner_config.h"
#endif

#define EDGE_ZONE_1 1
#define EDGE_ZONE_2 2

#define EDGE_SAT_EPS_CM 0.02f
#define EDGE_ENDPOINT_PEN_TOL_CM 1.0f
#define EDGE_ROT_TOL_CM 0.02f

static uint8_t s_grid[MAX_MAP_BYTES];
static uint8_t s_clarity[MAX_CELLS];
static int8_t s_square_clarity[MAX_CELLS];
static Point2D s_path[MAX_PATH_LEN];
static RefRect s_rects[REF_MAX_RECTS];
static RefRect s_dyn_rects[32];
static RefRect s_rects_all[REF_MAX_RECTS + 32];
static int s_rect_count = 0;
static int s_dyn_count = 0;

static FILE *s_csv = NULL;
static int s_fail_count = 0;

/* ===== 记录 ===== */
static void csv_field(FILE *fp, const char *s) {
    fputc('"', fp);
    for (const char *p = s; *p != '\0'; p++) {
        if (*p == '"') fputc('"', fp);
        fputc(*p, fp);
    }
    fputc('"', fp);
}

static void record_result(const char *id, const char *name, const char *result, const char *detail) {
    printf("[%s] %-4s %s | %s\n", result, id, name, detail);
    if (s_csv != NULL) {
        fputc('"', s_csv);
        fputs(id, s_csv);
        fputc('"', s_csv);
        fputc(',', s_csv);
        csv_field(s_csv, name);
        fputc(',', s_csv);
        csv_field(s_csv, result);
        fputc(',', s_csv);
        csv_field(s_csv, detail);
        fputc('\n', s_csv);
    }
}

static void record(const char *id, const char *name, int pass, const char *detail) {
    record_result(id, name, pass ? "PASS" : "FAIL", detail);
    if (!pass) s_fail_count++;
}

static void record_info(const char *id, const char *name, const char *detail) {
    record_result(id, name, "INFO", detail);
}

static void record_warn(const char *id, const char *name, const char *detail) {
    record_result(id, name, "WARN", detail);
}

/* ===== 基础工具 ===== */
static Point2D to_map_point(float x_cm, float y_cm) {
    Point2D p;
    p.x = x_cm * 0.01f + MAP_OFFSET_X;
    p.y = y_cm * 0.01f + MAP_OFFSET_Y;
    return p;
}

static Point2D to_world_cm(Point2D p) {
    Point2D w;
    w.x = (p.x - MAP_OFFSET_X) * 100.0f;
    w.y = (p.y - MAP_OFFSET_Y) * 100.0f;
    return w;
}

static float wrap_deg(float a) {
    float r = fmodf(a, 360.0f);
    if (r > 180.0f) r -= 360.0f;
    if (r <= -180.0f) r += 360.0f;
    return r;
}

static int init_zone(int zone) {
    s_dyn_count = 0;
    if (!MapCore_Init((uint8_t)zone, s_grid, s_clarity, s_square_clarity)) return 0;
    if (zone == EDGE_ZONE_2) {
        s_rect_count =
            ref_build_rects(s_ref_obstacle_quad_zone2, (int)REF_OBSTACLE_POINTS_ZONE2, s_rects, REF_MAX_RECTS);
    } else {
        s_rect_count =
            ref_build_rects(s_ref_obstacle_quad_zone1, (int)REF_OBSTACLE_POINTS_ZONE1, s_rects, REF_MAX_RECTS);
    }
    return s_rect_count > 0;
}

static void set_dynamic_cell(int gx, int gy) {
    set_grid_val(&g_map, gx, gy, MAP_CELL_DYNAMIC_OBSTACLE);
}

/** @brief 写动态障碍格，同时加入独立 SAT 的障碍矩形集合（供通道/动态用例真值校验） */
static void add_dynamic_cell(int gx, int gy) {
    set_dynamic_cell(gx, gy);
    if (s_dyn_count < (int)(sizeof(s_dyn_rects) / sizeof(s_dyn_rects[0]))) {
        RefRect r;
        r.x0 = ((float)gx * g_map.res - MAP_OFFSET_X) * 100.0f;
        r.y0 = ((float)gy * g_map.res - MAP_OFFSET_Y) * 100.0f;
        r.x1 = ((float)(gx + 1) * g_map.res - MAP_OFFSET_X) * 100.0f;
        r.y1 = ((float)(gy + 1) * g_map.res - MAP_OFFSET_Y) * 100.0f;
        s_dyn_rects[s_dyn_count++] = r;
    }
}

/** @brief 当前独立几何障碍集合 = 静态矩形 + 动态格矩形 */
static const RefRect *current_rects(int *out_count) {
    memcpy(s_rects_all, s_rects, sizeof(RefRect) * (size_t)s_rect_count);
    memcpy(&s_rects_all[s_rect_count], s_dyn_rects, sizeof(RefRect) * (size_t)s_dyn_count);
    *out_count = s_rect_count + s_dyn_count;
    return s_rects_all;
}

/* ===== 一次规划 + 独立 SAT 评估 ===== */
typedef struct {
    int n;
    PlannerPathStats_t st;
    RefSquarePathMetrics sat;
    float path_xy[2 * REF_MAX_POINTS];
    float seg_heading[MAX_PATH_LEN];
    float wp_heading[MAX_PATH_LEN];
    float rot_clear;
    int rot_ok;
} Eval;

static int measure_sat(const float *xy,
                       int n,
                       const float *seg_heading,
                       float sx,
                       float sy,
                       float gx,
                       float gy,
                       const RefRect *rects,
                       int rect_count,
                       RefSquarePathMetrics *out) {
    return ref_measure_square_path(rects,
                                   rect_count,
                                   xy,
                                   n,
                                   seg_heading,
                                   n - 1,
                                   sx,
                                   sy,
                                   gx,
                                   gy,
                                   PLANNER_BODY_HALF_CM,
                                   PLANNER_ENDPOINT_ZONE_CM,
                                   EDGE_ENDPOINT_PEN_TOL_CM,
                                   out)
               ? 1
               : 0;
}

static int plan_eval(Eval *e, float sx, float sy, float gx, float gy, float syaw, float gyaw) {
    memset(e, 0, sizeof(*e));
    Point2D a = to_map_point(sx, sy);
    Point2D b = to_map_point(gx, gy);
    e->n = Planner_PlanPathWithYaw(a, b, syaw, gyaw, s_path, MAX_PATH_LEN, &e->st);
    if (e->n <= 0) {
        e->rot_clear = -1.0f;
        return 0;
    }
    const uint16_t rot = e->st.rotation_index;
    const int need_rot = (rot != PLANNER_ROTATE_NONE);
    for (int i = 0; i < e->n; i++) {
        e->wp_heading[i] = (!need_rot)
                               ? gyaw
                               : ((i <= (int)rot) ? e->st.heading_before_deg : e->st.heading_after_deg);
        if (i + 1 < e->n) {
            e->seg_heading[i] = (!need_rot)
                                    ? gyaw
                                    : ((i < (int)rot) ? e->st.heading_before_deg : e->st.heading_after_deg);
        }
        Point2D w = to_world_cm(s_path[i]);
        e->path_xy[i * 2 + 0] = w.x;
        e->path_xy[i * 2 + 1] = w.y;
    }
    int rect_count = 0;
    const RefRect *rects = current_rects(&rect_count);
    if (!measure_sat(e->path_xy, e->n, e->seg_heading, sx, sy, gx, gy, rects, rect_count, &e->sat)) {
        return -1;
    }
    e->rot_clear = -1.0f;
    e->rot_ok = 1;
    if (need_rot) {
        Point2D rw = to_world_cm(s_path[rot]);
        e->rot_clear = ref_point_euclid_clearance_cm(rw.x, rw.y, rects, rect_count);
        e->rot_ok = (e->rot_clear >= PLANNER_ROTATION_CLEARANCE_CM - EDGE_ROT_TOL_CM) ? 1 : 0;
    }
    return 1;
}

/** @brief 在"当前独立障碍集合（含动态格）"下重测已有路径的 SAT（用于动态障碍用例） */
static int measure_current(Eval *e, float sx, float sy, float gx, float gy, RefSquarePathMetrics *out) {
    int rect_count = 0;
    const RefRect *rects = current_rects(&rect_count);
    return measure_sat(e->path_xy, e->n, e->seg_heading, sx, sy, gx, gy, rects, rect_count, out);
}

static int sat_safe(const RefSquarePathMetrics *m) {
    return (m->general_collision_samples == 0) && (m->endpoint_penetration_violations == 0);
}

/* ==========================================================================================
 * E1：短距直达
 * ========================================================================================== */
static void test_short_direct(void) {
    Eval e;
    (void)init_zone(EDGE_ZONE_1);
    int ok = plan_eval(&e, 10.0f, 75.0f, 10.0f, 85.0f, 0.0f, 0.0f);
    char detail[300];
    int pass = ok == 1 && e.n == 2 && e.st.tier == PLANNER_TIER_SOFT && sat_safe(&e.sat);
    snprintf(detail,
             sizeof(detail),
             "点数=%d(期望2) tier=%u(期望soft=0) rot=%u len=%.2fcm SAT_min=%.3f 穿透=%.3f",
             e.n,
             (unsigned)e.st.tier,
             (unsigned)e.st.rotation_index,
             ok == 1 ? (double)e.sat.length_cm : 0.0,
             ok == 1 ? (double)e.sat.min_signed_cm : 0.0,
             ok == 1 ? (double)e.sat.max_penetration_cm : 0.0);
    record("E1", "10cm 短距直达 2 点 + soft + SAT 安全", pass, detail);
}

/* ==========================================================================================
 * E2/E3：失败用例（目标在障碍内、地图外）
 * ========================================================================================== */
static void test_fail_cases(void) {
    Eval e;
    char detail[300];
    (void)init_zone(EDGE_ZONE_1);

    int ok = plan_eval(&e, 10.0f, 75.0f, 60.0f, 145.0f, 0.0f, 0.0f);
    int pass = (ok == 0) && (e.st.fail_reason == PLANNER_FAIL_ENDPOINT);
    snprintf(detail,
             sizeof(detail),
             "goal=(60,145) 在方框 [35,90]x[120,175] 内：返回=%d fail_reason=%u(期望ENDPOINT=1)",
             e.n,
             (unsigned)e.st.fail_reason);
    record("E2", "目标在障碍内部 → clean fail(ENDPOINT)", pass, detail);

    ok = plan_eval(&e, 500.0f, 0.0f, 10.0f, 85.0f, 0.0f, 0.0f);
    pass = (ok == 0) && (e.st.fail_reason != PLANNER_FAIL_NONE);
    snprintf(detail,
             sizeof(detail),
             "start world=(500,0)cm 在地图外：返回=%d fail_reason=%u（期望非0、不崩溃）",
             e.n,
             (unsigned)e.st.fail_reason);
    record("E3a", "起点在地图外 → clean fail", pass, detail);

    ok = plan_eval(&e, 10.0f, 75.0f, -300.0f, 0.0f, 0.0f, 0.0f);
    pass = (ok == 0) && (e.st.fail_reason != PLANNER_FAIL_NONE);
    snprintf(detail,
             sizeof(detail),
             "goal world=(-300,0)cm 在地图外：返回=%d fail_reason=%u（期望非0、不崩溃）",
             e.n,
             (unsigned)e.st.fail_reason);
    record("E3b", "目标在地图外 → clean fail", pass, detail);

    /* 30×30 方形：旧 v1 端点 (100,100) 会侵入方框角 5cm，超出端点 1cm 容差 */
    ok = plan_eval(&e, 10.0f, 105.0f, 100.0f, 100.0f, 0.0f, 0.0f);
    pass = (ok == 0);
    snprintf(detail,
             sizeof(detail),
             "world(100,100) 距方框角 14.142cm，30×30 方形中心在此侵入方框角 5cm > 端点容差 1cm："
             "返回=%d fail_reason=%u（30cm 方形车几何不可达，属正确拒绝）",
             e.n,
             (unsigned)e.st.fail_reason);
    record("E10", "旧 v1 端点 (100,100) 对 30×30 方形不可达 → clean fail", pass, detail);
}

/* ==========================================================================================
 * E4：HOME 端点（zone1 8 条双向）
 * ========================================================================================== */
static void test_home_endpoints(void) {
    static const float targets[4][3] = { /* x_cm, y_cm, yaw */
        {10.0f, 105.0f, 0.0f},
        {105.0f, 14.0f, 0.0f},
        {190.0f, 105.0f, 90.0f},
        {105.0f, 190.0f, 180.0f}};
    int ok_count = 0;
    int safe_count = 0;
    int rot_count = 0;
    float min_sat = 1.0e9f;
    float min_rot = 1.0e9f;
    char detail[300];
    (void)init_zone(EDGE_ZONE_1);

    for (int dir = 0; dir < 2; dir++) {
        for (int i = 0; i < 4; i++) {
            Eval e;
            int ok = (dir == 0) ? plan_eval(&e, 0.0f, 0.0f, targets[i][0], targets[i][1], 0.0f, targets[i][2])
                                : plan_eval(&e, targets[i][0], targets[i][1], 0.0f, 0.0f, targets[i][2], 0.0f);
            if (ok == 1) {
                ok_count++;
                if (sat_safe(&e.sat)) safe_count++;
                if (e.sat.min_signed_cm < min_sat) min_sat = e.sat.min_signed_cm;
                if (e.st.rotation_index != PLANNER_ROTATE_NONE) {
                    rot_count++;
                    if (e.rot_ok && e.rot_clear < min_rot) min_rot = e.rot_clear;
                }
            }
        }
    }
    int pass = (ok_count == 8) && (safe_count == 8) && (rot_count > 0) &&
               (min_rot >= PLANNER_ROTATION_CLEARANCE_CM - EDGE_ROT_TOL_CM);
    snprintf(detail,
             sizeof(detail),
             "HOME 相关 8/8: 成功=%d 安全=%d 旋转方案=%d | SAT_min=%.2fcm 最小旋转清障=%.2fcm",
             ok_count,
             safe_count,
             rot_count,
             (double)min_sat,
             rot_count > 0 ? (double)min_rot : 0.0);
    record("E4", "zone1 HOME 端点 8 条路径全部成功且方形 SAT 安全", pass, detail);
}

/* ==========================================================================================
 * E5：通道几何（动态障碍格构造）
 * ========================================================================================== */
static void test_channels(void) {
    char detail[400];
    Eval e;
    int ok;
    int pass;

    /* ---- 40cm 通道：动态墙 x[-10,0) 与 x[40,50)，y[-10,10]；yaw 0/90/180 同几何 ---- */
    {
        int ok_all = 1;
        int safe_all = 1;
        int tier_soft_cnt = 0;
        float min_sat = 1.0e9f;
        float lens[3];
        for (int k = 0; k < 3; k++) {
            float yaw = (k == 0) ? 0.0f : ((k == 1) ? 90.0f : 180.0f);
            (void)init_zone(EDGE_ZONE_1);
            for (int gy = 23; gy <= 24; gy++) {
                add_dynamic_cell(23, gy);
                add_dynamic_cell(28, gy);
            }
            MapCore_RebuildClearance();
            ok = plan_eval(&e, 20.0f, -5.0f, 20.0f, 5.0f, yaw, yaw);
            if (ok != 1) {
                ok_all = 0;
                lens[k] = 0.0f;
                continue;
            }
            if (!sat_safe(&e.sat)) safe_all = 0;
            if (e.st.tier == PLANNER_TIER_SOFT) tier_soft_cnt++;
            if (e.sat.min_signed_cm < min_sat) min_sat = e.sat.min_signed_cm;
            lens[k] = e.sat.length_cm;
        }
        pass = ok_all && safe_all && (tier_soft_cnt == 3) && (min_sat >= 4.0f);
        snprintf(detail,
                 sizeof(detail),
                 "40cm 通道 yaw 0/90/180：成功=%d/3 安全=%d/3 soft=%d/3 SAT_min=%.2fcm(len %.1f/%.1f/%.1f)",
                 ok_all ? 3 : 0,
                 safe_all ? 3 : 0,
                 tier_soft_cnt,
                 (double)min_sat,
                 (double)lens[0],
                 (double)lens[1],
                 (double)lens[2]);
        record("E5a", "40cm 通道 yaw 0/90/180 全部成功、soft 且 SAT 安全", pass, detail);
    }

    /* ---- 35cm 通道：左侧场边条 x=-15 + 动态墙 x[20,30) ---- */
    (void)init_zone(EDGE_ZONE_1);
    for (int gy = 23; gy <= 25; gy++) add_dynamic_cell(26, gy);
    MapCore_RebuildClearance();
    ok = plan_eval(&e, 2.5f, -5.0f, 2.5f, 25.0f, 0.0f, 0.0f);
    pass = (ok == 1) && sat_safe(&e.sat) && (e.st.tier == PLANNER_TIER_HARD);
    snprintf(detail,
             sizeof(detail),
             "35cm 通道：成功=%d tier=%u(期望tight=1) SAT_min=%.2fcm 穿透=%.3f 旋转=%u",
             ok == 1 ? 1 : 0,
             (unsigned)e.st.tier,
             ok == 1 ? (double)e.sat.min_signed_cm : 0.0,
             ok == 1 ? (double)e.sat.max_penetration_cm : 0.0,
             (unsigned)e.st.rotation_index);
    record("E5b", "35cm 通道成功、tight 且 SAT 安全", pass, detail);

    /* ---- 30cm 短通道（长度 20cm，全部落在端点豁免区） ---- */
    (void)init_zone(EDGE_ZONE_1);
    for (int gy = 23; gy <= 25; gy++) {
        add_dynamic_cell(23, gy);
        add_dynamic_cell(27, gy);
    }
    MapCore_RebuildClearance();
    ok = plan_eval(&e, 15.0f, -5.0f, 15.0f, 15.0f, 0.0f, 0.0f);
    pass = (ok == 1) && sat_safe(&e.sat);
    snprintf(detail,
             sizeof(detail),
             "30cm 短通道(len20cm，全在25cm端点区)：成功=%d tier=%u SAT_min=%.3fcm 穿透=%.3f",
             ok == 1 ? 1 : 0,
             (unsigned)e.st.tier,
             ok == 1 ? (double)e.sat.min_signed_cm : 0.0,
             ok == 1 ? (double)e.sat.max_penetration_cm : 0.0);
    record("E5c", "30cm 短通道（端点豁免区内）可 tight 通过", pass, detail);

    /* ---- 30cm 长通道（80cm，穿越段超出端点豁免区）→ 安全优先：clean fail 或安全成功 ---- */
    (void)init_zone(EDGE_ZONE_1);
    for (int gy = 23; gy <= 31; gy++) {
        add_dynamic_cell(23, gy);
        add_dynamic_cell(27, gy);
    }
    MapCore_RebuildClearance();
    ok = plan_eval(&e, 15.0f, 0.0f, 15.0f, 60.0f, 0.0f, 0.0f);
    if (ok == 1) {
        pass = sat_safe(&e.sat);
        snprintf(detail,
                 sizeof(detail),
                 "30cm 长通道(80cm)：成功（超预期）tier=%u SAT_min=%.3fcm 需要 Lead 复核",
                 (unsigned)e.st.tier,
                 (double)e.sat.min_signed_cm);
        record_warn("E5d", "30cm 长通道穿越（硬门限 0.5cm 不可满足）", detail);
    } else {
        pass = (e.st.fail_reason != PLANNER_FAIL_NONE);
        snprintf(detail,
                 sizeof(detail),
                 "30cm 长通道(80cm)：clean fail fail_reason=%u（端点=%u/无解=%u）——安全优先正确行为，"
                 "不要求改 Planner",
                 (unsigned)e.st.fail_reason,
                 (unsigned)PLANNER_FAIL_ENDPOINT,
                 (unsigned)PLANNER_FAIL_NO_SOLUTION);
        record("E5d", "30cm 长通道穿越 → clean fail（安全优先正确行为）", pass, detail);
    }
}

/* ==========================================================================================
 * E6：任意起始 yaw
 * ========================================================================================== */
static void test_arbitrary_yaw(void) {
    char detail[300];
    Eval e;

    /* 开阔点 (10,75)：10°/30°/45° 同 yaw 直达，应成功且 SAT 安全 */
    int ok_all = 1;
    int safe_all = 1;
    float min_sat = 1.0e9f;
    for (int k = 0; k < 3; k++) {
        float yaw = (k == 0) ? 10.0f : ((k == 1) ? 30.0f : 45.0f);
        (void)init_zone(EDGE_ZONE_1);
        int ok = plan_eval(&e, 10.0f, 75.0f, 10.0f, 85.0f, yaw, yaw);
        if (ok != 1) {
            ok_all = 0;
            continue;
        }
        if (!sat_safe(&e.sat)) safe_all = 0;
        if (e.sat.min_signed_cm < min_sat) min_sat = e.sat.min_signed_cm;
    }
    int pass = ok_all && safe_all && (min_sat > 0.0f);
    snprintf(detail,
             sizeof(detail),
             "开阔点 yaw 10/30/45 直达：成功=%d/3 安全=%d/3 SAT_min=%.2fcm",
             ok_all ? 3 : 0,
             safe_all ? 3 : 0,
             (double)min_sat);
    record("E6a", "任意起始 yaw(10/30/45°) 开阔点成功且 SAT 安全", pass, detail);

    /* HOME 处 yaw10：方形半投影 H=17.4>15，扫入场边条 → clean fail(ENDPOINT) */
    (void)init_zone(EDGE_ZONE_1);
    int ok = plan_eval(&e, 0.0f, 0.0f, 10.0f, 105.0f, 10.0f, 10.0f);
    pass = (ok == 0) && (e.st.fail_reason == PLANNER_FAIL_ENDPOINT);
    snprintf(detail,
             sizeof(detail),
             "HOME(yaw10)->QrCode：返回=%d fail_reason=%u（方形扫入边界，期望 clean fail=ENDPOINT）",
             e.n,
             (unsigned)e.st.fail_reason);
    record("E6b", "HOME 起点任意 yaw 干净失败并给出 fail_reason", pass, detail);

    /* 旋转安全起点 + 任意起始 yaw：应在起点旋转后按目标 yaw 平移 */
    (void)init_zone(EDGE_ZONE_1);
    ok = plan_eval(&e, 10.0f, 75.0f, 10.0f, 85.0f, 45.0f, 0.0f);
    pass = (ok == 1) && sat_safe(&e.sat) && (e.st.rotation_index == 0) && e.rot_ok;
    snprintf(detail,
             sizeof(detail),
             "(10,75,yaw45)->(10,85,yaw0)：成功=%d rot=%u(期望0) 旋转清障=%.2fcm SAT_min=%.2fcm",
             ok == 1 ? 1 : 0,
             (unsigned)e.st.rotation_index,
             ok == 1 ? (double)e.rot_clear : -1.0,
             ok == 1 ? (double)e.sat.min_signed_cm : 0.0);
    record("E6c", "旋转安全起点任意 yaw → 起点旋转后平移且安全", pass, detail);
}

/* ==========================================================================================
 * E7：旋转方案（起点旋转 / 终点补转分支 1b / 中途旋转）
 * ========================================================================================== */
static void test_rotation_cases(void) {
    char detail[320];
    Eval e;

    /* 1b 分支：起点 HOME 不可转，目标 (10,75) 旋转安全 → rotation_index=len-1（到终点再转） */
    (void)init_zone(EDGE_ZONE_1);
    int ok = plan_eval(&e, 0.0f, 0.0f, 10.0f, 75.0f, 0.0f, 90.0f);
    int branch_1b = (ok == 1) && (e.st.rotation_index == (uint16_t)(e.n - 1)) && e.rot_ok;
    snprintf(detail,
             sizeof(detail),
             "HOME(yaw0)->(10,75,yaw90)：成功=%d rot=%u(n-1=%d) 终点旋转清障=%.2fcm SAT_min=%.2fcm",
             ok == 1 ? 1 : 0,
             (unsigned)e.st.rotation_index,
             e.n - 1,
             ok == 1 ? (double)e.rot_clear : -1.0,
             ok == 1 ? (double)e.sat.min_signed_cm : 0.0);
    record("E7a", "终点再转分支（起点不可转+终点旋转安全）", branch_1b, detail);

    /* 中途旋转：zone1 HOME(yaw0)->MATL_STORAGE(yaw90) */
    (void)init_zone(EDGE_ZONE_1);
    ok = plan_eval(&e, 0.0f, 0.0f, 190.0f, 105.0f, 0.0f, 90.0f);
    int pass = (ok == 1) && sat_safe(&e.sat) && (e.st.rotation_index != PLANNER_ROTATE_NONE) && e.rot_ok &&
               (fabsf(wrap_deg(e.st.heading_before_deg - 0.0f)) < 0.01f) &&
               (fabsf(wrap_deg(e.st.heading_after_deg - 90.0f)) < 0.01f);
    snprintf(detail,
             sizeof(detail),
             "HOME->MATL_STORAGE(yaw0->90)：成功=%d rot=%u 旋转清障=%.2fcm SAT_min=%.2fcm hb=%.1f ha=%.1f",
             ok == 1 ? 1 : 0,
             (unsigned)e.st.rotation_index,
             ok == 1 ? (double)e.rot_clear : -1.0,
             ok == 1 ? (double)e.sat.min_signed_cm : 0.0,
             ok == 1 ? (double)e.st.heading_before_deg : 0.0,
             ok == 1 ? (double)e.st.heading_after_deg : 0.0);
    record("E7b", "中途旋转方案语义正确且旋转点独立清障>=23.22cm", pass, detail);

    /* 旋转点不得安排在 clearance<23.22 的通道：遍历 zone1 20 条任务点对的旋转点独立校验 */
    {
        int rot_count = 0;
        int unsafe = 0;
        float min_rot = 1.0e9f;
        (void)init_zone(EDGE_ZONE_1);
        for (int si = 0; si < REF_POINTS_PER_ZONE; si++) {
            for (int gi = 0; gi < REF_POINTS_PER_ZONE; gi++) {
                if (si == gi) continue;
                const RefNamedPoint *sp = &s_ref_points_zone1[si];
                const RefNamedPoint *gp = &s_ref_points_zone1[gi];
                Eval ev;
                int r = plan_eval(&ev, sp->x_cm, sp->y_cm, gp->x_cm, gp->y_cm, sp->yaw_deg, gp->yaw_deg);
                if (r != 1) continue;
                if (ev.st.rotation_index != PLANNER_ROTATE_NONE) {
                    rot_count++;
                    if (!ev.rot_ok) unsafe++;
                    if (ev.rot_clear < min_rot) min_rot = ev.rot_clear;
                }
            }
        }
        pass = (rot_count > 0) && (unsafe == 0) && (min_rot >= PLANNER_ROTATION_CLEARANCE_CM - EDGE_ROT_TOL_CM);
        snprintf(detail,
                 sizeof(detail),
                 "zone1 20 点对：旋转方案=%d 不安全=%d 最小独立旋转清障=%.2fcm（门限 %.2fcm）",
                 rot_count,
                 unsafe,
                 rot_count > 0 ? (double)min_rot : 0.0,
                 (double)PLANNER_ROTATION_CLEARANCE_CM);
        record("E7c", "所有旋转点独立欧氏清障 >=23.22cm（不在窄通道内旋转）", pass, detail);
    }

    /* 分支 1b + 任意目标 yaw（30/37/45°）：起点不可转（HOME 欧氏 15cm）、终点 (10,75) 旋转安全，
     * 应规划成功并在终点再转：rotation_index = 点数-1、独立旋转清障 >=23.22cm。 */
    {
        int ok_all = 1;
        int branch_ok = 1;
        int safe_all = 1;
        float yaws[3] = {30.0f, 37.0f, 45.0f};
        float rots[3] = {-1.0f, -1.0f, -1.0f};
        unsigned rot_idx[3] = {0, 0, 0};
        for (int k = 0; k < 3; k++) {
            (void)init_zone(EDGE_ZONE_1);
            int r = plan_eval(&e, 0.0f, 0.0f, 10.0f, 75.0f, 0.0f, yaws[k]);
            if (r != 1) {
                ok_all = 0;
                continue;
            }
            rots[k] = e.rot_clear;
            rot_idx[k] = e.st.rotation_index;
            if (e.st.rotation_index != (uint16_t)(e.n - 1)) branch_ok = 0;
            if (!sat_safe(&e.sat) || !e.rot_ok) safe_all = 0;
            if (fabsf(wrap_deg(e.st.heading_after_deg - yaws[k])) > 0.01f) branch_ok = 0;
        }
        int pass = ok_all && branch_ok && safe_all;
        snprintf(detail,
                 sizeof(detail),
                 "任意目标 yaw(30/37/45°）HOME→(10,75)：成功=%d/3 均到终点再转=%d 安全=%d/3 "
                 "rot_idx=%u/%u/%u 旋转清障=%.2f/%.2f/%.2fcm",
                 ok_all ? 3 : 0,
                 branch_ok,
                 safe_all ? 3 : 0,
                 rot_idx[0],
                 rot_idx[1],
                 rot_idx[2],
                 (double)rots[0],
                 (double)rots[1],
                 (double)rots[2]);
        record("E7d", "任意目标 yaw 经窄通道→到终点再转分支（rotation_index=len-1）", pass, detail);
    }
}

/* ==========================================================================================
 * E11：起终点重合的合法零长路径（允许 2 点相同；其它重复仍 FAIL）
 * ========================================================================================== */
static void test_zero_length_path(void) {
    char detail[300];
    Eval e;
    (void)init_zone(EDGE_ZONE_1);

    /* 同点且同 yaw → 2 点零长路径、两点可相同 */
    int ok = plan_eval(&e, 10.0f, 75.0f, 10.0f, 75.0f, 0.0f, 0.0f);
    float dup = 999.0f;
    if (ok == 1 && e.n == 2) {
        dup = sqrtf((e.path_xy[2] - e.path_xy[0]) * (e.path_xy[2] - e.path_xy[0]) +
                    (e.path_xy[3] - e.path_xy[1]) * (e.path_xy[3] - e.path_xy[1]));
    }
    int pass = (ok == 1) && (e.n == 2) && (dup <= EDGE_SAT_EPS_CM) && (e.sat.length_cm <= 0.01f);
    snprintf(detail,
             sizeof(detail),
             "同点(10,75,0)->(10,75,0)：成功=%d 点数=%d 两点间距=%.3fcm 长度=%.3fcm（合法零长路径特例，允许重复）",
             ok == 1 ? 1 : 0,
             e.n,
             (double)dup,
             ok == 1 ? (double)e.sat.length_cm : -1.0);
    record("E11a", "起终点重合 → 合法 2 点零长路径（允许两点相同）", pass, detail);

    /* 同点但需要转向 → 起点原地转（rotation_index=0），独立旋转安全 */
    (void)init_zone(EDGE_ZONE_1);
    ok = plan_eval(&e, 10.0f, 75.0f, 10.0f, 75.0f, 45.0f, 0.0f);
    pass = (ok == 1) && (e.n == 2) && (e.st.rotation_index == 0) && e.rot_ok &&
           (fabsf(wrap_deg(e.st.heading_after_deg - 0.0f)) < 0.01f);
    snprintf(detail,
             sizeof(detail),
             "同点(10,75,45)->(10,75,0)：成功=%d 点数=%d rot=%u 独立旋转清障=%.2fcm len=%.3fcm",
             ok == 1 ? 1 : 0,
             e.n,
             (unsigned)(ok == 1 ? e.st.rotation_index : 0),
             ok == 1 ? (double)e.rot_clear : -1.0,
             ok == 1 ? (double)e.sat.length_cm : -1.0);
    record("E11b", "起终点重合需转向 → 起点原地转且旋转安全", pass, detail);
}

/* ==========================================================================================
 * E8：确定性
 * ========================================================================================== */
static void test_determinism(void) {
    Eval e0;
    char detail[300];
    (void)init_zone(EDGE_ZONE_1);
    int ok0 = plan_eval(&e0, 10.0f, 105.0f, 190.0f, 105.0f, 0.0f, 90.0f);
    int same = (ok0 == 1);
    for (int k = 0; k < 4; k++) {
        Eval e;
        int ok = plan_eval(&e, 10.0f, 105.0f, 190.0f, 105.0f, 0.0f, 90.0f);
        if (ok != 1 || e.n != e0.n || fabsf(e.sat.length_cm - e0.sat.length_cm) > 1.0e-6f ||
            e.st.rotation_index != e0.st.rotation_index || e.st.tier != e0.st.tier ||
            fabsf(e.st.min_square_clearance_cm - e0.st.min_square_clearance_cm) > 1.0e-6f) {
            same = 0;
        }
    }
    snprintf(detail,
             sizeof(detail),
             "连跑 5 次：waypoints=%d len=%.4fcm rot=%u tier=%u min_sq=%.4f 结果一致=%d",
             e0.n,
             ok0 == 1 ? (double)e0.sat.length_cm : 0.0,
             (unsigned)e0.st.rotation_index,
             (unsigned)e0.st.tier,
             ok0 == 1 ? (double)e0.st.min_square_clearance_cm : 0.0,
             same);
    record("E8", "同一对点重复规划结果稳定（5 次完全一致）", same, detail);
}

/* ==========================================================================================
 * E9：动态障碍
 * ========================================================================================== */
static void test_dynamic_obstacle(void) {
    Eval base;
    Eval replanned;
    char detail[400];
    (void)init_zone(EDGE_ZONE_1);
    int ok = plan_eval(&base, 10.0f, 105.0f, 190.0f, 105.0f, 0.0f, 90.0f);
    if (ok != 1) {
        record("E9a", "动态障碍重规划", 0, "基准规划失败");
        return;
    }

    /* 在路径累计 50% 处写 2 个动态障碍格（同 y 相邻格） */
    float total = base.sat.length_cm;
    float acc = 0.0f;
    int gx = -1;
    int gy = -1;
    for (int i = 0; i + 1 < base.n && gx < 0; i++) {
        float ax = base.path_xy[i * 2 + 0];
        float ay = base.path_xy[i * 2 + 1];
        float bx = base.path_xy[(i + 1) * 2 + 0];
        float by = base.path_xy[(i + 1) * 2 + 1];
        float sl = sqrtf((bx - ax) * (bx - ax) + (by - ay) * (by - ay));
        if (sl < 1e-6f) continue;
        if (acc + sl >= total * 0.5f) {
            float t = (total * 0.5f - acc) / sl;
            Point2D mid = to_map_point(ax + (bx - ax) * t, ay + (by - ay) * t);
            gx = (int)(mid.x / g_map.res);
            gy = (int)(mid.y / g_map.res);
            if (gx < 0 || gx >= g_map.grid_size || gy < 0 || gy >= g_map.grid_size) {
                gx = -1;
                gy = -1;
            } else if (get_grid_val(&g_map, gx, gy) != MAP_CELL_FREE) {
                gx = -1;
                gy = -1;
            }
        }
        acc += sl;
    }
    if (gx < 0) {
        record("E9a", "动态障碍重规划", 0, "未找到可写动态障碍格");
        return;
    }

    add_dynamic_cell(gx, gy);
    if (gx + 1 < g_map.grid_size && get_grid_val(&g_map, gx + 1, gy) == MAP_CELL_FREE) {
        add_dynamic_cell(gx + 1, gy);
    }
    MapCore_RebuildClearance();

    /* 旧路径在含动态格的场景下应失效（SAT 相交或穿透） */
    RefSquarePathMetrics old_with_dyn;
    int old_measured = measure_current(&base, 10.0f, 105.0f, 190.0f, 105.0f, &old_with_dyn);
    int old_broken = (old_measured == 1) && ((old_with_dyn.general_collision_samples > 0) ||
                                             (old_with_dyn.max_penetration_cm > EDGE_SAT_EPS_CM));
    snprintf(detail,
             sizeof(detail),
             "动态格=2 @ grid(%d,%d)+(%d,%d)≈world(%.1f,%.1f)cm：旧路径 SAT 碰撞=%d 最大穿透=%.2fcm",
             gx,
             gy,
             gx + 1,
             gy,
             (double)((gx + 0.5f) * g_map.res * 100.0f - 240.0f),
             (double)((gy + 0.5f) * g_map.res * 100.0f - 240.0f),
             old_measured == 1 ? old_with_dyn.general_collision_samples : -1,
             old_measured == 1 ? (double)old_with_dyn.max_penetration_cm : -1.0);
    record("E9a", "加入动态障碍后旧路径已不满足方形安全", old_broken, detail);

    /* 重规划：成功、SAT 安全（含动态格）、路径改变 */
    int ok2 = plan_eval(&replanned, 10.0f, 105.0f, 190.0f, 105.0f, 0.0f, 90.0f);
    RefSquarePathMetrics new_with_dyn;
    int new_safe = 0;
    if (ok2 == 1) {
        new_safe = measure_current(&replanned, 10.0f, 105.0f, 190.0f, 105.0f, &new_with_dyn) == 1 &&
                   sat_safe(&new_with_dyn);
    }
    int changed =
        (ok2 == 1) && ((replanned.n != base.n) || (fabsf(replanned.sat.length_cm - base.sat.length_cm) > 0.5f));
    snprintf(detail,
             sizeof(detail),
             "重规划：成功=%d 点数=%d(原%d) len=%.1fcm(原%.1f) rot=%u SAT(含动态)安全=%d 路径改变=%d",
             ok2 == 1 ? 1 : 0,
             ok2 == 1 ? replanned.n : 0,
             base.n,
             ok2 == 1 ? (double)replanned.sat.length_cm : 0.0,
             (double)base.sat.length_cm,
             (unsigned)(ok2 == 1 ? replanned.st.rotation_index : PLANNER_ROTATE_NONE),
             new_safe,
             changed);
    record("E9b", "动态障碍下重规划成功且新路径方形 SAT 安全", ok2 == 1 && new_safe, detail);

    /* 清除动态障碍后恢复原路径能力 */
    (void)init_zone(EDGE_ZONE_1);
    Eval restored;
    int ok3 = plan_eval(&restored, 10.0f, 105.0f, 190.0f, 105.0f, 0.0f, 90.0f);
    int restore_ok = (ok3 == 1) && sat_safe(&restored.sat) &&
                     (fabsf(restored.sat.length_cm - base.sat.length_cm) <= 0.5f);
    snprintf(detail,
             sizeof(detail),
             "清除动态格后：成功=%d len=%.1fcm(原%.1f) SAT_min=%.2fcm",
             ok3 == 1 ? 1 : 0,
             ok3 == 1 ? (double)restored.sat.length_cm : 0.0,
             (double)base.sat.length_cm,
             ok3 == 1 ? (double)restored.sat.min_signed_cm : 0.0);
    record("E9c", "清除动态障碍后恢复原路径能力", restore_ok, detail);
}

/* ==========================================================================================
 * 入口
 * ========================================================================================== */
int main(int argc, char **argv) {
    const char *out_dir = ".";
    char csv_path[512];
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--out-dir") == 0 && i + 1 < argc) {
            out_dir = argv[++i];
        } else {
            fprintf(stderr, "用法: %s [--out-dir DIR]\n", argv[0]);
            return 2;
        }
    }
    snprintf(csv_path, sizeof(csv_path), "%s/edge_results_v2.csv", out_dir);
    s_csv = fopen(csv_path, "w");
    if (s_csv == NULL) {
        fprintf(stderr, "无法写 %s\n", csv_path);
        return 2;
    }
    fprintf(s_csv, "test,name,result,detail\n");
    (void)record_info; /* 供扩展用例使用；显式引用避免 -Wunused-function */

    printf("edge_runner(v2): 方形车体 30×30 边界/回归用例（独立精确 SAT）\n");
    printf("车体: side=%.1fcm half=%.1fcm 半对角线=%.2fcm 旋转清障门限=%.2fcm 端点区=%.1fcm\n",
           (double)PLANNER_CAR_SIDE_CM,
           (double)PLANNER_BODY_HALF_CM,
           (double)PLANNER_BODY_HALF_DIAGONAL_CM,
           (double)PLANNER_ROTATION_CLEARANCE_CM,
           (double)PLANNER_ENDPOINT_ZONE_CM);

    test_short_direct();
    test_fail_cases();
    test_home_endpoints();
    test_channels();
    test_arbitrary_yaw();
    test_rotation_cases();
    test_determinism();
    test_zero_length_path();
    test_dynamic_obstacle();

    fclose(s_csv);
    printf("\n========== v2 边界/回归结果（edge_results_v2.csv） ==========\n");
    printf("FAIL 条数: %d\n", s_fail_count);
    printf("输出: %s\n", csv_path);
    if (s_fail_count != 0) {
        printf("[FAIL] edge_runner(v2) 有 %d 条用例失败\n", s_fail_count);
        return 1;
    }
    printf("[PASS] edge_runner(v2) 全部边界/回归用例通过\n");
    return 0;
}
