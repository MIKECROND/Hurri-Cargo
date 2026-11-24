/**
 * @file new_runner.c
 * @brief 重构版 v2（方形车体 30×30）主机评测驱动：带 yaw 规划 + 独立精确 SAT 真值 + 硬性验收断言
 *
 * @details 被测接口：
 *          - MapCore_Init(zone, grid, clarity, square_clarity)
 *          - Planner_PlanPathWithYaw(start, goal, start_yaw, goal_yaw, ...)
 *          输入：2 zone × 5 点位（yaw 取自 nav_map_data.c 规划器版）的全部有向点对 = 40 条。
 *
 *          独立真值（不调用 MapCore 判定）：
 *          - 精确 SAT：30×30 正方形按"每段实际执行航向"旋转，与轴对齐障碍矩形/地图边界求
 *            相交与穿透深度（reference_metrics.c 的 ref_square_scene_clearance_cm）；
 *          - 按执行语义逐段 1cm 采样：航点 i<=rotation_index 用 heading_before，i>rotation_index
 *            用 heading_after；平移段朝向决定方形足迹；
 *          - 旋转安全：旋转点到障碍/边界欧氏距离 >= PLANNER_ROTATION_CLEARANCE_CM(23.22cm)；
 *          - 端点豁免区：到起终点欧氏 <=PLANNER_ENDPOINT_ZONE_CM(25cm)，允许 <=1cm 模型侵入。
 *
 *          验收（任一失败退出码非 0，结果写 acceptance_v2.csv）：
 *            1) 40/40 规划成功；
 *            2) 一般路段方形净空 >=1.5cm（独立 1cm SAT；= hard 2.0 - VALIDATION_TOL 0.25 - 余量 0.25）；
 *            3) 端点豁免区侵入 <= 1cm；
 *            4) 旋转点独立几何欧氏清障 >= 23.22cm（含终点补转）；
 *            5) 朝向语义：heading_before=起始实际 yaw、heading_after=目标 yaw（无需旋转时两者=目标 yaw）；
 *            6) 终点精确、相邻点无重复、stats.waypoints/length 与实测一致。
 *
 * 编译（源码用 v2 工作区）：
 *   gcc -std=c99 -O2 -Wall -Wextra -DPLANNER_HOST_TEST -msse2 -mfpmath=sse \
 *       -I Applications/Planner -I Applications/Planner/utils -I Applications/Planner/tests/harness \
 *       Applications/Planner/utils/Cstar_map_core.c Applications/Planner/utils/planner_algo.c \
 *       Applications/Planner/utils/priority_queue.c \
 *       Applications/Planner/tests/harness/reference_metrics.c \
 *       Applications/Planner/tests/harness/new_runner.c -o new_runner.exe -lm
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#if defined(_WIN32)
#include <windows.h>
#endif

#include "Cstar_map_core.h"
#include "planner_algo.h"
#include "planner_config.h"
#include "reference_metrics.h"
#include "reference_obstacles.h"

#if !PLANNER_ENABLED
#error "new_runner 需要 PLANNER_ENABLED=1 的 planner_config.h"
#endif

#define NEW_ZONE_1 1
#define NEW_ZONE_2 2
#define NEW_MIN_SUCCESS 40

/* 独立真值判定门限 */
#define NEW_SAT_COLLISION_EPS_CM 0.02f /* SAT signed 噪声容差：低于 -0.02cm 才算"相交" */
/* 一般路段独立 1cm 采样方形净空合格门限：hard=2.0 - VALIDATION_TOL(0.25) - 采样/数值余量 0.25 = 1.5cm */
#define NEW_GENERAL_MIN_SIGNED_CM 1.5f
#define NEW_ENDPOINT_PEN_TOL_CM 1.0f /* 端点豁免区允许的模型侵入深度 (cm) */
#define NEW_ENDPOINT_TOL_CM 1.0e-3f /* 起点/终点精确一致容差 (cm) */
#define NEW_DUP_TOL_CM 1.0e-3f /* 相邻点"重复"判定 */
#define NEW_ROT_CLEARANCE_TOL_CM 0.02f /* 旋转清障独立值相比门限的噪声容差 */
#define NEW_STATS_LEN_DIFF_MAX_CM 0.5f /* stats.length 与独立测量允许差 */
#define NEW_HEADING_TOL_DEG 1.0e-2f /* 朝向语义允许差（度） */

static uint8_t s_grid[MAX_MAP_BYTES];
static uint8_t s_clarity[MAX_CELLS];
static int8_t s_square_clarity[MAX_CELLS];
static Point2D s_path[MAX_PATH_LEN];
static float s_path_xy[2 * REF_MAX_POINTS];
static float s_seg_heading[MAX_PATH_LEN];
static float s_wp_heading[MAX_PATH_LEN];
static RefRect s_rects[REF_MAX_RECTS];
static int s_rect_count = 0;

/* ===== PASS/FAIL 记录 ===== */
static FILE *s_accept_csv = NULL;
static int s_fail_count = 0;

static void csv_field(FILE *fp, const char *s) {
    fputc('"', fp);
    for (const char *p = s; *p != '\0'; p++) {
        if (*p == '"') fputc('"', fp);
        fputc(*p, fp);
    }
    fputc('"', fp);
}

static void record(const char *check, int pass, const char *detail) {
    printf("[%s] %s | %s\n", pass ? "PASS" : "FAIL", check, detail);
    if (s_accept_csv != NULL) {
        csv_field(s_accept_csv, check);
        fputc(',', s_accept_csv);
        csv_field(s_accept_csv, pass ? "PASS" : "FAIL");
        fputc(',', s_accept_csv);
        csv_field(s_accept_csv, detail);
        fputc('\n', s_accept_csv);
    }
    if (!pass) s_fail_count++;
}

static void record_info(const char *check, const char *detail) {
    printf("[INFO] %s | %s\n", check, detail);
    if (s_accept_csv != NULL) {
        csv_field(s_accept_csv, check);
        fputc(',', s_accept_csv);
        csv_field(s_accept_csv, "INFO");
        fputc(',', s_accept_csv);
        csv_field(s_accept_csv, detail);
        fputc('\n', s_accept_csv);
    }
}

/** @brief 记录 INFO（不计入失败） */

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

static float dist_world_cm(float ax, float ay, float bx, float by) {
    float dx = ax - bx;
    float dy = ay - by;
    return sqrtf(dx * dx + dy * dy);
}

static float wrap_deg(float a) {
    float r = fmodf(a, 360.0f);
    if (r > 180.0f) r -= 360.0f;
    if (r <= -180.0f) r += 360.0f;
    return r;
}

static double now_ms(void) {
#if defined(_WIN32)
    static LARGE_INTEGER freq;
    static int inited = 0;
    LARGE_INTEGER c;
    if (!inited) {
        QueryPerformanceFrequency(&freq);
        inited = 1;
    }
    QueryPerformanceCounter(&c);
    return (double)c.QuadPart * 1000.0 / (double)freq.QuadPart;
#else
    return (double)clock() * 1000.0 / (double)CLOCKS_PER_SEC;
#endif
}

static int build_reference_rects(int zone) {
    if (zone == NEW_ZONE_2) {
        return ref_build_rects(
            s_ref_obstacle_quad_zone2, (int)REF_OBSTACLE_POINTS_ZONE2, s_rects, REF_MAX_RECTS);
    }
    return ref_build_rects(s_ref_obstacle_quad_zone1, (int)REF_OBSTACLE_POINTS_ZONE1, s_rects, REF_MAX_RECTS);
}

/* ===== zone 汇总 ===== */
typedef struct {
    int total;
    int ok;
    int fail;
    int soft;
    int tight;
    int rotations;
    int rot_start; /* rotation_index == 0 */
    int rot_end; /* rotation_index == waypoints-1 */
    double sum_len;
    double min_len;
    double max_len;
    int sum_wp;
    int max_wp;
    double sum_plan_ms;
    double min_plan_ms;
    double max_plan_ms;
    float min_sat_signed;
    float min_general_signed; /* 一般路段（端点区外）最小独立 SAT signed */
    float min_rot_clear;
    int sat_collision_pairs;
    int general_below_min_pairs; /* 一般路段最小 signed < NEW_GENERAL_MIN_SIGNED_CM 的点对 */
    int endpoint_pen_violation_pairs;
    int rot_unsafe_pairs;
    int heading_bad_pairs;
    int endpoint_bad_pairs;
    int adjacent_bad_pairs;
    int zero_len_pairs; /* 起终点几乎重合（<1cm）的合法零长路径特例 */
    int stats_bad_pairs;
} ZoneSummary;

static ZoneSummary g_z1;
static ZoneSummary g_z2;

static void run_zone(int zone, const RefNamedPoint *pts, const char *out_dir) {
    char pair_path[512];
    char path_path[512];
    snprintf(pair_path, sizeof(pair_path), "%s/newv2_zone%d.csv", out_dir, zone);
    snprintf(path_path, sizeof(path_path), "%s/newv2_paths_zone%d.csv", out_dir, zone);

    FILE *f_pair = fopen(pair_path, "w");
    FILE *f_path = fopen(path_path, "w");
    if (f_pair == NULL || f_path == NULL) {
        printf("[FAIL] zone%d 无法写输出 CSV\n", zone);
        if (f_pair) fclose(f_pair);
        if (f_path) fclose(f_path);
        s_fail_count++;
        return;
    }

    fprintf(f_pair,
            "zone,start,goal,start_yaw_deg,goal_yaw_deg,ok,waypoints,expanded_nodes,rotation_index,"
            "heading_before_deg,heading_after_deg,length_cm,min_square_clearance_cm,rotation_clearance_cm,"
            "min_clearance_cm,tier,fail_reason,plan_ms,sat_min_signed_cm,sat_min_general_signed_cm,"
            "sat_min_endpoint_signed_cm,sat_max_penetration_cm,sat_general_collision_samples,"
            "sat_endpoint_pen_violations,sat_rot_clear_cm,sat_rot_safe,heading_semantics_ok,"
            "endpoint_exact_ok,adjacent_ok,min_adjacent_cm,zero_len_path\n");
    fprintf(f_path,
            "zone,start,goal,start_yaw_deg,goal_yaw_deg,wp_idx,world_x_cm,world_y_cm,heading_deg,"
            "rotation_index,square_clearance_cm,map_square_clearance_cm\n");

    ZoneSummary sum;
    memset(&sum, 0, sizeof(sum));
    sum.min_len = 1.0e9;
    sum.min_plan_ms = 1.0e9;
    sum.min_sat_signed = 1.0e9;
    sum.min_general_signed = 1.0e9;
    sum.min_rot_clear = 1.0e9;

    printf("\n===== v2 new zone%d（方形车体 30×30，Planner_PlanPathWithYaw）=====\n", zone);

    for (int si = 0; si < REF_POINTS_PER_ZONE; si++) {
        for (int gi = 0; gi < REF_POINTS_PER_ZONE; gi++) {
            if (si == gi) continue;
            const RefNamedPoint *sp = &pts[si];
            const RefNamedPoint *gp = &pts[gi];
            sum.total++;

            Point2D start_map = to_map_point(sp->x_cm, sp->y_cm);
            Point2D goal_map = to_map_point(gp->x_cm, gp->y_cm);

            PlannerPathStats_t stats;
            memset(&stats, 0, sizeof(stats));
            double t0 = now_ms();
            int n = Planner_PlanPathWithYaw(
                start_map, goal_map, sp->yaw_deg, gp->yaw_deg, s_path, MAX_PATH_LEN, &stats);
            double plan_ms = now_ms() - t0;

            if (n <= 0) {
                sum.fail++;
                fprintf(f_pair,
                        "%d,%s,%s,%.1f,%.1f,0,0,0,65535,%.1f,%.1f,0,0,0,0,%u,%u,%.3f,-1,-1,-1,-1,0,0,-1,0,0,0,0,0,0\n",
                        zone,
                        sp->name,
                        gp->name,
                        (double)sp->yaw_deg,
                        (double)gp->yaw_deg,
                        (double)stats.heading_before_deg,
                        (double)stats.heading_after_deg,
                        (unsigned)stats.tier,
                        (unsigned)stats.fail_reason,
                        plan_ms);
                printf("[zone%d] %-15s -> %-15s FAIL(fail_reason=%u)\n",
                       zone,
                       sp->name,
                       gp->name,
                       (unsigned)stats.fail_reason);
                fclose(f_pair);
                fclose(f_path);
                if (zone == NEW_ZONE_1) {
                    g_z1 = sum;
                } else {
                    g_z2 = sum;
                }
                return; /* 40/40 必须成功，避免半截数据继续 */
            }

            const uint16_t rot = stats.rotation_index;
            const int need_rotation = (rot != PLANNER_ROTATE_NONE);

            /* ===== 执行语义：逐航点/逐段航向 ===== */
            for (int i = 0; i < n; i++) {
                if (!need_rotation) {
                    s_wp_heading[i] = gp->yaw_deg;
                } else {
                    s_wp_heading[i] = (i <= (int)rot) ? stats.heading_before_deg : stats.heading_after_deg;
                }
                if (i + 1 < n) {
                    if (!need_rotation) {
                        s_seg_heading[i] = gp->yaw_deg;
                    } else {
                        s_seg_heading[i] = (i < (int)rot) ? stats.heading_before_deg : stats.heading_after_deg;
                    }
                }
            }

            /* ===== 世界坐标 + 独立 SAT 指标 ===== */
            for (int i = 0; i < n; i++) {
                Point2D w = to_world_cm(s_path[i]);
                s_path_xy[i * 2 + 0] = w.x;
                s_path_xy[i * 2 + 1] = w.y;
            }
            RefSquarePathMetrics sat;
            if (!ref_measure_square_path(s_rects,
                                         s_rect_count,
                                         s_path_xy,
                                         n,
                                         s_seg_heading,
                                         n - 1,
                                         sp->x_cm,
                                         sp->y_cm,
                                         gp->x_cm,
                                         gp->y_cm,
                                         PLANNER_BODY_HALF_CM,
                                         PLANNER_ENDPOINT_ZONE_CM,
                                         NEW_ENDPOINT_PEN_TOL_CM,
                                         &sat)) {
                printf("[FAIL] zone%d %s->%s SAT 指标计算失败\n", zone, sp->name, gp->name);
                s_fail_count++;
                fclose(f_pair);
                fclose(f_path);
                return;
            }

            /* ===== 旋转点独立检查 ===== */
            float sat_rot_clear = -1.0f;
            int rot_ok = 1;
            if (need_rotation) {
                Point2D rw = to_world_cm(s_path[rot]);
                sat_rot_clear = ref_point_euclid_clearance_cm(rw.x, rw.y, s_rects, s_rect_count);
                rot_ok = (sat_rot_clear >= PLANNER_ROTATION_CLEARANCE_CM - NEW_ROT_CLEARANCE_TOL_CM) ? 1 : 0;
                if (!rot_ok) sum.rot_unsafe_pairs++;
                sum.rotations++;
                if (rot == 0) sum.rot_start++;
                if (rot == (uint16_t)(n - 1)) sum.rot_end++;
                if (sat_rot_clear < sum.min_rot_clear) sum.min_rot_clear = sat_rot_clear;
            }

            /* ===== 朝向语义检查 ===== */
            int heading_ok = 1;
            if (need_rotation) {
                if (fabsf(wrap_deg(stats.heading_before_deg - sp->yaw_deg)) > NEW_HEADING_TOL_DEG) heading_ok = 0;
                if (fabsf(wrap_deg(stats.heading_after_deg - gp->yaw_deg)) > NEW_HEADING_TOL_DEG) heading_ok = 0;
            } else {
                if (fabsf(wrap_deg(stats.heading_before_deg - gp->yaw_deg)) > NEW_HEADING_TOL_DEG) heading_ok = 0;
                if (fabsf(wrap_deg(stats.heading_after_deg - gp->yaw_deg)) > NEW_HEADING_TOL_DEG) heading_ok = 0;
                if (fabsf(wrap_deg(gp->yaw_deg - sp->yaw_deg)) > PLANNER_HEADING_DEADBAND_DEG + 0.1f) heading_ok = 0;
            }
            if (!heading_ok) sum.heading_bad_pairs++;

            /* ===== 端点精确 / 相邻点 ===== */
            float start_err = dist_world_cm(s_path_xy[0], s_path_xy[1], sp->x_cm, sp->y_cm);
            float goal_err =
                dist_world_cm(s_path_xy[(n - 1) * 2], s_path_xy[(n - 1) * 2 + 1], gp->x_cm, gp->y_cm);
            float min_adjacent = 1.0e9f;
            for (int i = 0; i + 1 < n; i++) {
                float d = dist_world_cm(s_path_xy[i * 2],
                                        s_path_xy[i * 2 + 1],
                                        s_path_xy[(i + 1) * 2],
                                        s_path_xy[(i + 1) * 2 + 1]);
                if (d < min_adjacent) min_adjacent = d;
            }
            int endpoint_ok = (start_err <= NEW_ENDPOINT_TOL_CM && goal_err <= NEW_ENDPOINT_TOL_CM) ? 1 : 0;
            int adjacent_ok = (min_adjacent > NEW_DUP_TOL_CM) ? 1 : 0;
            /* 唯一合法例外：起终点几乎重合（<1cm）时允许 2 点零长路径（原地转/原地不动），
             * 两点可相同；其它任何相邻重复都算 FAIL。 */
            float start_goal_dist = dist_world_cm(sp->x_cm, sp->y_cm, gp->x_cm, gp->y_cm);
            int zero_len_path =
                (n == 2 && start_goal_dist < 1.0f && min_adjacent <= NEW_DUP_TOL_CM) ? 1 : 0;
            if (!endpoint_ok) sum.endpoint_bad_pairs++;
            if (!adjacent_ok && !zero_len_path) {
                sum.adjacent_bad_pairs++;
            } else if (zero_len_path) {
                sum.zero_len_pairs++;
            }

            /* ===== 验收断言（按点对累计） ===== */
            if (sat.general_collision_samples > 0) sum.sat_collision_pairs++;
            if (sat.min_general_signed_cm < NEW_GENERAL_MIN_SIGNED_CM - NEW_SAT_COLLISION_EPS_CM) {
                sum.general_below_min_pairs++;
            }
            if (sat.min_general_signed_cm < sum.min_general_signed) {
                sum.min_general_signed = sat.min_general_signed_cm;
            }
            if (sat.endpoint_penetration_violations > 0) sum.endpoint_pen_violation_pairs++;
            float stats_len_diff = fabsf(stats.length_cm - sat.length_cm);
            int stats_ok = (stats.waypoints == (uint16_t)n) && (stats_len_diff <= NEW_STATS_LEN_DIFF_MAX_CM) ? 1 : 0;
            if (!stats_ok) sum.stats_bad_pairs++;

            /* ===== 汇总 ===== */
            sum.ok++;
            sum.sum_len += sat.length_cm;
            if (sat.length_cm < sum.min_len) sum.min_len = sat.length_cm;
            if (sat.length_cm > sum.max_len) sum.max_len = sat.length_cm;
            sum.sum_wp += n;
            if (n > sum.max_wp) sum.max_wp = n;
            sum.sum_plan_ms += plan_ms;
            if (plan_ms < sum.min_plan_ms) sum.min_plan_ms = plan_ms;
            if (plan_ms > sum.max_plan_ms) sum.max_plan_ms = plan_ms;
            if (sat.min_signed_cm < sum.min_sat_signed) sum.min_sat_signed = sat.min_signed_cm;
            if (stats.tier == PLANNER_TIER_SOFT) {
                sum.soft++;
            } else {
                sum.tight++;
            }

            /* ===== 写主 CSV ===== */
            fprintf(f_pair,
                    "%d,%s,%s,%.1f,%.1f,1,%d,%u,%u,%.2f,%.2f,%.4f,%.4f,%.4f,%.4f,%u,%u,%.4f,%.4f,%.4f,"
                    "%.4f,%.4f,%d,%d,%.4f,%d,%d,%d,%d,%.4f,%d\n",
                    zone,
                    sp->name,
                    gp->name,
                    (double)sp->yaw_deg,
                    (double)gp->yaw_deg,
                    n,
                    (unsigned)stats.expanded_nodes,
                    (unsigned)stats.rotation_index,
                    (double)stats.heading_before_deg,
                    (double)stats.heading_after_deg,
                    (double)stats.length_cm,
                    (double)stats.min_square_clearance_cm,
                    (double)stats.rotation_clearance_cm,
                    (double)stats.min_clearance_cm,
                    (unsigned)stats.tier,
                    (unsigned)stats.fail_reason,
                    plan_ms,
                    (double)sat.min_signed_cm,
                    (double)sat.min_general_signed_cm,
                    (double)sat.min_endpoint_signed_cm,
                    (double)sat.max_penetration_cm,
                    sat.general_collision_samples,
                    sat.endpoint_penetration_violations,
                    (double)sat_rot_clear,
                    rot_ok,
                    heading_ok,
                    endpoint_ok,
                    adjacent_ok,
                    (double)min_adjacent,
                    zero_len_path);

            /* ===== 写航点 CSV（含独立方形净空） ===== */
            for (int i = 0; i < n; i++) {
                float clr = ref_square_scene_clearance_cm(
                    s_path_xy[i * 2], s_path_xy[i * 2 + 1], s_wp_heading[i], PLANNER_BODY_HALF_CM, s_rects,
                    s_rect_count);
                float map_half = Planner_HeadingFootprintHalfCm(s_wp_heading[i]);
                float map_clr = MapCore_PointSquareClearanceCm(s_path[i], map_half);
                fprintf(f_path,
                        "%d,%s,%s,%.1f,%.1f,%d,%.4f,%.4f,%.2f,%u,%.4f,%.4f\n",
                        zone,
                        sp->name,
                        gp->name,
                        (double)sp->yaw_deg,
                        (double)gp->yaw_deg,
                        i,
                        (double)s_path_xy[i * 2],
                        (double)s_path_xy[i * 2 + 1],
                        (double)s_wp_heading[i],
                        (unsigned)rot,
                        (double)clr,
                        (double)map_clr);
            }

            printf("[zone%d] %-15s(yaw%3.0f) -> %-15s(yaw%3.0f) ok wp=%2d len=%6.1f tier=%u rot=%5u "
                   "rotclr=%6.2f sat_min=%7.2f pen=%5.2f plan=%6.2fms\n",
                   zone,
                   sp->name,
                   (double)sp->yaw_deg,
                   gp->name,
                   (double)gp->yaw_deg,
                   n,
                   (double)sat.length_cm,
                   (unsigned)stats.tier,
                   (unsigned)stats.rotation_index,
                   (double)sat_rot_clear,
                   (double)sat.min_signed_cm,
                   (double)sat.max_penetration_cm,
                   plan_ms);
        }
    }

    fclose(f_pair);
    fclose(f_path);

    double avg_len = (sum.ok > 0) ? sum.sum_len / (double)sum.ok : 0.0;
    double avg_wp = (sum.ok > 0) ? (double)sum.sum_wp / (double)sum.ok : 0.0;
    double avg_ms = (sum.ok > 0) ? sum.sum_plan_ms / (double)sum.ok : 0.0;
    printf("\n[zone%d] 汇总: 点对=%d 成功=%d 失败=%d | soft=%d tight=%d | 旋转=%d (起点%d/终点%d) | "
           "平均航点=%.2f 最大=%d | 平均长=%.1fcm (min %.1f / max %.1f) | 最小SAT signed=%.2fcm | "
           "一般路段min signed=%.2fcm (门限%.2f, 不足点对%d) | "
           "最小旋转清障=%.2fcm | 规划耗时 avg=%.2fms min=%.2f max=%.2fms\n",
           zone,
           sum.total,
           sum.ok,
           sum.fail,
           sum.soft,
           sum.tight,
           sum.rotations,
           sum.rot_start,
           sum.rot_end,
           avg_wp,
           sum.max_wp,
           avg_len,
           sum.min_len,
           sum.max_len,
           (double)sum.min_sat_signed,
           (double)sum.min_general_signed,
           (double)NEW_GENERAL_MIN_SIGNED_CM,
           sum.general_below_min_pairs,
           (sum.rotations > 0) ? (double)sum.min_rot_clear : 0.0,
           avg_ms,
           sum.min_plan_ms,
           sum.max_plan_ms);
    printf("[zone%d] 断言异常: SAT碰撞点对=%d 端点侵入超限点对=%d 旋转不安全=%d 朝向语义=%d 端点=%d "
           "相邻点=%d 零长路径特例=%d stats不一致=%d\n",
           zone,
           sum.sat_collision_pairs,
           sum.endpoint_pen_violation_pairs,
           sum.rot_unsafe_pairs,
           sum.heading_bad_pairs,
           sum.endpoint_bad_pairs,
           sum.adjacent_bad_pairs,
           sum.zero_len_pairs,
           sum.stats_bad_pairs);

    if (zone == NEW_ZONE_1) {
        g_z1 = sum;
    } else {
        g_z2 = sum;
    }
}

int main(int argc, char **argv) {
    const char *out_dir = ".";
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--out-dir") == 0 && i + 1 < argc) {
            out_dir = argv[++i];
        } else {
            fprintf(stderr, "用法: %s [--out-dir DIR]\n", argv[0]);
            return 2;
        }
    }

    char accept_path[512];
    snprintf(accept_path, sizeof(accept_path), "%s/acceptance_v2.csv", out_dir);
    s_accept_csv = fopen(accept_path, "w");
    if (s_accept_csv == NULL) {
        fprintf(stderr, "无法写 %s\n", accept_path);
        return 2;
    }
    fprintf(s_accept_csv, "check,result,detail\n");

    printf("new_runner(v2): 方形车体 30×30 规划评测（Planner_PlanPathWithYaw + 独立精确 SAT）\n");
    printf("车体: side=%.1fcm half=%.1fcm 半对角线=%.2fcm 旋转清障门限=%.2fcm 端点区=%.1fcm\n",
           (double)PLANNER_CAR_SIDE_CM,
           (double)PLANNER_BODY_HALF_CM,
           (double)PLANNER_BODY_HALF_DIAGONAL_CM,
           (double)PLANNER_ROTATION_CLEARANCE_CM,
           (double)PLANNER_ENDPOINT_ZONE_CM);

    memset(&g_z1, 0, sizeof(g_z1));
    memset(&g_z2, 0, sizeof(g_z2));

    for (int zone = NEW_ZONE_1; zone <= NEW_ZONE_2; zone++) {
        if (!MapCore_Init((uint8_t)zone, s_grid, s_clarity, s_square_clarity)) {
            printf("[FAIL] zone%d MapCore_Init 失败\n", zone);
            fclose(s_accept_csv);
            return 1;
        }
        s_rect_count = build_reference_rects(zone);
        if (s_rect_count <= 0) {
            printf("[FAIL] zone%d 独立障碍矩形构造失败\n", zone);
            fclose(s_accept_csv);
            return 1;
        }
        run_zone(zone, (zone == NEW_ZONE_1) ? s_ref_points_zone1 : s_ref_points_zone2, out_dir);
    }

    /* ===== 全局验收 ===== */
    int total_ok = g_z1.ok + g_z2.ok;
    int total_fail = g_z1.fail + g_z2.fail;
    int coll = g_z1.sat_collision_pairs + g_z2.sat_collision_pairs;
    int pen_viol = g_z1.endpoint_pen_violation_pairs + g_z2.endpoint_pen_violation_pairs;
    int rot_unsafe = g_z1.rot_unsafe_pairs + g_z2.rot_unsafe_pairs;
    int head_bad = g_z1.heading_bad_pairs + g_z2.heading_bad_pairs;
    int endpoint_bad = g_z1.endpoint_bad_pairs + g_z2.endpoint_bad_pairs;
    int adjacent_bad = g_z1.adjacent_bad_pairs + g_z2.adjacent_bad_pairs;
    int stats_bad = g_z1.stats_bad_pairs + g_z2.stats_bad_pairs;
    int general_below = g_z1.general_below_min_pairs + g_z2.general_below_min_pairs;
    float min_general = (g_z1.min_general_signed < g_z2.min_general_signed) ? g_z1.min_general_signed
                                                                            : g_z2.min_general_signed;
    int rotations = g_z1.rotations + g_z2.rotations;
    float min_rot;
    if (g_z1.rotations > 0 && g_z2.rotations > 0) {
        min_rot = (g_z1.min_rot_clear < g_z2.min_rot_clear) ? g_z1.min_rot_clear : g_z2.min_rot_clear;
    } else if (g_z1.rotations > 0) {
        min_rot = g_z1.min_rot_clear;
    } else if (g_z2.rotations > 0) {
        min_rot = g_z2.min_rot_clear;
    } else {
        min_rot = -1.0f;
    }
    float min_sat = (g_z1.min_sat_signed < g_z2.min_sat_signed) ? g_z1.min_sat_signed : g_z2.min_sat_signed;
    char detail[420];

    snprintf(detail, sizeof(detail), "成功 %d/%d，失败 %d", total_ok, NEW_MIN_SUCCESS, total_fail);
    record("40/40 任务点对全部规划成功", (total_fail == 0 && total_ok == NEW_MIN_SUCCESS) ? 1 : 0, detail);

    snprintf(detail,
             sizeof(detail),
             "一般路段独立 1cm 采样最小 signed=%.3fcm（门限 %.2fcm），不足点对=%d，SAT 碰撞点对=%d，"
             "全程最小 signed=%.3fcm",
             (double)min_general,
             (double)NEW_GENERAL_MIN_SIGNED_CM,
             general_below,
             coll,
             (double)min_sat);
    record("一般路段方形净空 >=1.5cm（独立 1cm SAT）", (general_below == 0 && coll == 0) ? 1 : 0, detail);

    snprintf(detail,
             sizeof(detail),
             "超限点对=%d（端点区=到起终点欧氏<=%.0fcm，允许侵入<=%.1fcm）",
             pen_viol,
             (double)PLANNER_ENDPOINT_ZONE_CM,
             (double)NEW_ENDPOINT_PEN_TOL_CM);
    record("端点豁免区模型侵入 <=1.0cm", pen_viol == 0 ? 1 : 0, detail);

    snprintf(detail,
             sizeof(detail),
             "旋转方案=%d，不安全=%d，最小独立旋转清障=%.2fcm（门限%.2fcm）",
             g_z1.rotations + g_z2.rotations,
             rot_unsafe,
             (double)min_rot,
             (double)PLANNER_ROTATION_CLEARANCE_CM);
    record("旋转点独立欧氏清障 >=23.22cm", rot_unsafe == 0 ? 1 : 0, detail);

    snprintf(detail, sizeof(detail), "语义错误点对=%d（不旋转时两者均=目标 yaw）", head_bad);
    record("朝向方案语义正确（before=起始yaw / after=目标yaw）", head_bad == 0 ? 1 : 0, detail);

    snprintf(detail,
             sizeof(detail),
             "端点异常点对=%d，相邻点重复点对=%d，合法零长路径特例=%d（起终点<1cm，允许两点相同）",
             endpoint_bad,
             adjacent_bad,
             g_z1.zero_len_pairs + g_z2.zero_len_pairs);
    if (endpoint_bad == 0 && adjacent_bad == 0) {
        record("起终点精确一致 / 相邻点无重复", 1, detail);
    } else {
        record("起终点精确一致 / 相邻点无重复", 0, detail);
    }

    snprintf(detail, sizeof(detail), "不一致点对=%d（长度差<=%.1fcm）", stats_bad, (double)NEW_STATS_LEN_DIFF_MAX_CM);
    record("stats.waypoints/length 与实测一致", stats_bad == 0 ? 1 : 0, detail);

    snprintf(detail,
             sizeof(detail),
             "soft=%d tight=%d（zone1 %d/%d, zone2 %d/%d）",
             g_z1.soft + g_z2.soft,
             g_z1.tight + g_z2.tight,
             g_z1.soft,
             g_z1.tight,
             g_z2.soft,
             g_z2.tight);
    record_info("tier 分布", detail);

    snprintf(detail,
             sizeof(detail),
             "旋转方案=%d（起点旋转 %d / 终点补转 %d）；平均长度=%.1fcm；规划耗时 avg=%.2fms max=%.2fms",
             rotations,
             g_z1.rot_start + g_z2.rot_start,
             g_z1.rot_end + g_z2.rot_end,
             (double)((g_z1.sum_len + g_z2.sum_len) / (double)(total_ok > 0 ? total_ok : 1)),
             (g_z1.sum_plan_ms + g_z2.sum_plan_ms) / (double)(total_ok > 0 ? total_ok : 1),
             (g_z1.max_plan_ms > g_z2.max_plan_ms) ? g_z1.max_plan_ms : g_z2.max_plan_ms);
    record_info("旋转方案与耗时", detail);

    fclose(s_accept_csv);
    printf("\n========== v2 验收汇总（acceptance_v2.csv） ==========\n");
    printf("成功=%d/%d 失败=%d | SAT碰撞点对=%d | 端点侵入超限=%d | 旋转不安全=%d | 朝向语义错误=%d | "
           "端点异常=%d | stats不一致=%d\n",
           total_ok,
           NEW_MIN_SUCCESS,
           total_fail,
           coll,
           pen_viol,
           rot_unsafe,
           head_bad,
           endpoint_bad,
           stats_bad);
    printf("输出: %s\n", accept_path);

    if (s_fail_count != 0) {
        printf("[FAIL] new_runner(v2) 有 %d 条验收断言失败\n", s_fail_count);
        return 1;
    }
    printf("[PASS] new_runner(v2) 全部验收断言通过\n");
    return 0;
}
