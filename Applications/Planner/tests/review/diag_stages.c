/**
 * @file diag_stages.c
 * @brief [v2 方形车体] 评审诊断：分阶段定位规划失败（含“是否存在合法旋转方案”独立证明）
 *
 * @details 通过 #include "planner_algo.c" 访问其 static 函数（plan_translation_path 等）取证，
 *          不修改被测源码。存在性证明口径：存在旋转安全点 p，使得
 *          start→p 按 start_yaw 的投影半长可规划、p→goal 按 goal_yaw 的投影半长可规划，
 *          且两段都通过独立 tier 校验 → 至少存在一种合法方案；若实现返回失败即为过度保守。
 *
 * 编译（%TEMP%）：
 *   gcc -std=c99 -O2 -DPLANNER_HOST_TEST -msse2 -mfpmath=sse \
 *       -I Applications/Planner -I Applications/Planner/utils \
 *       Applications/Planner/tests/review/diag_stages.c \
 *       Applications/Planner/utils/Cstar_map_core.c \
 *       Applications/Planner/utils/priority_queue.c -o %TEMP%/diag_stages.exe -lm
 */

#define PLANNER_HOST_TEST 1

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "Cstar_map_core.h"
#include "planner_config.h"
#include "priority_queue.h"
#include "planner_algo.c"

static uint8_t s_grid[MAX_MAP_BYTES];
static uint8_t s_clear[MAX_CELLS];
static int8_t s_square[MAX_CELLS];
static Point2D s_out[MAX_PATH_LEN];
static Point2D s_a[MAX_PATH_LEN];
static Point2D s_b[MAX_PATH_LEN];

static Point2D w2m(float wx, float wy) {
    Point2D p;
    p.x = wx * 0.01f + MAP_OFFSET_X;
    p.y = wy * 0.01f + MAP_OFFSET_Y;
    return p;
}
static void wcm(Point2D p, float *wx, float *wy) {
    *wx = (p.x - MAP_OFFSET_X) * 100.0f;
    *wy = (p.y - MAP_OFFSET_Y) * 100.0f;
}
static float dist_cm2(Point2D a, Point2D b) {
    float dx = (a.x - b.x) * 100.0f, dy = (a.y - b.y) * 100.0f;
    return sqrtf(dx * dx + dy * dy);
}

/* 独立存在性证明：存在旋转安全点 p，两段分别用各自 heading 半长可规划并通过校验 */
static int rev_prove_feasible(Point2D start, Point2D goal, float syaw, float gyaw, float required,
                              float endpoint_allowed) {
    float hs = Planner_HeadingFootprintHalfCm(syaw);
    float hg = Planner_HeadingFootprintHalfCm(gyaw);
    uint8_t fail;
    int tried = 0;
    uint16_t exp;
    for (int gx = 0; gx < g_map.grid_size && tried < 600; gx++) {
        for (int gy = 0; gy < g_map.grid_size && tried < 600; gy++) {
            Point2D p = cell_center(gx, gy);
            if (!MapCore_RotationSafe(p)) continue;
            if (dist_cm2(p, start) < 1.0f || dist_cm2(p, goal) < 1.0f) continue;
            tried++;
            int na = plan_translation_path(start, p, hs, required, endpoint_allowed, s_a, MAX_PATH_LEN, &exp, &fail);
            if (na < 2) continue;
            int nb = plan_translation_path(p, goal, hg, required, endpoint_allowed, s_b, MAX_PATH_LEN, &exp, &fail);
            if (nb < 2) continue;
            float msq = 0, mcl = 0;
            if (!Planner_SquarePathValidate(s_a, na, hs, required, endpoint_allowed, &msq, &mcl)) continue;
            if (!Planner_SquarePathValidate(s_b, nb, hg, required, endpoint_allowed, &msq, &mcl)) continue;
            printf("    [FEASIBLE] via rotation world=(%.1f,%.1f) euclid=%.2fcm len_a=%d len_b=%d\n",
                   (double)((p.x - MAP_OFFSET_X) * 100.0f), (double)((p.y - MAP_OFFSET_Y) * 100.0f),
                   (double)MapCore_PointClearanceCm(p), na, nb);
            return 1;
        }
    }
    return 0;
}

static void diag_case(const char *tag, float sx, float sy, float syaw, float gx, float gy, float gyaw) {
    printf("\n---- [%s] start=(%.1f,%.1f,%.1fdeg) goal=(%.1f,%.1f,%.1fdeg) ----\n", tag, (double)sx, (double)sy,
           (double)syaw, (double)gx, (double)gy, (double)gyaw);
    Point2D start = w2m(sx, sy), goal = w2m(gx, gy);
    printf("  start: euclid=%.2f half(syaw)=%.2f square=%.2f rotSafe=%d\n", (double)MapCore_PointClearanceCm(start),
           (double)Planner_HeadingFootprintHalfCm(syaw),
           (double)MapCore_PointSquareClearanceCm(start, Planner_HeadingFootprintHalfCm(syaw)),
           (int)MapCore_RotationSafe(start));
    printf("  goal : euclid=%.2f half(gyaw)=%.2f square=%.2f rotSafe=%d\n", (double)MapCore_PointClearanceCm(goal),
           (double)Planner_HeadingFootprintHalfCm(gyaw),
           (double)MapCore_PointSquareClearanceCm(goal, Planner_HeadingFootprintHalfCm(gyaw)),
           (int)MapCore_RotationSafe(goal));

    PlannerPathStats_t stats;
    memset(&stats, 0, sizeof(stats));
    int n = Planner_PlanPathWithYaw(start, goal, syaw, gyaw, s_out, MAX_PATH_LEN, &stats);
    printf("  result: n=%d tier=%u fail=%u rot=%d hb=%.1f ha=%.1f min_sq=%.2f len=%.1f\n", n, (unsigned)stats.tier,
           (unsigned)stats.fail_reason,
           (stats.rotation_index == PLANNER_ROTATE_NONE) ? -1 : (int)stats.rotation_index,
           (double)stats.heading_before_deg, (double)stats.heading_after_deg, (double)stats.min_square_clearance_cm,
           (double)stats.length_cm);

    if (n <= 0) {
        if (rev_prove_feasible(start, goal, syaw, gyaw, PLANNER_SQUARE_HARD_MARGIN_CM,
                               (PLANNER_SQUARE_HARD_MARGIN_CM - PLANNER_ENDPOINT_SQUARE_TOL_CM < 0.0f)
                                   ? 0.0f
                                   : PLANNER_SQUARE_HARD_MARGIN_CM - PLANNER_ENDPOINT_SQUARE_TOL_CM)) {
            printf("  => [MAJOR-HINT] 存在合法旋转方案但实现返回失败（过度保守）\n");
        } else {
            printf("  => 未找到其他合法方案（物理/端点受限）\n");
        }
    } else {
        if (stats.rotation_index != PLANNER_ROTATE_NONE && stats.rotation_index < n) {
            float rx, ry;
            wcm(s_out[stats.rotation_index], &rx, &ry);
            printf("  rotation@wp%u world=(%.1f,%.1f) euclid=%.2f\n", (unsigned)stats.rotation_index, (double)rx,
                   (double)ry, (double)MapCore_PointClearanceCm(s_out[stats.rotation_index]));
        }
        for (int i = 0; i < n; i++) {
            float wx, wy;
            wcm(s_out[i], &wx, &wy);
            float yaw = (stats.rotation_index != PLANNER_ROTATE_NONE && i > (int)stats.rotation_index)
                            ? stats.heading_after_deg
                            : stats.heading_before_deg;
            printf("   wp%02d (%.1f,%.1f) sq=%.2f half=%.2f\n", i, (double)wx, (double)wy,
                   (double)MapCore_PointSquareClearanceCm(s_out[i], Planner_HeadingFootprintHalfCm(yaw)),
                   (double)Planner_HeadingFootprintHalfCm(yaw));
        }
    }
}

int main(void) {
    MapCore_Init(1, s_grid, s_clear, s_square);
    diag_case("rotate-at-start(open)", 20.0f, 195.0f, 10.0f, 200.0f, 195.0f, 100.0f);
    diag_case("rotate-in-30cm-corridor", 105.0f, 20.0f, 0.0f, 105.0f, 160.0f, 90.0f);
    diag_case("arbitrary-yaw-37", 20.0f, 195.0f, 0.0f, 200.0f, 195.0f, 37.0f);
    diag_case("invalid-goal-yaw-123", 105.0f, 20.0f, 0.0f, 105.0f, 205.0f, -123.0f);
    diag_case("same-point-rotate", 105.0f, 195.0f, 0.0f, 105.0f, 195.0f, 90.0f);
    diag_case("35cm-channel-straight", 50.0f, 192.0f, 0.0f, 160.0f, 192.0f, 0.0f);
    return 0;
}
