/**
 * @file profile_runner.c
 * @brief 航点执行剖面纯函数单元测试（执行层 Blocker 回归，不依赖 RTOS）
 *
 * @details 背景：`planner_port.c` 曾把 `Planner_PointSegmentDistanceCm`（点到线段距离）
 *          误当"两点段长"使用，导致所有中间航点段长恒为 0、剩余路程全 0、航点被跳过。
 *          修复方式是把执行剖面逻辑抽到算法层纯函数（`planner_algo.c`）：
 *            Planner_PointDistanceCm / Planner_BuildRemainingCm /
 *            Planner_WaypointSpeedCmS / Planner_WaypointTimeoutMs
 *          本文件在主机上直接单元测试这些函数，并专门回归"路径 ≥2 段时 remaining
 *          必须非零且单调递减"（旧错误实现会全 0）。
 *
 *          输出：<out-dir>/profile_results.csv（test,name,result,detail）；
 *          任一断言失败退出码非 0，由 run_new.ps1 纳入 PASS/FAIL 汇总。
 *
 * 编译（与 new_runner 相同，-DPLANNER_HOST_TEST -msse2 -mfpmath=sse -lm）：
 *   gcc -std=c99 -O2 -Wall -Wextra -DPLANNER_HOST_TEST -msse2 -mfpmath=sse \
 *       -I Applications/Planner -I Applications/Planner/utils \
 *       Applications/Planner/utils/Cstar_map_core.c Applications/Planner/utils/planner_algo.c \
 *       Applications/Planner/utils/priority_queue.c \
 *       Applications/Planner/tests/harness/profile_runner.c -o profile_runner.exe -lm
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Cstar_map_core.h"
#include "planner_algo.h"
#include "planner_config.h"

#if !PLANNER_ENABLED
#error "profile_runner 需要 PLANNER_ENABLED=1 的 planner_config.h"
#endif

/* 断言容差 */
#define PROF_FLOAT_TOL_CM 1.0e-3f /* 距离/剩余路程 (cm) */
#define PROF_SPEED_TOL 1.0e-3f /* 速度 (cm/s) */
#define PROF_TIMEOUT_TOL_MS 1.0 /* 超时 (ms) */

/* 静态缓冲区（供真实规划路径的集成回归用） */
static uint8_t s_grid[MAX_MAP_BYTES];
static uint8_t s_clarity[MAX_CELLS];
static int8_t s_square_clarity[MAX_CELLS];
static Point2D s_plan_path[MAX_PATH_LEN];
static float s_remaining[MAX_PATH_LEN];

static FILE *s_csv = NULL;
static int s_fail_count = 0;
static int s_pass_count = 0;

/* ==========================================================================================
 * 记录与基础工具
 * ========================================================================================== */

/** @brief 写一个 CSV 字段（加引号，内部引号双写） */
static void csv_field(FILE *fp, const char *s) {
    fputc('"', fp);
    for (const char *p = s; *p != '\0'; p++) {
        if (*p == '"') fputc('"', fp);
        fputc(*p, fp);
    }
    fputc('"', fp);
}

/** @brief 记录一条 PASS/FAIL 断言（打印 + CSV） */
static void record(const char *id, const char *name, int pass, const char *detail) {
    printf("[%s] %-4s %s | %s\n", pass ? "PASS" : "FAIL", id, name, detail);
    if (s_csv != NULL) {
        csv_field(s_csv, id);
        fputc(',', s_csv);
        csv_field(s_csv, name);
        fputc(',', s_csv);
        csv_field(s_csv, pass ? "PASS" : "FAIL");
        fputc(',', s_csv);
        csv_field(s_csv, detail);
        fputc('\n', s_csv);
    }
    if (pass) {
        s_pass_count++;
    } else {
        s_fail_count++;
    }
}

/** @brief 世界坐标 (cm) -> 地图坐标 (m)（与 Planner 的坐标约定一致） */
static Point2D to_map(float x_cm, float y_cm) {
    Point2D p;
    p.x = x_cm * 0.01f + MAP_OFFSET_X;
    p.y = y_cm * 0.01f + MAP_OFFSET_Y;
    return p;
}

/** @brief 浮点近似比较 */
static int near_f(float actual, float expect, float tol) {
    return fabsf(actual - expect) <= tol;
}

/* ==========================================================================================
 * P1：Planner_PointDistanceCm 与 Planner_PointSegmentDistanceCm 语义区分
 * ========================================================================================== */
static void test_point_distance(void) {
    char detail[320];

    /* P1a：世界 (0,0)-(300,400)cm 的欧氏距离 = 500cm */
    {
        float d = Planner_PointDistanceCm(to_map(0.0f, 0.0f), to_map(300.0f, 400.0f));
        snprintf(detail, sizeof(detail), "world(0,0)-(300,400)cm → %.4fcm（期望 500.0000 ±0.001）", (double)d);
        record("P1a", "两点距离 = 500cm（3-4-5 直角三角形）", near_f(d, 500.0f, PROF_FLOAT_TOL_CM), detail);
    }

    /* P1b：零距离 */
    {
        Point2D p = to_map(12.0f, 34.0f);
        float d = Planner_PointDistanceCm(p, p);
        snprintf(detail, sizeof(detail), "同一点距离 → %.6fcm（期望 0）", (double)d);
        record("P1b", "两点距离：零距离 = 0", fabsf(d) <= PROF_FLOAT_TOL_CM, detail);
    }

    /* P1c：点-线段距离与两点距离必须区分开（旧 Blocker 的语义混用就在这）
     * p = 线段起点 → 点-线段距离 = 0；而两点距离 = 段长 100cm */
    {
        Point2D p = to_map(0.0f, 0.0f);
        Point2D a = to_map(0.0f, 0.0f);
        Point2D b = to_map(100.0f, 0.0f);
        float seg_d = Planner_PointSegmentDistanceCm(p, a, b);
        float pt_d = Planner_PointDistanceCm(a, b);
        snprintf(detail, sizeof(detail), "p=线段起点: 点-线段距离=%.6fcm（期望0）, 两点距离=%.4fcm（期望100）",
                 (double)seg_d, (double)pt_d);
        record("P1c", "点-线段距离(0) 与 两点段长(100) 语义区分",
               fabsf(seg_d) <= PROF_FLOAT_TOL_CM && near_f(pt_d, 100.0f, PROF_FLOAT_TOL_CM), detail);
    }
}

/* ==========================================================================================
 * P2：Planner_BuildRemainingCm
 * ========================================================================================== */
static void test_build_remaining(void) {
    Point2D path[3];
    float remaining[3];
    char detail[320];

    path[0] = to_map(0.0f, 0.0f);
    path[1] = to_map(100.0f, 0.0f);
    path[2] = to_map(100.0f, 100.0f);

    /* P2a：remaining = {200, 100, 0} */
    {
        int ok = Planner_BuildRemainingCm(path, 3, remaining, 3) ? 1 : 0;
        int values_ok = ok && near_f(remaining[0], 200.0f, PROF_FLOAT_TOL_CM) &&
                        near_f(remaining[1], 100.0f, PROF_FLOAT_TOL_CM) &&
                        near_f(remaining[2], 0.0f, PROF_FLOAT_TOL_CM);
        snprintf(detail, sizeof(detail), "路径(0,0)→(100,0)→(100,100)cm: remaining={%.4f, %.4f, %.4f}"
                                         "（期望 {200,100,0}）",
                 ok ? (double)remaining[0] : -1.0, ok ? (double)remaining[1] : -1.0,
                 ok ? (double)remaining[2] : -1.0);
        record("P2a", "剩余路程 = {200,100,0}cm", values_ok, detail);
    }

    /* P2b：参数校验：out_cap 不足 / len<=0 / 空指针 → false */
    {
        float small[2];
        int cap_bad = Planner_BuildRemainingCm(path, 3, small, 2) ? 1 : 0;
        int len_zero = Planner_BuildRemainingCm(path, 0, remaining, 3) ? 1 : 0;
        int len_neg = Planner_BuildRemainingCm(path, -1, remaining, 3) ? 1 : 0;
        int ptr_bad = Planner_BuildRemainingCm(path, 3, NULL, 3) ? 1 : 0;
        snprintf(detail, sizeof(detail), "out_cap=2→返回%d, len=0→返回%d, len=-1→返回%d, out=NULL→返回%d（期望全 false）",
                 cap_bad, len_zero, len_neg, ptr_bad);
        record("P2b", "剩余路程：容量不足/非法长度/空指针返回 false",
               cap_bad == 0 && len_zero == 0 && len_neg == 0 && ptr_bad == 0, detail);
    }
}

/* ==========================================================================================
 * P3：Planner_WaypointSpeedCmS（减速区 / cruise / tight 限速 / 非负）
 * ========================================================================================== */
static void test_waypoint_speed(void) {
    char detail[360];

    struct {
        float remaining;
        bool tight;
        float expect;
    } cases[] = {
        {0.0f, false, 40.0f}, /* 终点：终速 40 */
        {25.0f, false, 80.0f}, /* 减速区中点：40+(120-40)*0.5 = 80 */
        {50.0f, false, 120.0f}, /* 减速区边界（严格 <50 才减速）：巡航 120 */
        {100.0f, false, 120.0f}, /* 巡航 */
        {0.0f, true, 40.0f}, /* tight 下限仍是 40 */
        {25.0f, true, 80.0f}, /* tight：80 恰好等于限速 */
        {50.0f, true, 80.0f}, /* tight：巡航被限到 80 */
        {100.0f, true, 80.0f},
    };
    int all_ok = 1;
    int n = (int)(sizeof(cases) / sizeof(cases[0]));
    for (int i = 0; i < n; i++) {
        float s = Planner_WaypointSpeedCmS(cases[i].remaining, cases[i].tight);
        if (!near_f(s, cases[i].expect, PROF_SPEED_TOL)) all_ok = 0;
    }
    snprintf(detail, sizeof(detail),
             "remaining/tight→speed: 0/F→40, 25/F→80, 50/F→120, 100/F→120, 0/T→40, 25/T→80, 50/T→80, 100/T→80"
             "（共 %d 个用例）",
             n);
    record("P3a", "减速区/巡航/tight 限速速度剖面（40/80/120）", all_ok, detail);

    /* P3b：任意 remaining（含负数、超大值）不出现负值，且 tight 不超过 80 */
    {
        int range_ok = 1;
        float min_speed = 1.0e9f;
        float max_speed_tight = -1.0e9f;
        for (float r = -100.0f; r <= 300.0f; r += 1.0f) {
            float soft = Planner_WaypointSpeedCmS(r, false);
            float tight = Planner_WaypointSpeedCmS(r, true);
            if (soft < 40.0f - PROF_SPEED_TOL || soft > 120.0f + PROF_SPEED_TOL) range_ok = 0;
            if (tight < 40.0f - PROF_SPEED_TOL || tight > 80.0f + PROF_SPEED_TOL) range_ok = 0;
            if (soft < min_speed) min_speed = soft;
            if (tight > max_speed_tight) max_speed_tight = tight;
        }
        snprintf(detail, sizeof(detail),
                 "remaining∈[-100,300] 扫描: soft 速度 ∈ [%.2f, 120]，tight 最大 %.2f（期望 soft≥40 且无负值、tight≤80）",
                 (double)min_speed, (double)max_speed_tight);
        record("P3b", "速度剖面：不出现负值 / tight 不超过 80cm/s", range_ok, detail);
    }
}

/* ==========================================================================================
 * P4：Planner_WaypointTimeoutMs（估算公式 / 除零保护 / 上限）
 * ========================================================================================== */
static void test_waypoint_timeout(void) {
    char detail[360];

    /* P4a：seg=100cm, speed=120 → 2000 + 100/120*1000*1.8 = 3500ms（±1ms） */
    {
        uint32_t t = Planner_WaypointTimeoutMs(100.0f, 120.0f);
        double diff = fabs((double)t - 3500.0);
        snprintf(detail, sizeof(detail), "seg=100cm, speed=120cm/s → %ums（期望 2000 + 100/120*1000*1.8 = 3500 ±1）",
                 (unsigned)t);
        record("P4a", "超时估算公式 = 3500ms", diff <= PROF_TIMEOUT_TOL_MS, detail);
    }

    /* P4b：speed<=0 不得除零，应走 1cm/s 保护并命中 60000ms 上限 */
    {
        uint32_t t0 = Planner_WaypointTimeoutMs(100.0f, 0.0f);
        uint32_t tn = Planner_WaypointTimeoutMs(100.0f, -5.0f);
        uint32_t ts = Planner_WaypointTimeoutMs(10000.0f, 1.0f);
        int ok = (t0 == 60000u) && (tn == 60000u) && (ts == 60000u);
        snprintf(detail, sizeof(detail),
                 "speed=0→%ums, speed=-5→%ums, seg=10000/speed=1→%ums（期望均 60000 上限，无除零）",
                 (unsigned)t0, (unsigned)tn, (unsigned)ts);
        record("P4b", "超时：speed≤0 除零保护 + 60000ms 上限", ok, detail);
    }

    /* P4c：seg=0 → 只剩固定余量 2000ms；且结果不低于 2000ms */
    {
        uint32_t t = Planner_WaypointTimeoutMs(0.0f, 120.0f);
        snprintf(detail, sizeof(detail), "seg=0, speed=120 → %ums（期望 2000 固定余量）", (unsigned)t);
        record("P4c", "超时：零段长只保留 2000ms 固定余量", t == 2000u, detail);
    }
}

/* ==========================================================================================
 * P5：执行层 Blocker 回归（路径 ≥2 段时 remaining 必须非零且单调递减）
 * ========================================================================================== */
static void test_remaining_regression(void) {
    char detail[400];

    /* P5a：纯几何路径 3 段，remaining 应为 {210,160,80,0}，严格递减且差值=段长。
     * 旧错误实现把点到线段距离当段长，会得到全 0。 */
    {
        Point2D path[4];
        float remaining[4];
        path[0] = to_map(0.0f, 0.0f);
        path[1] = to_map(50.0f, 0.0f);
        path[2] = to_map(50.0f, 80.0f);
        path[3] = to_map(130.0f, 80.0f);

        int ok = Planner_BuildRemainingCm(path, 4, remaining, 4) ? 1 : 0;
        int values_ok = ok && near_f(remaining[0], 210.0f, PROF_FLOAT_TOL_CM) &&
                        near_f(remaining[1], 160.0f, PROF_FLOAT_TOL_CM) &&
                        near_f(remaining[2], 80.0f, PROF_FLOAT_TOL_CM) &&
                        near_f(remaining[3], 0.0f, PROF_FLOAT_TOL_CM);
        int decreasing = ok && (remaining[0] > remaining[1]) && (remaining[1] > remaining[2]) &&
                         (remaining[2] > remaining[3]) && remaining[0] > 0.0f;
        int diffs_ok = ok;
        for (int i = 0; i + 1 < 4; i++) {
            float seg = Planner_PointDistanceCm(path[i], path[i + 1]);
            float diff = remaining[i] - remaining[i + 1];
            if (!near_f(diff, seg, PROF_FLOAT_TOL_CM) || seg <= 0.0f) diffs_ok = 0;
        }
        snprintf(detail, sizeof(detail),
                 "3 段路径 remaining={%.4f,%.4f,%.4f,%.4f}（期望 {210,160,80,0}），严格递减=%d，差值=各段长=%d"
                 "（旧实现会全 0）",
                 ok ? (double)remaining[0] : -1.0, ok ? (double)remaining[1] : -1.0,
                 ok ? (double)remaining[2] : -1.0, ok ? (double)remaining[3] : -1.0, decreasing, diffs_ok);
        record("P5a", "多段路径 remaining 非零且严格递减（Blocker 回归）", values_ok && decreasing && diffs_ok,
               detail);
    }

    /* P5b：真实规划路径（zone1 HOME→MATL_PROCESSING 4 点 3 段）：
     * remaining[0] 必须等于规划长度，且逐段差值=段长、严格递减。 */
    {
        int ok = 0;
        if (MapCore_Init(1, s_grid, s_clarity, s_square_clarity)) {
            Point2D start = to_map(0.0f, 0.0f);
            Point2D goal = to_map(105.0f, 190.0f);
            PlannerPathStats_t stats;
            int len = Planner_PlanPath(start, goal, s_plan_path, MAX_PATH_LEN, &stats);
            if (len >= 3) {
                if (Planner_BuildRemainingCm(s_plan_path, len, s_remaining, MAX_PATH_LEN)) {
                    ok = near_f(s_remaining[0], stats.length_cm, 0.01f) && (s_remaining[len - 1] == 0.0f);
                    for (int i = 0; i + 1 < len; i++) {
                        float seg = Planner_PointDistanceCm(s_plan_path[i], s_plan_path[i + 1]);
                        float diff = s_remaining[i] - s_remaining[i + 1];
                        if (seg <= 0.0f || !near_f(diff, seg, PROF_FLOAT_TOL_CM)) ok = 0;
                        if (s_remaining[i] < s_remaining[i + 1]) ok = 0;
                    }
                }
                snprintf(detail, sizeof(detail),
                         "真实规划 zone1 HOME→MATL_PROCESSING: 点数=%d, stats.length=%.4fcm, remaining[0]=%.4fcm,"
                         " remaining[len-1]=%.4f, 全部段长>0 且单调递减=%d",
                         len, (double)stats.length_cm, (double)s_remaining[0],
                         (double)s_remaining[len > 0 ? len - 1 : 0], ok);
            } else {
                snprintf(detail, sizeof(detail), "真实规划失败（len=%d），无法回归", len);
            }
        } else {
            snprintf(detail, sizeof(detail), "MapCore_Init 失败，无法回归");
        }
        record("P5b", "真实规划路径 remaining[0]=规划长度且严格递减（端到端回归）", ok, detail);
    }
}

/* ==========================================================================================
 * P6：Planner_SegmentSpeedLimitCmS（按段长限制入段速度，sqrt(2*a*s) 反推 + clamp）
 * ========================================================================================== */
static void test_segment_speed_limit(void) {
    char detail[400];

    /* P6a：给定样例 seg=30 → 72.25（sqrt(2*90*29)），seg=300 → 120 封顶，seg=0.2 → 40 下限 */
    {
        float v30 = Planner_SegmentSpeedLimitCmS(30.0f);
        float v300 = Planner_SegmentSpeedLimitCmS(300.0f);
        float v02 = Planner_SegmentSpeedLimitCmS(0.2f);
        float v0 = Planner_SegmentSpeedLimitCmS(0.0f);
        int ok = near_f(v30, 72.25f, 0.05f) && near_f(v300, 120.0f, PROF_SPEED_TOL) &&
                 near_f(v02, 40.0f, PROF_SPEED_TOL) && near_f(v0, 40.0f, PROF_SPEED_TOL);
        snprintf(detail, sizeof(detail),
                 "seg=30→%.4fcm/s（期望72.25）, seg=300→%.2f（期望120封顶）, seg=0.2→%.2f（期望40下限）, "
                 "seg=0→%.2f",
                 (double)v30, (double)v300, (double)v02, (double)v0);
        record("P6a", "段长限速：sqrt(2*a*s) 反推 + [40,120] clamp", ok, detail);
    }

    /* P6b：段长扫描：结果始终在 [40,120] 且随段长单调不减 */
    {
        int ok = 1;
        float prev = -1.0f;
        float max_v = -1.0f;
        for (float seg = 0.0f; seg <= 500.0f; seg += 0.5f) {
            float v = Planner_SegmentSpeedLimitCmS(seg);
            if (v < 40.0f - PROF_SPEED_TOL || v > 120.0f + PROF_SPEED_TOL) ok = 0;
            if (v < prev - PROF_SPEED_TOL) ok = 0;
            prev = v;
            if (v > max_v) max_v = v;
        }
        snprintf(detail, sizeof(detail), "seg∈[0,500] 扫描: 速度单调不减、∈[40,120]，最大 %.2fcm/s", (double)max_v);
        record("P6b", "段长限速：单调性与安全区间", ok, detail);
    }
}

/* ==========================================================================================
 * P7：执行层实际剖面 = min(剩余路程速度, 段长限速)（过冲修复 + 旧 Blocker 回归）
 * ========================================================================================== */
static void test_execution_profile_combined(void) {
    char detail[420];

    /* P7a：合成 2 段路径（最后一段只有 30cm）：段长限速应压过剩余路程速度。
     * path: (0,0)→(100,0)→(130,0)cm；remaining={130,30,0}
     * 中间点：WaypointSpeed(30)=88 cm/s，而 SegmentLimit(30)=72.25 → 实际取 72.25（减少过冲）。 */
    {
        Point2D path[3];
        float remaining[3];
        path[0] = to_map(0.0f, 0.0f);
        path[1] = to_map(100.0f, 0.0f);
        path[2] = to_map(130.0f, 0.0f);
        int ok = Planner_BuildRemainingCm(path, 3, remaining, 3) ? 1 : 0;
        float seg1 = Planner_PointDistanceCm(path[1], path[2]);
        float speed_remaining = Planner_WaypointSpeedCmS(remaining[1], false);
        float speed_limit = Planner_SegmentSpeedLimitCmS(seg1);
        float speed_final = fminf(speed_remaining, speed_limit);
        ok = ok && near_f(remaining[0], 130.0f, PROF_FLOAT_TOL_CM) &&
             near_f(remaining[1], 30.0f, PROF_FLOAT_TOL_CM) && near_f(seg1, 30.0f, PROF_FLOAT_TOL_CM);
        snprintf(detail, sizeof(detail),
                 "2 段路径 remaining={130,30,0}: 中间点 WaypointSpeed=%.2f, SegmentLimit=%.2f, 实际=min=%.2f"
                 "（期望 72.25，说明段长限速生效）",
                 (double)speed_remaining, (double)speed_limit, (double)speed_final);
        record("P7a", "执行速度 = min(剩余路程速度, 段长限速)（短末段限速 72.25）",
               ok && near_f(speed_final, 72.25f, 0.05f), detail);
    }

    /* P7b：真实规划路径（zone1 HOME→MATL_PROCESSING，4 点 3 段）的整条执行剖面：
     * 首点速度>40（旧 bug 会恒 40）、末点=40、全程 ∈[40,120]、且不是全 40。
     * 旧 bug 特征：段长恒 0 → remaining 全 0 → WaypointSpeed 恒 40、SegmentLimit 恒 40。 */
    {
        int ok = 0;
        char profile[200];
        int off = 0;
        profile[0] = '\0';
        if (MapCore_Init(1, s_grid, s_clarity, s_square_clarity)) {
            Point2D start = to_map(0.0f, 0.0f);
            Point2D goal = to_map(105.0f, 190.0f);
            PlannerPathStats_t stats;
            int len = Planner_PlanPath(start, goal, s_plan_path, MAX_PATH_LEN, &stats);
            if (len >= 3 && Planner_BuildRemainingCm(s_plan_path, len, s_remaining, MAX_PATH_LEN)) {
                ok = 1;
                int all_40 = 1;
                float first_speed = 0.0f;
                for (int i = 0; i < len; i++) {
                    float seg = (i + 1 < len) ? Planner_PointDistanceCm(s_plan_path[i], s_plan_path[i + 1]) : 0.0f;
                    float base = Planner_WaypointSpeedCmS(s_remaining[i], true); /* 实际执行按 tight 限速 */
                    float limit = Planner_SegmentSpeedLimitCmS(seg);
                    float speed = fminf(base, limit);
                    if (speed < 40.0f - PROF_SPEED_TOL || speed > 120.0f + PROF_SPEED_TOL) ok = 0;
                    if (fabsf(speed - 40.0f) > PROF_SPEED_TOL) all_40 = 0;
                    if (i == 0) first_speed = speed;
                    off += snprintf(profile + off, sizeof(profile) - (size_t)off, "%s%.1f", (i == 0) ? "" : ",",
                                    (double)speed);
                    if (off > (int)sizeof(profile) - 16) off = (int)sizeof(profile) - 16;
                }
                /* 末点（目标点）速度应为终速 40 */
                float last_seg = Planner_PointDistanceCm(s_plan_path[len - 2], s_plan_path[len - 1]);
                float last_speed = fminf(Planner_WaypointSpeedCmS(0.0f, true), Planner_SegmentSpeedLimitCmS(last_seg));
                /* v2 方形车路径更绕/末段更短：首点可能被段长限速压到 40，只要求首点>=40 且剖面非全 40 */
                ok = ok && (first_speed >= 40.0f - PROF_SPEED_TOL) && all_40 == 0 &&
                     near_f(last_speed, 40.0f, PROF_SPEED_TOL) && near_f(s_remaining[len - 1], 0.0f, PROF_FLOAT_TOL_CM);
                snprintf(detail, sizeof(detail),
                         "真实路径 %d 点: 执行速度剖面=[%s]cm/s；首点 %.1f(>=40)、末点 %.1f(=40)、非全 40=%d"
                         "（旧 bug 会全 40）",
                         len, profile, (double)first_speed, (double)last_speed, all_40 ? 0 : 1);
            } else {
                snprintf(detail, sizeof(detail), "真实规划失败（len=%d），无法回归", len);
            }
        } else {
            snprintf(detail, sizeof(detail), "MapCore_Init 失败，无法回归");
        }
        record("P7b", "真实路径执行剖面：首点>=40、末点=40、非全 40（过冲修复 + 旧 Blocker 回归）", ok, detail);
    }
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

    snprintf(csv_path, sizeof(csv_path), "%s/profile_results.csv", out_dir);
    s_csv = fopen(csv_path, "w");
    if (s_csv == NULL) {
        fprintf(stderr, "无法写 %s\n", csv_path);
        return 2;
    }
    fprintf(s_csv, "test,name,result,detail\n");

    printf("profile_runner: 航点执行剖面纯函数单元测试（执行层 Blocker 回归）\n");
    printf("常量: 巡航=%.0fcm/s, 终速=%.0fcm/s, 减速区=%.0fcm, tight限速=%.0fcm/s, "
           "超时基数=%dms, 放大=%.2f\n",
           (double)PLANNER_WAYPOINT_SPEED_CM_S,
           (double)PLANNER_FINAL_SPEED_CM_S,
           (double)PLANNER_SLOWDOWN_DIST_CM,
           (double)PLANNER_TIGHT_SPEED_CM_S,
           PLANNER_WAYPOINT_TIMEOUT_BASE_MS,
           (double)PLANNER_WAYPOINT_TIMEOUT_SCALE);

    test_point_distance();
    test_build_remaining();
    test_waypoint_speed();
    test_waypoint_timeout();
    test_segment_speed_limit();
    test_remaining_regression();
    test_execution_profile_combined();

    fclose(s_csv);
    printf("\n========== 执行剖面单元测试结果 ==========\n");
    printf("PASS=%d, FAIL=%d\n", s_pass_count, s_fail_count);
    printf("输出: %s\n", csv_path);

    if (s_fail_count != 0) {
        printf("[FAIL] profile_runner 有 %d 条断言失败\n", s_fail_count);
        return 1;
    }
    printf("[PASS] profile_runner 全部断言通过\n");
    return 0;
}
