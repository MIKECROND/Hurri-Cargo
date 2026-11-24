/**
 * @file port_distance_repro.c
 * @brief 复现并回归验证：planner_port.c 段长/剩余路程/速度/超时计算
 *
 * @details 历史缺陷（已由 Lead 修复，本文件保留为回归用例）：
 *          原 planner_port.c 把 `Planner_PointSegmentDistanceCm(path[i], path[i], path[i+1])`
 *          误当段长（p 与 a 同点，恒为 0），导致 execute_path 跳过所有航点、速度恒 40cm/s。
 *          修复后改为调用算法层纯函数：
 *            Planner_PointDistanceCm / Planner_BuildRemainingCm /
 *            Planner_WaypointSpeedCmS / Planner_SegmentSpeedLimitCmS / Planner_WaypointTimeoutMs。
 *
 *          本程序断言这些纯函数的数值，并检查旧误用（p 与 a 同点）确实恒为 0（说明当初的根因）。
 *
 * 编译（%TEMP%）：
 *   gcc -std=c99 -O2 -DPLANNER_HOST_TEST -msse2 -mfpmath=sse \
 *       -I Applications/Planner -I Applications/Planner/utils \
 *       Applications/Planner/tests/review/port_distance_repro.c \
 *       Applications/Planner/utils/Cstar_map_core.c Applications/Planner/utils/planner_algo.c \
 *       Applications/Planner/utils/priority_queue.c -o %TEMP%/port_distance_repro.exe -lm
 */

#define PLANNER_HOST_TEST 1

#include <math.h>
#include <stdio.h>

#include "Cstar_map_core.h"
#include "planner_algo.h"
#include "planner_config.h"

static int g_fail = 0;

static void expect_near(const char *tag, float got, float want, float tol) {
    int ok = fabsf(got - want) <= tol;
    printf("  [%s] %s: got=%.4f want~%.4f tol=%.4f\n", ok ? "OK" : "FAIL", tag, (double)got, (double)want,
           (double)tol);
    if (!ok) g_fail++;
}

static void expect_uint(const char *tag, unsigned got, unsigned want) {
    int ok = (got == want);
    printf("  [%s] %s: got=%u want=%u\n", ok ? "OK" : "FAIL", tag, got, want);
    if (!ok) g_fail++;
}

static void expect_bool(const char *tag, int got, int want) {
    int ok = (got == want);
    printf("  [%s] %s: got=%d want=%d\n", ok ? "OK" : "FAIL", tag, got, want);
    if (!ok) g_fail++;
}

int main(void) {
    /* 地图坐标（米）：世界坐标 (0,0) -> (100,0) -> (100,100) cm */
    Point2D path[3] = {{2.40f, 2.40f}, {3.40f, 2.40f}, {3.40f, 3.40f}};

    printf("=== 1. 两点距离（修复后 execute_path:31x 的段长来源） ===\n");
    expect_near("Planner_PointDistanceCm(p0,p1)", Planner_PointDistanceCm(path[0], path[1]), 100.0f, 1e-3f);
    expect_near("Planner_PointDistanceCm(p1,p2)", Planner_PointDistanceCm(path[1], path[2]), 100.0f, 1e-3f);
    expect_near("Planner_PointDistanceCm(p0,p0)", Planner_PointDistanceCm(path[0], path[0]), 0.0f, 1e-3f);

    printf("=== 2. 历史根因复核：p 与 a 同点时 PointSegmentDistance 恒为 0（旧代码就是这样传参的） ===\n");
    expect_near("PointSegmentDist(p0,p0,p1) 旧误用", Planner_PointSegmentDistanceCm(path[0], path[0], path[1]), 0.0f,
                1e-3f);
    expect_near("PointSegmentDist(p1,p1,p2) 旧误用", Planner_PointSegmentDistanceCm(path[1], path[1], path[2]), 0.0f,
                1e-3f);
    /* 正确用法仍供执行期偏离监测使用：点到线段距离 */
    Point2D q = {3.40f, 2.40f}; /* 落在 p1→p2 线段起点 */
    expect_near("PointSegmentDist(线外点, p0, p1) 正确用法", Planner_PointSegmentDistanceCm((Point2D){3.40f, 3.40f}, path[0], path[1]),
                100.0f, 1e-3f);
    (void)q;

    printf("=== 3. 剩余路程 Planner_BuildRemainingCm（execute_path:320） ===\n");
    float rem[3] = {-1.0f, -1.0f, -1.0f};
    int ok = Planner_BuildRemainingCm(path, 3, rem, 3);
    expect_bool("BuildRemaining(3点,cap=3)", ok, 1);
    expect_near("remaining[0] (p0->goal)", rem[0], 200.0f, 1e-3f);
    expect_near("remaining[1] (p1->goal)", rem[1], 100.0f, 1e-3f);
    expect_near("remaining[2] (终点)", rem[2], 0.0f, 1e-3f);
    expect_bool("BuildRemaining(NULL)", Planner_BuildRemainingCm(NULL, 3, rem, 3), 0);
    expect_bool("BuildRemaining(cap<len)", Planner_BuildRemainingCm(path, 3, rem, 2), 0);
    expect_bool("BuildRemaining(len=0)", Planner_BuildRemainingCm(path, 0, rem, 3), 0);

    printf("=== 4. 航点速度 Planner_WaypointSpeedCmS（execute_path:337） ===\n");
    expect_near("soft rem=0 (终点)", Planner_WaypointSpeedCmS(0.0f, false), 40.0f, 1e-3f);
    expect_near("soft rem=25 (减速区中)", Planner_WaypointSpeedCmS(25.0f, false), 80.0f, 1e-3f);
    expect_near("soft rem=50 (减速区边界)", Planner_WaypointSpeedCmS(50.0f, false), 120.0f, 1e-3f);
    expect_near("soft rem=100 (巡航)", Planner_WaypointSpeedCmS(100.0f, false), 120.0f, 1e-3f);
    expect_near("tight rem=100 (限速)", Planner_WaypointSpeedCmS(100.0f, true), 80.0f, 1e-3f);
    expect_near("tight rem=50 (限速)", Planner_WaypointSpeedCmS(50.0f, true), 80.0f, 1e-3f);
    expect_near("tight rem=10 (终点前)", Planner_WaypointSpeedCmS(10.0f, true), 56.0f, 1e-3f);
    expect_near("soft rem<0 (防御)", Planner_WaypointSpeedCmS(-5.0f, false), 40.0f, 1e-3f);

    printf("=== 5. 单段限速 Planner_SegmentSpeedLimitCmS（execute_path:339） ===\n");
    expect_near("seg=100cm", Planner_SegmentSpeedLimitCmS(100.0f), 120.0f, 1e-3f);
    expect_near("seg=20cm", Planner_SegmentSpeedLimitCmS(20.0f), sqrtf(2.0f * 90.0f * 19.0f), 1e-2f);
    expect_near("seg=5cm", Planner_SegmentSpeedLimitCmS(5.0f), 40.0f, 1e-3f);
    expect_near("seg=0cm", Planner_SegmentSpeedLimitCmS(0.0f), 40.0f, 1e-3f);

    printf("=== 6. 航点超时 Planner_WaypointTimeoutMs（execute_path:343） ===\n");
    expect_uint("seg=100,speed=120", Planner_WaypointTimeoutMs(100.0f, 120.0f), 3500u);
    expect_uint("seg=100,speed=40", Planner_WaypointTimeoutMs(100.0f, 40.0f), 6500u);
    expect_uint("seg=50,speed=120", Planner_WaypointTimeoutMs(50.0f, 120.0f), 2750u);
    expect_uint("seg=0,speed=40(下限2000)", Planner_WaypointTimeoutMs(0.0f, 40.0f), 2000u);
    expect_uint("seg=10000(上限60000)", Planner_WaypointTimeoutMs(10000.0f, 120.0f), 60000u);
    expect_uint("speed=0(防御)", Planner_WaypointTimeoutMs(100.0f, 0.0f), 60000u);

    printf("\n结果：%s（失败断言 %d 个）\n", (g_fail == 0) ? "ALL PASS" : "FAILED", g_fail);
    return (g_fail == 0) ? 0 : 1;
}
