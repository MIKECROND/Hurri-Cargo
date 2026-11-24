/**
 * @file planner_port.c
 * @author T-Car Team
 * @brief 路径规划器移植层（方形车体 30×30）：定位输入 + 规划调度 + 航点/旋转执行 + 安全监测
 * @details
 *  执行流程（每次 Planner_GoTo_* 请求触发一次）：
 *    1) 读定位（重试直到有效），把场地坐标转成地图坐标；
 *    2) 调算法层 Planner_PlanPathWithYaw(start, goal, 当前yaw, 目标yaw)：
 *       方形车体净空 A* → 直线化 → 逐段方形校验 → 旋转安全点/朝向方案；
 *    3) 按方案执行：
 *       - 若规划在起点旋转：先原地转到目标朝向；
 *       - 平移段保持固定朝向（麦轮直行/蟹行，40cm/35cm 通道内绝不计旋转）；
 *       - 在规划出的旋转安全点原地转到目标朝向（该点欧氏清障 >= PLANNER_ROTATION_CLEARANCE_CM）；
 *       - 终点位置/朝向精度校验；仅当目标点旋转安全时才允许原地补转 yaw；
 *    4) 执行期监测：轨迹偏离 + 当前位姿/实际 yaw 的方形净空；旋转阶段额外监测
 *       欧氏清障 >= 旋转扫掠半径 PLANNER_ROTATION_SWEEP_RADIUS_CM（默认 21.22cm，
 *       车辆真实外缘不对称/有凸出时按实测配置）。
 */

#include "planner_port.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "planner_algo.h"
#include "log.h"
#include "Events.h"
#include "Cstar_map_core.h"
#include "planner_config.h"
#include "nav_map.h"
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

#if PLANNER_ENABLED && (PLANNER_PLATFORM == PLATFORM_MCU)

extern osMutexId_t Planner_MutexHandle;

/* ==========================================================================
 * 全局状态（供工具/LCD/任务查询）
 * ========================================================================== */
bool is_planner_running = false; // 运行状态，默认未运行

Point2D start_w_extend = {0.0f, 0.0f}; // 起始点（场地坐标，米）
Point2D goal_w_extend = {0.0f, 0.0f}; // 目标点（场地坐标，米）
float goal_yaw = 0.0f; // 目标朝向（度）

Point2D g_plan_path[MAX_PATH_LEN]; // 最近一次规划路径（地图坐标，米）
int g_plan_path_len = 0; // 路径节点数，0 表示当前无有效路径
PlannerReport_t g_planner_report; // 最近一次规划/执行报告

/* ==========================================================================
 * 静态存储（地图网格/距离场放 SRAM2，减少主 RAM 占用）
 * ========================================================================== */
PLACE_IN_SRAM2 static uint8_t s_map_grid[MAX_MAP_BYTES];
PLACE_IN_SRAM2 static uint8_t s_map_clearance[MAX_CELLS];
PLACE_IN_SRAM2 static int8_t s_map_square[MAX_CELLS];

static Point2D s_path[MAX_PATH_LEN]; // 当前执行路径（地图坐标，米）
static float s_remaining_cm[MAX_PATH_LEN]; // 每个航点之后剩余路程 (cm)
static TargetPoint_t s_waypoint; // 当前航点（导航层只缓存指针，必须静态存储）

static volatile bool s_planner_abort = false; // 外部中止请求（Planner_Stop 置位）
static volatile bool s_planner_dry_run = false; // 空跑标志：只规划不下发（Planner_PlanDry 置位）

/* 执行结果 */
typedef enum {
    EXEC_OK = 0, // 整条路径执行完成且终点精度达标
    EXEC_ABORT, // 外部请求中止
    EXEC_NAV_FAIL, // 导航失败（超时/提前结束）
    EXEC_REJECTED, // 导航拒绝目标点（参数非法/未初始化）
    EXEC_SECURITY, // 执行期安全监测触发（轨迹偏离/方形净空/旋转安全）
} ExecResult_t;

/* ==========================================================================
 * 坐标换算
 * ========================================================================== */

/** @brief 场地坐标（物理位置） -> 地图坐标 */
static inline Point2D to_map_coord(float wx, float wy) {
    Point2D p = {wx + MAP_OFFSET_X, wy + MAP_OFFSET_Y};
    return p;
}

/** @brief 地图坐标 -> 场地坐标（物理位置） */
static inline Point2D to_world_coord(float mx, float my) {
    Point2D p = {mx - MAP_OFFSET_X, my - MAP_OFFSET_Y};
    return p;
}

/** @brief 目标角度误差（归一化到 [-180, 180] 度） */
static float yaw_error_deg(float target, float current) {
    float e = fmodf(target - current, 360.0f);
    if (e > 180.0f) e -= 360.0f;
    if (e < -180.0f) e += 360.0f;
    return e;
}

/** @brief 规划失败原因的可读字符串 */
static const char *fail_reason_text(uint8_t reason) {
    switch (reason) {
    case PLANNER_FAIL_ENDPOINT:
        return "endpoint-in-square-zone";
    case PLANNER_FAIL_NO_SOLUTION:
        return "no-solution";
    case PLANNER_FAIL_NO_ROTATION_POINT:
        return "no-safe-rotation-point";
    case PLANNER_FAIL_BAD_PARAM:
        return "bad-param";
    default:
        return "unknown";
    }
}

/* ==========================================================================
 * 中止请求
 * ========================================================================== */

void Planner_Stop(void) {
    s_planner_abort = true;
}

/* ==========================================================================
 * 导航下发与等待
 * ========================================================================== */

/**
 * @brief 下发一个航点（可单独指定角速度，用于原地慢转）
 */
static bool send_waypoint_ex(Point2D wp,
                             float yaw_deg,
                             float speed_cm_s,
                             float angular_speed_deg_s,
                             float angular_accel_deg_s2,
                             ArriveCheckMode_t mode,
                             float dist_th_cm,
                             float yaw_th_deg,
                             uint32_t timeout_ms) {
    s_waypoint.pose.x = wp.x * 100.0f;
    s_waypoint.pose.y = wp.y * 100.0f;
    s_waypoint.pose.yaw = yaw_deg;
    s_waypoint.motion.target_speed = speed_cm_s;
    s_waypoint.motion.target_angular_speed = angular_speed_deg_s;
    s_waypoint.motion.acceleration = PLANNER_WAYPOINT_ACCEL_CM_S2;
    s_waypoint.motion.deceleration = PLANNER_WAYPOINT_DECEL_CM_S2;
    s_waypoint.motion.angular_acceleration = angular_accel_deg_s2;
    s_waypoint.arrive.check_mode = mode;
    s_waypoint.arrive.distance_threshold = dist_th_cm;
    s_waypoint.arrive.yaw_threshold = yaw_th_deg;
    s_waypoint.arrive.timeout_ms = (uint16_t)((timeout_ms > 60000U) ? 60000U : timeout_ms);
    return Nav_GoToDirect(&s_waypoint);
}

/**
 * @brief 下发一个平移航点（默认角速度）
 */
static bool send_waypoint(Point2D wp,
                          float yaw_deg,
                          float speed_cm_s,
                          ArriveCheckMode_t mode,
                          float dist_th_cm,
                          float yaw_th_deg,
                          uint32_t timeout_ms) {
    return send_waypoint_ex(wp,
                            yaw_deg,
                            speed_cm_s,
                            PLANNER_WAYPOINT_ANGULAR_SPEED_DEG_S,
                            PLANNER_WAYPOINT_ANGULAR_ACCEL_DEG_S2,
                            mode,
                            dist_th_cm,
                            yaw_th_deg,
                            timeout_ms);
}

/**
 * @brief 等待当前航点结束，期间做执行期安全监测
 * @param seg_from       本段起点（地图坐标，米）
 * @param seg_to         本段终点（地图坐标，米）
 * @param deadline_tick  规划器侧兜底截止时间（系统 tick）
 * @param rotation_phase true = 原地旋转阶段（增加"旋转扫掠半径"监测）
 */
static ExecResult_t wait_waypoint(Point2D seg_from, Point2D seg_to, uint32_t deadline_tick, bool rotation_phase) {
    for (;;) {
        if (s_planner_abort) {
            Nav_Stop();
            return EXEC_ABORT;
        }

        if ((int32_t)(osKernelGetTickCount() - deadline_tick) > 0) {
            Nav_Stop();
            logError("Planner: waypoint wait hard timeout, abort segment");
            return EXEC_NAV_FAIL;
        }

        NavState_t state = Nav_GetState();
        if (state == NAV_STATE_COMPLETE) {
            return EXEC_OK;
        }
        if (state == NAV_STATE_ERROR) {
            return EXEC_NAV_FAIL;
        }
        if (state != NAV_STATE_RUNNING && state != NAV_STATE_STOPPING) {
            return EXEC_NAV_FAIL;
        }

        /* 执行期安全监测：轨迹偏离 / 实际位姿（含实际 yaw）的方形净空 / 旋转安全 */
        if (PLANNER_TRACK_DEVIATION_CM > 0.0f || PLANNER_TRACK_MIN_SQUARE_CLEARANCE_CM > 0.0f || rotation_phase) {
            PoseTimestamp_t pose;
            if (Loc_Get(&pose)) {
                Point2D pose_w = {pose.pose.x * 0.01f, pose.pose.y * 0.01f};
                Point2D pose_map = to_map_coord(pose_w.x, pose_w.y);

                if (PLANNER_TRACK_DEVIATION_CM > 0.0f) {
                    float deviation_cm = Planner_PointSegmentDistanceCm(pose_map, seg_from, seg_to);
                    if (deviation_cm > PLANNER_TRACK_DEVIATION_CM) {
                        Nav_Stop();
                        logError("Planner: track deviation %.1fcm > %.1fcm, stop and replan",
                                 deviation_cm,
                                 PLANNER_TRACK_DEVIATION_CM);
                        return EXEC_SECURITY;
                    }
                }

                /* 平移阶段才做"实际 yaw 的方形净空"监测；旋转阶段该指标会扫过 45°，
                 * 轴对齐保守模型会明显低估真实旋转安全（真实扫掠是扫掠半径
                 * PLANNER_ROTATION_SWEEP_RADIUS_CM 的圆），
                 * 因此旋转阶段改由下面的旋转扫掠包络监测负责。 */
                if (!rotation_phase && PLANNER_TRACK_MIN_SQUARE_CLEARANCE_CM > 0.0f) {
                    float half_cm = Planner_HeadingFootprintHalfCm(pose.pose.yaw);
                    float square_cm = MapCore_PointSquareClearanceCm(pose_map, half_cm);
                    float allowed_cm = PLANNER_TRACK_MIN_SQUARE_CLEARANCE_CM;

                    /* 端点豁免区：任务点本身可能恰好贴墙/贴角（方形净空=0），
                     * 允许低到 -PLANNER_ENDPOINT_PENETRATION_TOL_CM，避免被误判碰撞。 */
                    if (g_plan_path_len >= 2) {
                        Point2D path_goal_map = g_plan_path[g_plan_path_len - 1];
                        Point2D path_start_map = g_plan_path[0];
                        float d_goal_cm = Planner_PointDistanceCm(pose_map, path_goal_map);
                        float d_start_cm = Planner_PointDistanceCm(pose_map, path_start_map);
                        if (d_goal_cm <= PLANNER_ENDPOINT_ZONE_CM || d_start_cm <= PLANNER_ENDPOINT_ZONE_CM) {
                            allowed_cm = -PLANNER_ENDPOINT_PENETRATION_TOL_CM;
                        }
                    }

                    if (square_cm < allowed_cm) {
                        Nav_Stop();
                        logError("Planner: square clearance %.2fcm < %.2fcm, emergency stop", square_cm, allowed_cm);
                        return EXEC_SECURITY;
                    }
                }

                /* 旋转阶段：车体任意朝向的扫掠包络 = 以车位姿点（旋转中心）为圆心、
                 * 半径 PLANNER_ROTATION_SWEEP_RADIUS_CM 的圆（默认 = 30×30 半对角线
                 * 21.22cm；车体有凸出时按实测最大外缘半径配置）。
                 * 中心欧氏清障 < 扫掠半径 + 0.5cm 时立即停车。 */
                if (rotation_phase) {
                    float circle_cm = MapCore_PointClearanceCm(pose_map);
                    const float rotation_safe_cm =
                        PLANNER_ROTATION_SWEEP_RADIUS_CM + PLANNER_ROTATION_MONITOR_EXTRA_CM;
                    if (circle_cm < rotation_safe_cm) {
                        Nav_Stop();
                        logError("Planner: rotation unsafe (clr %.1fcm < %.1fcm), emergency stop",
                                 circle_cm,
                                 rotation_safe_cm);
                        return EXEC_SECURITY;
                    }
                }
            }
        }

        osDelay(PLANNER_EXEC_MONITOR_PERIOD_MS);
    }
}

/* ==========================================================================
 * 原地旋转
 * ========================================================================== */

/**
 * @brief 原地旋转到目标朝向
 * @param wp_map         旋转位置（地图坐标，米）
 * @param target_yaw_deg 目标朝向（度）
 * @param delta_deg      需要旋转的角度（绝对值，用于超时估算）
 * @param dist_th_cm     位置阈值（中途旋转点放宽到航点到达阈值，避免旋转后额外平移）
 */
static ExecResult_t rotate_in_place(Point2D wp_map, float target_yaw_deg, float delta_deg, float dist_th_cm) {
    Point2D wp_world = to_world_coord(wp_map.x, wp_map.y);
    uint32_t timeout_ms = Planner_RotationTimeoutMs(delta_deg);
    if (!send_waypoint_ex(wp_world,
                          target_yaw_deg,
                          PLANNER_FINAL_SPEED_CM_S,
                          PLANNER_ROTATION_SPEED_DEG_S,
                          PLANNER_ROTATION_ACCEL_DEG_S2,
                          ARRIVE_CHECK_BOTH,
                          dist_th_cm,
                          PLANNER_FINAL_ALIGN_YAW_DEG,
                          timeout_ms)) {
        return EXEC_REJECTED;
    }
    uint32_t deadline_tick = osKernelGetTickCount() + timeout_ms + PLANNER_WAYPOINT_TIMEOUT_BASE_MS;
    return wait_waypoint(wp_map, wp_map, deadline_tick, true);
}

/* ==========================================================================
 * 路径执行
 * ========================================================================== */

/**
 * @brief 终点精度校验与补偿（方形车体：只有目标点旋转安全时才允许原地补转 yaw）
 * @param goal_map 终点（地图坐标，米）
 * @return true 位置精度达标
 */
static bool verify_goal_precision(Point2D goal_map) {
    Point2D goal_w = to_world_coord(goal_map.x, goal_map.y);
    int corrections = 0;
    const bool can_rotate_at_goal = MapCore_RotationSafe(goal_map) && PLANNER_GOAL_YAW_ALIGN_IF_SAFE;

    g_planner_report.final_pos_err_cm = -1.0f;
    g_planner_report.final_yaw_err_deg = -1.0f;

    for (;;) {
        PoseTimestamp_t pose;
        if (!Loc_Get(&pose)) {
            logWarning("Planner: localization lost, skip final precision check");
            return true;
        }

        Point2D pose_w = {pose.pose.x * 0.01f, pose.pose.y * 0.01f};
        float pos_err_cm = Planner_PointDistanceCm(pose_w, goal_w);
        float yaw_err_deg = fabsf(yaw_error_deg(goal_yaw, pose.pose.yaw));

        g_planner_report.final_pos_err_cm = pos_err_cm;
        g_planner_report.final_yaw_err_deg = yaw_err_deg;

        bool pos_ok = pos_err_cm <= PLANNER_FINAL_POS_TOL_CM;
        bool yaw_ok = yaw_err_deg <= PLANNER_FINAL_YAW_TOL_DEG;
        logInfo("Planner: final pose error pos=%.2fcm yaw=%.2fdeg", pos_err_cm, yaw_err_deg);

        if (pos_ok && yaw_ok) {
            return true;
        }

        /* 1) yaw 修正：只有终点本身旋转安全时才允许原地补转（否则会扫入障碍） */
        if (!yaw_ok && can_rotate_at_goal) {
            if (corrections >= PLANNER_FINAL_CORRECT_MAX) {
                logError("Planner: final yaw error out of tolerance");
                return false;
            }
            corrections++;
            logWarning("Planner: correct final yaw in place (%.2fdeg)", yaw_err_deg);
            if (rotate_in_place(goal_map, goal_yaw, yaw_err_deg, PLANNER_FINAL_ARRIVE_DIST_CM) != EXEC_OK) {
                return false;
            }
            continue;
        }
        if (!yaw_ok) {
            logWarning("Planner: goal point is not rotation-safe, cannot correct yaw here (yaw_err=%.2fdeg)",
                       yaw_err_deg);
        }

        /* 2) 位置修正：终点不安全时用实测 yaw + 仅距离模式，绝不触发导航层原地旋转 */
        if (!pos_ok) {
            if (corrections >= PLANNER_FINAL_CORRECT_MAX) {
                break;
            }
            corrections++;
            float keep_yaw = can_rotate_at_goal ? goal_yaw : pose.pose.yaw;
            ArriveCheckMode_t mode = can_rotate_at_goal ? ARRIVE_CHECK_BOTH : ARRIVE_CHECK_DISTANCE;
            float yaw_tol = can_rotate_at_goal ? PLANNER_FINAL_ALIGN_YAW_DEG : 180.0f;
            if (!send_waypoint(goal_w,
                               keep_yaw,
                               PLANNER_FINAL_SPEED_CM_S,
                               mode,
                               PLANNER_FINAL_ARRIVE_DIST_CM,
                               yaw_tol,
                               PLANNER_FINAL_CORRECT_TIMEOUT_MS)) {
                return false;
            }
            uint32_t deadline = osKernelGetTickCount() + PLANNER_FINAL_CORRECT_TIMEOUT_MS +
                                PLANNER_WAYPOINT_TIMEOUT_BASE_MS;
            if (wait_waypoint(goal_map, goal_map, deadline, false) != EXEC_OK) {
                return false;
            }
            continue;
        }

        /* 位置达标、yaw 超差但终点不可旋转：接受（平移段朝向已尽量保证） */
        logWarning("Planner: accept final yaw error %.2fdeg at non-rotation-safe goal", yaw_err_deg);
        return true;
    }

    /* 修正次数用尽：按最终误差判定 */
    if (g_planner_report.final_pos_err_cm <= PLANNER_FINAL_POS_TOL_CM * 2.0f) {
        if (g_planner_report.final_yaw_err_deg > PLANNER_FINAL_YAW_TOL_DEG) {
            if (can_rotate_at_goal) {
                logError("Planner: final yaw error out of tolerance");
                return false;
            }
            logWarning("Planner: goal not rotation-safe, accept yaw error %.2fdeg",
                       g_planner_report.final_yaw_err_deg);
        }
        logWarning("Planner: final position error slightly over tolerance, accepted");
        return true;
    }
    logError("Planner: final position error out of tolerance");
    return false;
}

/**
 * @brief 执行整条路径：按规划方案平移/旋转 → 终点精度校验
 * @param path          路径（地图坐标，米）
 * @param len           路径点数
 * @param start_yaw_deg 出发时实际朝向（旋转前平移段保持该朝向）
 * @return 执行结果
 */
static ExecResult_t execute_path(const Point2D *path, int len, float start_yaw_deg) {
    if (path == NULL || len < 2) {
        return EXEC_NAV_FAIL;
    }

    if (!Planner_BuildRemainingCm(path, len, s_remaining_cm, MAX_PATH_LEN)) {
        return EXEC_NAV_FAIL;
    }

    const bool tight = (g_planner_report.tier == PLANNER_TIER_HARD);
    const uint16_t rot_idx = g_planner_report.rotation_index;
    const float heading_before = g_planner_report.heading_before_deg;
    const float heading_after = g_planner_report.heading_after_deg;
    bool rotation_done = false;

    /* 规划在起点旋转：先转好再走 */
    if (rot_idx != PLANNER_ROTATE_NONE && rot_idx == 0) {
        float delta = fabsf(yaw_error_deg(heading_after, heading_before));
        ExecResult_t r = rotate_in_place(path[0], heading_after, delta, PLANNER_WAYPOINT_ARRIVE_DIST_CM);
        if (r != EXEC_OK) {
            return r;
        }
        rotation_done = true;
    }

    for (int i = 1; i < len; i++) {
        if (s_planner_abort) {
            return EXEC_ABORT;
        }

        Point2D wp = to_world_coord(path[i].x, path[i].y);
        float seg_cm = Planner_PointDistanceCm(path[i - 1], path[i]);
        if (seg_cm < 0.5f) {
            continue; /* 重复点直接跳过 */
        }

        /* 旋转点之后的航点用旋转后朝向；之前（含旋转点本身）用旋转前朝向 */
        bool after_rotation = rotation_done || (rot_idx != PLANNER_ROTATE_NONE && rot_idx != 0 && i > (int)rot_idx);
        float yaw_cmd = after_rotation ? heading_after : heading_before;

        const bool is_final = (i == len - 1);
        float speed_cm_s = Planner_WaypointSpeedCmS(s_remaining_cm[i], tight);
        float seg_speed_limit_cm_s = Planner_SegmentSpeedLimitCmS(seg_cm);
        if (seg_speed_limit_cm_s < speed_cm_s) {
            speed_cm_s = seg_speed_limit_cm_s;
        }
        uint32_t timeout_ms = Planner_WaypointTimeoutMs(seg_cm, speed_cm_s);

        ArriveCheckMode_t mode = ARRIVE_CHECK_DISTANCE;
        float dist_th_cm = PLANNER_WAYPOINT_ARRIVE_DIST_CM;
        float yaw_th_deg = 0.0f;
        if (is_final) {
            dist_th_cm = PLANNER_FINAL_ARRIVE_DIST_CM;
        }

        if (!send_waypoint(wp, yaw_cmd, speed_cm_s, mode, dist_th_cm, yaw_th_deg, timeout_ms)) {
            logError("Planner: nav rejected waypoint %d/%d", i + 1, len);
            return EXEC_REJECTED;
        }

        uint32_t deadline_tick = osKernelGetTickCount() + timeout_ms + PLANNER_WAYPOINT_TIMEOUT_BASE_MS;
        ExecResult_t result = wait_waypoint(path[i - 1], path[i], deadline_tick, false);
        if (result != EXEC_OK) {
            if (result == EXEC_ABORT) {
                return EXEC_ABORT;
            }
            logError("Planner: waypoint %d/%d failed (result=%d), nav state %d",
                     i + 1,
                     len,
                     (int)result,
                     (int)Nav_GetState());
            return result;
        }

        /* 到达旋转点：原地转到目标朝向 */
        if (!rotation_done && rot_idx != PLANNER_ROTATE_NONE && rot_idx != 0 && i == (int)rot_idx) {
            float delta = fabsf(yaw_error_deg(heading_after, heading_before));
            /* 目标点旋转用终点位置阈值，中途旋转点用普通航点阈值 */
            float rot_dist_th_cm = (i == len - 1) ? PLANNER_FINAL_ARRIVE_DIST_CM : PLANNER_WAYPOINT_ARRIVE_DIST_CM;
            ExecResult_t r = rotate_in_place(path[i], heading_after, delta, rot_dist_th_cm);
            if (r != EXEC_OK) {
                return r;
            }
            rotation_done = true;
        }
    }

    if (!verify_goal_precision(path[len - 1])) {
        return EXEC_NAV_FAIL;
    }

    return EXEC_OK;
}

/* ==========================================================================
 * 规划调度
 * ========================================================================== */

/**
 * @brief 规划一条路径（含朝向/旋转方案）并缓存/报告
 * @return 路径点数；0 表示失败
 */
static int plan_path(Point2D start_map, Point2D goal_map, float start_yaw_local, float goal_yaw_local) {
    PlannerPathStats_t stats;
    uint32_t plan_start_tick = osKernelGetTickCount();

    for (int attempt = 0; attempt < PLANNER_ASTAR_RETRY_MAX; attempt++) {
        int len = Planner_PlanPathWithYaw(
            start_map, goal_map, start_yaw_local, goal_yaw_local, s_path, MAX_PATH_LEN, &stats);
        if (len > 0) {
            g_planner_report.plan_ms = osKernelGetTickCount() - plan_start_tick;
            g_planner_report.waypoints = stats.waypoints;
            g_planner_report.expanded_nodes = stats.expanded_nodes;
            g_planner_report.rotation_index = stats.rotation_index;
            g_planner_report.heading_before_deg = stats.heading_before_deg;
            g_planner_report.heading_after_deg = stats.heading_after_deg;
            g_planner_report.rotation_clearance_cm = stats.rotation_clearance_cm;
            g_planner_report.length_cm = stats.length_cm;
            g_planner_report.min_square_clearance_cm = stats.min_square_clearance_cm;
            g_planner_report.min_clearance_cm = stats.min_clearance_cm;
            g_planner_report.tier = stats.tier;
            g_planner_report.fail_reason = PLANNER_FAIL_NONE;

            memcpy(g_plan_path, s_path, sizeof(Point2D) * (size_t)len);
            g_plan_path_len = len;

            Point2D start_w = to_world_coord(start_map.x, start_map.y);
            Point2D goal_w = to_world_coord(goal_map.x, goal_map.y);
            if (stats.rotation_index == PLANNER_ROTATE_NONE) {
                logInfo("Planner: (%.1f,%.1f,%.0fdeg)->(%.1f,%.1f,%.0fdeg)cm -> %d wp, %.0fcm, sqClr=%.2fcm, %s, no-rotate, %ums",
                        start_w.x * 100.0f,
                        start_w.y * 100.0f,
                        start_yaw_local,
                        goal_w.x * 100.0f,
                        goal_w.y * 100.0f,
                        goal_yaw_local,
                        len,
                        stats.length_cm,
                        stats.min_square_clearance_cm,
                        (stats.tier == PLANNER_TIER_SOFT) ? "soft" : "tight",
                        (unsigned int)g_planner_report.plan_ms);
            } else {
                logInfo("Planner: (%.1f,%.1f,%.0fdeg)->(%.1f,%.1f,%.0fdeg)cm -> %d wp, %.0fcm, sqClr=%.2fcm, %s, rotate@%d %.0f->%.0fdeg clear=%.1fcm, %ums",
                        start_w.x * 100.0f,
                        start_w.y * 100.0f,
                        start_yaw_local,
                        goal_w.x * 100.0f,
                        goal_w.y * 100.0f,
                        goal_yaw_local,
                        len,
                        stats.length_cm,
                        stats.min_square_clearance_cm,
                        (stats.tier == PLANNER_TIER_SOFT) ? "soft" : "tight",
                        (int)stats.rotation_index,
                        stats.heading_before_deg,
                        stats.heading_after_deg,
                        stats.rotation_clearance_cm,
                        (unsigned int)g_planner_report.plan_ms);
            }
            return len;
        }

        if (s_planner_abort) {
            return 0;
        }
        osDelay(200);
    }

    Point2D start_w = to_world_coord(start_map.x, start_map.y);
    Point2D goal_w = to_world_coord(goal_map.x, goal_map.y);
    g_planner_report.fail_reason = stats.fail_reason;

    /* 失败诊断：起点/终点的方形净空、A* 展开节点数（便于区分"端点被卡死"与"搜索无解"） */
    float start_half_cm = Planner_HeadingFootprintHalfCm(start_yaw_local);
    float goal_half_cm = Planner_HeadingFootprintHalfCm(goal_yaw_local);
    float start_sq_cm = MapCore_PointSquareClearanceCm(start_map, start_half_cm);
    float goal_sq_cm = MapCore_PointSquareClearanceCm(goal_map, goal_half_cm);
    logPrintln("Planner: FAIL(%s) start=(%.1f,%.1f,%.0fdeg) goal=(%.1f,%.1f,%.0fdeg)cm, tries=%d, "
               "startSq=%.2f goalSq=%.2f half=%.1f/%.1f nodes=%u",
               fail_reason_text(stats.fail_reason),
               start_w.x * 100.0f,
               start_w.y * 100.0f,
               start_yaw_local,
               goal_w.x * 100.0f,
               goal_w.y * 100.0f,
               goal_yaw_local,
               PLANNER_ASTAR_RETRY_MAX,
               start_sq_cm,
               goal_sq_cm,
               start_half_cm,
               goal_half_cm,
               (unsigned int)stats.expanded_nodes);
    return 0;
}

/**
 * @brief 空跑模式：打印路径、旋转方案与每个航点的净空
 */
static void print_dry_run_path(const Point2D *path, int len) {
    if (g_planner_report.rotation_index == PLANNER_ROTATE_NONE) {
        logPrintln("  rotation: none");
    } else {
        logPrintln("  rotation: @wp%d %.0fdeg -> %.0fdeg (clearance=%.1fcm)",
                   (int)g_planner_report.rotation_index,
                   g_planner_report.heading_before_deg,
                   g_planner_report.heading_after_deg,
                   g_planner_report.rotation_clearance_cm);
    }

    for (int i = 0; i < len; i++) {
        Point2D wp = to_world_coord(path[i].x, path[i].y);
        float half_cm = Planner_HeadingFootprintHalfCm(
            (g_planner_report.rotation_index != PLANNER_ROTATE_NONE && i > (int)g_planner_report.rotation_index)
                ? g_planner_report.heading_after_deg
                : g_planner_report.heading_before_deg);
        float square_cm = MapCore_PointSquareClearanceCm(path[i], half_cm);
        logPrintln("  [%d] x=%.1f y=%.1f cm squareClr=%.2fcm", i, wp.x * 100.0f, wp.y * 100.0f, square_cm);
    }
}

/* ==========================================================================
 * 一次完整规划/执行
 * ========================================================================== */

static bool read_current_pose(PoseTimestamp_t *pose) {
    int fail_count = 0;
    while (!Loc_Get(pose)) {
        if (s_planner_abort) {
            return false;
        }
        if (++fail_count >= 50) {
            logPrintln("Planner: FAIL(localization) pose unavailable, abort");
            return false;
        }
        osDelay(100);
    }
    return true;
}

static bool planner_run_once(void) {
    PoseTimestamp_t pose;

    if (!read_current_pose(&pose)) {
        return false;
    }

    Point2D start_w = {pose.pose.x * 0.01f, pose.pose.y * 0.01f};
    float start_yaw_local = pose.pose.yaw;

    Point2D goal_w;
    float goal_yaw_local = 0.0f;
    osMutexAcquire(Planner_MutexHandle, osWaitForever);
    goal_w = goal_w_extend;
    goal_yaw_local = goal_yaw;
    osMutexRelease(Planner_MutexHandle);

    start_w_extend = start_w;
    g_plan_path_len = 0;
    memset(&g_planner_report, 0, sizeof(g_planner_report));
    g_planner_report.rotation_index = PLANNER_ROTATE_NONE;
    g_planner_report.rotation_clearance_cm = -1.0f;
    g_planner_report.final_pos_err_cm = -1.0f;
    g_planner_report.final_yaw_err_deg = -1.0f;

    uint32_t exec_start_tick = osKernelGetTickCount();
    bool ok = false;

    for (int exec_try = 0; exec_try <= PLANNER_EXEC_RETRY_MAX; exec_try++) {
        Point2D start_map = to_map_coord(start_w.x, start_w.y);
        Point2D goal_map = to_map_coord(goal_w.x, goal_w.y);

        int len = plan_path(start_map, goal_map, start_yaw_local, goal_yaw_local);
        if (len <= 0) {
            break;
        }

        if (s_planner_dry_run) {
            logPrintln("Planner dry run: %d waypoints, sqClr=%.2fcm", len, g_planner_report.min_square_clearance_cm);
            print_dry_run_path(s_path, len);
            return true;
        }

        ExecResult_t result = execute_path(s_path, len, start_yaw_local);
        if (result == EXEC_OK) {
            ok = true;
            break;
        }

        if (result == EXEC_ABORT) {
            logPrintln("Planner: aborted by request");
            break;
        }

        if (exec_try >= PLANNER_EXEC_RETRY_MAX) {
            logPrintln("Planner: FAIL(exec) path execution failed (code %d)", (int)result);
            break;
        }

        g_planner_report.exec_retries++;
        logWarning("Planner: execution failed (code %d), replanning from current pose", (int)result);
        osDelay(300);

        if (!read_current_pose(&pose)) {
            break;
        }
        start_w.x = pose.pose.x * 0.01f;
        start_w.y = pose.pose.y * 0.01f;
        start_yaw_local = pose.pose.yaw;
        start_w_extend = start_w;
    }

    g_planner_report.exec_ms = osKernelGetTickCount() - exec_start_tick;
    if (ok) {
        logInfo("Planner completed: %d wp, %.0fcm, sqClr=%.2fcm",
                g_planner_report.waypoints,
                g_planner_report.length_cm,
                g_planner_report.min_square_clearance_cm);
    }
    return ok;
}

/* ==========================================================================
 * 初始化
 * ========================================================================== */

/**
 * @brief 启动时检查任务点位的方形净空，提示"方形车刚好贴边/贴角"的点位
 * @note  例如 30×30 车体在 HOME/STORAGE/PROCESSING 的规划净空=0（恰好相切），
 *        实际定位/摆放有误差时容易触发端点失败或执行期安全停车；建议把点位
 *        往自由空间挪 2~3cm（改 nav_map_data.c），而不是放宽规划安全门限。
 */
static void check_mission_point_clearance(void) {
    static const char *const names[] = {"HOME", "QrCode", "MATL_GRAB", "MATL_STORAGE", "MATL_PROCESSING"};
    const int count = (int)(sizeof(names) / sizeof(names[0]));

    for (int i = 0; i < count; i++) {
        TargetPoint_t *pt = Map_GetPointByName(names[i]);
        if (pt == NULL) {
            continue;
        }
        Point2D pos = to_map_coord(pt->pose.x * 0.01f, pt->pose.y * 0.01f);
        float square_cm = MapCore_PointSquareClearanceCm(pos, PLANNER_BODY_HALF_CM);
        if (square_cm < PLANNER_SQUARE_HARD_MARGIN_CM) {
            logWarning("Planner: mission point %s square30 clearance=%.2fcm (<%.2fcm), car touches obstacle; "
                       "consider moving point 2-3cm into free space",
                       names[i],
                       square_cm,
                       PLANNER_SQUARE_HARD_MARGIN_CM);
        }
    }
}

bool Planner_Init(uint8_t zone) {
    if (!MapCore_Init(zone, s_map_grid, s_map_clearance, s_map_square)) {
        logPrintln("Planner: FAIL(map-init)");
        return false;
    }

    check_mission_point_clearance();

    g_plan_path_len = 0;
    logInfo("Planner: map ready zone=%u, rects=%u, obstacleCells=%u, res=%ucm, body=%ux%ucm, "
            "rotSweep=%.1fcm, rotClear=%.1fcm",
            (unsigned int)zone,
            (unsigned int)MapCore_StaticRectCount(),
            (unsigned int)MapCore_ObstacleCells(),
            PLANNER_MAP_RES_CM,
            (unsigned int)PLANNER_CAR_SIDE_CM,
            (unsigned int)PLANNER_CAR_SIDE_CM,
            PLANNER_ROTATION_SWEEP_RADIUS_CM,
            PLANNER_ROTATION_CLEARANCE_CM);
    return true;
}

/* ==========================================================================
 * 对外请求接口
 * ========================================================================== */

static bool planner_request(TargetPoint_t *target, bool dry_run) {
    if (target == NULL) {
        return false;
    }

    osMutexAcquire(Planner_MutexHandle, osWaitForever);

    if (is_planner_running) {
        osMutexRelease(Planner_MutexHandle);
        return false;
    }

    goal_w_extend.x = target->pose.x * 0.01f;
    goal_w_extend.y = target->pose.y * 0.01f;
    goal_yaw = target->pose.yaw;
    s_planner_abort = false;
    s_planner_dry_run = dry_run;

    osEventFlagsClear(Planner_EventHandle, PLANNER_EVT_DONE | PLANNER_EVT_ERROR);

    is_planner_running = true;
    osMutexRelease(Planner_MutexHandle);

    uint32_t flags = osEventFlagsWait(
        Planner_EventHandle, PLANNER_EVT_DONE | PLANNER_EVT_ERROR, osFlagsWaitAny, osWaitForever);

    if (flags == PLANNER_EVT_DONE) {
        return true;
    }
    return false;
}

bool Planner_GoToDirect(TargetPoint_t *target) {
    return planner_request(target, false);
}

bool Planner_PlanDry(TargetPoint_t *target) {
    return planner_request(target, true);
}

bool Planner_GoTo_fromName(const char *name) {
    TargetPoint_t *target = Map_GetPointByName(name);
    if (target == NULL) {
        return false;
    }
    return Planner_GoToDirect(target);
}

/* ==========================================================================
 * 规划任务
 * ========================================================================== */

void Planner_Task(void *argument) {
    (void)argument;
    osEventFlagsWait(System_StatusHandle, SYS_INIT_COMPLETE, osFlagsNoClear, osWaitForever);

    for (;;) {
        osDelay(1);

        osMutexAcquire(Planner_MutexHandle, osWaitForever);
        if (!is_planner_running) {
            osMutexRelease(Planner_MutexHandle);
            continue;
        }
        osMutexRelease(Planner_MutexHandle);

        bool ok = planner_run_once();

        osMutexAcquire(Planner_MutexHandle, osWaitForever);
        is_planner_running = false;
        osEventFlagsSet(Planner_EventHandle, ok ? PLANNER_EVT_DONE : PLANNER_EVT_ERROR);
        osMutexRelease(Planner_MutexHandle);
        osDelay(200);
    }
}

#elif PLANNER_ENABLED && (PLANNER_PLATFORM == PLATFORM_RASPBERRY)

bool Planner_GoToDirect(TargetPoint_t *target) {
    (void)target;
    return true;
}

bool Planner_GoTo_fromName(const char *name) {
    (void)name;
    return true;
}

bool Planner_PlanDry(TargetPoint_t *target) {
    (void)target;
    return true;
}

void Planner_Stop(void) {
}

void Planner_Task(void *argument) {
    (void)argument;
    osEventFlagsWait(System_StatusHandle, SYS_INIT_COMPLETE, osFlagsNoClear, osWaitForever);
    osThreadExit();
}

#else

__weak void Planner_Task(void *argument) {
    (void)argument;
    osEventFlagsWait(System_StatusHandle, SYS_INIT_COMPLETE, osFlagsNoClear, osWaitForever);
    osThreadExit();
}

#endif /* PLANNER_ENABLED && PLANNER_PLATFORM == PLATFORM_MCU */
