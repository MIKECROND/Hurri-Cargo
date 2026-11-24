/**
 * @file planner_tools.c
 * @author T-Car Team
 * @brief 路径规划器 shell 调试工具
 * @details
 *  命令组 `planner`：
 *    map               打印地图栅格（'#' 静态障碍，'@' 动态障碍，'.' 空闲）
 *    set_obs x y 1|0   在世界坐标 (x_cm, y_cm) 添加/清除一个动态障碍格
 *    clr x y           查询世界坐标 (x_cm, y_cm) 的精确清障距离
 *    stat              打印最近一次规划/执行的报告（航点数、清障、终点误差等）
 *    init 1|2          重新载入指定启动区的地图（规划器空闲时才允许）
 *    go x y yaw        规划并执行到世界坐标 (x_cm, y_cm, yaw)
 *    to id             规划并执行到地图点位 id
 *    dry id | dry x y yaw  空跑：只规划并打印路径/清障，不下发导航
 */

#include "shell.h"
#include "shell_cmd_group.h"
#include "log.h"
#include "cmsis_os.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "Events.h"
#include "planner_port.h"
#include "planner_algo.h"
#include "Cstar_map_core.h"

#if PLANNER_ENABLED && (PLANNER_PLATFORM == PLATFORM_MCU)

TargetPoint_t tool_point;

/**
 * @brief 用默认运动参数填充一个目标点（go/dry 命令行用）
 */
static void fill_tool_point(float x_cm, float y_cm, float yaw_deg) {
    tool_point.pose.x = x_cm;
    tool_point.pose.y = y_cm;
    tool_point.pose.yaw = yaw_deg;
    tool_point.motion.target_speed = PLANNER_WAYPOINT_SPEED_CM_S;
    tool_point.motion.target_angular_speed = PLANNER_WAYPOINT_ANGULAR_SPEED_DEG_S;
    tool_point.motion.acceleration = PLANNER_WAYPOINT_ACCEL_CM_S2;
    tool_point.motion.deceleration = PLANNER_WAYPOINT_DECEL_CM_S2;
    tool_point.motion.angular_acceleration = PLANNER_WAYPOINT_ANGULAR_ACCEL_DEG_S2;
    tool_point.arrive.check_mode = ARRIVE_CHECK_BOTH;
    tool_point.arrive.distance_threshold = PLANNER_FINAL_ARRIVE_DIST_CM;
    tool_point.arrive.yaw_threshold = PLANNER_FINAL_ALIGN_YAW_DEG;
    tool_point.arrive.timeout_ms = (uint16_t)(PLANNER_WAYPOINT_TIMEOUT_BASE_MS + 8000);
}

/**
 * @brief 校验 g_map 结构是否可用（防止结构被写坏后野指针）
 */
static bool map_ready(void) {
    if (g_map.grid == NULL || g_map.clearance == NULL) {
        logPrintln("ERROR: planner map not initialized");
        return false;
    }
    if (g_map.res <= 0.0f || g_map.res > 1.0f || g_map.grid_size <= 0 || g_map.grid_size > MAX_GRID_SIZE) {
        logPrintln("ERROR: g_map corrupted (grid=0x%08x, res=%.3f, grid_size=%d)",
                   (unsigned int)g_map.grid,
                   g_map.res,
                   g_map.grid_size);
        return false;
    }
    return true;
}

/**
 * @brief 世界坐标 (cm) -> 地图坐标 (m)
 */
static bool world_to_map(float x_cm, float y_cm, Point2D *out) {
    out->x = x_cm * 0.01f + MAP_OFFSET_X;
    out->y = y_cm * 0.01f + MAP_OFFSET_Y;
    if (out->x < 0.0f || out->x >= g_map.grid_size * g_map.res || out->y < 0.0f ||
        out->y >= g_map.grid_size * g_map.res) {
        logPrintln("Invalid world coordinates: (%.1f, %.1f)cm out of map", x_cm, y_cm);
        return false;
    }
    return true;
}

/**
 * @brief 添加/清除一个动态障碍格（雷达/手动调试用）
 */
static void PlannerTools_SetObstacle(int argc, char *argv[]) {
    if (argc != 4) {
        logPrintln("Usage: set_obs <x_cm> <y_cm> <0|1>  (1=set obstacle, 0=clear obstacle)");
        return;
    }
    if (!map_ready()) {
        return;
    }

    float wx = atof(argv[1]);
    float wy = atof(argv[2]);
    int value = atoi(argv[3]);
    if (value != 0 && value != 1) {
        logPrintln("Invalid value: must be 0 or 1");
        return;
    }

    Point2D p;
    if (!world_to_map(wx, wy, &p)) {
        return;
    }

    int gx = (int)(p.x / g_map.res);
    int gy = (int)(p.y / g_map.res);
    set_grid_val(&g_map, gx, gy, value ? MAP_CELL_DYNAMIC_OBSTACLE : MAP_CELL_FREE);
    MapCore_RebuildClearance();

    logPrintln("%s at world (%.1f, %.1f)cm -> grid (%d, %d), clr=%.1fcm",
               value ? "Dynamic obstacle set" : "Dynamic obstacle cleared",
               wx,
               wy,
               gx,
               gy,
               MapCore_PointClearanceCm(p));
}

/**
 * @brief 查询任意点的精确清障距离（调试安全余量）
 */
static void PlannerTools_Clearance(int argc, char *argv[]) {
    if (argc != 3) {
        logPrintln("Usage: clr <x_cm> <y_cm>");
        return;
    }
    if (!map_ready()) {
        return;
    }

    Point2D p;
    if (!world_to_map(atof(argv[1]), atof(argv[2]), &p)) {
        return;
    }

    logPrintln("Clearance at (%.1f, %.1f)cm: euclid=%.2fcm, square30=%.2fcm",
               atof(argv[1]),
               atof(argv[2]),
               MapCore_PointClearanceCm(p),
               MapCore_PointSquareClearanceCm(p, PLANNER_BODY_HALF_CM));
}

/**
 * @brief 规划并执行到世界坐标目标
 */
static void PlannerTools_GoToDirect(int argc, char *argv[]) {
    if (argc != 4) {
        logPrintln("Usage: go <x_cm> <y_cm> <yaw>");
        return;
    }

    fill_tool_point(atof(argv[1]), atof(argv[2]), atof(argv[3]));
    logPrintln("planner go -> %s", Planner_GoToDirect(&tool_point) ? "ok" : "failed");
}

/**
 * @brief 规划并执行到地图点位 ID
 */
static void PlannerTools_GofromID(int argc, char *argv[]) {
    if (argc != 2) {
        logPrintln("Usage: to <id>");
        return;
    }

    uint8_t id = (uint8_t)atoi(argv[1]);
    TargetPoint_t *target = Map_GetPoint(id);
    if (target == NULL) {
        logPrintln("Point ID:%u not found", id);
        return;
    }

    logPrintln("planner to ID:%u %s -> %s", id, target->name, Planner_GoToDirect(target) ? "ok" : "failed");
}

/**
 * @brief 空跑规划：只算路径并打印航点/清障，不下发导航（车不动）
 * @details 用法同 to / go，可随后用 `lcd plan_view [thick|thin]` 查看本次路径
 */
static void PlannerTools_DryRun(int argc, char *argv[]) {
    if (argc != 2 && argc != 4) {
        logPrintln("Usage: dry <id> | dry <x_cm> <y_cm> <yaw>");
        return;
    }

    bool ok;
    if (argc == 2) {
        uint8_t id = (uint8_t)atoi(argv[1]);
        TargetPoint_t *target = Map_GetPoint(id);
        if (target == NULL) {
            logPrintln("Point ID:%u not found", id);
            return;
        }
        logPrintln("dry to ID:%u %s", id, target->name);
        ok = Planner_PlanDry(target);
    } else {
        fill_tool_point(atof(argv[1]), atof(argv[2]), atof(argv[3]));
        ok = Planner_PlanDry(&tool_point);
    }

    logPrintln("dry -> %s", ok ? "ok" : "failed");
}

/**
 * @brief 打印最近一次规划/执行报告
 */
static void PlannerTools_Report(int argc, char *argv[]) {
    (void)argc;
    (void)argv;
    if (g_plan_path_len <= 0 && g_planner_report.waypoints == 0 && g_planner_report.fail_reason == 0) {
        logPrintln("No plan yet");
        return;
    }

    logPrintln("Plan: %u wp, %.1fcm, squareClr=%.2fcm, euclidClr=%.1fcm, tier=%s, nodes=%u, fail=%u",
               (unsigned int)g_planner_report.waypoints,
               g_planner_report.length_cm,
               g_planner_report.min_square_clearance_cm,
               g_planner_report.min_clearance_cm,
               (g_planner_report.tier == PLANNER_TIER_HARD) ? "tight" : "soft",
               (unsigned int)g_planner_report.expanded_nodes,
               (unsigned int)g_planner_report.fail_reason);
    if (g_planner_report.rotation_index == PLANNER_ROTATE_NONE) {
        logPrintln("Heading: keep %.0fdeg (no rotation)", g_planner_report.heading_after_deg);
    } else {
        logPrintln("Heading: rotate@wp%u %.0f->%.0fdeg (clearance=%.1fcm)",
                   (unsigned int)g_planner_report.rotation_index,
                   g_planner_report.heading_before_deg,
                   g_planner_report.heading_after_deg,
                   g_planner_report.rotation_clearance_cm);
    }
    logPrintln("Exec: retries=%u, plan=%ums, exec=%ums, final pos=%.2fcm yaw=%.2fdeg",
               (unsigned int)g_planner_report.exec_retries,
               (unsigned int)g_planner_report.plan_ms,
               (unsigned int)g_planner_report.exec_ms,
               g_planner_report.final_pos_err_cm,
               g_planner_report.final_yaw_err_deg);
}

/**
 * @brief 重新载入地图（切换启动区/雷达数据异常时使用）
 */
static void PlannerTools_Reinit(int argc, char *argv[]) {
    if (argc != 2) {
        logPrintln("Usage: init <1|2>");
        return;
    }
    if (is_planner_running) {
        logPrintln("Planner busy, refuse to reinit map");
        return;
    }

    uint8_t zone = (uint8_t)atoi(argv[1]);
    logPrintln("planner init zone %u -> %s", zone, Planner_Init(zone) ? "ok" : "failed");
}

/**
 * @brief 打印当前地图（'#'=静态障碍 '@'=动态障碍 '.'=空闲），行标是格号 y，列是格号 x
 * @details 世界坐标(cm) 与格号的换算：格 = 世界_cm/10 + 24
 */
static void PlannerTools_ShowMap(int argc, char *argv[]) {
    (void)argc;
    (void)argv;
    if (!map_ready()) {
        return;
    }

    int static_cnt = 0;
    int dyn_cnt = 0;
    logPrintln("Map %dx%d res=%.2fm (num = world cm/10 + 24)", g_map.grid_size, g_map.grid_size, g_map.res);
    char line[MAX_GRID_SIZE + 1];
    for (int gy = g_map.grid_size - 1; gy >= 0; gy--) {
        for (int gx = 0; gx < g_map.grid_size; gx++) {
            uint8_t v = get_grid_val(&g_map, gx, gy);
            if (v == MAP_CELL_STATIC_OBSTACLE) {
                static_cnt++;
                line[gx] = '#';
            } else if (v == MAP_CELL_DYNAMIC_OBSTACLE) {
                dyn_cnt++;
                line[gx] = '@';
            } else {
                line[gx] = '.';
            }
        }
        line[g_map.grid_size] = '\0';
        logPrintln("y%02d %s", gy, line);
    }
    logPrintln("static cells=%d, dynamic cells=%d", static_cnt, dyn_cnt);
}

ShellCommand PlannerToolsGroup[] = {
    SHELL_CMD_GROUP_ITEM(
        SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, set_obs, PlannerTools_SetObstacle, Set Dynamic Obstacle),
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, clr, PlannerTools_Clearance, Query Clearance),
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, go, PlannerTools_GoToDirect, Go Goal),
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, to, PlannerTools_GofromID, To ID),
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, dry, PlannerTools_DryRun, Plan Only No Motion),
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, stat, PlannerTools_Report, Last Plan Report),
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, init, PlannerTools_Reinit, Reload Map),
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, map, PlannerTools_ShowMap, Show Map),
    SHELL_CMD_GROUP_END()};
SHELL_EXPORT_CMD_GROUP(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_DISABLE_RETURN,
                       planner,
                       PlannerToolsGroup,
                       Planner Tools);

#endif /* PLANNER_ENABLED && PLANNER_PLATFORM == PLATFORM_MCU */
