/**
 * @file baseline_runner.c
 * @brief 重构前（HEAD）规划器的主机复现驱动 + 独立几何指标评测
 *
 * @details 用途：在 PC 上复现 HEAD 版本（提交 6413fac，重构前）的完整规划流程，并对输出路径
 *          计算与 Planner 实现无关的几何指标（清障距离、碰撞、长度、航点数、转角等），
 *          作为重构后版本的对比基线。
 *
 *          复现链路（与 HEAD 一致）：
 *            1. baseline_map_init()：照抄 HEAD planner_port.c 的 Planner_Init（矩形填充逻辑）；
 *            2. astar_plan(&g_map, start, goal, NULL, 0, false, raw)：HEAD planner_algo.c 原函数；
 *            3. baseline_simplify_path()：照抄 HEAD planner_port.c 的 simplify_path / segment_is_free；
 *            4. reference_metrics.c：独立几何真值指标（不读地图网格、不调 Planner 碰撞判断）。
 *
 *          依赖 HEAD 源码快照：由 tests/harness/baseline/snapshot_baseline.py 从 git 提交
 *          6413fac 导出到 %TEMP%\planner_harness\baseline_src，编译时用 -I 指向该目录。
 *          详见 tests/README.md。
 *
 *          输入点位（hardcode，与 HEAD nav_map_data.c 的“规划器版”完全一致）：
 *            zone1: HOME(0,0), QrCode(10,105), MATL_GRAB(105,14), MATL_STORAGE(190,105),
 *                   MATL_PROCESSING(105,190)                              单位：cm
 *            zone2: HOME(0,0), QrCode(105,10), MATL_GRAB(196,105), MATL_STORAGE(105,190),
 *                   MATL_PROCESSING(20,105)                               单位：cm
 *          对每个 zone 跑全部有向点对（排除起点 == 终点），共 20 条/zone。
 *
 *          输出（写到 --out-dir）：
 *            baseline_zone1.csv / baseline_zone2.csv    每条点对的汇总指标
 *            baseline_paths_zone1.csv / ..._zone2.csv   每条路径的简化航点明细（世界坐标）
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* HEAD 快照头文件（编译时 -I 指向 %TEMP%\planner_harness\baseline_src） */
#include "Cstar_map_core.h"
#include "planner_algo.h"
#include "planner_config.h"
#include "priority_queue.h"

/* 独立几何/指标库 */
#include "reference_metrics.h"

#if !PLANNER_ENABLED
#error "baseline_runner 需要 PLANNER_ENABLED=1 的 HEAD 快照头文件"
#endif

/* 与 HEAD Applications/nav/utils/nav_map.h 的 MAP_ZONE_1 / MAP_ZONE_2 取值一致 */
#define BASELINE_ZONE_1 1
#define BASELINE_ZONE_2 2

/* 每个 zone 的点位数（HEAD nav_map_data.c 规划器版） */
#define BASELINE_POINTS_PER_ZONE 5

/* 两个 zone 的障碍顶点数取最大，作为四边形坐标缓冲容量 */
#define BASELINE_MAX_OBSTACLE_POINTS \
    ((PRE_OBSTACLE_NUM_ZONE1 > PRE_OBSTACLE_NUM_ZONE2) ? PRE_OBSTACLE_NUM_ZONE1 : PRE_OBSTACLE_NUM_ZONE2)

/* ==========================================================================================
 * 1. HEAD 版地图初始化
 *    以下 baseline_map_init() 与 HEAD planner_port.c 的 Planner_Init 逐行一致，
 *    仅做重命名（to_map_coord -> baseline_to_map_coord）并省略日志，仅用于基线复现。
 * ========================================================================================== */

/* 与 HEAD planner_port.c 的 s_map_grid 定义一致；PLACE_IN_SRAM2 在主机 gcc 下展开为空 */
static uint8_t s_map_grid[MAX_MAP_BYTES];

/** @brief 场地坐标（物理位置） -> 地图坐标（与 HEAD planner_port.c to_map_coord 逐行一致） */
static inline Point2D baseline_to_map_coord(float wx, float wy) {
    Point2D p = {wx + MAP_OFFSET_X, wy + MAP_OFFSET_Y};
    return p;
}

/** @brief 地图坐标 -> 场地坐标（物理位置）（与 HEAD planner_port.c to_world_coord 逐行一致） */
static inline Point2D baseline_to_world_coord(float mx, float my) {
    Point2D p = {mx - MAP_OFFSET_X, my - MAP_OFFSET_Y};
    return p;
}

/**
 * @brief 复现 HEAD Planner_Init：初始化地图网格并按启动区载入预定义障碍物
 * @details 与 HEAD planner_port.c 的 Planner_Init 逐行一致：世界坐标先平移 +MAP_OFFSET，
 *          每 4 个点构成一个轴对齐方框，按 min/max 求网格范围并整块填 1；
 *          上边界用 (max - 1e-4) / res 避免正好落在格线上时多占一格。
 * @param zone 启动区（BASELINE_ZONE_1 / BASELINE_ZONE_2）
 * @return 恒为 true（与 HEAD 行为一致）
 */
static bool baseline_map_init(uint8_t zone) {
    g_map.grid = s_map_grid;
    memset(s_map_grid, 0, sizeof(s_map_grid)); /* 初始全空闲 */

    /* 按启动区选择预定义障碍物 */
    const Point2D *obstacles;
    int obstacle_num;
    if (zone == BASELINE_ZONE_2) {
        obstacles = s_pre_obstacles_zone2;
        obstacle_num = (int)PRE_OBSTACLE_NUM_ZONE2;
    } else {
        obstacles = s_pre_obstacles_zone1;
        obstacle_num = (int)PRE_OBSTACLE_NUM_ZONE1;
    }

    /* 障碍物为场地坐标，先平移到地图坐标；每 4 个点 = 一个轴对齐方框的 4 个顶点，填充整块矩形 */
    for (int i = 0; i + 3 < obstacle_num; i += 4) {
        Point2D p = baseline_to_map_coord(obstacles[i].x, obstacles[i].y);
        float min_x = p.x, max_x = p.x;
        float min_y = p.y, max_y = p.y;
        for (int k = 1; k < 4; k++) {
            Point2D q = baseline_to_map_coord(obstacles[i + k].x, obstacles[i + k].y);
            if (q.x < min_x) min_x = q.x;
            if (q.x > max_x) max_x = q.x;
            if (q.y < min_y) min_y = q.y;
            if (q.y > max_y) max_y = q.y;
        }

        int gx0 = (int)(min_x / g_map.res); /* 单位：m -> 地图网格索引 */
        int gx1 = (int)((max_x - 1e-4f) / g_map.res); /* 减极小量：避免上边界正好落在格线上时多占一格 */
        int gy0 = (int)(min_y / g_map.res);
        int gy1 = (int)((max_y - 1e-4f) / g_map.res);

        for (int gx = gx0; gx <= gx1; gx++) {
            for (int gy = gy0; gy <= gy1; gy++) {
                if (gx >= 0 && gx < g_map.grid_size && gy >= 0 && gy < g_map.grid_size) {
                    set_grid_val(&g_map, gx, gy, 1); /* 1 = 障碍 */
                }
            }
        }
    }

    return true;
}

/* ==========================================================================================
 * 2. HEAD 版路径直线化
 *    以下 segment_is_free() / baseline_simplify_path() 与 HEAD planner_port.c 的
 *    segment_is_free / simplify_path 逐行一致（仅重命名 simplify_path 以免与未来重构版重名），
 *    仅用于基线复现。
 * ========================================================================================== */

/**
 * @brief 检查两点之间的直线是否整段可通行（与 HEAD planner_port.c 逐行一致）
 * @details 按 PATH_SMOOTH_SAMPLE_STEP_M 采样，每个采样点用与 astar_plan 相同的物理半径判据
 *          （r_phys2 = r*(r+1)，即 3×3 邻域）检查。
 */
static bool segment_is_free(Point2D a, Point2D b) {
    int r = (int)(PHYSICAL_RADIUS / g_map.res);
    int r2 = r * (r + 1); /* 与 astar_plan 的 r_phys2 保持一致 */

    float dx = b.x - a.x, dy = b.y - a.y;
    float len = sqrtf(dx * dx + dy * dy);
    int steps = (int)(len / PATH_SMOOTH_SAMPLE_STEP_M);
    if (steps < 1) steps = 1;

    for (int k = 0; k <= steps; k++) {
        float t = (float)k / (float)steps;
        int cx = (int)((a.x + dx * t) / g_map.res);
        int cy = (int)((a.y + dy * t) / g_map.res);

        for (int i = -r; i <= r; i++) {
            for (int j = -r; j <= r; j++) {
                if (i * i + j * j > r2) continue;
                int nx = cx + i, ny = cy + j;
                if (nx < 0 || nx >= g_map.grid_size || ny < 0 || ny >= g_map.grid_size) return false;
                if (get_grid_val(&g_map, nx, ny) == 1) return false;
            }
        }
    }
    return true;
}

/**
 * @brief 路径直线化（string pulling）（与 HEAD planner_port.c 逐行一致）
 * @details 从起点开始尽量一步跳到最远的、与当前点之间整段可通行的路径点，只保留拐点。
 * @param path 路径点数组（原地压缩）
 * @param len 压缩前节点数
 * @return 压缩后节点数
 */
static int baseline_simplify_path(Point2D *path, int len) {
    if (len <= 2) return len;

    int out = 0;
    int i = 0;
    path[out++] = path[0]; /* path[0] 起点必留 */
    while (i < len - 1) {
        int j = len - 1;
        while (j > i + 1 && !segment_is_free(path[i], path[j]))
            j--;
        path[out++] = path[j];
        i = j;
    }
    return out;
}

/* ==========================================================================================
 * 3. 输入点位（HEAD nav_map_data.c 规划器版，单位 cm）
 * ========================================================================================== */

typedef struct {
    const char *name;
    float x_cm;
    float y_cm;
} NamedPoint;

static const NamedPoint s_zone1_points[BASELINE_POINTS_PER_ZONE] = {
    {"HOME", 0.0f, 0.0f},
    {"QrCode", 10.0f, 105.0f},
    {"MATL_GRAB", 105.0f, 14.0f},
    {"MATL_STORAGE", 190.0f, 105.0f},
    {"MATL_PROCESSING", 105.0f, 190.0f},
};

static const NamedPoint s_zone2_points[BASELINE_POINTS_PER_ZONE] = {
    {"HOME", 0.0f, 0.0f},
    {"QrCode", 105.0f, 10.0f},
    {"MATL_GRAB", 196.0f, 105.0f},
    {"MATL_STORAGE", 105.0f, 190.0f},
    {"MATL_PROCESSING", 20.0f, 105.0f},
};

/* ==========================================================================================
 * 4. 评测统计与输出
 * ========================================================================================== */

/** @brief 单个 zone 的汇总统计 */
typedef struct {
    int total; /* 点对总数 */
    int ok; /* A* + simplify 成功条数 */
    int fail; /* A* 失败条数 */
    int collision; /* 直线化后路径最小清障 < 机器人半径的条数（物理碰撞） */
    int raw_collision; /* A* 原始逐格路径最小清障 < 机器人半径的条数 */
    int sum_waypoints; /* 成功路径 simplify 后航点总数 */
    int max_waypoints;
    int sum_astar_pts; /* 成功路径 A* 原始格点总数 */
    double sum_length_cm;
    double min_clearance_cm; /* 本 zone 全部路径中的最小清障 */
    const char *worst_start;
    const char *worst_goal;
    float worst_x_cm;
    float worst_y_cm;
} ZoneStats;

/* 复用的静态缓冲（不占栈；MAX_PATH_LEN 来自旧版 config.h） */
static Point2D s_raw_path[MAX_PATH_LEN]; /* A* 原始逐格路径（地图坐标，m） */
static Point2D s_simplified_path[MAX_PATH_LEN]; /* 直线化后的路径（地图坐标，m） */
static float s_raw_xy[2 * REF_MAX_POINTS]; /* A* 原始路径（世界坐标，cm） */
static float s_path_xy[2 * REF_MAX_POINTS]; /* 直线化路径（世界坐标，cm） */
static float s_wp_clearance[REF_MAX_POINTS];
static float s_turn_deg[REF_MAX_POINTS];
static RefSegMetrics s_seg_metrics[REF_MAX_SEGMENTS];

/**
 * @brief 把 HEAD 版输出路径（地图坐标，m）转成世界坐标折线（cm）
 */
static void path_to_world_cm(const Point2D *path, int len, float *out_xy) {
    for (int i = 0; i < len; i++) {
        Point2D w = baseline_to_world_coord(path[i].x, path[i].y);
        out_xy[i * 2 + 0] = w.x * 100.0f;
        out_xy[i * 2 + 1] = w.y * 100.0f;
    }
}

/**
 * @brief 跑一个 zone 的全部有向点对，写 CSV 并打印控制台摘要
 * @param zone 启动区编号
 * @param out_dir 输出目录
 * @return 0 成功；非 0 表示文件打开失败等 IO 错误
 */
static int run_zone(uint8_t zone, const char *out_dir) {
    const NamedPoint *pts = (zone == BASELINE_ZONE_2) ? s_zone2_points : s_zone1_points;
    const Point2D *obstacles = (zone == BASELINE_ZONE_2) ? s_pre_obstacles_zone2 : s_pre_obstacles_zone1;
    int obstacle_num =
        (int)((zone == BASELINE_ZONE_2) ? PRE_OBSTACLE_NUM_ZONE2 : PRE_OBSTACLE_NUM_ZONE1);

    if (!baseline_map_init(zone)) return 1;

    /* 独立几何真值：由旧 config.h 的障碍四边形（世界坐标）构造轴对齐矩形，单位 cm */
    float quad_xy[2 * BASELINE_MAX_OBSTACLE_POINTS];
    for (int i = 0; i < obstacle_num; i++) {
        quad_xy[i * 2 + 0] = obstacles[i].x * 100.0f;
        quad_xy[i * 2 + 1] = obstacles[i].y * 100.0f;
    }
    RefRect rects[REF_MAX_RECTS];
    int rect_count = ref_build_rects(quad_xy, obstacle_num, rects, REF_MAX_RECTS);
    if (rect_count <= 0) {
        fprintf(stderr, "zone%u: 构造障碍矩形失败\n", (unsigned)zone);
        return 1;
    }

    char csv_path[512];
    char paths_path[512];
    char rects_path[512];
    char points_path[512];
    snprintf(csv_path, sizeof(csv_path), "%s/baseline_zone%u.csv", out_dir, (unsigned)zone);
    snprintf(paths_path, sizeof(paths_path), "%s/baseline_paths_zone%u.csv", out_dir, (unsigned)zone);
    snprintf(rects_path, sizeof(rects_path), "%s/baseline_rects_zone%u.csv", out_dir, (unsigned)zone);
    snprintf(points_path, sizeof(points_path), "%s/baseline_points_zone%u.csv", out_dir, (unsigned)zone);

    /* 点位表：打印到文件，报告工具据此在 markdown 里还原点位坐标 */
    FILE *points_csv = fopen(points_path, "w");
    if (points_csv == NULL) {
        fprintf(stderr, "无法写 %s\n", points_path);
        return 1;
    }
    fprintf(points_csv, "idx,name,x_cm,y_cm\n");
    for (int i = 0; i < BASELINE_POINTS_PER_ZONE; i++) {
        fprintf(points_csv, "%d,%s,%.1f,%.1f\n", i, pts[i].name, (double)pts[i].x_cm, (double)pts[i].y_cm);
    }
    fclose(points_csv);

    FILE *rects_csv = fopen(rects_path, "w");
    if (rects_csv == NULL) {
        fprintf(stderr, "无法写 %s\n", rects_path);
        return 1;
    }
    fprintf(rects_csv, "idx,x0_cm,y0_cm,x1_cm,y1_cm\n");
    for (int i = 0; i < rect_count; i++) {
        fprintf(rects_csv,
                "%d,%.4f,%.4f,%.4f,%.4f\n",
                i,
                rects[i].x0,
                rects[i].y0,
                rects[i].x1,
                rects[i].y1);
    }
    fclose(rects_csv);

    FILE *csv = fopen(csv_path, "w");
    if (csv == NULL) {
        fprintf(stderr, "无法写 %s\n", csv_path);
        return 1;
    }
    FILE *paths = fopen(paths_path, "w");
    if (paths == NULL) {
        fprintf(stderr, "无法写 %s\n", paths_path);
        fclose(csv);
        return 1;
    }

    fprintf(csv,
            "zone,start,goal,ok,astar_pts,waypoints,length_cm,min_clearance_cm,collision,min_clr_wp_idx,"
            "min_clr_seg,min_clr_rect,min_clr_x_cm,min_clr_y_cm,segs_over_radius,wp_over_radius,goal_error_cm,"
            "max_turn_deg,sum_turn_deg,raw_min_clearance_cm,raw_collision,raw_min_clr_x_cm,raw_min_clr_y_cm,"
            "raw_min_clr_rect\n");
    fprintf(paths, "zone,start,goal,wp_idx,world_x_cm,world_y_cm,clearance_cm,seg_len_cm,turn_deg\n");

    ZoneStats stats;
    memset(&stats, 0, sizeof(stats));
    stats.min_clearance_cm = 1.0e9;

    printf("\n===== baseline zone%u（HEAD 6413fac）: %d 个点位, %d 条有向点对 =====\n",
           (unsigned)zone,
           BASELINE_POINTS_PER_ZONE,
           BASELINE_POINTS_PER_ZONE * (BASELINE_POINTS_PER_ZONE - 1));

    for (int si = 0; si < BASELINE_POINTS_PER_ZONE; si++) {
        for (int gi = 0; gi < BASELINE_POINTS_PER_ZONE; gi++) {
            if (si == gi) continue;
            stats.total++;

            /* cm -> m，再平移到地图坐标（与 HEAD 流程一致） */
            Point2D start = baseline_to_map_coord(pts[si].x_cm / 100.0f, pts[si].y_cm / 100.0f);
            Point2D goal = baseline_to_map_coord(pts[gi].x_cm / 100.0f, pts[gi].y_cm / 100.0f);

            int raw_len = astar_plan(&g_map, start, goal, NULL, 0, false, s_raw_path);
            if (raw_len <= 0) {
                stats.fail++;
                fprintf(csv, "%u,%s,%s,0,-1,-1,-1,-1,0,-1,-1,-1,-1,-1,0,0,-1,-1,-1,-1,0,-1,-1,-1\n",
                        (unsigned)zone, pts[si].name, pts[gi].name);
                printf("[zone%u] %-15s -> %-15s  FAIL(A* 无解)\n", (unsigned)zone, pts[si].name, pts[gi].name);
                continue;
            }

            int astar_pts = raw_len; /* A* 原始逐格点数 */
            memcpy(s_simplified_path, s_raw_path, sizeof(Point2D) * (size_t)raw_len);
            int wp_count = baseline_simplify_path(s_simplified_path, raw_len); /* 直线化后的航点数 */

            /* 先评估 A* 原始逐格路径，用于区分“搜索判据本身不安全”还是“直线化把安全的路径带偏” */
            path_to_world_cm(s_raw_path, astar_pts, s_raw_xy);
            RefPathMetrics raw_m;
            if (!ref_measure_path(rects,
                                  rect_count,
                                  s_raw_xy,
                                  astar_pts,
                                  pts[gi].x_cm,
                                  pts[gi].y_cm,
                                  &raw_m,
                                  NULL,
                                  0)) {
                fprintf(stderr, "zone%u: %s -> %s 原始路径指标计算失败\n", (unsigned)zone, pts[si].name, pts[gi].name);
                fclose(csv);
                fclose(paths);
                return 1;
            }
            if (raw_m.collision) stats.raw_collision++;

            /* 再评估直线化后的路径（实际下发给导航的航点） */
            path_to_world_cm(s_simplified_path, wp_count, s_path_xy);

            RefPathMetrics m;
            if (!ref_measure_path(rects,
                                  rect_count,
                                  s_path_xy,
                                  wp_count,
                                  pts[gi].x_cm,
                                  pts[gi].y_cm,
                                  &m,
                                  s_seg_metrics,
                                  REF_MAX_SEGMENTS)) {
                fprintf(stderr, "zone%u: %s -> %s 指标计算失败\n", (unsigned)zone, pts[si].name, pts[gi].name);
                fclose(csv);
                fclose(paths);
                return 1;
            }

            stats.ok++;
            stats.sum_waypoints += m.point_count;
            stats.sum_astar_pts += astar_pts;
            stats.sum_length_cm += m.length_cm;
            if (m.point_count > stats.max_waypoints) stats.max_waypoints = m.point_count;
            if (m.collision) stats.collision++;
            if (m.min_clearance_cm < stats.min_clearance_cm) {
                stats.min_clearance_cm = m.min_clearance_cm;
                stats.worst_start = pts[si].name;
                stats.worst_goal = pts[gi].name;
                stats.worst_x_cm = m.min_clearance_x_cm;
                stats.worst_y_cm = m.min_clearance_y_cm;
            }

            fprintf(csv,
                    "%u,%s,%s,1,%d,%d,%.4f,%.4f,%d,%d,%d,%d,%.4f,%.4f,%d,%d,%.4f,%.4f,%.4f,%.4f,%d,%.4f,%.4f,%d\n",
                    (unsigned)zone,
                    pts[si].name,
                    pts[gi].name,
                    astar_pts,
                    m.point_count,
                    m.length_cm,
                    m.min_clearance_cm,
                    m.collision ? 1 : 0,
                    m.min_clearance_wp,
                    m.min_clearance_seg,
                    m.min_clearance_rect,
                    m.min_clearance_x_cm,
                    m.min_clearance_y_cm,
                    m.segs_over_radius,
                    m.waypoints_over_radius,
                    m.goal_error_cm,
                    m.max_turn_deg,
                    m.sum_turn_deg,
                    raw_m.min_clearance_cm,
                    raw_m.collision ? 1 : 0,
                    raw_m.min_clearance_x_cm,
                    raw_m.min_clearance_y_cm,
                    raw_m.min_clearance_rect);

            /* 航点明细：清障距离、段长、转角 */
            (void)ref_waypoint_clearances(rects, rect_count, s_path_xy, wp_count, s_wp_clearance, REF_MAX_POINTS);
            (void)ref_turn_angles(s_path_xy, wp_count, s_turn_deg, REF_MAX_POINTS);
            for (int k = 0; k < wp_count; k++) {
                float seg_len = (k == 0) ? 0.0f : s_seg_metrics[k - 1].len_cm;
                float turn = (k >= 1 && k <= wp_count - 2) ? s_turn_deg[k - 1] : 0.0f;
                fprintf(paths,
                        "%u,%s,%s,%d,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                        (unsigned)zone,
                        pts[si].name,
                        pts[gi].name,
                        k,
                        s_path_xy[k * 2 + 0],
                        s_path_xy[k * 2 + 1],
                        s_wp_clearance[k],
                        seg_len,
                        turn);
            }

            printf("[zone%u] %-15s -> %-15s ok astar=%4d wp=%2d len=%7.1fcm min_clr=%6.1fcm raw_clr=%6.1fcm%s",
                   (unsigned)zone,
                   pts[si].name,
                   pts[gi].name,
                   astar_pts,
                   m.point_count,
                   m.length_cm,
                   m.min_clearance_cm,
                   raw_m.min_clearance_cm,
                   m.collision ? "  <<< COLLISION" : "");
            if (m.collision) {
                printf(" at (%.1f,%.1f)cm", m.min_clearance_x_cm, m.min_clearance_y_cm);
            }
            printf("\n");
        }
    }

    fclose(csv);
    fclose(paths);

    /* zone 汇总 */
    double avg_wp = (stats.ok > 0) ? (double)stats.sum_waypoints / (double)stats.ok : 0.0;
    double avg_astar = (stats.ok > 0) ? (double)stats.sum_astar_pts / (double)stats.ok : 0.0;
    double avg_len = (stats.ok > 0) ? stats.sum_length_cm / (double)stats.ok : 0.0;
    printf("\n[zone%u] 汇总: 点对=%d 成功=%d 失败=%d 碰撞路径=%d (A*原始路径碰撞=%d) | 平均航点=%.2f 最大航点=%d "
           "平均A*格点=%.1f | 平均路径长=%.1fcm | 最小清障=%.2fcm",
           (unsigned)zone,
           stats.total,
           stats.ok,
           stats.fail,
           stats.collision,
           stats.raw_collision,
           avg_wp,
           stats.max_waypoints,
           avg_astar,
           avg_len,
           stats.min_clearance_cm);
    if (stats.worst_start != NULL) {
        printf(" (%s -> %s at (%.1f,%.1f)cm)",
               stats.worst_start,
               stats.worst_goal,
               stats.worst_x_cm,
               stats.worst_y_cm);
    }
    printf("\n输出: %s , %s\n", csv_path, paths_path);

    return 0;
}

/**
 * @brief 入口
 * @details 用法：baseline_runner [--zone 1|2|all] [--out-dir DIR]
 */
int main(int argc, char **argv) {
    int zone_sel = 0; /* 0 = all */
    const char *out_dir = ".";

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--zone") == 0 && i + 1 < argc) {
            zone_sel = atoi(argv[++i]);
            if (zone_sel != 0 && zone_sel != 1 && zone_sel != 2) {
                fprintf(stderr, "--zone 只能是 1 / 2 / 0(all)\n");
                return 2;
            }
        } else if (strcmp(argv[i], "--out-dir") == 0 && i + 1 < argc) {
            out_dir = argv[++i];
        } else {
            fprintf(stderr, "用法: %s [--zone 1|2|0(all)] [--out-dir DIR]\n", argv[0]);
            return 2;
        }
    }

    printf("baseline_runner: HEAD 6413fac 规划器主机复现（独立几何真值指标）\n");
    printf("地图: %d x %d 格 @ %.3fm/格; 机器人半径 %.1fcm; 路径采样步长 %.1fcm\n",
           g_map.grid_size,
           g_map.grid_size,
           (double)g_map.res,
           (double)REF_ROBOT_RADIUS_CM,
           (double)REF_SAMPLE_STEP_CM);

    int rc = 0;
    if (zone_sel == 0 || zone_sel == 1) rc |= run_zone(BASELINE_ZONE_1, out_dir);
    if (zone_sel == 0 || zone_sel == 2) rc |= run_zone(BASELINE_ZONE_2, out_dir);
    return rc;
}
