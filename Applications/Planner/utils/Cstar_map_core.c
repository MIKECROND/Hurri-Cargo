/**
 * @file Cstar_map_core.c
 * @author T-Car Team
 * @brief 地图核心实现：栅格位图、欧氏/方形两层清障距离场与连续几何碰撞判定
 * @details
 *  设计要点（与 Cstar_map_core.h 的说明对应）：
 *   1) 预定义障碍物数据以"世界坐标（米）的 4 顶点一组"形式保存在本文件，加载时
 *      平移到地图坐标并构造轴对齐矩形 s_static_rects[]，这是碰撞/余量判定的精确几何；
 *   2) 栅格位图 g_map.grid（2bit/格）只用于显示/调试与记录动态障碍，写入静态障碍
 *      时采用"格覆盖区间与矩形有交集即标记"的保守策略，不参与精确判定；
 *   3) 欧氏清障距离场 g_map.clearance[] 缓存每格中心的精确清障距离（cm，饱和到
 *      PLANNER_CLEARANCE_CAP_CM），用于原地旋转安全性判断与诊断；雷达更新动态障碍后
 *      调用 MapCore_RebuildClearance() 重建；重建时先扫描位图缓存动态障碍格（容量
 *      不足退化为全图扫描，绝不漏检）；
 *   4) 方形带符号清障场 g_map.square_clearance[]（int8，cm）缓存"方形车体中心到
 *      障碍按半长 H0=PLANNER_BODY_HALF_CM 做 Minkowski 扩张后的中心禁区"的带符号
 *      距离：>0 余量、0 相切、<0 侵入深度。扩张量在 x/y 两轴相同，故任意朝向 psi
 *      的投影半长 H(psi)=half*(|cos|+|sin|) 的净空 = 基值 - (H(psi)-H0)，A* 可
 *      O(1) 查表判断任意朝向可行性；障碍格直接存足够负的值，保证不会被当作可通行；
 *   5) 所有精确查询都用"距离平方取最小、最后统一开方"的写法：结果与逐个开方取
 *      最小完全一致，但重建整张距离场时只需 ~MAX_CELLS 次 sqrtf，满足毫秒级要求。
 *      方形净空额外区分"在扩张矩形内"（负值 = 侵入深度，取四边最小穿透）与
 *      "在外"（正值 = 欧氏距离）两种情况，比较在米域/平方域完成，最后统一换算 cm。
 *
 *  纯 C99：无动态内存、无 FreeRTOS/HAL 依赖、无 ARM 专有属性；定义 PLANNER_HOST_TEST
 *  后可直接用 PC 编译器（gcc）编译做单元测试。
 */

#include "Cstar_map_core.h"

#if PLANNER_ENABLED && (PLANNER_PLATFORM == PLATFORM_MCU)

#include <math.h>
#include <string.h>

/* ==========================================================================
 * 1. 内部常量
 * ========================================================================== */

/* 启动区编号：与 nav_map.h 的 MAP_ZONE_1 / MAP_ZONE_2 取值一致。
 * 本模块不包含 nav 头文件，避免地图核心反向依赖导航层。 */
#define MAP_CORE_ZONE_1 1u
#define MAP_CORE_ZONE_2 2u

/* 栅格化浮点容差 (m)：min/max 正好落在格线上时避免因浮点误差多占/少占一格 */
#define MAP_CORE_GRID_TOL 1e-6f

/* 距离无穷大 (m)：求最小值时的初值（用有限大值，避免 inf 参与算术运算） */
#define MAP_CORE_DIST_INF 1.0e9f

/* 动态障碍格缓存容量：超过后精确查询退化为全图扫描，保证不漏检动态障碍 */
#define MAP_CORE_DYNAMIC_CELL_CAP 512

/* 线段采样步数上限：正常地图内最长线段约 340 步，此上限只用于防御非法输入 */
#define MAP_CORE_SEGMENT_MAX_STEPS 2048

/* 米 → 厘米换算系数 */
#define MAP_CORE_CM_PER_M 100.0f

/* "满足清障要求"的比较容差 (cm)：吸收浮点误差，与 Cstar_map_core.h 的约定一致 */
#define MAP_CORE_FREE_TOL_CM 1e-3f

/* 清障距离场取整容差 (cm)：格中心到障碍的距离数学真值经常正好是整数（例如 300mm
 * 窄缝中心/350mm 通道中心恰好 15cm），但浮点表示会算成 14.99998；先加该容差再向下
 * 取整，保证存入的是"数学真值的向下取整"。否则这些恰好卡在机器人半径上的格会被存成
 * 半径-1，A* 的格清障硬过滤会把本可 tight 通过的窄通道整条判死（实测会导致 zone2
 * 部分点位不可达）。容差远小于 1cm，不影响保守性。 */
#define MAP_CORE_CLEARANCE_ROUND_TOL_CM 1e-3f

/* 方形带符号清障场的存储范围 (cm)：int8 存储，先向下取整再饱和到该范围。
 * 障碍格（位图非 0）直接存 MIN，表示"格中心在障碍内，任何朝向都碰撞"，
 * 保证 A* 绝不会把障碍格当成可通行格。 */
#define MAP_CORE_SQUARE_FIELD_MIN (-100)
#define MAP_CORE_SQUARE_FIELD_MAX 100

/* ==========================================================================
 * 2. 预定义障碍物数据（世界坐标，单位：米，左下角为原点）
 *
 *  每 4 个点 = 一个轴对齐方框的 4 个顶点（顺序任意），加载时取该组 4 点的
 *  min/max 构造一个轴对齐矩形（与旧版 Planner_Init 的矩形填充分组方式一致）。
 *  数据原样迁移自旧 planner_config.h（重构后该文件不再放地图数据）。
 * ========================================================================== */

/* ---------- 原始障碍点表：X(x, y) ---------- */
#define PRE_OBSTACLES_TABLE(X)      \
    X(0.45f, 0.45f)                 \
    X(1.65f, 0.45f)                 \
    X(0.45f, 1.65f)                 \
    X(1.65f, 1.65f)                 \

/* ---------- zone1：原样 ---------- */
static const Point2D s_pre_obstacles_zone1[] = {
#define X(x, y) { (x), (y) },
    PRE_OBSTACLES_TABLE(X)
#undef X
};

/* ---------- zone2：顺时针旋转 90°，(x,y) -> (y,-x) ---------- */
static const Point2D s_pre_obstacles_zone2[] = {
#define X(x, y) { (y), -(x) },
    PRE_OBSTACLES_TABLE(X)
#undef X
};

#define PRE_OBSTACLE_NUM_ZONE1 (sizeof(s_pre_obstacles_zone1) / sizeof(s_pre_obstacles_zone1[0]))
#define PRE_OBSTACLE_NUM_ZONE2 (sizeof(s_pre_obstacles_zone2) / sizeof(s_pre_obstacles_zone2[0]))

/* ==========================================================================
 * 3. 全局对象（定义在本文件，供 priority_queue / 算法层 / 移植层引用）
 * ========================================================================== */

/* 地图对象：grid/clearance/square_clearance 缓冲区由移植层（planner_port.c）提供并
 * 传入 MapCore_Init，本模块只负责填充内容。初始化列表覆盖 5 个字段
 * （grid, clearance, square_clearance, res, grid_size）。 */
ServerMap g_map = {NULL, NULL, NULL, PLANNER_MAP_RES, MAX_GRID_SIZE};

/* A* 优先队列的分时共享内存池：priority_queue.c 的 pq_init() 会把 pq->data 指向
 * g_mem_pool.pq_data，因此实例必须定义在此文件（类型与 extern 声明在 planner_config.h）。 */
PlannerMemoryPool g_mem_pool;

/* ==========================================================================
 * 4. 内部数据
 * ========================================================================== */

/* 轴对齐障碍矩形（地图坐标，米；约定 min <= max） */
typedef struct {
    float min_x;
    float min_y;
    float max_x;
    float max_y;
} MapRect;

/* 当前启动区的静态障碍矩形（精确几何，来自预定义障碍物数据） */
static MapRect s_static_rects[PLANNER_MAX_STATIC_RECTS];
static uint8_t s_static_rect_count = 0;

/* 动态障碍格缓存（存 gx * grid_size + gy；容量不足时置溢出标志，查询退化全图扫描） */
static uint16_t s_dynamic_cells[MAP_CORE_DYNAMIC_CELL_CAP];
static int s_dynamic_cell_count = 0;
static bool s_dynamic_overflow = false;

/* ==========================================================================
 * 5. 内部几何工具
 * ========================================================================== */

/**
 * @brief 点到轴对齐矩形的欧氏距离平方 (m²)
 * @details 分别把 x、y 坐标夹到矩形区间外再求平方和；点在矩形内部（含边界）时为 0。
 * @param p     查询点（地图坐标，米）
 * @param min_x 矩形最小 x (m)
 * @param min_y 矩形最小 y (m)
 * @param max_x 矩形最大 x (m)
 * @param max_y 矩形最大 y (m)
 * @return 点到矩形的距离平方 (m²)
 */
static float map_rect_distance_sq(Point2D p, float min_x, float min_y, float max_x, float max_y) {
    float dx = 0.0f;
    float dy = 0.0f;

    if (p.x < min_x) {
        dx = min_x - p.x;
    } else if (p.x > max_x) {
        dx = p.x - max_x;
    }
    if (p.y < min_y) {
        dy = min_y - p.y;
    } else if (p.y > max_y) {
        dy = p.y - max_y;
    }
    return dx * dx + dy * dy;
}

/**
 * @brief 累加"查询点到某个障碍矩形的 Minkowski 扩张矩形"的带符号距离 (m)
 * @details 扩张矩形 = [min_x-half, max_x+half] × [min_y-half, max_y+half]
 *          （方形车体与轴对齐障碍的碰撞等价于车体中心进入该矩形）。
 *          - 查询点在扩张矩形内：计算到四边的穿透深度，取最小值（轴对齐侵入深度），
 *            与已有候选比较取"最深"（最大值），存入 *inside_depth_m >= 0；
 *          - 查询点在扩张矩形外：计算欧氏距离平方，与已有候选取最小，存入 *outside_sq。
 *          两类候选分开累计：侵入（负值）在语义上永远优于外部距离（正值），
 *          因此最后只需判断 *inside_depth_m 是否有有效值（初值 <0 表示无侵入）。
 * @param p              查询点（地图坐标，米）
 * @param min_x          障碍矩形最小 x (m)
 * @param min_y          障碍矩形最小 y (m)
 * @param max_x          障碍矩形最大 x (m)
 * @param max_y          障碍矩形最大 y (m)
 * @param half_m         车体半长（Minkowski 扩张量，米）
 * @param inside_depth_m 输入/输出：已有最大侵入深度 (m)，初值应 < 0
 * @param outside_sq     输入/输出：已有最小外部距离平方 (m²)，初值应很大
 */
static void map_core_accumulate_rect_signed(Point2D p, float min_x, float min_y, float max_x, float max_y,
                                            float half_m, float *inside_depth_m, float *outside_sq) {
    float ex0 = min_x - half_m;
    float ey0 = min_y - half_m;
    float ex1 = max_x + half_m;
    float ey1 = max_y + half_m;

    /* 在扩张矩形内（含边界）：侵入深度 = min(四边穿透)，取各障碍中最深者 */
    if (p.x >= ex0 && p.x <= ex1 && p.y >= ey0 && p.y <= ey1) {
        float depth = p.x - ex0;
        float margin = ex1 - p.x;
        if (margin < depth) {
            depth = margin;
        }
        margin = p.y - ey0;
        if (margin < depth) {
            depth = margin;
        }
        margin = ey1 - p.y;
        if (margin < depth) {
            depth = margin;
        }
        if (depth > *inside_depth_m) {
            *inside_depth_m = depth;
        }
        return;
    }

    /* 在扩张矩形外：欧氏距离平方取最小 */
    {
        float d2 = map_rect_distance_sq(p, ex0, ey0, ex1, ey1);
        if (d2 < *outside_sq) {
            *outside_sq = d2;
        }
    }
}

/**
 * @brief 累加查询点到所有动态障碍格的最近距离平方
 * @details 正常情况下遍历 MapCore_RebuildClearance() 缓存的动态格列表；列表溢出时
 *          退化为全图扫描位图，保证不会漏掉动态障碍（性能较差但正确）。
 * @param p       查询点（地图坐标，米）
 * @param best_sq 输入/输出：当前最近距离平方 (m²)，发现更近的障碍时更新
 */
static void map_core_accumulate_dynamic_sq(Point2D p, float *best_sq) {
    if (g_map.grid == NULL || g_map.grid_size <= 0) {
        return;
    }

    if (!s_dynamic_overflow) {
        for (int i = 0; i < s_dynamic_cell_count; i++) {
            int idx = (int)s_dynamic_cells[i];
            int gx = idx / g_map.grid_size;
            int gy = idx % g_map.grid_size;
            float x0 = (float)gx * g_map.res;
            float y0 = (float)gy * g_map.res;
            float d2 = map_rect_distance_sq(p, x0, y0, x0 + g_map.res, y0 + g_map.res);
            if (d2 < *best_sq) {
                *best_sq = d2;
            }
        }
        return;
    }

    /* 缓存溢出：全图扫描动态障碍格（不能漏检） */
    for (int gx = 0; gx < g_map.grid_size; gx++) {
        for (int gy = 0; gy < g_map.grid_size; gy++) {
            if (get_grid_val(&g_map, gx, gy) != MAP_CELL_DYNAMIC_OBSTACLE) {
                continue;
            }
            float x0 = (float)gx * g_map.res;
            float y0 = (float)gy * g_map.res;
            float d2 = map_rect_distance_sq(p, x0, y0, x0 + g_map.res, y0 + g_map.res);
            if (d2 < *best_sq) {
                *best_sq = d2;
            }
        }
    }
}

/**
 * @brief 累加查询点到所有动态障碍格（按 10cm 方格）的 Minkowski 扩张带符号距离
 * @details 与 map_core_accumulate_dynamic_sq 相同的列表遍历/溢出退化策略，区别是每个
 *          动态格按"障碍矩形"处理，先按车体半长 half_m 扩张，再按带符号距离累计
 *          （扩张矩形内为侵入深度，外部为欧氏距离）。
 * @param p              查询点（地图坐标，米）
 * @param half_m         车体半长（Minkowski 扩张量，米）
 * @param inside_depth_m 输入/输出：已有最大侵入深度 (m)
 * @param outside_sq     输入/输出：已有最小外部距离平方 (m²)
 */
static void map_core_accumulate_dynamic_signed(Point2D p, float half_m, float *inside_depth_m,
                                               float *outside_sq) {
    if (g_map.grid == NULL || g_map.grid_size <= 0) {
        return;
    }

    if (!s_dynamic_overflow) {
        for (int i = 0; i < s_dynamic_cell_count; i++) {
            int idx = (int)s_dynamic_cells[i];
            int gx = idx / g_map.grid_size;
            int gy = idx % g_map.grid_size;
            float x0 = (float)gx * g_map.res;
            float y0 = (float)gy * g_map.res;
            map_core_accumulate_rect_signed(p, x0, y0, x0 + g_map.res, y0 + g_map.res, half_m,
                                            inside_depth_m, outside_sq);
        }
        return;
    }

    /* 缓存溢出：全图扫描动态障碍格（不能漏检） */
    for (int gx = 0; gx < g_map.grid_size; gx++) {
        for (int gy = 0; gy < g_map.grid_size; gy++) {
            if (get_grid_val(&g_map, gx, gy) != MAP_CELL_DYNAMIC_OBSTACLE) {
                continue;
            }
            float x0 = (float)gx * g_map.res;
            float y0 = (float)gy * g_map.res;
            map_core_accumulate_rect_signed(p, x0, y0, x0 + g_map.res, y0 + g_map.res, half_m,
                                            inside_depth_m, outside_sq);
        }
    }
}

/**
 * @brief 由预定义障碍物顶点数组构造静态障碍矩形（世界坐标 → 地图坐标）
 * @details 每 4 个点构成一个轴对齐矩形：先对每个点做世界 → 地图平移（+MAP_OFFSET_X/Y），
 *          再取该组 4 点的 min/max。与旧版 Planner_Init 的分组/取 min-max 逻辑一致。
 * @param world_quads 顶点数组（世界坐标，米），每 4 个点一组
 * @param point_count 顶点个数
 */
static void map_core_load_static_rects(const Point2D *world_quads, int point_count) {
    s_static_rect_count = 0;

    for (int i = 0; i + 3 < point_count; i += 4) {
        if (s_static_rect_count >= PLANNER_MAX_STATIC_RECTS) {
            break;
        }

        float min_x = world_quads[i].x + MAP_OFFSET_X;
        float max_x = min_x;
        float min_y = world_quads[i].y + MAP_OFFSET_Y;
        float max_y = min_y;

        for (int k = 1; k < 4; k++) {
            float x = world_quads[i + k].x + MAP_OFFSET_X;
            float y = world_quads[i + k].y + MAP_OFFSET_Y;
            if (x < min_x) {
                min_x = x;
            }
            if (x > max_x) {
                max_x = x;
            }
            if (y < min_y) {
                min_y = y;
            }
            if (y > max_y) {
                max_y = y;
            }
        }

        MapRect *rect = &s_static_rects[s_static_rect_count];
        rect->min_x = min_x;
        rect->min_y = min_y;
        rect->max_x = max_x;
        rect->max_y = max_y;
        s_static_rect_count++;
    }
}

/**
 * @brief 把静态障碍矩形保守地写入栅格位图（只用于显示/调试）
 * @details 格 gx 的覆盖区间 [gx*res, (gx+1)*res) 与矩形 [min_x, max_x] 有交集即标记为
 *          MAP_CELL_STATIC_OBSTACLE：gx0 = floor(min_x/res)，gx1 = ceil(max_x/res) - 1，
 *          再 clamp 到地图范围。MAP_CORE_GRID_TOL 用于吸收浮点误差，避免正好落在格线上的
 *          边界因表示误差多占一格。
 */
static void map_core_mark_static_cells(void) {
    if (g_map.grid == NULL || g_map.grid_size <= 0) {
        return;
    }

    int last = g_map.grid_size - 1;

    for (int i = 0; i < s_static_rect_count; i++) {
        const MapRect *rect = &s_static_rects[i];

        /* gx0 = floor(min/res)，gx1 = ceil(max/res) - 1；max 侧减容差防止上边界多占一格 */
        int gx0 = (int)floorf((rect->min_x + MAP_CORE_GRID_TOL) / g_map.res);
        int gx1 = (int)ceilf((rect->max_x - MAP_CORE_GRID_TOL) / g_map.res) - 1;
        int gy0 = (int)floorf((rect->min_y + MAP_CORE_GRID_TOL) / g_map.res);
        int gy1 = (int)ceilf((rect->max_y - MAP_CORE_GRID_TOL) / g_map.res) - 1;

        /* clamp 到地图范围；完全在地图外的矩形跳过 */
        if (gx0 < 0) {
            gx0 = 0;
        }
        if (gy0 < 0) {
            gy0 = 0;
        }
        if (gx1 > last) {
            gx1 = last;
        }
        if (gy1 > last) {
            gy1 = last;
        }
        if (gx0 > gx1 || gy0 > gy1) {
            continue;
        }

        for (int gx = gx0; gx <= gx1; gx++) {
            for (int gy = gy0; gy <= gy1; gy++) {
                set_grid_val(&g_map, gx, gy, MAP_CELL_STATIC_OBSTACLE);
            }
        }
    }
}

/* ==========================================================================
 * 6. 生命周期与统计
 * ========================================================================== */

/**
 * @brief 初始化地图：清空栅格、按启动区载入预定义障碍物、重建两层清障距离场
 * @param zone             启动区（MAP_ZONE_1 / MAP_ZONE_2，取值 1 / 2；其它值按启动区 1 处理）
 * @param grid             外部提供的栅格位图缓冲区，长度 MAX_MAP_BYTES
 * @param clearance        外部提供的欧氏清障距离场缓冲区，长度 MAX_CELLS
 * @param square_clearance 外部提供的方形带符号清障场缓冲区，长度 MAX_CELLS
 * @return true 初始化成功；false 表示参数非法（指针为空 / res 或 grid_size 非法）
 */
bool MapCore_Init(uint8_t zone, uint8_t *grid, uint8_t *clearance, int8_t *square_clearance) {
    if (grid == NULL || clearance == NULL || square_clearance == NULL) {
        return false;
    }

    /* 设置并校验地图对象字段 */
    g_map.grid = grid;
    g_map.clearance = clearance;
    g_map.square_clearance = square_clearance;
    g_map.res = PLANNER_MAP_RES;
    g_map.grid_size = MAX_GRID_SIZE;
    if (!(g_map.res > 0.0f) || (g_map.grid_size <= 0) || (g_map.grid_size > MAX_GRID_SIZE)) {
        g_map.grid = NULL;
        g_map.clearance = NULL;
        g_map.square_clearance = NULL;
        return false;
    }

    /* 清空位图（动态/静态障碍全部复位） */
    memset(grid, 0, MAX_MAP_BYTES);
    s_dynamic_cell_count = 0;
    s_dynamic_overflow = false;

    /* 按启动区载入预定义障碍物（世界坐标 → 地图坐标 → 轴对齐矩形） */
    if (zone == MAP_CORE_ZONE_2) {
        map_core_load_static_rects(s_pre_obstacles_zone2, (int)PRE_OBSTACLE_NUM_ZONE2);
    } else {
        map_core_load_static_rects(s_pre_obstacles_zone1, (int)PRE_OBSTACLE_NUM_ZONE1);
    }

    /* 位图只为显示/调试服务：保守标记静态障碍格 */
    map_core_mark_static_cells();

    /* 重建两层清障距离场（同时重建动态障碍格列表；刚初始化时动态障碍为空） */
    MapCore_RebuildClearance();
    return true;
}

/**
 * @brief 重建两层清障距离场（动态障碍变更后调用，规划前也会自动重建）
 * @details 分两步：
 *          1) 全图扫描位图，重建动态障碍格缓存列表（容量不足时置溢出标志，
 *             后续精确查询退化为全图扫描，保证不漏检）；
 *          2) 逐格计算两层距离场：
 *             - 欧氏场 g_map.clearance[]：位图非空的格直接记 0（保守，与位图的显示语义
 *               一致，格中心不允许作为可通行点），空闲格取 MapCore_PointClearanceCm(格中心)；
 *               向下取整后饱和到 PLANNER_CLEARANCE_CAP_CM；
 *             - 方形场 g_map.square_clearance[]：位图非空的格直接记
 *               MAP_CORE_SQUARE_FIELD_MIN（足够负，任何朝向都碰撞，A* 不会当作可通行），
 *               空闲格取 MapCore_PointSquareClearanceCm(格中心, PLANNER_BODY_HALF_CM)，
 *               向下取整（负值向更负取整，更保守）后饱和到 [MIN, MAX]。
 */
void MapCore_RebuildClearance(void) {
    if (g_map.grid == NULL || g_map.grid_size <= 0 || !(g_map.res > 0.0f)) {
        return;
    }
    if (g_map.clearance == NULL && g_map.square_clearance == NULL) {
        return;
    }

    /* 1) 重建动态障碍格列表 */
    s_dynamic_cell_count = 0;
    s_dynamic_overflow = false;
    for (int gx = 0; gx < g_map.grid_size; gx++) {
        for (int gy = 0; gy < g_map.grid_size; gy++) {
            if (get_grid_val(&g_map, gx, gy) != MAP_CELL_DYNAMIC_OBSTACLE) {
                continue;
            }
            if (s_dynamic_cell_count < MAP_CORE_DYNAMIC_CELL_CAP) {
                s_dynamic_cells[s_dynamic_cell_count] = (uint16_t)(gx * g_map.grid_size + gy);
                s_dynamic_cell_count++;
            } else {
                s_dynamic_overflow = true;
            }
        }
    }

    /* 2) 逐格计算两层距离场 (cm) */
    float half_res = g_map.res * 0.5f;
    for (int gx = 0; gx < g_map.grid_size; gx++) {
        for (int gy = 0; gy < g_map.grid_size; gy++) {
            int idx = gx * g_map.grid_size + gy;
            uint8_t cell = get_grid_val(&g_map, gx, gy);

            Point2D center;
            center.x = (float)gx * g_map.res + half_res;
            center.y = (float)gy * g_map.res + half_res;

            /* 欧氏场：障碍格记 0（与位图的显示语义一致） */
            if (g_map.clearance != NULL) {
                if (cell != MAP_CELL_FREE) {
                    g_map.clearance[idx] = 0;
                } else {
                    int clearance_cm =
                        (int)floorf(MapCore_PointClearanceCm(center) + MAP_CORE_CLEARANCE_ROUND_TOL_CM);
                    if (clearance_cm < 0) {
                        clearance_cm = 0;
                    }
                    if (clearance_cm > PLANNER_CLEARANCE_CAP_CM) {
                        clearance_cm = PLANNER_CLEARANCE_CAP_CM; /* uint8 存储饱和 */
                    }
                    g_map.clearance[idx] = (uint8_t)clearance_cm;
                }
            }

            /* 方形场：障碍格存足够负的值；空闲格存带符号净空的向下取整 */
            if (g_map.square_clearance != NULL) {
                if (cell != MAP_CELL_FREE) {
                    g_map.square_clearance[idx] = (int8_t)MAP_CORE_SQUARE_FIELD_MIN;
                } else {
                    float square_cm = MapCore_PointSquareClearanceCm(center, PLANNER_BODY_HALF_CM);
                    int square_val = (int)floorf(square_cm + MAP_CORE_CLEARANCE_ROUND_TOL_CM);
                    if (square_val < MAP_CORE_SQUARE_FIELD_MIN) {
                        square_val = MAP_CORE_SQUARE_FIELD_MIN; /* int8 下限饱和 */
                    }
                    if (square_val > MAP_CORE_SQUARE_FIELD_MAX) {
                        square_val = MAP_CORE_SQUARE_FIELD_MAX; /* int8 上限饱和 */
                    }
                    g_map.square_clearance[idx] = (int8_t)square_val;
                }
            }
        }
    }
}

/**
 * @brief 统计当前障碍格数量（静态 + 动态）
 * @return 位图值 != 0 的格数
 */
uint16_t MapCore_ObstacleCells(void) {
    if (g_map.grid == NULL || g_map.grid_size <= 0) {
        return 0;
    }

    uint16_t count = 0;
    for (int gx = 0; gx < g_map.grid_size; gx++) {
        for (int gy = 0; gy < g_map.grid_size; gy++) {
            if (get_grid_val(&g_map, gx, gy) != MAP_CELL_FREE) {
                count++;
            }
        }
    }
    return count;
}

/**
 * @brief 当前启动区加载的静态障碍矩形数量（调试用）
 * @return 矩形数量
 */
uint8_t MapCore_StaticRectCount(void) {
    return s_static_rect_count;
}

/* ==========================================================================
 * 7. 欧氏清障查询（旋转安全 / 诊断用；单位 cm，地图坐标 m）
 * ========================================================================== */

/**
 * @brief 查询格中心清障距离（缓存值，cm，0 表示格中心在障碍内）
 * @param gx 格 x 索引
 * @param gy 格 y 索引
 * @return 缓存的清障距离 (cm)；越界或地图未初始化返回 0
 */
uint8_t MapCore_CellClearanceCm(int gx, int gy) {
    if (g_map.clearance == NULL || g_map.grid_size <= 0) {
        return 0;
    }
    if (gx < 0 || gx >= g_map.grid_size || gy < 0 || gy >= g_map.grid_size) {
        return 0;
    }
    return g_map.clearance[gx * g_map.grid_size + gy];
}

/**
 * @brief 精确查询任意点清障距离 (cm)：到最近静态矩形/动态障碍格/地图边界的距离
 * @details 三类距离取最小：
 *          a) 到所有静态障碍矩形的欧氏距离（点在矩形内为 0）；
 *          b) 到所有动态障碍格（10cm 方格）的欧氏距离；
 *          c) 到地图四边界的距离（地图外未知，按障碍处理，保守）。
 *          实现上先取三类距离平方的最小值，最后统一开方；数学结果与"逐个开方取最小"
 *          一致，但整张距离场重建时只需 ~MAX_CELLS 次 sqrtf。
 * @param p 查询点（地图坐标，米）
 * @return 清障距离 (cm)，0 表示压在障碍上或超出地图范围；不饱和到 CAP（调用方自行比较）
 */
float MapCore_PointClearanceCm(Point2D p) {
    if (g_map.grid_size <= 0 || !(g_map.res > 0.0f)) {
        return 0.0f;
    }

    float width = (float)g_map.grid_size * g_map.res;
    float height = (float)g_map.grid_size * g_map.res;

    /* 超出地图：地图外区域未知，按障碍处理 */
    if (p.x < 0.0f || p.y < 0.0f || p.x >= width || p.y >= height) {
        return 0.0f;
    }

    /* a) 静态障碍矩形（精确几何） */
    float best_sq = MAP_CORE_DIST_INF;
    for (int i = 0; i < s_static_rect_count; i++) {
        const MapRect *rect = &s_static_rects[i];
        float d2 = map_rect_distance_sq(p, rect->min_x, rect->min_y, rect->max_x, rect->max_y);
        if (d2 < best_sq) {
            best_sq = d2;
        }
    }

    /* b) 动态障碍格（精确几何，10cm 方格） */
    map_core_accumulate_dynamic_sq(p, &best_sq);

    /* c) 地图边界：min(p.x, p.y, W-p.x, H-p.y) */
    float dx_edge = (p.x < width - p.x) ? p.x : (width - p.x);
    float dy_edge = (p.y < height - p.y) ? p.y : (height - p.y);
    float boundary = (dx_edge < dy_edge) ? dx_edge : dy_edge;
    if (boundary * boundary < best_sq) {
        best_sq = boundary * boundary;
    }

    return sqrtf(best_sq) * MAP_CORE_CM_PER_M;
}

/**
 * @brief 精确查询线段的最小清障距离 (cm)：按 PLANNER_PATH_SAMPLE_STEP_CM 采样
 * @details steps = ceil(线段长度 / PLANNER_PATH_SAMPLE_STEP_CM)，至少 1 步；采样包含
 *          首末点（t = k/steps，k = 0..steps），因此起终点一定被检查到。
 * @param a 起点（地图坐标，米）
 * @param b 终点（地图坐标，米）
 * @return 线段上采样点的最小清障距离 (cm)；任一采样点在地图外时结果为 0
 */
float MapCore_SegmentMinClearanceCm(Point2D a, Point2D b) {
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    float len_cm = sqrtf(dx * dx + dy * dy) * MAP_CORE_CM_PER_M;

    int steps = 1;
    if (len_cm > PLANNER_PATH_SAMPLE_STEP_CM) {
        float steps_f = ceilf(len_cm / PLANNER_PATH_SAMPLE_STEP_CM);
        if (steps_f < (float)MAP_CORE_SEGMENT_MAX_STEPS) {
            steps = (int)steps_f;
        } else {
            steps = MAP_CORE_SEGMENT_MAX_STEPS; /* 防御非法超长输入 */
        }
    }

    float min_cm = MAP_CORE_DIST_INF;
    for (int k = 0; k <= steps; k++) {
        float t = (float)k / (float)steps;
        Point2D p;
        p.x = a.x + dx * t;
        p.y = a.y + dy * t;

        float clearance_cm = MapCore_PointClearanceCm(p);
        if (clearance_cm < min_cm) {
            min_cm = clearance_cm;
        }
    }
    return min_cm;
}

/**
 * @brief 点是否满足最小清障要求
 * @details 清障 >= required_cm - 1e-3f 即视为通过（1e-3cm 用于吸收浮点误差）。
 * @param p           地图坐标（米）
 * @param required_cm 要求的最小清障距离 (cm)
 * @return true 满足要求
 */
bool MapCore_PointFree(Point2D p, float required_cm) {
    return MapCore_PointClearanceCm(p) >= (required_cm - MAP_CORE_FREE_TOL_CM);
}

/**
 * @brief 线段是否满足最小清障要求（起点/终点/整段采样）
 * @details 先显式检查起点与终点（语义清晰），再按 PLANNER_PATH_SAMPLE_STEP_CM 采样整段
 *          取最小清障；全部 >= required_cm - 1e-3f 才视为通过。
 * @param a           起点（地图坐标，米）
 * @param b           终点（地图坐标，米）
 * @param required_cm 要求的最小清障距离 (cm)
 * @return true 整段满足要求
 */
bool MapCore_SegmentFree(Point2D a, Point2D b, float required_cm) {
    float need_cm = required_cm - MAP_CORE_FREE_TOL_CM;

    if (MapCore_PointClearanceCm(a) < need_cm) {
        return false;
    }
    if (MapCore_PointClearanceCm(b) < need_cm) {
        return false;
    }
    return MapCore_SegmentMinClearanceCm(a, b) >= need_cm;
}

/* ==========================================================================
 * 8. 方形车体几何查询（Minkowski 扩张，带符号净空；单位 cm，地图坐标 m）
 *
 *  方形车体（半长 half_cm）与轴对齐障碍矩形 [x0,x1]×[y0,y1] 的碰撞等价于：
 *  车体中心进入"障碍矩形按 half 在 x/y 各向外扩"的中心禁区
 *  [x0-half, x1+half] × [y0-half, y1+half]（Minkowski 扩张，轴对齐近似）。
 *  带符号净空：在扩张矩形内为 -min(四边穿透)（负 = 侵入深度，取各障碍最深者），
 *  在外为到扩张矩形的欧氏距离（正）。动态障碍格按 10cm 方格同样处理；
 *  地图边界要求方形完整在地图内，带符号值 = min(x, y, W-x, H-y) - half。
 * ========================================================================== */

/**
 * @brief 查询方形车体中心的带符号净空 (cm)
 * @details 候选 = min(所有静态矩形扩张、所有动态障碍格扩张、地图边界扩张)：
 *          - 若查询点落在某个扩张矩形内：候选值 = -min(到扩张矩形四边的距离)，
 *            即负的侵入深度；多个障碍同时侵入时取最深（最负）值；
 *          - 若落在所有扩张矩形外：候选值 = 到最近扩张矩形的欧氏距离（正）；
 *          - 地图边界：min(p.x, p.y, W-p.x, H-p.y) - half（cm，地图外为负）。
 *          侵入（负）与外部（正）两类候选分开累计，比较在米域/平方域完成，
 *          最后统一换算 cm，避免先开方再取负带来的精度与语义问题。
 * @param p       中心点（地图坐标，米）
 * @param half_cm 方形半长（轴对齐朝向为 15cm；任意朝向按投影半长 H(psi) 传入）
 * @return >0 到中心禁区的余量 (cm)；0 相切；<0 已侵入（负值为侵入深度）
 */
float MapCore_PointSquareClearanceCm(Point2D p, float half_cm) {
    if (g_map.grid_size <= 0 || !(g_map.res > 0.0f)) {
        return 0.0f;
    }

    float half_m = half_cm / MAP_CORE_CM_PER_M;

    /* 侵入深度初值 < 0 表示"尚未侵入任何扩张矩形" */
    float inside_depth_m = -1.0f;
    float outside_sq = MAP_CORE_DIST_INF;

    /* a) 静态障碍矩形：按 Minkowski 扩张后求带符号距离 */
    for (int i = 0; i < s_static_rect_count; i++) {
        const MapRect *rect = &s_static_rects[i];
        map_core_accumulate_rect_signed(p, rect->min_x, rect->min_y, rect->max_x, rect->max_y, half_m,
                                        &inside_depth_m, &outside_sq);
    }

    /* b) 动态障碍格：按 10cm 方格同样扩张后求带符号距离 */
    map_core_accumulate_dynamic_signed(p, half_m, &inside_depth_m, &outside_sq);

    /* 侵入（负值）永远优于外部距离（正值），因此有侵入时不需要外部距离 */
    float value_cm;
    if (inside_depth_m >= 0.0f) {
        value_cm = -(inside_depth_m * MAP_CORE_CM_PER_M);
    } else {
        value_cm = sqrtf(outside_sq) * MAP_CORE_CM_PER_M;
    }

    /* c) 地图边界：方形必须完整在地图内 → min(x, y, W-x, H-y) - half（cm，地图外为负） */
    {
        float width = (float)g_map.grid_size * g_map.res;
        float height = (float)g_map.grid_size * g_map.res;
        float dx_edge = (p.x < width - p.x) ? p.x : (width - p.x);
        float dy_edge = (p.y < height - p.y) ? p.y : (height - p.y);
        float boundary_m = (dx_edge < dy_edge) ? dx_edge : dy_edge;
        float boundary_cm = boundary_m * MAP_CORE_CM_PER_M - half_cm;
        if (boundary_cm < value_cm) {
            value_cm = boundary_cm;
        }
    }

    return value_cm;
}

/**
 * @brief 线段上的最小方形净空 (cm)：按 PLANNER_PATH_SAMPLE_STEP_CM 采样（含首末点）
 * @details 采样步数与 MapCore_SegmentMinClearanceCm 一致：
 *          steps = ceil(长度 / PLANNER_PATH_SAMPLE_STEP_CM)，至少 1 步。
 * @param a       起点（地图坐标，米）
 * @param b       终点（地图坐标，米）
 * @param half_cm 方形半长 (cm)
 * @return 采样点上的最小带符号净空 (cm)；线段穿过障碍扩张区时为负
 */
float MapCore_SquareSegmentMinClearanceCm(Point2D a, Point2D b, float half_cm) {
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    float len_cm = sqrtf(dx * dx + dy * dy) * MAP_CORE_CM_PER_M;

    int steps = 1;
    if (len_cm > PLANNER_PATH_SAMPLE_STEP_CM) {
        float steps_f = ceilf(len_cm / PLANNER_PATH_SAMPLE_STEP_CM);
        if (steps_f < (float)MAP_CORE_SEGMENT_MAX_STEPS) {
            steps = (int)steps_f;
        } else {
            steps = MAP_CORE_SEGMENT_MAX_STEPS; /* 防御非法超长输入 */
        }
    }

    float min_cm = MAP_CORE_DIST_INF;
    for (int k = 0; k <= steps; k++) {
        float t = (float)k / (float)steps;
        Point2D p;
        p.x = a.x + dx * t;
        p.y = a.y + dy * t;

        float clearance_cm = MapCore_PointSquareClearanceCm(p, half_cm);
        if (clearance_cm < min_cm) {
            min_cm = clearance_cm;
        }
    }
    return min_cm;
}

/**
 * @brief 点是否满足方形净空要求
 * @details 净空 >= required_cm - 1e-3f 即视为通过（1e-3cm 用于吸收浮点误差）。
 *          required_cm = 0 时"恰好相切"（净空 0，例如 30cm 窄缝）视为可通行。
 * @param p           地图坐标（米）
 * @param half_cm     方形半长 (cm)
 * @param required_cm 要求的最小方形净空 (cm)
 * @return true 满足要求
 */
bool MapCore_SquareFree(Point2D p, float half_cm, float required_cm) {
    return MapCore_PointSquareClearanceCm(p, half_cm) >= (required_cm - MAP_CORE_FREE_TOL_CM);
}

/**
 * @brief 线段是否满足方形净空要求（先端点后整段采样）
 * @details 先显式检查起点与终点，再按 PLANNER_PATH_SAMPLE_STEP_CM 采样整段取最小净空；
 *          全部 >= required_cm - 1e-3f 才视为通过。
 * @param a           起点（地图坐标，米）
 * @param b           终点（地图坐标，米）
 * @param half_cm     方形半长 (cm)
 * @param required_cm 要求的最小方形净空 (cm)
 * @return true 整段满足要求
 */
bool MapCore_SquareSegmentFree(Point2D a, Point2D b, float half_cm, float required_cm) {
    float need_cm = required_cm - MAP_CORE_FREE_TOL_CM;

    if (MapCore_PointSquareClearanceCm(a, half_cm) < need_cm) {
        return false;
    }
    if (MapCore_PointSquareClearanceCm(b, half_cm) < need_cm) {
        return false;
    }
    return MapCore_SquareSegmentMinClearanceCm(a, b, half_cm) >= need_cm;
}

/**
 * @brief 查询格中心的方形净空基值 (cm，带符号)
 * @details 缓存值按 PLANNER_BODY_HALF_CM 计算并向下取整；任意朝向 psi 的净空 =
 *          返回值 - (H(psi) - PLANNER_BODY_HALF_CM)。障碍格缓存为
 *          MAP_CORE_SQUARE_FIELD_MIN（-100），任何朝向都判定为碰撞。
 * @param gx 格 x 索引
 * @param gy 格 y 索引
 * @return 缓存的带符号净空 (cm)；越界或地图未初始化返回 -100（保守）
 */
int16_t MapCore_CellSquareClearanceCm(int gx, int gy) {
    if (g_map.square_clearance == NULL || g_map.grid_size <= 0) {
        return (int16_t)MAP_CORE_SQUARE_FIELD_MIN;
    }
    if (gx < 0 || gx >= g_map.grid_size || gy < 0 || gy >= g_map.grid_size) {
        return (int16_t)MAP_CORE_SQUARE_FIELD_MIN;
    }
    return (int16_t)g_map.square_clearance[gx * g_map.grid_size + gy];
}

/* ==========================================================================
 * 9. 原地旋转安全性（扫掠半径 = PLANNER_ROTATION_SWEEP_RADIUS_CM）
 * ========================================================================== */

/**
 * @brief 该点位能否安全完成车体原地旋转
 * @details 原地旋转扫过所有朝向，扫掠半径 = PLANNER_ROTATION_SWEEP_RADIUS_CM
 *          （默认 30×30 半对角线 21.22cm；车体有凸出时按实测最大外缘半径配置），
 *          要求中心到最近障碍/地图边界的欧氏距离 >= PLANNER_ROTATION_CLEARANCE_CM
 *          （默认 23.22cm，含 2cm 余量）。与车体朝向无关，因此直接用欧氏距离场判断。
 * @param p 地图坐标（米）
 * @return true 可安全原地旋转
 */
bool MapCore_RotationSafe(Point2D p) {
    return MapCore_PointClearanceCm(p) >= (PLANNER_ROTATION_CLEARANCE_CM - MAP_CORE_FREE_TOL_CM);
}

/**
 * @brief 在整张地图中找离 near 最近的旋转安全格中心
 * @details 遍历全图格中心（48×48），跳过障碍格，取欧氏清障 >= PLANNER_ROTATION_CLEARANCE_CM
 *          且离 near 欧氏距离平方最小的格中心。只在需要绕行/旋转安全点时调用，
 *          ~几千次精确几何查询的开销可接受。
 * @param near 参考点（地图坐标，米）
 * @param out  输出：旋转安全点（格中心，地图坐标，米）
 * @return true 找到并写出 out；false 表示无安全格或参数非法
 */
bool MapCore_FindRotationSafePoint(Point2D near, Point2D *out) {
    if (out == NULL || g_map.grid == NULL || g_map.grid_size <= 0 || !(g_map.res > 0.0f)) {
        return false;
    }

    /* 与 MapCore_RotationSafe 保持同一容差，保证输出的点一定也被判为安全 */
    float need_cm = PLANNER_ROTATION_CLEARANCE_CM - MAP_CORE_FREE_TOL_CM;
    float half_res = g_map.res * 0.5f;
    float best_sq = MAP_CORE_DIST_INF;
    Point2D best = {0.0f, 0.0f};
    bool found = false;

    for (int gx = 0; gx < g_map.grid_size; gx++) {
        for (int gy = 0; gy < g_map.grid_size; gy++) {
            if (get_grid_val(&g_map, gx, gy) != MAP_CELL_FREE) {
                continue; /* 障碍格不能作为旋转点 */
            }

            Point2D center;
            center.x = (float)gx * g_map.res + half_res;
            center.y = (float)gy * g_map.res + half_res;
            if (MapCore_PointClearanceCm(center) < need_cm) {
                continue;
            }

            float dx = center.x - near.x;
            float dy = center.y - near.y;
            float d2 = dx * dx + dy * dy;
            if (!found || d2 < best_sq) {
                best_sq = d2;
                best = center;
                found = true;
            }
        }
    }

    if (found) {
        *out = best;
    }
    return found;
}

#endif /* PLANNER_ENABLED && PLANNER_PLATFORM == PLATFORM_MCU */
