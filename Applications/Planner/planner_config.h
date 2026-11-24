/**
 * @file planner_config.h
 * @brief 路径规划器配置：地图栅格 / 机器人几何与安全余量 / A* 代价 / 路径后处理 / 航点执行
 * @details
 *  本文件只放"可调参数与派生常量"，不放地图数据与算法实现：
 *   - 预定义障碍物数据：Cstar_map_core.c（按启动区区分，世界坐标）
 *   - 地图栅格、清障距离场、几何碰撞判定：Cstar_map_core.c
 *   - A* 搜索、路径直线化/重采样/安全校验：planner_algo.c
 *   - 航点下发、终点对齐、执行期安全监测：planner_port.c
 *
 *  安全模型（方形车体，30cm × 30cm）：
 *   1) 平移段车头保持固定朝向，方形车体与轴对齐障碍的碰撞用"Minkowski 扩张"精确判定：
 *      中心禁区 = 障碍矩形按车体半长 H 在 x/y 各向外扩；轴对齐朝向时 H = 15cm，
 *      任意朝向 psi 用投影上界 H(psi)=15*(|cos|+|sin|) ≤ 21.22cm（保守，只大不小）；
 *   2) 原地旋转会扫过所有朝向，需要中心到障碍欧氏距离 ≥ 旋转扫掠半径 + 2cm。
 *      默认扫掠半径 = 30×30 方形半对角线 21.22cm；若车体相对轮组/定位点不对称
 *      （某一侧凸出、含机械臂/外壳），按实测最大外缘距离配置
 *      PLANNER_ROTATION_SWEEP_RADIUS_CM。旋转中心本身由运动学
 *      chassis_config.h 的 CHASSIS_ROTATION_CENTER_OFFSET_X/Y 决定（均应等于
 *      导航/OPS 位姿点，否则定位点会在原地旋转时画圈）。
 *      30×30 方形转 45° 需要 42.43cm 宽，40cm/35cm 通道内禁止旋转（物理不可能）；
 *      因此执行策略是"旋转只在旋转安全点进行，平移段固定朝向，麦轮蟹行入位"。
 *   3) 起终点本身可能贴障碍（如 HOME 方形净空=0 恰好贴墙），按端点容差处理并标记 tight。
 *
 *  单位约定：几何量统一用 cm（与导航层一致），坐标转换只在 planner_port.c 做。
 */

#ifndef __PLANNER_CONFIG_H__
#define __PLANNER_CONFIG_H__

#include <stdint.h>
#include "planner_switch.h"

/* ===== 运行平台 ===== */
#define PLATFORM_MCU 0 /* 单片机 */
#define PLATFORM_RASPBERRY 1 /* 树莓派 */

#ifndef PLANNER_PLATFORM
#define PLANNER_PLATFORM PLATFORM_MCU
#endif

#if (PLANNER_PLATFORM != PLATFORM_MCU) && (PLANNER_PLATFORM != PLATFORM_RASPBERRY)
#error "PLANNER_PLATFORM 取值非法，只能是 PLATFORM_MCU 或 PLATFORM_RASPBERRY"
#endif

#if PLANNER_ENABLED && (PLANNER_PLATFORM == PLATFORM_MCU)

/* ==========================================================================
 * 1. 坐标与栅格
 * ========================================================================== */
#define PLANNER_MAP_RES_CM 10 /* 栅格分辨率 (cm)：1 格 = 10cm */
#define PLANNER_MAP_RES (PLANNER_MAP_RES_CM * 0.01f) /* 栅格分辨率 (m) */

/* 场地坐标 → 地图坐标平移：地图覆盖 [0, MAX_GRID_SIZE*res)，场地 [-2.4, 2.4)m */
#define MAP_OFFSET_X 2.4f
#define MAP_OFFSET_Y 2.4f

#define MAX_GRID_SIZE 48 /* 地图边长格数：48 * 10cm = 4.8m */
#define MAX_CELLS (MAX_GRID_SIZE * MAX_GRID_SIZE)

/* 栅格位图：2bit/格（0=空闲,1=静态障碍,2=动态障碍，见 Cstar_map_core.h） */
#define MAX_MAP_BYTES ((MAX_CELLS + 3) / 4)

/* 清障距离场存储上限 (cm)：uint8 存储，超过该值按上限处理。
 * 只用于"越远越好"的代价比较，取值不影响碰撞判定。 */
#define PLANNER_CLEARANCE_CAP_CM 100

/* 预定义障碍物最大矩形数（每个启动区 10 个，预留余量） */
#define PLANNER_MAX_STATIC_RECTS 32

/* ==========================================================================
 * 2. 方形车体几何与安全余量（单位 cm）
 * ==========================================================================
 *  车体 30×30cm 方形：半边长 15cm，半对角线 21.22cm。
 *  - 平移时车头固定朝向；方形与轴对齐障碍的碰撞用 Minkowski 扩张精确判定：
 *    车体半长 H → 中心禁区 = 障碍矩形在 x/y 各向外扩 H；
 *    轴对齐朝向（0/90/180/270°）H = 15cm 精确；任意朝向 psi 用保守投影上界
 *    H(psi) = 15*(|cos psi|+|sin psi|)（最大 21.22cm），只大不小，安全。
 *  - 原地旋转会扫过所有朝向，需要中心欧氏清障 >= 旋转扫掠半径 + 2cm；默认按
 *    半对角线 21.22 + 2 = 23.22cm。若车体相对轮组/定位点不对称（凸出、机械臂、
 *    外壳），按实测最大外缘距离改 PLANNER_ROTATION_SWEEP_RADIUS_CM。
 *    40cm/35cm 通道内不允许旋转（30×30 方形转 45° 需要 42.43cm 宽）。
 *  - 执行策略：旋转只在旋转安全点进行，平移段固定朝向，麦轮可蟹行入位。
 */
#define PLANNER_CAR_SIDE_CM 30.0f
#define PLANNER_BODY_HALF_CM (PLANNER_CAR_SIDE_CM * 0.5f) /* 15cm */
#define PLANNER_BODY_HALF_DIAGONAL_CM 21.22f /* 15*sqrt(2)，30×30 方形半对角线 */

/* 原地旋转扫掠半径 (cm)：以车体位姿点（= 旋转中心，见 chassis_config.h）为圆心，
 * 车体最外缘（含凸出部分/机械臂/外壳）到该点的最大距离。
 *   - 30×30 对称方形：等于半对角线 21.22cm（默认值，保持原有安全门限 23.22cm）；
 *   - 车体相对轮组/定位点不对称（某一侧凸出）：按实测最大外缘距离填写，
 *     例如 30(宽)×34(长)cm、旋转中心在几何中心 → sqrt(15²+17²)=22.67cm。
 * 注意：该半径只用于原地旋转的安全判定/执行期监测，与平移方形模型解耦，
 *       平移碰撞仍按 PLANNER_CAR_SIDE_CM 的方形计算。 */
#define PLANNER_ROTATION_SWEEP_RADIUS_CM PLANNER_BODY_HALF_DIAGONAL_CM

/* 方形净空（cm，带符号：>0 在"方形中心禁区"外仍有余量，0 相切，<0 已侵入=碰撞） */
/* 软目标：路径尽量保持方形净空 >= 4cm（40cm 通道居中时恰好 5cm） */
#define PLANNER_SAFE_MARGIN_CM 4.0f
/* 硬约束（采样门限）：一般路段方形净空 >= 2cm（1cm 采样 + 0.25cm 校验容差下，
 * 真实净空约 >=1.5cm，给实车轨迹跟踪误差留余量；35cm 通道居中 2.5cm 刚好可过）。
 * 实车反馈：门限 1cm 时路径规划最小净空约 1cm，跟踪误差 0.5~1cm 就会在 35cm 通道
 * 触发执行期方形净空监测急停，因此提高到 2cm。 */
#define PLANNER_SQUARE_HARD_MARGIN_CM 2.0f
/* 端点容差：起终点由任务给定（如 HOME 方形净空=0 恰好贴墙/贴角），允许比硬门限低 2cm；
 * 其中最多 1cm 可侵入模型禁区，用于吸收定位/摆放误差（仅端点豁免区生效，一般路段不允许）。 */
#define PLANNER_ENDPOINT_SQUARE_TOL_CM 2.0f
#define PLANNER_ENDPOINT_PENETRATION_TOL_CM 1.0f
/* 端点豁免区半径：采样点到起终点欧氏距离 <= 该值时允许端点容差 */
#define PLANNER_ENDPOINT_ZONE_CM 25.0f
/* 路径校验容差：1cm 采样间隔 + 栅格离散误差（一般路段采样净空允许比 tier 要求低 0.25cm；
 * 端点豁免区单独按端点容差处理） */
#define PLANNER_VALIDATION_TOL_CM 0.25f

/* ===== 路径连续化/净空优化 =====
 * A* 在 10cm 栅格上搜索，窄通道（如 35cm 通道、方形车净空仅 2.5cm）可能出现
 * "格中心都贴边/不可用"的量化问题；因此 A* 用 0 净空（允许相切）保证连通性，
 * 再对路径做连续化 + 逐点净空优化，最后按 tier 的 required 校验。
 * 优化：把 >10cm 的段细分到 ≤10cm，每个中间点在其 ±6cm 邻域内搜索使
 * "相邻两段方形净空最小值"最大的位置，迭代 3 次。 */
#define PLANNER_REFINE_SEG_CM 10.0f
#define PLANNER_REFINE_MAX_OFFSET_CM 6.0f
#define PLANNER_REFINE_STEP_CM 2.0f
#define PLANNER_REFINE_MAX_PASSES 3
/* 净空优化目标比 tier 要求再高一点，抵消采样/离散误差，保证真实净空 >= 0 */
#define PLANNER_REFINE_TARGET_EXTRA_CM 1.0f

/* ===== 原地旋转安全（扫掠半径见上）===== */
#define PLANNER_ROTATION_MARGIN_CM 2.0f
#define PLANNER_ROTATION_CLEARANCE_CM (PLANNER_ROTATION_SWEEP_RADIUS_CM + PLANNER_ROTATION_MARGIN_CM)
#define PLANNER_ROTATION_SCAN_STEP_CM 5.0f
#define PLANNER_ROTATION_VIA_MAX_CANDIDATES 12
#define PLANNER_HEADING_DEADBAND_DEG 1.5f
#define PLANNER_GOAL_YAW_ALIGN_IF_SAFE 1
#define PLANNER_ROTATION_TIMEOUT_BASE_MS 1500
#define PLANNER_ROTATION_TIMEOUT_PER_DEG_MS 25

/* 原地旋转速度：比平移角速度（220°/s）慢，减小旋转漂移/打滑；180° 约 2s。
 * 旋转阶段监测的是"扫掠半径 + 0.5cm"的包络，慢转才能让监测来得及兜住。 */
#define PLANNER_ROTATION_SPEED_DEG_S 90.0f
#define PLANNER_ROTATION_ACCEL_DEG_S2 180.0f
/* 旋转阶段欧氏清障监测余量：实际中心清障 < 旋转扫掠半径 + 该值 立即停车
 * （默认 21.22 + 0.5 = 21.72cm，比真实接触包络 21.22cm 提前 0.5cm 停车） */
#define PLANNER_ROTATION_MONITOR_EXTRA_CM 0.5f

/* ==========================================================================
 * 3. 执行期安全监测
 * ========================================================================== */
/* 轨迹偏离监测：航点跟踪过程中，实际位置到"当前航点段"直线的横向偏差超过该值
 * 视为跟踪失控，立即停车并重新规划（0 = 关闭）。 */
#define PLANNER_TRACK_DEVIATION_CM 10.0f

/* 执行期方形碰撞监测：按实际位姿/实际 yaw 计算方形净空（cm），低于该值视为即将
 * 碰撞，立即停车并重新规划（0 = 关闭）。 */
#define PLANNER_TRACK_MIN_SQUARE_CLEARANCE_CM 0.3f

/* 执行期安全监测周期 (ms)：查询定位并计算轨迹偏离/方形净空，越小响应越快、开销越大 */
#define PLANNER_EXEC_MONITOR_PERIOD_MS 10

/* 兼容旧名字：LCD 路径显示仍用 PHYSICAL_RADIUS（单位米）按半车宽加粗路径。 */
#define PHYSICAL_RADIUS (PLANNER_BODY_HALF_CM * 0.01f)

/* ==========================================================================
 * 4. A* 代价与搜索限制
 * ========================================================================== */
#define COST_SCALE 10 /* 浮点代价放大倍数（固定点整数搜索） */
#define STEP_STRAIGHT 10 /* 直行代价 (1.0 * 10) */
#define STEP_DIAG 14 /* 斜行代价 (1.414 * 10) */
#define MAX_COST_INF 65535 /* uint16 的无穷大 */

/* 贴障碍代价上限（×10）：方形净空从"软目标"降到"硬门限"时平方曲线加速惩罚；
 * 取值越大越不愿意钻窄缝/贴角。必须足够大：方形车在 30cm 缝（净空=0）里的
 * 通行代价要显著高于绕行，避免 A* 选一条"物理上过不去/零余量"的捷径。 */
#define PLANNER_CLEARANCE_PENALTY_MAX 200

#define MAX_PQ_SIZE 600 /* 优先队列最大容量：A* 实测 expanded_nodes 峰值 226，600 保留 >2 倍余量 */
#define MAX_PATH_LEN 200 /* 路径最大节点数 */

/* 分时共享内存池（当前只有 A* 优先队列使用；定义在 Cstar_map_core.c 的 BSS 段） */
typedef union {
    uint32_t pq_data[MAX_PQ_SIZE];
} PlannerMemoryPool;

extern PlannerMemoryPool g_mem_pool;

/* 反向搜索（终点→起点）终止容差：找到距起点格 <= 该值的格即收尾，
 * 再用精确起点替换第一个航点（起点可能贴障碍，不能要求起点格必须可通行）。 */
#define ASTAR_START_TOL_GRIDS 1

/* ==========================================================================
 * 5. A* 失败重试
 * ========================================================================== */
/* 规划失败后的重试次数（每次间隔 200ms）。定位数据在刷新，重试可能改变结果 */
#define PLANNER_ASTAR_RETRY_MAX 3

/* ==========================================================================
 * 6. 路径后处理
 * ========================================================================== */
/* 路径直线化/校验的采样步长 (cm)：1cm。2cm 采样会漏掉窄通道内的净空低谷
 * （实车 0.7cm 谷值被 2cm 采样漏检、执行期才暴露），A* 与校验统一用 1cm。 */
#define PLANNER_PATH_SAMPLE_STEP_CM 1.0f

/* 长直线段重采样上限 (cm)：0 = 不重采样（推荐，避免每个重采样点都产生一次停走）。
 * >0 时把长段均分成不超过该长度的子段，用于限制单段跟踪误差。 */
#define PLANNER_MAX_SEGMENT_CM 0.0f

/* 短距离直达阈值 (cm)：起点到终点小于该距离且直线可通行时，直接生成 2 点路径，
 * 不走 A*（用于标定微调、原地小位移等场景）。距离放大后必须走 A*，
 * 避免"直线刚好够硬约束"的偷懒路径绕过绕障搜索。 */
#define PLANNER_DIRECT_PATH_MAX_CM 20.0f

/* ==========================================================================
 * 7. 航点执行参数（导航层单位：cm、cm/s、deg、deg/s）
 * ========================================================================== */
/* 航点线速度：直线段巡航速度 */
#define PLANNER_WAYPOINT_SPEED_CM_S 120.0f
/* 终点减速速度：剩余路程进入减速区后，速度从巡航值线性降到该值 */
#define PLANNER_FINAL_SPEED_CM_S 80.0f
/* 终点减速区长度 (cm)：剩余路程小于该值时开始降速 */
#define PLANNER_SLOWDOWN_DIST_CM 15.0f
/* tight 路径（裸奔硬约束）限速：贴障碍走时降速通过 */
#define PLANNER_TIGHT_SPEED_CM_S 80.0f

/* 加速度/减速度/角速度/角加速度：梯形速度规划要求非 0 */
#define PLANNER_WAYPOINT_ACCEL_CM_S2 160.0f
#define PLANNER_WAYPOINT_DECEL_CM_S2 90.0f
#define PLANNER_WAYPOINT_ANGULAR_SPEED_DEG_S 220.0f
#define PLANNER_WAYPOINT_ANGULAR_ACCEL_DEG_S2 350.0f

/* 中间航点到达阈值 (cm)：中间点只要求位置到位，姿态保持出发时的朝向；
 * 该残余误差由下一段导航自然纠正，2cm 可显著减少停走时间。 */
#define PLANNER_WAYPOINT_ARRIVE_DIST_CM 2.0f

/* 单个航点超时：固定余量 + 按段长/实际速度估算耗时再放大。
 * 不能用固定值：长直线段（1m 以上）在低速度下必然超时。 */
#define PLANNER_WAYPOINT_TIMEOUT_BASE_MS 2000
#define PLANNER_WAYPOINT_TIMEOUT_SCALE 1.8f

/* ==========================================================================
 * 8. 终点精度（两段式：先走到位置，再原地对齐朝向）
 * ========================================================================== */
/* 是否启用终点两段式对齐：位置到位后原地旋转到目标 yaw。
 * 平移过程中保持出发朝向，避免"边走边转"导致麦轮打滑、终点位置超差。 */
#define PLANNER_FINAL_ALIGN_ENABLE 1

/* 位置阶段到达阈值 (cm)：先走到目标 xy，此时 yaw 无要求 */
#define PLANNER_FINAL_ARRIVE_DIST_CM 1.0f

/* 原地对齐的 yaw 阈值 (deg) 与超时 */
#define PLANNER_FINAL_ALIGN_YAW_DEG 2.0f
#define PLANNER_FINAL_ALIGN_TIMEOUT_MS 6000

/* 终点精度校验与补偿：两段式结束后测量位置/朝向误差，超差则补发一次微调目标 */
#define PLANNER_FINAL_VERIFY_ENABLE 1
#define PLANNER_FINAL_POS_TOL_CM 1.2f
#define PLANNER_FINAL_YAW_TOL_DEG 2.5f
#define PLANNER_FINAL_CORRECT_MAX 1
#define PLANNER_FINAL_CORRECT_TIMEOUT_MS 3000

/* ==========================================================================
 * 9. 执行失败重规划
 * ========================================================================== */
/* 航点导航失败/跟踪偏离/清障告警后的重规划次数：从当前实际位姿重新规划剩余路径 */
#define PLANNER_EXEC_RETRY_MAX 1

/* ==========================================================================
 * 10. 静态内存放置
 * ========================================================================== */
/* SRAM2 (RW_IRAM2: 0x2001C000, 16KB) 专用放置宏。
 * 用法: PLACE_IN_SRAM2 static uint8_t buf[N];
 * 配套: MDK-ARM/Hurri-Cargo_Code.sct 中 RW_IRAM2 已选择 .bss.map_sram2 段。
 * 注意: STM32F407 的 SRAM2 无 DMA 能力, 仅适合 CPU 独占数据。
 * PLANNER_HOST_TEST 用于 PC 端单元测试（无 ARM 编译器属性）。 */
#if defined(PLANNER_HOST_TEST)
#define PLACE_IN_SRAM2
#elif defined(__CC_ARM) /* AC5 (armcc) */
#define PLACE_IN_SRAM2 __attribute__((section(".bss.map_sram2"), zero_init))
#elif defined(__ARMCC_VERSION) /* AC6 (armclang) */
#define PLACE_IN_SRAM2 __attribute__((section(".bss.map_sram2")))
#else
#define PLACE_IN_SRAM2
#endif

/* ===== 基础几何类型（米制，供地图/算法层内部使用） ===== */
typedef struct {
    float x;
    float y;
} Point2D;

/* ==========================================================================
 * 11. 编译期参数自检
 * ========================================================================== */
#if (MAX_GRID_SIZE < 8) || (MAX_GRID_SIZE > 255)
#error "MAX_GRID_SIZE 必须在 [8, 255] 内（栅格索引用 int，地图不能为空）"
#endif

#if (PLANNER_MAP_RES_CM < 1) || (PLANNER_MAP_RES_CM > 50)
#error "PLANNER_MAP_RES_CM 必须在 [1, 50] 内"
#endif

#if (PLANNER_CLEARANCE_CAP_CM < 1) || (PLANNER_CLEARANCE_CAP_CM > 255)
#error "PLANNER_CLEARANCE_CAP_CM 必须在 [1, 255] 内（uint8 存储）"
#endif

#if (MAX_PATH_LEN < 8) || (MAX_PATH_LEN > 4096)
#error "MAX_PATH_LEN 必须在 [8, 4096] 内"
#endif

#endif /* PLANNER_ENABLED && PLANNER_PLATFORM == PLATFORM_MCU */

#endif /* __PLANNER_CONFIG_H__ */
