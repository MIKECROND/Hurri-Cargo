/**
 * @file planner_port.h
 * @author T-Car Team
 * @brief 路径规划器移植层（对外接口）
 * @details
 *  对外提供：初始化、点名/坐标导航、空跑规划、中止请求、任务入口。
 *  内部实现：读定位 → 调算法层 Planner_PlanPath() → 航点分段下发导航 →
 *  终点两段式对齐（位置 + 朝向）→ 精度校验与补偿，全程支持中止与执行期安全监测。
 *
 *  与导航层的关系：Planner 通过 Nav_GoToDirect() 逐航点下发，每个航点都等待
 *  NAV_STATE_COMPLETE 后再发下一个；任务层只调用 Planner_GoTo_*，不感知航点细节。
 */

#ifndef __PLANNER_PORT_H__
#define __PLANNER_PORT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "planner_config.h"
#include "nav_core.h"
#include "nav_local.h"

#if PLANNER_ENABLED
#if PLANNER_PLATFORM == PLATFORM_MCU

extern bool is_planner_running;

/* 本次/最近一次规划的原始请求（场地坐标，米） */
extern Point2D start_w_extend;
extern Point2D goal_w_extend;
extern float goal_yaw;

/* 最近一次规划得到的路径（地图坐标，米），供调试显示使用。
 * g_plan_path_len == 0 表示当前没有有效路径。 */
extern Point2D g_plan_path[MAX_PATH_LEN];
extern int g_plan_path_len;

/**
 * @brief 最近一次规划/执行的报告，供日志与 shell 工具查看
 */
typedef struct {
    uint16_t waypoints; /* 航点数（含起点与精确终点） */
    uint16_t expanded_nodes; /* A* 展开节点数 */
    uint16_t rotation_index; /* 原地旋转发生在第几个航点（PLANNER_ROTATE_NONE=无） */
    float heading_before_deg; /* 旋转前平移朝向 */
    float heading_after_deg; /* 旋转后平移朝向（应等于目标 yaw） */
    float rotation_clearance_cm; /* 旋转点欧氏清障 */
    float length_cm; /* 路径总长 */
    float min_square_clearance_cm; /* 路径最小方形净空（带符号） */
    float min_clearance_cm; /* 路径最小欧氏清障（诊断） */
    uint8_t tier; /* 0 = 满足安全余量；1 = tight */
    uint8_t fail_reason; /* PlannerFailReason_t，0=成功 */
    uint8_t exec_retries; /* 本次执行触发的重规划次数 */
    float final_pos_err_cm; /* 终点位置误差（实测） */
    float final_yaw_err_deg; /* 终点朝向误差（实测） */
    uint32_t plan_ms; /* 规划耗时 */
    uint32_t exec_ms; /* 执行耗时 */
} PlannerReport_t;

extern PlannerReport_t g_planner_report;

bool Planner_Init(uint8_t zone);
#else /* PLANNER_PLATFORM == PLATFORM_RASPBERRY */

#endif /* PLANNER_PLATFORM */

bool Planner_GoToDirect(TargetPoint_t *target);
bool Planner_GoTo_fromName(const char *name);
bool Planner_PlanDry(TargetPoint_t *target); /* 空跑：只做规划与显示，不下发航点 */
void Planner_Stop(void); /* 请求中止当前规划（其它任务可调用） */
void Planner_Task(void *argument);

#endif /* PLANNER_ENABLED */

#ifdef __cplusplus
}
#endif

#endif /* __PLANNER_PORT_H__ */
