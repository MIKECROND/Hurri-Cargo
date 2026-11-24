/**
 * @file mission_route.c
 * @author T-Car Team
 * @brief 任务固定路线表实现（数据层）
 *
 * 本文件只放"路线数据 + 按启动区/轮次选路线"，不含任何流程控制；
 * mission.c 的流程代码只通过 Mission_Route*() 取路线后交给 mission_nav 执行。
 *
 * 字段含义见 mission_nav.h 的 MissionLeg_t：
 * 可调参数见 mission_config.h / Applications/mission/README.md。
 */

#include "mission_route.h"
#include "mission_config.h"
#include "nav_map.h"
#include "planner_switch.h"
#include "DIP_Switch.h"
#include <stddef.h>

/* 路线对象初始化：按数组元素个数自动填 count */
#define MISSION_ROUTE_INIT(legs_)                                                                                      \
    {                                                                                                                  \
        .legs = (legs_),                                                                                               \
        .count = (uint8_t)(sizeof(legs_) / sizeof((legs_)[0])),                                                        \
    }

/* ==========================================================================
 * 固定路线表 —— 命名与选择逻辑说明
 * ==========================================================================
 *
 * 【数据结构】
 *   MissionLeg_t   : 一段路线（Leg = 段），含目标点/临时点、导航模式、对齐、标定等字段
 *   MissionRoute_t : 一条完整路线（Route = 线），由 legs 数组 + count 组成
 *   关系：多条 Leg 组成一条 Route；legs 是"段数组"，route 是"整条线"。
 *
 * 【符号命名】
 *   s_legs_xxx    : 路线段数组（static，文件内私有）
 *   s_route_xxx   : 由对应 legs 数组构造出的路线对象
 *   命名后缀固定顺序：_z<区>_l<轮>
 *       _z1 / _z2   : 启动区 1 / 启动区 2（zone，对应 DIP_Switch_GetStartZone）
 *       _l0 / _l1   : 第 0 轮 / 第 1 轮（layer，对应 process_layer 参数，0-based）
 *                     注意：_l0 在注释叙述中称"第一轮/第一层"，命名用 0-based。
 *       每个阶段都有 _z1 / _z2 两张段表（两区各一份，二区默认按一区原样写）；
 *       只有空路线 s_route_none 两区共用。
 *
 * 【阶段语义】
 *   qr         : 扫码点
 *   grab       : 进入原料区抓取
 *   processing : 粗加工区
 *   storage    : 暂存区
 *   home       : 回家（两区分开写，见 s_legs_home_z2）
 *   none       : 空路线（该启动区/轮次无此阶段时使用，执行时直接返回成功）
 *
 * 【宏命名】
 *   MISSION_TARGET_xxx          : 目标点名（nav_map 点位；两区各用自己那张独立点位表）
 *   MISSION_POINT_xxx_OFFSET_Z1_X/Y、_Z2_X/Y : 各点机体系固定偏移(mm, X 前 / Y 左；
 *                 按启动区分两套，启动区2 默认值等于启动区1，见 mission_config.h)
 *   MISSION_CAL_DISK/RING_FIRST/RING_SECOND/STORAGE : 二次标定类型（圆盘/圆环第一次/圆环第二次/暂存区专用 0xE1）
 *   MISSION_NAV_MODE_AUTO/PLANNER/DIRECT/ROTATE/PASS : 导航方式（ROTATE=原地旋转，只用 yaw_deg；
 *                 PASS=不停车通过点，到触发距离即切下一段目标）
 *
 * 【对外 API】
 *   Mission_RouteXxx()          : 按阶段取路线，内部按启动区/轮次选具体 s_route_xxx
 * ========================================================================== */

/* 扫码点（启动区1） */
static const MissionLeg_t s_legs_qr_z1[] = {
    {.target_name = MISSION_TARGET_QR, .mode = MISSION_NAV_MODE_DIRECT},
};
/* 扫码点（启动区2） */
static const MissionLeg_t s_legs_qr_z2[] = {
    {.target_name = MISSION_TARGET_QR, .mode = MISSION_NAV_MODE_DIRECT},
};

/* 进入原料区（启动区1，第一轮） */
static const MissionLeg_t s_legs_grab_z1_l0[] = {
    {.x_cm = 10.0f, .y_cm = 40.0f, .yaw_deg = 0, .mode = MISSION_NAV_MODE_PASS},
    {.target_name = MISSION_TARGET_MATL_GRAB,
     .mode = MISSION_NAV_MODE_DIRECT,
     .cal_type = MISSION_CAL_DISK,
     .cal_offset_x_mm = MISSION_POINT_GRAB_OFFSET_Z1_X,
     .cal_offset_y_mm = MISSION_POINT_GRAB_OFFSET_Z1_Y},
};
/* 进入原料区（启动区2，第一轮） */
static const MissionLeg_t s_legs_grab_z2_l0[] = {
    {.x_cm = 10.0f, .y_cm = -170.0f, .yaw_deg = 0, .mode = MISSION_NAV_MODE_PASS},
    {.target_name = MISSION_TARGET_MATL_GRAB,
     .mode = MISSION_NAV_MODE_DIRECT,
     .cal_type = MISSION_CAL_DISK,
     .cal_offset_x_mm = MISSION_POINT_GRAB_OFFSET_Z2_X,
     .cal_offset_y_mm = MISSION_POINT_GRAB_OFFSET_Z2_Y},
};

/* 进入原料区（启动区1，第二轮） */
static const MissionLeg_t s_legs_grab_z1_l1[] = {
    {.yaw_deg = 0, .mode = MISSION_NAV_MODE_ROTATE},
    {.target_name = MISSION_TARGET_OA2, .mode = MISSION_NAV_MODE_PASS},
    {.target_name = MISSION_TARGET_MATL_GRAB,
     .mode = MISSION_NAV_MODE_DIRECT,
     .cal_type = MISSION_CAL_DISK,
     .cal_offset_x_mm = MISSION_POINT_GRAB_OFFSET_Z1_X,
     .cal_offset_y_mm = MISSION_POINT_GRAB_OFFSET_Z1_Y},
};
/* 进入原料区（启动区2，第二轮） */
static const MissionLeg_t s_legs_grab_z2_l1[] = {
    {.yaw_deg = 0, .mode = MISSION_NAV_MODE_ROTATE},
    {.target_name = MISSION_TARGET_OA2, .mode = MISSION_NAV_MODE_PASS},
    {.target_name = MISSION_TARGET_MATL_GRAB,
     .mode = MISSION_NAV_MODE_DIRECT,
     .cal_type = MISSION_CAL_DISK,
     .cal_offset_x_mm = MISSION_POINT_GRAB_OFFSET_Z2_X,
     .cal_offset_y_mm = MISSION_POINT_GRAB_OFFSET_Z2_Y},
};

/* 粗加工区（启动区1，第一二轮共用）*/
static const MissionLeg_t s_legs_processing_z1[] = {
    {.yaw_deg = 180, .mode = MISSION_NAV_MODE_ROTATE},
    {.target_name = MISSION_TARGET_MATL_PROCESSING,
     .mode = MISSION_NAV_MODE_DIRECT,
     .align = true,
     .cal_type = MISSION_CAL_RING_FIRST,
     .cal_offset_x_mm = MISSION_POINT_PROCESSING_OFFSET_Z1_X,
     .cal_offset_y_mm = MISSION_POINT_PROCESSING_OFFSET_Z1_Y},
};
/* 粗加工区（启动区2，第一二轮共用） */
static const MissionLeg_t s_legs_processing_z2[] = {
    {.yaw_deg = 180, .mode = MISSION_NAV_MODE_ROTATE},
    {.target_name = MISSION_TARGET_MATL_PROCESSING,
     .mode = MISSION_NAV_MODE_DIRECT,
     .align = true,
     .cal_type = MISSION_CAL_RING_FIRST,
     .cal_offset_x_mm = MISSION_POINT_PROCESSING_OFFSET_Z2_X,
     .cal_offset_y_mm = MISSION_POINT_PROCESSING_OFFSET_Z2_Y},
};

/* 暂存区（启动区1，第一二轮） */
static const MissionLeg_t s_legs_storage_z1_l0[] = {
    {.yaw_deg = 90, .mode = MISSION_NAV_MODE_ROTATE},
    {.target_name = MISSION_TARGET_OA1, .mode = MISSION_NAV_MODE_PASS},
    {.target_name = MISSION_TARGET_MATL_STORAGE,
     .mode = MISSION_NAV_MODE_DIRECT,
     .align = true,
     .cal_type = MISSION_CAL_STORAGE,
     .cal_offset_x_mm = MISSION_POINT_STORAGE_OFFSET_Z1_X,
     .cal_offset_y_mm = MISSION_POINT_STORAGE_OFFSET_Z1_Y},
};
static const MissionLeg_t s_legs_storage_z1_l1[] = {
    {.yaw_deg = 90, .mode = MISSION_NAV_MODE_ROTATE},
    {.target_name = MISSION_TARGET_OA1, .mode = MISSION_NAV_MODE_PASS},
    {.target_name = MISSION_TARGET_MATL_STORAGE,
     .mode = MISSION_NAV_MODE_DIRECT,
     .align = true,
     .cal_type = MISSION_CAL_STORAGE,
     .cal_offset_x_mm = MISSION_POINT_STORAGE_OFFSET_Z1_X,
     .cal_offset_y_mm = MISSION_POINT_STORAGE_OFFSET_Z1_Y},
};

/* 暂存区（启动区2，第一二轮） */
static const MissionLeg_t s_legs_storage_z2_l0[] = {
    {.yaw_deg = 90, .mode = MISSION_NAV_MODE_ROTATE},
    {.target_name = MISSION_TARGET_OA1, .mode = MISSION_NAV_MODE_PASS},
    {.target_name = MISSION_TARGET_MATL_STORAGE,
     .mode = MISSION_NAV_MODE_DIRECT,
     .align = true,
     .cal_type = MISSION_CAL_STORAGE,
     .cal_offset_x_mm = MISSION_POINT_STORAGE_OFFSET_Z2_X,
     .cal_offset_y_mm = MISSION_POINT_STORAGE_OFFSET_Z2_Y},
};
static const MissionLeg_t s_legs_storage_z2_l1[] = {
    {.yaw_deg = 90, .mode = MISSION_NAV_MODE_ROTATE},
    {.target_name = MISSION_TARGET_OA1, .mode = MISSION_NAV_MODE_PASS},
    {.target_name = MISSION_TARGET_MATL_STORAGE,
     .mode = MISSION_NAV_MODE_DIRECT,
     .align = true,
     .cal_type = MISSION_CAL_STORAGE,
     .cal_offset_x_mm = MISSION_POINT_STORAGE_OFFSET_Z2_X,
     .cal_offset_y_mm = MISSION_POINT_STORAGE_OFFSET_Z2_Y},
};

/* 回家路线（启动区1） */
static const MissionLeg_t s_legs_home_z1[] = {
    {.target_name = MISSION_TARGET_OA2, .mode = MISSION_NAV_MODE_PASS},
    {.target_name = MISSION_TARGET_MATL_GRAB, .mode = MISSION_NAV_MODE_PASS},
    {.target_name = MISSION_TARGET_HOME, .mode = MISSION_NAV_MODE_DIRECT},
};

/* 回家路线（启动区2） */
static const MissionLeg_t s_legs_home_z2[] = {
    {.target_name = MISSION_TARGET_OA1, .mode = MISSION_NAV_MODE_PASS},
    {.target_name = MISSION_TARGET_MATL_PROCESSING, .mode = MISSION_NAV_MODE_PASS},
    {.target_name = MISSION_TARGET_HOME, .mode = MISSION_NAV_MODE_DIRECT, .align = true},
};

/* 由路线段数组构造路线对象 */
static const MissionRoute_t s_route_qr_z1 = MISSION_ROUTE_INIT(s_legs_qr_z1);
static const MissionRoute_t s_route_qr_z2 = MISSION_ROUTE_INIT(s_legs_qr_z2);
static const MissionRoute_t s_route_grab_z1_l0 = MISSION_ROUTE_INIT(s_legs_grab_z1_l0);
static const MissionRoute_t s_route_grab_z2_l0 = MISSION_ROUTE_INIT(s_legs_grab_z2_l0);
static const MissionRoute_t s_route_grab_z1_l1 = MISSION_ROUTE_INIT(s_legs_grab_z1_l1);
static const MissionRoute_t s_route_grab_z2_l1 = MISSION_ROUTE_INIT(s_legs_grab_z2_l1);
static const MissionRoute_t s_route_processing_z1 = MISSION_ROUTE_INIT(s_legs_processing_z1);
static const MissionRoute_t s_route_processing_z2 = MISSION_ROUTE_INIT(s_legs_processing_z2);
static const MissionRoute_t s_route_storage_z1_l0 = MISSION_ROUTE_INIT(s_legs_storage_z1_l0);
static const MissionRoute_t s_route_storage_z1_l1 = MISSION_ROUTE_INIT(s_legs_storage_z1_l1);
static const MissionRoute_t s_route_storage_z2_l0 = MISSION_ROUTE_INIT(s_legs_storage_z2_l0);
static const MissionRoute_t s_route_storage_z2_l1 = MISSION_ROUTE_INIT(s_legs_storage_z2_l1);
static const MissionRoute_t s_route_home_z1 = MISSION_ROUTE_INIT(s_legs_home_z1);
static const MissionRoute_t s_route_home_z2 = MISSION_ROUTE_INIT(s_legs_home_z2);

/* 空路线：不同启动区/轮次没有该阶段时使用，执行时直接返回成功 */
static const MissionRoute_t s_route_none = {.legs = NULL, .count = 0};

/* ==========================================================================
 * 路线选择（按启动区 / 轮次）
 * ========================================================================== */

/**
 * @brief 当前是否从启动区2 出发
 */
static bool IsStartZone2(void) {
    return (DIP_Switch_GetStartZone() == START_ZONE_2);
}

/**
 * @brief 扫码点路线（两区各一套，二区默认按一区写）
 */
const MissionRoute_t *Mission_RouteQr(void) {
    return IsStartZone2() ? &s_route_qr_z2 : &s_route_qr_z1;
}

/**
 * @brief 选择"进入原料区"的路线（按启动区/轮次选择，两区各一套）
 * @param process_layer 轮次：0=第一轮, 1=第二轮
 */
const MissionRoute_t *Mission_RouteGrab(uint8_t process_layer) {
    if (process_layer == 1) {
        return IsStartZone2() ? &s_route_grab_z2_l1 : &s_route_grab_z1_l1;
    }
    return IsStartZone2() ? &s_route_grab_z2_l0 : &s_route_grab_z1_l0;
}

/**
 * @brief 选择"粗加工区"路线（两区各一套，二区入口临时点用二区独立坐标）
 */
const MissionRoute_t *Mission_RouteProcessing(void) {
    return IsStartZone2() ? &s_route_processing_z2 : &s_route_processing_z1;
}

/**
 * @brief 选择"暂存区"路线（两区各一套，二区入口临时点用二区独立坐标；第一二轮均用暂存区专用标定）
 * @param process_layer 粗加工区叠放层：0=第一轮, 1=第二轮
 */
const MissionRoute_t *Mission_RouteStorage(uint8_t process_layer) {
    if (process_layer == 1) {
        return IsStartZone2() ? &s_route_storage_z2_l1 : &s_route_storage_z1_l1;
    }
    return IsStartZone2() ? &s_route_storage_z2_l0 : &s_route_storage_z1_l0;
}

/**
 * @brief 选择"回家"路线（两区 via 点不同：各自沿自己那侧走廊回 Home，见 s_legs_home_z2）
 */
const MissionRoute_t *Mission_RouteHome(void) {
    return IsStartZone2() ? &s_route_home_z2 : &s_route_home_z1;
}
