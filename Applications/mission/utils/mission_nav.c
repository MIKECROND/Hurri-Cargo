/**
 * @file mission_nav.c
 * @author T-Car Team
 * @brief 任务导航封装实现
 *
 * 本文件把"规划 / 直连 / 等待 / 重试 / 中止 / 二次标定 / 到位精修"收口到一处：
 *   - StartAndWait()：按指定方式启动一次导航并等待到位；
 *   - NavToTarget()：在 StartAndWait 外包一层有限重试与中止检查；
 *   - CalibrateAt()：到位后调用树莓派视觉二次标定，并用底盘位置模式走完修正量；
 *   - RunPassChain()：执行"不停车通过点（过弯）"链：到通过点触发距离即无缝换下一段目标；
 *   - Mission_NavGoToLeg()：执行任务路线中的一段（点名或临时点，可选二次标定/位置精修）；
 *   - Mission_NavGoToRoute()：按顺序执行整条固定路线。
 * 任务流程（mission.c）只描述阶段流程，路线数据由 mission_route.c 提供，
 * 不再出现裸 Nav_GoTo_* / osDelay 伪等待。
 * 可调参数见 mission_config.h / Applications/mission/README.md。
 */

#include "mission_nav.h"
#include "arm_action.h"
#include "chassis_config.h"
#include "cmsis_os2.h"
#include "Events.h"
#include "log.h"
#include "mission_config.h"
#include "mission_timing.h"
#include "motion_control.h"
#include "nav_core.h"
#include "nav_local.h"
#include "ops.h"
#include "planner_port.h"
#include "rpi_port.h"
#include <math.h>
#include <string.h>

/* 临时途经点的名字：仅用于导航日志（Nav_GoToDirect 会打印目标名） */
#define MISSION_VIA_POINT_NAME "MissionVia"
/* 原地旋转段的名字：仅用于导航日志 */
#define MISSION_ROTATE_POINT_NAME "MissionRotate"
/* 准备期间预旋转的名字：仅用于导航日志 */
#define MISSION_PRE_ROTATE_POINT_NAME "MissionPreRotate"

/* Nav_GoToDirect 只缓存目标点指针，导航期间目标点必须持续有效，故用静态存储 */
static TargetPoint_t s_via_target; // 路线表里的临时点
static TargetPoint_t s_pre_rotate_target; // 准备期间"车体原地预旋转"的目标点
static bool s_pre_rotate_active = false; // 是否已启动预旋转、尚未等待完成
static TargetPoint_t s_align_target; // 到位精修目标
/* 通过点（PASS）目标缓冲区：需要在 map 点基础上改写到达门限，避免在触发距离前被判到点停车。
 * 过点链里最多同时有两个目标有效（当前通过点 + 下一段），用 2 个缓冲轮换；
 * 导航层缓存的是最近一次下发的目标，换目标后旧缓冲即可复用。 */
static TargetPoint_t s_pass_target[2];
static uint8_t s_pass_target_idx = 0;
#if MISSION_CAL_ENABLE
static TargetPoint_t s_cal_target; // 树莓派标定输出的相对修正量(机体系: pose.x/y 位置 cm, pose.yaw 角度 deg)
#endif

/**
 * @brief 任务是否已被请求中止
 */
static bool IsAborted(void) {
    return (osEventFlagsGet(System_StatusHandle) & MISSION_ABORT) != 0;
}

#if MISSION_CAL_ENABLE
/**
 * @brief 任务级标定类型 -> 树莓派标定命令类型
 * @param type 任务级标定类型
 * @return RPI_Cal_t 树莓派标定类型
 */
static RPI_Cal_t ToRpiCal(MissionCal_t type) {
    switch (type) {
    case MISSION_CAL_DISK:
        return CAL_DISK;
    case MISSION_CAL_RING_FIRST:
        return CAL_RING_FIRST;
    case MISSION_CAL_RING_SECOND:
        return CAL_RING_SECOND;
    case MISSION_CAL_STORAGE:
        return CAL_STORAGE;
    case MISSION_CAL_NONE:
    default:
        return CAL_NONE;
    }
}

/**
 * @brief 标定目标点名 -> 机械臂预备位姿(按目标区域)
 * @param target_name 标定目标点名
 * @param pose 输出: 机械臂预备位姿(基座旋转/伸展/抬升)
 * @return true=已按区域填充位姿; false=参数为空/该点未配置预备位姿
 * @note  三个区域的预备位姿在 mission_config.h 的 MISSION_CAL_ARM_* 配置。
 */
static bool ToArmCalPose(const char *target_name, ArmCalPose_t *pose) {
    if (target_name == NULL || pose == NULL) {
        return false;
    }

    if (strcmp(target_name, MISSION_TARGET_MATL_GRAB) == 0) {
        pose->base_angle_deg = (int32_t)MISSION_CAL_ARM_GRAB_BASE_ANGLE_DEG;
        pose->extend_mm = MISSION_CAL_ARM_GRAB_EXTEND_MM;
        pose->lift_mm = MISSION_CAL_ARM_GRAB_LIFT_MM;
        return true;
    }
    if (strcmp(target_name, MISSION_TARGET_MATL_PROCESSING) == 0) {
        pose->base_angle_deg = (int32_t)MISSION_CAL_ARM_PROCESSING_BASE_ANGLE_DEG;
        pose->extend_mm = MISSION_CAL_ARM_PROCESSING_EXTEND_MM;
        pose->lift_mm = MISSION_CAL_ARM_PROCESSING_LIFT_MM;
        return true;
    }
    if (strcmp(target_name, MISSION_TARGET_MATL_STORAGE) == 0) {
        pose->base_angle_deg = (int32_t)MISSION_CAL_ARM_STORAGE_BASE_ANGLE_DEG;
        pose->extend_mm = MISSION_CAL_ARM_STORAGE_EXTEND_MM;
        pose->lift_mm = MISSION_CAL_ARM_STORAGE_LIFT_MM;
        return true;
    }

    logError("MissionNav: no arm calibration pose configured for '%s'", target_name);
    return false;
}
#endif /* MISSION_CAL_ENABLE */

/**
 * @brief 按指定方式启动一次导航并等待到位（单次，不含重试）
 * @param target 目标点（导航层只缓存指针，须持续有效）
 * @param mode   导航方式
 * @return true 到位
 */
static bool StartAndWait(TargetPoint_t *target, MissionNavMode_t mode) {
#if PLANNER_ENABLED == 1
    if (mode != MISSION_NAV_MODE_DIRECT && mode != MISSION_NAV_MODE_PASS) {
        /* 规划器接口是阻塞式的：内部完成"规划 -> 逐航点下发 -> 终点精度校验"后返回，
         * 返回 true 即已到位，无需再等导航事件。 */
        return Planner_GoToDirect(target);
    }
#endif /* PLANNER_ENABLED */

    (void)mode;
    osEventFlagsClear(System_StatusHandle, NAV_EVT_DONE | NAV_EVT_ERROR);
    if (!Nav_GoToDirect(target)) {
        return false;
    }
    return Wait_Arrive();
}

/**
 * @brief 导航到目标点（失败自动重试，支持任务中止）
 * @param target 目标点（须持续有效）
 * @param mode   导航方式
 * @return true 到位；false 重试后仍失败或任务被中止
 */
static bool NavToTarget(TargetPoint_t *target, MissionNavMode_t mode) {
    for (uint32_t attempt = 0; attempt <= (uint32_t)MISSION_NAV_RETRY_MAX; attempt++) {
        if (IsAborted()) {
            return false;
        }

        if (attempt > 0) {
            bool aborted;
            MissionTiming_Count(MISSION_TC_NAV_RETRY, 1);
            logWarning("MissionNav: retry %u/%u", (unsigned int)attempt, (unsigned int)MISSION_NAV_RETRY_MAX);
            MissionTiming_Begin(MISSION_TM_NAV_RETRY);
            osDelay(MISSION_NAV_RETRY_DELAY_MS);
            aborted = IsAborted();
            MissionTiming_End(MISSION_TM_NAV_RETRY);
            if (aborted) {
                return false;
            }
        }

        if (StartAndWait(target, mode)) {
            return true;
        }

        /* 清理失败/超时后的导航状态与运动参数，避免影响下一次导航 */
        Nav_Stop();
    }
    return false;
}

/**
 * @brief 由路线表的临时点构造目标点（含车体参考点偏移换算）
 * @param leg 路线表中的一段
 * @note  与 nav_map_data.c 的任务点使用同一坐标约定（声明坐标 = OPS/声明坐标 P 系）：
 *        按 CHASSIS_BODY_REF_OFFSET 换算成车体参考点位姿，保证与点名点位同一物理落点约定。
 *        注意 yaw 也参与换算：同一声明 (x,y) 但 yaw 不同的两段，换算后的目标点会相差
 *        (R(yaw2)-R(yaw1))*(G-P)，即"同点异向"并不等于原地旋转；需要原地转体请用
 *        MISSION_NAV_MODE_ROTATE 段（见 FillRotateTarget）。
 *        历史注意：旧 mission.c 的 zone2 内联 avoid_point 是直接下发 Nav_GoToDirect 的
 *        导航位姿 G 坐标（未换算），迁移到路线表后物理落点会偏移 R(yaw)*offset
 *        （yaw=0 时约 +3.5cm/+5.0cm），实车标定时需确认（见 mission_config.h 注释）。
 */
static void FillViaTarget(const MissionLeg_t *leg) {
    s_via_target.id = 0xFF;
    strncpy(s_via_target.name, MISSION_VIA_POINT_NAME, sizeof(s_via_target.name) - 1);
    s_via_target.name[sizeof(s_via_target.name) - 1] = '\0';
    s_via_target.pose.x = leg->x_cm;
    s_via_target.pose.y = leg->y_cm;
    s_via_target.pose.yaw = leg->yaw_deg;

    pose_apply_body_ref_offset(&s_via_target.pose.x,
                               &s_via_target.pose.y,
                               s_via_target.pose.yaw,
                               CHASSIS_BODY_REF_OFFSET_X,
                               CHASSIS_BODY_REF_OFFSET_Y);

    s_via_target.motion.target_speed = MISSION_VIA_SPEED_CM_S;
    s_via_target.motion.target_angular_speed = MISSION_VIA_ANGULAR_SPEED_DEG_S;
    s_via_target.motion.acceleration = MISSION_VIA_ACCEL_CM_S2;
    s_via_target.motion.deceleration = MISSION_VIA_DECEL_CM_S2;
    s_via_target.motion.angular_acceleration = 0.0f;

    s_via_target.arrive.check_mode = ARRIVE_CHECK_BOTH;
    s_via_target.arrive.distance_threshold = MISSION_VIA_ARRIVE_DIST_CM;
    s_via_target.arrive.yaw_threshold = MISSION_VIA_ARRIVE_YAW_DEG;
    s_via_target.arrive.timeout_ms = MISSION_VIA_TIMEOUT_MS;

    /* 打印"声明坐标(P 系) -> 实际下发目标(G 系)"，现场排查看这一行就能发现是否被偏移带偏 */
    logInfo("MissionNav: via (%.1f, %.1f, %.0fdeg) -> target (%.2f, %.2f, %.0fdeg)",
            leg->x_cm,
            leg->y_cm,
            leg->yaw_deg,
            s_via_target.pose.x,
            s_via_target.pose.y,
            s_via_target.pose.yaw);
}

/**
 * @brief 构造"位置锁当前实测位姿、只改朝向"的原地旋转目标点
 * @param yaw_deg 目标朝向(°)
 * @param name    目标点名(仅日志用, 须非空)
 * @param out     输出目标点
 * @return true=构造成功; false=取不到当前位姿(此时不生成目标)
 * @note  位置直接取当前实测导航位姿、不做车体参考点换算, 所以无论 CHASSIS_BODY_REF_OFFSET
 *        是否为非零, 都只转体、不平移; 运动/到达参数沿用固定 via 点参数。
 */
static bool FillRotateTargetAt(float yaw_deg, const char *name, TargetPoint_t *out) {
    PoseTimestamp_t pose;

    if (!Loc_Get(&pose)) {
        logError("MissionNav: rotate in place but pose unavailable");
        return false;
    }

    out->id = 0xFF;
    strncpy(out->name, name, sizeof(out->name) - 1);
    out->name[sizeof(out->name) - 1] = '\0';
    out->pose.x = pose.pose.x;
    out->pose.y = pose.pose.y;
    out->pose.yaw = yaw_deg;

    out->motion.target_speed = MISSION_VIA_SPEED_CM_S;
    out->motion.target_angular_speed = MISSION_VIA_ANGULAR_SPEED_DEG_S;
    out->motion.acceleration = MISSION_VIA_ACCEL_CM_S2;
    out->motion.deceleration = MISSION_VIA_DECEL_CM_S2;
    out->motion.angular_acceleration = 0.0f;

    out->arrive.check_mode = ARRIVE_CHECK_BOTH;
    out->arrive.distance_threshold = MISSION_VIA_ARRIVE_DIST_CM;
    out->arrive.yaw_threshold = MISSION_VIA_ARRIVE_YAW_DEG;
    out->arrive.timeout_ms = MISSION_VIA_TIMEOUT_MS;
    return true;
}

/**
 * @brief 由路线表的"原地旋转"段构造目标点（位置锁当前导航位姿，只改朝向）
 * @param leg 路线表中的一段（只用 yaw_deg）
 * @return true 构造成功；false 取不到当前位姿
 * @note  位置直接取段开始时的实测导航位姿 G、不做车体参考点换算，所以旋转中心就是当前
 *        位姿点：无论 CHASSIS_BODY_REF_OFFSET 是否非零，都不会一边转一边整体平移。
 *        这解决了"同一声明点、yaw 0->180"在 G≠P 时目标点相差 2|G-P| 的问题。
 */
static bool FillRotateTarget(const MissionLeg_t *leg) {
    if (!FillRotateTargetAt(leg->yaw_deg, MISSION_ROTATE_POINT_NAME, &s_via_target)) {
        return false;
    }

    logInfo("MissionNav: rotate in place at (%.2f, %.2f) -> yaw %.1f",
            s_via_target.pose.x,
            s_via_target.pose.y,
            s_via_target.pose.yaw);
    return true;
}

/**
 * @brief 解析一段路线对应的目标点
 * @param leg        路线段
 * @param out        输出目标信息（target 指针须持续有效）
 * @param pass_chain true=该段在"通过点链"中，PASS 段需要放宽到达门限（防止提前判到点停车）
 * @return true 解析成功；false 点名不存在/原地旋转段取不到位姿
 */
typedef struct {
    TargetPoint_t *target;
    const char *name;
    bool rotate_only;
} LegTarget_t;

#if MISSION_CAL_ENABLE
static bool CalibrateAt(const char *target_name, MissionCal_t cal_type, int cal_offset_x_mm, int cal_offset_y_mm);
#endif
static bool AlignPositionOnly(const TargetPoint_t *target);

static bool ResolveLegTarget(const MissionLeg_t *leg, LegTarget_t *out, bool pass_chain) {
    out->rotate_only = (leg->mode == MISSION_NAV_MODE_ROTATE);
    if (out->rotate_only) {
        if (!FillRotateTarget(leg)) {
            return false;
        }
        out->target = &s_via_target;
        out->name = MISSION_ROTATE_POINT_NAME;
        return true;
    }

    if (leg->target_name != NULL) {
        TargetPoint_t *point = Map_GetPointByName(leg->target_name);
        if (point == NULL) {
            logError("MissionNav: unknown point '%s'", leg->target_name);
            return false;
        }
        out->name = leg->target_name;
        out->target = point;
    } else {
        FillViaTarget(leg);
        out->name = MISSION_VIA_POINT_NAME;
        out->target = &s_via_target;
    }

    if (leg->mode == MISSION_NAV_MODE_PASS && pass_chain) {
        /* PASS 段由任务层在"触发距离"处无缝换目标，不能等导航层自己判到点：
         * 把到达门限收小到几乎不可达（0.3cm），朝向门限放宽（朝向由过弯过程处理），
         * 只用 tag 名保留日志可读性。 */
        TargetPoint_t *buf = &s_pass_target[s_pass_target_idx];
        s_pass_target_idx ^= 1u;
        *buf = *out->target;
        buf->arrive.check_mode = ARRIVE_CHECK_BOTH;
        buf->arrive.distance_threshold = MISSION_PASS_ARRIVE_GUARD_CM;
        buf->arrive.yaw_threshold = MISSION_PASS_ARRIVE_GUARD_YAW_DEG;
        out->target = buf;
    }
    return true;
}

/**
 * @brief 到位后处理（树莓派二次标定 / 位置精修）
 * @param leg      路线段
 * @param target   已到位的目标点
 * @param leg_name 日志名
 * @return true 成功（精修失败只告警）
 */
static bool RunLegPostProcess(const MissionLeg_t *leg, TargetPoint_t *target, const char *leg_name) {
#if MISSION_CAL_ENABLE
    if (leg->cal_type != MISSION_CAL_NONE) {
        if (leg->target_name == NULL) {
            logError("MissionNav: calibration requires a named target");
            return false;
        }
        if (!CalibrateAt(leg->target_name, leg->cal_type, leg->cal_offset_x_mm, leg->cal_offset_y_mm)) {
            logError("MissionNav: calibration failed at '%s'", leg->target_name);
#if MISSION_STEP_SOFT_FAIL
            /* 软失败: 标定超时/失败只跳过该点的位置修正, 导航/后续动作照常继续 */
            if (IsAborted()) {
                return false;
            }
            logError("MissionNav: skip calibration correction at '%s', continue navigation (soft-fail)",
                     leg->target_name);
            return true;
#else
            return false;
#endif
        }
        return true;
    }
#endif /* MISSION_CAL_ENABLE */

    /* 到位精修是 best-effort：失败只告警，不中止任务 */
    if (leg->align && !IsAborted()) {
        if (!AlignPositionOnly(target)) {
            logWarning("MissionNav: position align at %s failed (continue)", leg_name);
        }
    }

    return true;
}

/**
 * @brief 到位位置精修：只修正位置，保持实测朝向
 * @param target 原目标点
 * @return true 已到位或已跳过；false 精修导航失败（调用方只告警，不中止任务）
 *
 * 终点朝向由规划器两段式对齐负责（目标点旋转不安全时规划器会拒绝原地补转）；
 * 任务层精修只用"距离到达"模式，目标 yaw 取当前实测 yaw，不强制在原地转体，
 * 避免任务点旋转不安全时扫入障碍。
 */
static bool AlignPositionOnly(const TargetPoint_t *target) {
#if MISSION_ALIGN_ENABLE
    PoseTimestamp_t pose;

    /* OPS 断帧/丢帧时拿到的"当前位姿"可能已过期，别用陈旧位姿去生成精修目标；
     * 精修本身是 best-effort，直接跳过即可(导航层对 OPS 过期有自己的停车/报错保护) */
    if (OPS_Is_Stale(MISSION_NAV_OPS_STALE_TIMEOUT_MS)) {
        logWarning("MissionNav: OPS stale, skip position align");
        return true;
    }

    if (!Loc_Get(&pose)) {
        logWarning("MissionNav: pose unavailable, skip position align");
        return true;
    }

    s_align_target = *target;
    s_align_target.pose.yaw = pose.pose.yaw;
    s_align_target.arrive.check_mode = ARRIVE_CHECK_DISTANCE;
    s_align_target.arrive.distance_threshold = MISSION_ALIGN_DIST_CM;
    s_align_target.arrive.timeout_ms = MISSION_ALIGN_TIMEOUT_MS;
    s_align_target.motion.target_speed = MISSION_ALIGN_SPEED_CM_S;

    osEventFlagsClear(System_StatusHandle, NAV_EVT_DONE | NAV_EVT_ERROR);
    if (!Nav_GoToDirect(&s_align_target)) {
        return false;
    }

    if (!Wait_Arrive()) {
        Nav_Stop();
        return false;
    }
    return true;
#else
    (void)target;
    return true;
#endif /* MISSION_ALIGN_ENABLE */
}

/**
 * @brief 等待本次导航到位
 * @note  阻塞等待导航完成/失败或任务中止事件，不再轮询状态
 * @return true 到位
 * @return false 导航失败或任务被中止（已顺带停止导航）
 */
bool Wait_Arrive(void) {
    uint32_t start_tick = osKernelGetTickCount();

    for (;;) {
        /* 任务被中止：立即停车并退出等待 */
        if (IsAborted()) {
            Mission_NavStop();
            return false;
        }

        NavState_t state = Nav_GetState();
        if (state == NAV_STATE_ERROR) {
            logError("Nav Failed");
            return false;
        }
        if (state == NAV_STATE_COMPLETE) {
            return true;
        }

        /* 兜底超时：导航层正常由 arrive.timeout_ms 产生 NAV_EVT_ERROR；
         * 这里再兜一层，防止导航状态机异常时任务永久阻塞在 osWaitForever。 */
        uint32_t elapsed = osKernelGetTickCount() - start_tick;
        if (elapsed >= MISSION_NAV_WAIT_TIMEOUT_MS) {
            logError("Nav wait timeout");
            Mission_NavStop();
            return false;
        }

        uint32_t flags = osEventFlagsWait(System_StatusHandle,
                                          NAV_EVT_DONE | NAV_EVT_ERROR | MISSION_ABORT,
                                          osFlagsWaitAny,
                                          MISSION_NAV_WAIT_TIMEOUT_MS - elapsed);
        if ((flags & osFlagsError) != 0) {
            if (flags == osFlagsErrorTimeout) {
                logError("Nav wait timeout");
            }
            Mission_NavStop();
            return false;
        }
        if ((flags & MISSION_ABORT) != 0) {
            Mission_NavStop();
            return false;
        }
    }
}

/**
 * @brief 启动一次"车体原地旋转"(非阻塞, 位置锁当前实测位姿, 只改朝向)
 * @param yaw_deg 目标朝向(°)
 * @return true=旋转已下发; false=取不到当前位姿/下发失败
 * @note  与路线表 ROTATE 段同一语义(位置不做车体参考点换算), 供"每段导航出发前的准备"期间
 *        把路线首段的原地旋转提前到物料盘归位等待中并行执行。
 */
bool Mission_NavStartRotateInPlace(float yaw_deg) {
    /* 位置锁当前实测位姿，只改朝向；目标点用静态存储，导航期间指针持续有效 */
    if (!FillRotateTargetAt(yaw_deg, MISSION_PRE_ROTATE_POINT_NAME, &s_pre_rotate_target)) {
        return false;
    }

    /* 清掉上一段导航遗留的完成/错误事件，避免 Wait_Arrive 被旧事件提前唤醒 */
    osEventFlagsClear(System_StatusHandle, NAV_EVT_DONE | NAV_EVT_ERROR);
    if (!Nav_GoToDirect(&s_pre_rotate_target)) {
        s_pre_rotate_active = false;
        return false;
    }

    s_pre_rotate_active = true;
    logInfo("MissionNav: pre-rotate in place at (%.2f, %.2f) -> yaw %.1f",
            s_pre_rotate_target.pose.x,
            s_pre_rotate_target.pose.y,
            s_pre_rotate_target.pose.yaw);
    return true;
}

/**
 * @brief 等待 Mission_NavStartRotateInPlace 启动的原地旋转到位
 * @return true=已到位(未启动过也返回 true); false=导航失败/超时/任务被中止
 */
bool Mission_NavWaitRotateInPlace(void) {
    if (!s_pre_rotate_active) {
        return true;
    }
    s_pre_rotate_active = false;
    return Wait_Arrive();
}

#if MISSION_CAL_ENABLE
/**
 * @brief 等待底盘位置模式移动到位（支持任务中止）
 * @param timeout_ms 最长等待时间(ms)
 * @return true=已到位; false=超时/任务被中止(已停车)
 * @note  位置模式在底盘层是开环轮位置指令，这里按"各轮实时位置到达目标并停稳"判到位；
 *        轮询间隔取 MOTOR_POS_POLL_MS(chassis_config.h)，中止时立即 MotionControl_Stop()。
 */
static bool WaitPositionMoveDone(uint32_t timeout_ms) {
    uint32_t start_tick = osKernelGetTickCount();

    for (;;) {
        if (IsAborted()) {
            MotionControl_Stop();
            return false;
        }
        if (MotionControl_PositionArrived()) {
            return true;
        }
        if ((osKernelGetTickCount() - start_tick) >= timeout_ms) {
            logError("MissionNav: position move wait timeout (%ums)", (unsigned)timeout_ms);
            MotionControl_Stop();
            return false;
        }
        osDelay(MOTOR_POS_POLL_MS);
    }
}

/**
 * @brief 标定后精确入位: 把 RPI_Cal 输出的修正量用底盘位置模式走完(不走 nav 速度闭环)
 * @param target_name     目标点名称(日志用)
 * @param cal_yaw_err_deg 视觉 yaw 误差(°); MISSION_CAL_YAW_FIX_WITH_ARM=1 时交给机械臂基座补偿
 * @return true=已到位; false=位置命令下发失败/等待超时
 * @note  RPI_Cal 输出的是**机体系**相对修正量(pose.x/y 位置 cm、pose.yaw 角度 deg)，
 *        与视觉脚本/相机-车体安装标定同一个方向约定(见 RPI_CAL_SIGN_X/Y/ANGLE)，
 *        因此直接作为机体系偏移下发给位置模式，不需要再用定位位姿做世界系换算。
 */
static bool PositionFixByCalibration(const char *target_name, float cal_yaw_err_deg) {
    float dx_body = s_cal_target.pose.x; // 机体系位置修正 (cm)
    float dy_body = s_cal_target.pose.y; // 机体系位置修正 (cm, 含固定偏移)
    float dyaw_deg = s_cal_target.pose.yaw; // 角度修正 (deg), 圆盘标定为 0

#if MISSION_CAL_YAW_FIX_WITH_ARM
    /* 车身 yaw 修正精度差: yaw 误差交给机械臂基座补偿, 本次位置模式只走位置 */
    dyaw_deg = 0.0f;
    Arm_Action_SetYawCorrection(cal_yaw_err_deg);
    logInfo("MissionNav: cal yaw error %.2fdeg -> arm base correction (car yaw kept)", (double)cal_yaw_err_deg);
#endif

    logInfo("MissionNav: pos fix '%s' body(%.2f, %.2f)cm %.2fdeg",
            target_name,
            (double)dx_body,
            (double)dy_body,
            (double)dyaw_deg);

    if (!MotionControl_SetPosition(dx_body, dy_body, dyaw_deg)) {
        return false;
    }
    return WaitPositionMoveDone(MISSION_CAL_POS_TIMEOUT_MS);
}

/**
 * @brief 到位后树莓派视觉二次标定 + 底盘位置模式精确入位
 * @param target_name   目标点名称（RPI_Cal 按点名查表并计算修正量）
 * @param cal_type      标定类型（圆盘/圆环第一次/圆环第二次/暂存区）
 * @param cal_offset_x_mm 目标点机体系 X 固定偏移(mm, +前)
 * @param cal_offset_y_mm 目标点机体系 Y 固定偏移(mm, +左)
 * @return true 已按标定结果修正到位；false 标定或位置精修失败
 *
 * 说明：RPI_Cal 会下发视觉标定命令（阻塞等待树莓派应答，最长 RPI_RSP_TIMEOUT_MS），
 * 成功后输出"相对当前位置的机体系视觉修正量"（x/y cm + yaw 角度，已含上述固定偏移），
 * 这里直接交给底盘位置模式（MotionControl_SetPosition）走完，
 * 不再用 nav 速度闭环——小距离/小角度修正下位置模式比速度闭环更直接、无导航超调。
 */
static bool CalibrateAt(const char *target_name, MissionCal_t cal_type, int cal_offset_x_mm, int cal_offset_y_mm) {
    ArmCalPose_t arm_pose;
    bool ok = false;
    float cal_yaw_err_deg = 0.0f;
    uint32_t cal_ms;
    uint32_t arm_ms = 0;
    uint32_t rpi_ms = 0;
    uint32_t pos_ms = 0;

    /* 中止请求已置位时不再下发标定命令（UART 应答期间无法中断，避免无谓等待） */
    if (IsAborted()) {
        return false;
    }

    logInfo("MissionNav: calibrate '%s' type=%d offset=(%d, %d)mm",
            target_name,
            (int)cal_type,
            cal_offset_x_mm,
            cal_offset_y_mm);

    /* 视觉标定前先把机械臂移动到该目标区域的预备位姿（旋转/伸展/抬升），
     * 再由 Arm_Action_CalibrationReady 把相机翻到标定角度；未到位则不发送标定命令。 */
    if (!ToArmCalPose(target_name, &arm_pose)) {
        return false;
    }

    /* 一次标定 = 机械臂预备 + 树莓派视觉应答 + 标定后位置精修，总耗时计入 CAL，
     * 三个子段单独计时（已在 CAL 内，只作分解，用于判断时间该找 arm 还是找 RPI 优化）。 */
    MissionTiming_Count(MISSION_TC_CAL, 1);
    MissionTiming_Begin(MISSION_TM_CAL);

    MissionTiming_Begin(MISSION_TM_CAL_ARM);
    if (!Arm_Action_CalibrationReady(&arm_pose)) {
        arm_ms = MissionTiming_End(MISSION_TM_CAL_ARM);
        logError("MissionNav: arm calibration pose not ready");
    } else {
        arm_ms = MissionTiming_End(MISSION_TM_CAL_ARM);
        if (IsAborted()) {
            /* 臂动作完成后再查一次中止：标定命令一旦下发就无法中断，被中止时不要继续 */
        } else {
            MissionTiming_Begin(MISSION_TM_CAL_RPI);
            if (!RPI_Cal(target_name,
                         cal_offset_x_mm,
                         cal_offset_y_mm,
                         ToRpiCal(cal_type),
                         &s_cal_target,
                         &cal_yaw_err_deg)) {
                rpi_ms = MissionTiming_End(MISSION_TM_CAL_RPI);
                logError("MissionNav: RPI_Cal failed at '%s'", target_name);
            } else {
                rpi_ms = MissionTiming_End(MISSION_TM_CAL_RPI);
                MissionTiming_Begin(MISSION_TM_CAL_POS);
                if (!PositionFixByCalibration(target_name, cal_yaw_err_deg)) {
                    pos_ms = MissionTiming_End(MISSION_TM_CAL_POS);
                    logError("MissionNav: position fix after calibration failed at '%s'", target_name);
                } else {
                    pos_ms = MissionTiming_End(MISSION_TM_CAL_POS);
                    ok = true;
                }
            }
        }
    }

    /* 标定结束(成功/失败/中止)统一收尾: 保持翻转舵机在标定角(颜色识别也用该视角), 不再翻回默认角;
     * 默认角只在整臂回零/伸展轴回零前或 `arm flip restore` 时恢复 */
    Arm_Action_CalibrationFinish();

    cal_ms = MissionTiming_End(MISSION_TM_CAL);
#if MISSION_TIMING_ENABLE && MISSION_TIMING_LOG_LEG
    logInfo("MissionTiming: [%s] cal '%s' %u ms (arm %u, rpi %u, pos %u)%s",
            MissionTiming_PhaseName(),
            target_name,
            (unsigned)cal_ms,
            (unsigned)arm_ms,
            (unsigned)rpi_ms,
            (unsigned)pos_ms,
            ok ? "" : " FAILED");
#else
    (void)cal_ms;
    (void)arm_ms;
    (void)rpi_ms;
    (void)pos_ms;
#endif

    return ok;
}
#endif /* MISSION_CAL_ENABLE */

/**
 * @brief 等待到达"通过点"的触发距离（不停车）
 * @param target 通过点目标
 * @return true 已进入触发距离（或已越过/导航层已判到点，由调用方切换下一段）；false 超时/失败/中止
 */
static bool WaitPassTrigger(TargetPoint_t *target) {
    uint32_t start_tick = osKernelGetTickCount();
    float best_dist = 1.0e9f;

    for (;;) {
        if (IsAborted()) {
            return false;
        }

        NavState_t state = Nav_GetState();
        if (state == NAV_STATE_ERROR) {
            logError("MissionNav: nav error while approaching pass point '%s'", target->name);
            return false;
        }
        /* 导航层已判到点/在停车：按"已通过"返回，后续换目标失败会走停车兜底 */
        if (state == NAV_STATE_STOPPING || state == NAV_STATE_COMPLETE || state == NAV_STATE_IDLE) {
            return true;
        }

        /* OPS 断帧/丢帧时位姿不再更新，这里会"距离一直不变"等不到触发(导航层虽会先报 ERROR，
         * 但提前判掉更干净、日志也能说明是定位链路问题而不是路线/触发距离问题) */
        if (OPS_Is_Stale(MISSION_NAV_OPS_STALE_TIMEOUT_MS)) {
            logError("MissionNav: OPS stale while approaching pass point '%s'", target->name);
            return false;
        }

        PoseTimestamp_t pose;
        if (Loc_Get(&pose)) {
            float dx = target->pose.x - pose.pose.x;
            float dy = target->pose.y - pose.pose.y;
            float dist = sqrtf(dx * dx + dy * dy);

            if (dist <= MISSION_PASS_TRIGGER_CM) {
                logInfo("MissionNav: pass '%s' triggered, dist=%.2fcm (continue)", target->name, dist);
                return true;
            }

            /* 兜底：横向偏差大于触发距离时距离会先减小后增大，检测"已越过"避免一直等不到 */
            if (dist < best_dist) {
                best_dist = dist;
            } else if (best_dist <= MISSION_PASS_MISS_MAX_CM && dist > best_dist + MISSION_PASS_MISS_BACK_CM) {
                logWarning("MissionNav: pass '%s' missed by %.2fcm, retarget anyway", target->name, best_dist);
                return true;
            }
        }

        if ((osKernelGetTickCount() - start_tick) >= MISSION_PASS_WAIT_TIMEOUT_MS) {
            logError("MissionNav: wait pass point '%s' timeout", target->name);
            return false;
        }

        osDelay(MISSION_PASS_POLL_MS);
    }
}

/**
 * @brief 执行一段"通过点链"：经过 PASS 段不停车，到第一个普通段才按原语义到点/标定/精修
 * @param route  路线
 * @param start  链首段索引（必须 mode==PASS 且还有下一段）
 * @param resume 输出：链执行完后的下一段索引
 * @return true 成功
 *
 * 流程：向链首通过点直连导航 -> 进入触发距离时 Nav_Retarget 换到下一段目标（不停车）
 *       -> 下一段仍是 PASS 就继续同样的过程 -> 下一段是普通段时等待到点并做标定/精修。
 */
static bool RunPassChain(const MissionRoute_t *route, uint8_t start, uint8_t *resume) {
    LegTarget_t cur;

    if (!ResolveLegTarget(&route->legs[start], &cur, true)) {
        return false;
    }
    if (cur.rotate_only) {
        logError("MissionNav: PASS leg cannot be a rotate leg");
        return false;
    }

    osEventFlagsClear(System_StatusHandle, NAV_EVT_DONE | NAV_EVT_ERROR);
    if (!Nav_GoToDirect(cur.target)) {
        return false;
    }

    uint8_t i = start;
    for (;;) {
        if (!WaitPassTrigger(cur.target)) {
            Nav_Stop();
            return false;
        }

        i++;
        if (i >= route->count) {
            /* 路线以 PASS 段结尾：退化为普通到点（该目标未被改写门限） */
            if (!Wait_Arrive()) {
                Nav_Stop();
                return false;
            }
            *resume = i;
            return true;
        }

        LegTarget_t next;
        /* 链中 PASS 段用放宽门限；链尾普通段用原门限 */
        bool next_pass_chain = (route->legs[i].mode == MISSION_NAV_MODE_PASS) && ((uint8_t)(i + 1) < route->count);
        if (!ResolveLegTarget(&route->legs[i], &next, next_pass_chain)) {
            Nav_Stop();
            return false;
        }
        if (next.rotate_only) {
            logError("MissionNav: PASS leg followed by a rotate leg is not supported");
            Nav_Stop();
            return false;
        }

#if MISSION_PASS_YAW_FAST
        /* 过弯后开启 yaw 快速跟随：边走边把车头转向下一段目标朝向 */
        Nav_SetYawFast(true);
#endif

        if (!Nav_Retarget(next.target)) {
            /* 导航层已不在 RUNNING（例如恰好到点/状态切换）：停车后按普通段重跑，保证行为正确 */
            logWarning("MissionNav: retarget failed, fall back to stop-and-go at '%s'", next.name);
            Nav_Stop();
            osDelay(MISSION_NAV_RETRY_DELAY_MS);
            if (!Mission_NavGoToLeg(&route->legs[i])) {
                return false;
            }
            *resume = (uint8_t)(i + 1);
            return true;
        }

        if (next_pass_chain) {
            cur = next;
            continue;
        }

        /* 链尾普通段：等待到点后做标定/精修 */
        if (!Wait_Arrive()) {
            logError("MissionNav: navigate to '%s' failed after pass chain", next.name);
            Nav_Stop();
            return false;
        }
#if MISSION_PASS_YAW_FAST
        Nav_SetYawFast(false);
#endif
        *resume = (uint8_t)(i + 1);
        return RunLegPostProcess(&route->legs[i], next.target, next.name);
    }
}

void Mission_NavAbort(void) {
#if PLANNER_ENABLED == 1
    /* 规划接口是阻塞式的（要等整条路径下发完），停止任务时必须主动中止，
     * 否则调用方要等本次规划全跑完才能退出 */
    Planner_Stop();
#endif /* PLANNER_ENABLED */
}
void Mission_NavStop(void) {
    /* 停止后预旋转随之作废: 之后 Mission_NavWaitRotateInPlace 不再等一个已被停掉的目标 */
    s_pre_rotate_active = false;
    Nav_Stop();
}

bool Mission_NavGoToEx(const char *target_name, MissionNavMode_t mode) {
    if (target_name == NULL) {
        logError("MissionNav: null target name");
        return false;
    }

    /* 单点导航开始时清掉上一工位遗留的机械臂 yaw 补偿 */
    Arm_Action_SetYawCorrection(0.0f);

    TargetPoint_t *target = Map_GetPointByName(target_name);
    if (target == NULL) {
        logError("MissionNav: unknown point '%s'", target_name);
        return false;
    }
    return NavToTarget(target, mode);
}

bool Mission_NavGoToLeg(const MissionLeg_t *leg) {
    if (leg == NULL) {
        return false;
    }

    LegTarget_t resolved;
    /* PASS 段单独执行（不在过点链里，例如路线最后一段）时按普通到点处理：
     * 不做触发距离换目标，也不需要放宽到达门限。 */
    if (!ResolveLegTarget(leg, &resolved, false)) {
        return false;
    }

    if (!NavToTarget(resolved.target, resolved.rotate_only ? MISSION_NAV_MODE_DIRECT : leg->mode)) {
        if (resolved.rotate_only) {
            logError("MissionNav: rotate in place to yaw %.0fdeg failed", leg->yaw_deg);
        } else if (leg->target_name != NULL) {
            logError("MissionNav: navigate to '%s' failed", leg->target_name);
        } else {
            logError("MissionNav: navigate to via (%.1f, %.1f, %.0fdeg) failed", leg->x_cm, leg->y_cm, leg->yaw_deg);
        }
        return false;
    }

    /* 原地旋转段转到位即完成，没有后续标定/精修语义 */
    if (resolved.rotate_only) {
        return true;
    }

    return RunLegPostProcess(leg, resolved.target, resolved.name);
}

bool Mission_NavGoToRoute(const MissionRoute_t *route, const char *tag) {
    /* 阶段开始时清掉上一工位的机械臂 yaw 补偿, 本阶段标定成功后重新设置
     * (放在空路线判断之前, 避免空路线时残留上一阶段的补偿) */
    Arm_Action_SetYawCorrection(0.0f);

    /* NULL 与 count==0 都按空路线处理（不同启动区/轮次没有该阶段） */
    if (route == NULL || route->count == 0) {
        return true;
    }
    if (route->legs == NULL) {
        return false;
    }

    for (uint8_t i = 0; i < route->count; i++) {
        const MissionLeg_t *leg = &route->legs[i];

        if (IsAborted()) {
            return false;
        }

        if (leg->mode == MISSION_NAV_MODE_ROTATE) {
            logInfo("Mission[%s] leg %u/%u -> rotate in place to yaw %.0fdeg",
                    tag,
                    (unsigned int)(i + 1),
                    (unsigned int)route->count,
                    leg->yaw_deg);
        } else if (leg->target_name != NULL) {
            logInfo("Mission[%s] leg %u/%u -> %s%s",
                    tag,
                    (unsigned int)(i + 1),
                    (unsigned int)route->count,
                    leg->target_name,
                    (leg->mode == MISSION_NAV_MODE_PASS) ? " (pass-through, no stop)" : "");
        } else {
            logInfo("Mission[%s] leg %u/%u -> via (%.1f, %.1f, %.0fdeg)%s",
                    tag,
                    (unsigned int)(i + 1),
                    (unsigned int)route->count,
                    leg->x_cm,
                    leg->y_cm,
                    leg->yaw_deg,
                    (leg->mode == MISSION_NAV_MODE_PASS) ? " (pass-through, no stop)" : "");
        }

        /* 通过点链：链首是"不停车通过点"且后面还有普通段时才走链式执行，
         * 链执行完会顺带把紧随其后的普通段（含标定/精修）跑完，返回下一段索引。
         * 若下一段是原地旋转段则不链式执行（PASS 段退化为普通到点）。 */
        if (leg->mode == MISSION_NAV_MODE_PASS && (uint8_t)(i + 1) < route->count &&
            route->legs[i + 1].mode != MISSION_NAV_MODE_ROTATE) {
            uint8_t resume = i;
            uint8_t covered = 0;
            uint8_t pass_legs = 0;
            bool tail_stop = false;
            bool unit_ok;
            uint32_t elapsed_ms;

            MissionTiming_Begin(MISSION_TM_NAV);
            unit_ok = RunPassChain(route, i, &resume);
            elapsed_ms = MissionTiming_End(MISSION_TM_NAV);

            /* 统计：链覆盖的段数 = resume-i；链尾若是普通到点段则它还会停一次车 */
            if (resume > i) {
                covered = (uint8_t)(resume - i);
                pass_legs = covered;
                if (route->legs[resume - 1].mode != MISSION_NAV_MODE_PASS) {
                    tail_stop = true;
                    pass_legs--;
                }
            }
            MissionTiming_Count(MISSION_TC_NAV_LEG, covered);
            MissionTiming_Count(MISSION_TC_NAV_PASS_LEG, pass_legs);
            if (tail_stop) {
                MissionTiming_Count(MISSION_TC_NAV_STOP, 1);
            }
#if MISSION_TIMING_ENABLE && MISSION_TIMING_LOG_LEG
            logInfo("MissionTiming: [%s] legs %u-%u/%u %u pass %u ms%s",
                    tag,
                    (unsigned)(i + 1),
                    (unsigned)(covered != 0 ? resume : (uint8_t)(i + 1)),
                    (unsigned)route->count,
                    (unsigned)pass_legs,
                    (unsigned)elapsed_ms,
                    unit_ok ? (tail_stop ? " (tail stop)" : "") : " FAILED");
#else
            (void)elapsed_ms;
#endif

            if (!unit_ok) {
#if MISSION_STEP_SOFT_FAIL
                /* 软失败: 通过点链超时/失败只记录, 继续执行后续段(车已按导航闭环移动到某处) */
                logError("MissionNav[%s] pass chain %u-%u failed/timeout, continue next leg (soft-fail)",
                         tag,
                         (unsigned)(i + 1),
                         (unsigned)(covered != 0 ? resume : (uint8_t)(i + 1)));
                continue;
#else
                return false;
#endif
            }
            i = (uint8_t)(resume - 1);
            if (IsAborted()) {
                return false;
            }
            continue;
        }

        {
            bool leg_ok;
            uint32_t elapsed_ms;

            MissionTiming_Begin(MISSION_TM_NAV);
            leg_ok = Mission_NavGoToLeg(leg);
            elapsed_ms = MissionTiming_End(MISSION_TM_NAV);

            MissionTiming_Count(MISSION_TC_NAV_LEG, 1);
            if (leg_ok) {
                MissionTiming_Count(MISSION_TC_NAV_STOP, 1); /* 普通到点段(含 ROTATE)到位都会触发 STOPPING */
            }
#if MISSION_TIMING_ENABLE && MISSION_TIMING_LOG_LEG
            if (leg->mode == MISSION_NAV_MODE_ROTATE) {
                logInfo("MissionTiming: [%s] leg %u/%u rotate %.0fdeg %u ms%s",
                        tag,
                        (unsigned)(i + 1),
                        (unsigned)route->count,
                        (double)leg->yaw_deg,
                        (unsigned)elapsed_ms,
                        leg_ok ? "" : " FAILED");
            } else if (leg->target_name != NULL) {
                logInfo("MissionTiming: [%s] leg %u/%u %s %u ms%s",
                        tag,
                        (unsigned)(i + 1),
                        (unsigned)route->count,
                        leg->target_name,
                        (unsigned)elapsed_ms,
                        leg_ok ? "" : " FAILED");
            } else {
                logInfo("MissionTiming: [%s] leg %u/%u via(%.1f,%.1f,%.0f) %u ms%s",
                        tag,
                        (unsigned)(i + 1),
                        (unsigned)route->count,
                        (double)leg->x_cm,
                        (double)leg->y_cm,
                        (double)leg->yaw_deg,
                        (unsigned)elapsed_ms,
                        leg_ok ? "" : " FAILED");
            }
#else
            (void)elapsed_ms;
#endif

            if (!leg_ok) {
#if MISSION_STEP_SOFT_FAIL
                /* 软失败: 单段导航超时/失败只记录, 继续执行本路线后续段 */
                logError("MissionNav[%s] leg %u/%u failed/timeout, continue next leg (soft-fail)",
                         tag,
                         (unsigned)(i + 1),
                         (unsigned)route->count);
                continue;
#else
                return false;
#endif
            }
        }

        if (IsAborted()) {
            return false;
        }
    }

    return true;
}

bool Mission_NavGoTo(const char *target_name, MissionCal_t cal_type, int cal_offset_x_mm, int cal_offset_y_mm) {
    if (!Mission_NavGoToEx(target_name, MISSION_NAV_MODE_AUTO)) {
        return false;
    }

    /* 仅对"需要校准"的目标点做二次标定导航：必须指定了校准类型（偏移 0 是合法值）。
     * Home/QrCode 等点传入 MISSION_CAL_NONE，应直接跳过，避免误下发标定命令。 */
    if (cal_type == MISSION_CAL_NONE) {
        return true;
    }

#if MISSION_CAL_ENABLE
    if (!CalibrateAt(target_name, cal_type, cal_offset_x_mm, cal_offset_y_mm)) {
#if MISSION_STEP_SOFT_FAIL
        /* 软失败: 标定超时/失败只跳过位置修正, 不中止任务 */
        if (IsAborted()) {
            return false;
        }
        logError("MissionNav: calibration failed/timeout at '%s', continue without correction (soft-fail)",
                 target_name);
        return true;
#else
        return false;
#endif
    }
    return true;
#else
    /* 编译期关闭二次标定：保持"已导航到位"语义，不中断任务 */
    logWarning("MissionNav: RPI calibration disabled (MISSION_CAL_ENABLE=0)");
    return true;
#endif /* MISSION_CAL_ENABLE */
}
