/**
 * @file arm_motion.c
 * @author T-Car Team
 * @brief 机械臂运动接口: 模块初始化、笛卡尔/极坐标移动、到位等待、当前位置读取
 *
 * 本层把坐标解算(arm_kinematics)与单轴控制(arm_ctrl)组合成多轴运动命令,
 * 并把单轴读回换算成笛卡尔/极坐标位姿(arm pos 的三种数据)。
 * 可调参数见 arm_config.h / Applications/arm。
 */

#include "arm_motion.h"
#include "arm_config.h"
#include "arm_ctrl.h"
#include "arm_kinematics.h"
#include "log.h"
#include <math.h>

/** 默认 profile(各轴默认速度): 调试命令/未指定动作时使用 */
static const ArmMotionProfile_t s_default_profile = ARM_PROFILE_DEFAULT_INIT;

/** 当前动作使用的 profile: 由动作层 Arm_SetMotionProfile() 切换 */
static ArmMotionProfile_t s_motion_profile = ARM_PROFILE_DEFAULT_INIT;

void Arm_SetMotionProfile(const ArmMotionProfile_t *profile) {
    if (profile == NULL) {
        s_motion_profile = s_default_profile;
        return;
    }
    s_motion_profile = *profile;
}

const ArmMotionProfile_t *Arm_GetMotionProfile(void) {
    return &s_motion_profile;
}

/**
 * @brief 按(基座角度, 伸展长度, 抬升高度)下发三条轴命令
 * @param base_deg  基座目标角度 (°)
 * @param extend_mm 同步带目标伸长量 (mm)
 * @param lift_mm   目标抬升高度 (mm), 0=最底部
 * @return true 三条命令均下发成功
 * @note 三条命令的下发结果都需要检查: 队列满/超限时会失败, 不能静默当作已移动。
 *       速度/加速度使用当前动作 profile(Arm_SetMotionProfile)。
 */
static bool arm_send_axes(int32_t base_deg, float extend_mm, float lift_mm) {
    bool ok = true;

    if (!Arm_BaseRotateAbs(base_deg, s_motion_profile.base_vel, s_motion_profile.base_acc)) {
        logWarning("arm: base rotate cmd rejected/dropped");
        ok = false;
    }
    if (!Arm_ExtendTo(extend_mm, s_motion_profile.extend_vel, s_motion_profile.extend_acc)) {
        logWarning("arm: extend cmd rejected/dropped");
        ok = false;
    }
    if (!Arm_LiftTo(lift_mm, s_motion_profile.lift_vel, s_motion_profile.lift_acc)) {
        logWarning("arm: lift cmd rejected/dropped");
        ok = false;
    }
    return ok;
}

/**
 * @brief 角度四舍五入到整度(位置命令按整度下发)
 */
static int32_t arm_round_deg(float angle_deg) {
    return (int32_t)(angle_deg + (angle_deg >= 0.0f ? 0.5f : -0.5f));
}

/**
 * @brief 检查三轴电机是否在线(能否读到位置/角度)
 * @return true=三轴均可读; false=存在离线电机(已打印具体轴/ID)
 * @note "读不到位置"与"位置正好为 0"是两回事: 电机未接/掉线/总线异常时绝对位置不可信,
 *       运动与到位判定必然失败; 先打印明确原因(电机 ID/轴名), 避免只看到"未到位/重试"
 *       而误判成运动逻辑问题。
 */
static bool arm_check_motors_online(void) {
    bool all_online = true;

    for (uint8_t i = 0; i < Arm_AxisCount(); i++) {
        uint8_t motor_id = Arm_AxisMotorId(i);
        float angle_deg = 0.0f;
        if (!Arm_MotorGetAngle(motor_id, &angle_deg)) {
            logError("arm: motor ID%d (%s) offline, check arm power/bus cable ('arm pos' shows per-axis status)",
                     (int)motor_id,
                     Arm_AxisName(i));
            all_online = false;
        }
    }

    return all_online;
}

/**
 * @brief 笛卡尔坐标解算为(基座目标角度, 同步带目标伸长量)
 * @param x_mm       目标 X (mm)
 * @param y_mm       目标 Y (mm)
 * @param base_deg   输出: 基座目标角度 (°, 已取整)
 * @param extend_mm  输出: 同步带目标伸长量 (mm)
 * @return false=目标不可达(不下发任何命令)
 */
static bool arm_cartesian_to_axes(float x_mm, float y_mm, int32_t *base_deg, float *extend_mm) {
    float base_angle_deg;
    float extend;

    if (!Arm_Kin_CartesianToPolar(x_mm, y_mm, &base_angle_deg, &extend)) {
        logWarning("arm: target (%.1f, %.1f) mm unreachable", (double)x_mm, (double)y_mm);
        return false;
    }

    logDebug("[arm] ik (%.1f, %.1f) mm -> base %.1f deg, extend %.1f mm",
             (double)x_mm,
             (double)y_mm,
             (double)base_angle_deg,
             (double)extend);

    *base_deg = arm_round_deg(base_angle_deg);
    *extend_mm = extend;
    return true;
}

/**
 * @brief 打印某段运动失败时的目标/实际位姿(便于定位"偶发不到位")
 * @param stage     阶段名("horizontal" / "lift")
 * @param base_deg  目标基座角度 (°)
 * @param extend_mm 目标伸展长度 (mm)
 * @param lift_mm   目标抬升高度 (mm)
 */
static void arm_log_stage_failure(const char *stage, int32_t base_deg, float extend_mm, float lift_mm) {
    ArmPolarPose_t actual;
    if (Arm_GetPolarPose(&actual)) {
        logWarning("arm: %s stage not reached (target base %ld deg, extend %.1f mm, lift %.1f mm; "
                   "actual base %.1f deg, extend %.1f mm, lift %.1f mm)",
                   stage,
                   (long)base_deg,
                   (double)extend_mm,
                   (double)lift_mm,
                   (double)actual.base_angle_deg,
                   (double)actual.extend_mm,
                   (double)actual.lift_mm);
    } else {
        logWarning("arm: %s stage not reached (current pose unavailable)", stage);
    }
}

/**
 * @brief 水平段: 下发基座旋转 + 同步带伸展, 并等待水平两轴到位
 * @return true=两轴到位; false=命令下发失败或等待超时/未到位
 */
static bool arm_move_horizontal_stage(int32_t base_deg, float extend_mm, uint32_t timeout_ms) {
    if (!Arm_BaseRotateAbs(base_deg, s_motion_profile.base_vel, s_motion_profile.base_acc)) {
        logWarning("arm: base rotate cmd rejected/dropped");
        return false;
    }
    if (!Arm_ExtendTo(extend_mm, s_motion_profile.extend_vel, s_motion_profile.extend_acc)) {
        logWarning("arm: extend cmd rejected/dropped");
        return false;
    }
    return Arm_WaitSettledHorizontal(timeout_ms);
}

/**
 * @brief 升降段: 下发升降命令并等待升降轴到位
 * @return true=升降到位; false=命令下发失败或等待超时/未到位
 */
static bool arm_move_lift_stage(float lift_mm, uint32_t timeout_ms) {
    if (!Arm_LiftTo(lift_mm, s_motion_profile.lift_vel, s_motion_profile.lift_acc)) {
        logWarning("arm: lift cmd rejected/dropped");
        return false;
    }
    return Arm_WaitSettledLift(timeout_ms);
}

/**
 * @brief 水平段带回退重试: 最终仍未到位只告警, 不当作整体失败(按实际可达位置继续)
 * @return true=到位; false=重试用尽仍未到位(已打印实际位姿, 调用方继续后续段)
 * @note 现场策略: 目标超出可达范围/被挡住时, 不强停等, 按当前实际位置继续后续动作。
 */
static bool arm_run_horizontal_stage_best_effort(int32_t base_deg,
                                                 float extend_mm,
                                                 float lift_mm,
                                                 uint32_t timeout_ms) {
    for (uint8_t attempt = 0; attempt <= ARM_MOVE_RETRY; attempt++) {
        if (arm_move_horizontal_stage(base_deg, extend_mm, timeout_ms)) {
            return true;
        }
        arm_log_stage_failure("horizontal", base_deg, extend_mm, lift_mm);
        if (attempt < ARM_MOVE_RETRY) {
            logWarning("arm: retry horizontal stage (%u/%u)", (unsigned)(attempt + 1), (unsigned)ARM_MOVE_RETRY);
        }
    }
    logWarning("arm: horizontal stage not reached, continue with actual pose (best effort)");
    return false;
}

/**
 * @brief 升降段带回退重试: 最终仍未到位只告警, 不当作整体失败(按实际可达位置继续)
 * @return true=到位; false=重试用尽仍未到位(已打印实际位姿, 调用方继续后续段)
 */
static bool arm_run_lift_stage_best_effort(int32_t base_deg, float extend_mm, float lift_mm, uint32_t timeout_ms) {
    for (uint8_t attempt = 0; attempt <= ARM_MOVE_RETRY; attempt++) {
        if (arm_move_lift_stage(lift_mm, timeout_ms)) {
            return true;
        }
        arm_log_stage_failure("lift", base_deg, extend_mm, lift_mm);
        if (attempt < ARM_MOVE_RETRY) {
            logWarning("arm: retry lift stage (%u/%u)", (unsigned)(attempt + 1), (unsigned)ARM_MOVE_RETRY);
        }
    }
    logWarning("arm: lift stage not reached, continue with actual pose (best effort)");
    return false;
}

/**
 * @brief 角度差归算到 ±180° 内(判断目标角与当前角是否已在同一位置)
 * @param a 角度 a (°)
 * @param b 角度 b (°)
 * @return a-b 归算到 [-180, 180]
 */
static float arm_angle_diff_deg(float a, float b) {
    float d = a - b;
    while (d > 180.0f)
        d -= 360.0f;
    while (d < -180.0f)
        d += 360.0f;
    return d;
}

/**
 * @brief 分步运动: 按升降方向决定"先水平还是先升降", 已在目标上的段直接跳过
 * @param base_deg    基座目标角度 (°)
 * @param extend_mm   同步带目标伸长量 (mm)
 * @param lift_mm     目标抬升高度 (mm), 0=最底部
 * @param timeout_ms  每段到位等待超时 (ms)
 * @return true=已执行完(每段尽力执行; 有段未到位时按实际可达位置继续, 仍返回 true)
 * @note 读当前位姿后分两种顺序(水平动作始终放在较高的那个高度上做):
 *       - 下降(目标低于当前): 先基座旋转+伸展, 水平到位后再下降;
 *       - 上升(目标高于当前): 先升到目标高度, 再基座旋转+伸展;
 *       等高(差在 ARM_MOVE_LIFT_DIR_TOL_MM 内)按下降处理。
 *       当前位姿读取失败时告警并退回"先水平后升降"(原逻辑)。
 *       **段跳过**: 目标与当前差值在 ARM_MOVE_SKIP_* 容差内时跳过该段(不发命令、不等到位),
 *       只升/降或只水平的动作(盘面下降/抬起、抓取后抬升、回初始位等)因此不再空等一次到位判定;
 *       水平/升降都在目标上时直接返回。
 *       每段超时后按 ARM_MOVE_RETRY 自动重发一次同样的绝对目标;
 *       重试用尽仍未到位时**只告警并继续后续段/动作**(best effort, 按实际可达位置走), 不再让整轮任务失败。
 */
static bool arm_move_sequence(int32_t base_deg, float extend_mm, float lift_mm, uint32_t timeout_ms) {
    float current_lift_mm = 0.0f;
    float current_extend_mm = 0.0f;
    float current_base_deg = 0.0f;
    bool have_pose = true;
    bool ascending = false; // 默认按"下降/等高"处理(先水平后升降)
    bool skip_horizontal = false;
    bool skip_lift = false;
    bool reached = true;

    if (!Arm_LiftGetHeight(&current_lift_mm)) {
        logWarning("arm: read lift height failed, fallback to horizontal-first");
        have_pose = false;
    }
    if (!Arm_ExtendGetLength(&current_extend_mm) || !Arm_MotorGetAngle(ARM_BASE_MOTOR_ID, &current_base_deg)) {
        logWarning("arm: read horizontal pose failed, no stage skip");
        have_pose = false;
    }

    if (have_pose) {
        ascending = (lift_mm > current_lift_mm + ARM_MOVE_LIFT_DIR_TOL_MM);
        skip_lift = (fabsf(lift_mm - current_lift_mm) <= ARM_MOVE_SKIP_LIFT_MM);
        skip_horizontal = (fabsf(extend_mm - current_extend_mm) <= ARM_MOVE_SKIP_HORIZONTAL_MM) &&
                          (fabsf(arm_angle_diff_deg((float)base_deg, current_base_deg)) <= ARM_MOVE_SKIP_BASE_DEG);
    }

    /* 水平/升降都已在目标上: 不下发命令、不做到位等待, 直接返回(省掉动作之间的无谓停顿) */
    if (skip_lift && skip_horizontal) {
        return true;
    }

    if (ascending) {
        /* 上升动作: 先升到目标高度, 到位后再旋转+伸展 */
        if (!skip_lift && !arm_run_lift_stage_best_effort(base_deg, extend_mm, lift_mm, timeout_ms)) {
            reached = false;
        }
        if (!skip_horizontal && !arm_run_horizontal_stage_best_effort(base_deg, extend_mm, lift_mm, timeout_ms)) {
            reached = false;
        }
    } else {
        /* 下降/等高: 先旋转+伸展到位, 再下降 */
        if (!skip_horizontal && !arm_run_horizontal_stage_best_effort(base_deg, extend_mm, lift_mm, timeout_ms)) {
            reached = false;
        }
        if (!skip_lift && !arm_run_lift_stage_best_effort(base_deg, extend_mm, lift_mm, timeout_ms)) {
            reached = false;
        }
    }

    if (!reached) {
        logWarning("arm: move finished with unreached stage(s), continue with actual pose (best effort; "
                   "target base %ld, extend %.1f, lift %.1f)",
                   (long)base_deg,
                   (double)extend_mm,
                   (double)lift_mm);
    }
    return true;
}

void Arm_Init(void) {
    Arm_CtrlInit();
}

bool Arm_MoveTo(float x_mm, float y_mm, float z_mm) {
    int32_t base_deg;
    float extend_mm;

    if (!arm_check_motors_online()) return false;
    if (!arm_cartesian_to_axes(x_mm, y_mm, &base_deg, &extend_mm)) return false;
    return arm_send_axes(base_deg, extend_mm, z_mm);
}

bool Arm_MoveToAndWait(float x_mm, float y_mm, float z_mm, uint32_t timeout_ms) {
    /* 整臂回零是异步执行的: 未完成前下发运动命令会与回零冲突, 先等待就绪 */
    if (!Arm_HomeWaitDone(ARM_HOME_READY_TIMEOUT_MS)) {
        logWarning("arm: homing not ready, movement may be inaccurate");
    }
    if (!arm_check_motors_online()) return false;

    int32_t base_deg;
    float extend_mm;
    if (!arm_cartesian_to_axes(x_mm, y_mm, &base_deg, &extend_mm)) return false;
    return arm_move_sequence(base_deg, extend_mm, z_mm, timeout_ms);
}

bool Arm_MovePolarTo(float extend_mm, float base_angle_deg, float lift_mm) {
    if (!arm_check_motors_online()) return false;

    return arm_send_axes(arm_round_deg(base_angle_deg), extend_mm, lift_mm);
}

bool Arm_MovePolarToAndWait(float extend_mm, float base_angle_deg, float lift_mm, uint32_t timeout_ms) {
    if (!Arm_HomeWaitDone(ARM_HOME_READY_TIMEOUT_MS)) {
        logWarning("arm: homing not ready, movement may be inaccurate");
    }
    if (!arm_check_motors_online()) return false;

    return arm_move_sequence(arm_round_deg(base_angle_deg), extend_mm, lift_mm, timeout_ms);
}

/*============================ 当前位置读取(三种数据) ============================*/

bool Arm_GetPolarPose(ArmPolarPose_t *pose) {
    if (pose == NULL) return false;

    float base_angle_deg;
    float extend_mm;
    float lift_mm;
    if (!Arm_MotorGetAngle(ARM_BASE_MOTOR_ID, &base_angle_deg)) return false;
    if (!Arm_ExtendGetLength(&extend_mm)) return false;
    if (!Arm_LiftGetHeight(&lift_mm)) return false;

    pose->extend_mm = extend_mm;
    pose->base_angle_deg = base_angle_deg;
    pose->lift_mm = lift_mm;
    return true;
}

bool Arm_GetCartesianPose(ArmCartesianPose_t *pose) {
    if (pose == NULL) return false;

    /* 先读极坐标, 再由正解换算笛卡尔坐标; z 即抬升高度 */
    ArmPolarPose_t polar;
    if (!Arm_GetPolarPose(&polar)) return false;

    float x_mm;
    float y_mm;
    if (!Arm_Kin_PolarToCartesian(polar.extend_mm, polar.base_angle_deg, &x_mm, &y_mm)) return false;

    pose->x_mm = x_mm;
    pose->y_mm = y_mm;
    pose->z_mm = polar.lift_mm;
    return true;
}

bool Arm_GetPos(ArmPos_t *pos) {
    if (pos == NULL) return false;

    *pos = (ArmPos_t){0};

    /* 1) 机械臂相关电机当前角度(相对机械零位, 下标顺序同 Arm_AxisCount) */
    for (uint8_t axis = 0; axis < Arm_AxisCount(); axis++) {
        uint8_t motor_id = Arm_AxisMotorId(axis);
        pos->motor_angle_valid[axis] = Arm_MotorGetAngle(motor_id, &pos->motor_angle_deg[axis]);
    }

    /* 2) 极坐标: 伸展长度/旋转角度/抬升高度 */
    pos->polar_valid = Arm_GetPolarPose(&pos->polar);

    /* 3) 笛卡尔坐标: 由第 2 步同一次采样换算, 保证与极坐标输出一致; z=抬升高度 */
    if (pos->polar_valid) {
        pos->cartesian_valid = Arm_Kin_PolarToCartesian(
            pos->polar.extend_mm, pos->polar.base_angle_deg, &pos->cartesian.x_mm, &pos->cartesian.y_mm);
        pos->cartesian.z_mm = pos->polar.lift_mm;
    }

    return pos->polar_valid;
}
