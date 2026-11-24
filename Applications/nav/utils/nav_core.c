/**
 * @file nav_core.c
 * @author Pcb-yun (pcbyinyun@163.com)
 * @brief 导航核心控制器源文件
 *
 * 控制架构（统一、连续，无远/近模式切换）：
 *   位置环: 速度幅值 = min(kp*剩余距离, 滞后补偿制动上限)，方向始终指向目标（全误差向量 P）。
 *           远处由制动上限主导（梯形巡航/减速），近处由 P 主导（指数收敛）；
 *           制动上限按 v²/(2a)+v*lag<=距离 求解，计入电机命令链/轮速滞后，抑制延迟超调；
 *           实测速度高于指令时进一步收紧上限（提前刹车）。横向偏差由"方向指向目标"天然消除，
 *           不再需要远距横向 PI / 近目标纯 P 两套模式与过渡混合。
 *   航向环: 前馈 = sign(err)*min(yaw_max, sqrt(2*a_yaw*|err|))（sqrt 减速剖面，远处饱和快转、
 *           近处平滑减速到 0，天然无过冲）+ PID 反馈修正（抗饱和 + D 低通阻尼），按 ff/fb 权重融合；
 *           最小角速度钳位只在 |err| 较大时生效，近目标允许平滑收敛，无零交叉锁存。
 *   输出:   机体系加减速限幅 + 速度限幅后下发。
 *
 * 状态机: IDLE -> RUNNING -> STOPPING -> COMPLETE（失败/定位丢失 -> ERROR）。
 * 安全门: OPS 新鲜度保护、导航超时、停稳滑动窗口 + 距离/航向复核 + 补位次数上限（见 nav_config.h）。
 * 可调参数见 nav_config.h / Applications/nav/README.md。
 */

#include "nav_core.h"
#include "nav_local.h"
#include "nav_math.h"
#include "nav_config.h"
#include "motion_control.h"
#include "cmsis_os2.h"
#include "Events.h"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include <chassis_config.h>
#include "log.h"
#include "zdt_v5_cfg.h"
#include "ops.h"

#if !MOTOR_VELOCITY_MODE
#error "MOTOR_VELOCITY_MODE must be enabled for velocity-based navigation"
#endif

/**
 * @brief PID控制器结构体
 */
typedef struct {
    float kp, ki, kd; // PID参数
    float integral; // 积分项
    float last_error; // 上一次的误差
    float d_filtered; // 低通滤波后的微分项
    float output; // 输出
    float output_limit; // 输出限制
} PIDController_t;

/**
 * @brief 导航核心控制器结构体
 */
typedef struct {
    NavState_t state; // 导航状态（IDLE/RUNNING/STOPPING/COMPLETE/ERROR）
    uint32_t task_start_tick; // 任务开始时间戳
    uint32_t last_control_tick; // 上次控制时间戳
    TargetPoint_t *cached_target; // 缓存的目标点
    float saved_linear_speed; // 保存的线速度（导航前状态）
    float saved_yaw_speed; // 保存的yaw速度（导航前状态）
    float saved_acc; // 保存的加速度（导航前状态）
    float saved_dec; // 保存的减速度（导航前状态）
    PIDController_t yaw_pid; // yaw角度PID控制器
    float last_vx; // 上一次的车身系X速度
    float last_vy; // 上一次的车身系Y速度
    float last_yaw_speed; // 上一次的yaw角速度
    float current_speed_magnitude; // 当前速度幅值（调试快照用）
    uint32_t arrive_timeout_tick; // 到达超时时间戳
    uint8_t arrive_check_count; // 连续到达检查计数器
    uint32_t stop_start_tick; // 停止开始时间戳（用于等待车辆静止）
    bool yaw_fast; // yaw 快速跟随标志（任务层过弯时置位，见 Nav_SetYawFast）
    Pose2D_t stop_win_pose; // STOPPING 停稳检测：滑动窗口参考位姿
    uint32_t stop_win_tick; // STOPPING 停稳检测：上次窗口采样时刻
    bool stop_sliding; // STOPPING 停稳检测：最近一个窗口内是否仍在移动
    uint8_t stop_resume_count; // 本段停稳/误差复核失败后恢复 RUNNING 的次数（限制反复补位）
    /* ===== 运行时整定参数（默认取 nav_config.h 宏，`nav par` 可在线修改） =====
     * 只放"控制律增益/速度阈值"这类现场调参常用量；逻辑/安全判据仍用编译期宏。
     * 初值在 Nav_Core_Init() 里统一从宏加载。 */
    float ff_weight; // 航向剖面馈权重 (NAV_YAW_FEEDFORWARD_WEIGHT)
    float fb_weight; // 航向 PID 反馈权重 (NAV_YAW_FEEDBACK_WEIGHT)
    float yaw_gain_far; // 远距离航向增益 (NAV_YAW_GAIN_FAR)
    float yaw_gain_fast; // 过弯快速跟随增益 (NAV_YAW_GAIN_FAST)
    float align_dist; // 航向远/近增益分界距离 (NAV_ALIGN_DIST)
    float xy_kp; // 位置 P 基础增益 (NAV_ALIGN_XY_KP)
    float xy_ki; // 保留兼容（新位置环无积分）(NAV_ALIGN_XY_KI)
    float near_gain; // 近目标位置增益倍率 (NAV_NEAR_TARGET_GAIN)
    float near_factor; // 近目标区半径倍率 (NAV_NEAR_TARGET_FACTOR)
    float traj_speed_cap; // 近目标区速度上限 (NAV_TRAJ_SPEED_CAP)
    float startup_speed; // 起步最小速度 (NAV_STARTUP_SPEED)
    float min_speed; // 保留兼容（起步下限统一用 startup_speed）(NAV_MIN_SPEED)
    float yaw_max_speed; // 航向环最大角速度 (NAV_ALIGN_YAW_MAX)
    float yaw_min_speed; // 航向环最小角速度 (NAV_ALIGN_YAW_MIN)
    float yaw_accel_limit; // 航向角加速度限幅/剖面减速度 (NAV_YAW_ACCEL_LIMIT)
    float yaw_d_filter_tau_s; // 航向 PID 微分低通时间常数 (NAV_YAW_PID_D_FILTER_TAU_S)
    uint32_t stop_settle_ms; // STOPPING 最少等待 (NAV_STOP_SETTLE_MS)
    bool use_meas_speed_decel; // 实测速度参与减速规划 (NAV_USE_MEAS_SPEED_DECEL)
    /* ===== OPS 位姿差分实测速度估计 ===== */
    bool est_valid; // 估计器是否已有有效基准
    Pose2D_t est_last_pose; // 上一次检测到位姿变化的位姿
    uint32_t est_last_pose_tick; // 上一次位姿变化的 tick
    uint32_t est_last_change_tick; // 最近一次检测到位姿变化的 tick
    float est_vx; // 机体系实测 X 速度 (cm/s)
    float est_vy; // 机体系实测 Y 速度 (cm/s)
    float est_speed; // 实测速度幅值 (cm/s)
    uint32_t est_age_ms; // 距最近一次位姿变化的毫秒数
    bool ops_stale_reported; // OPS 丢失错误是否已上报（避免每周期刷日志）
    /* ===== 调试快照（shell `nav trace` 读取；每周期刷新） ===== */
    NavDebug_t debug;
} NavCore_t;

static NavCore_t *g_nav_core = NULL;

/* 静态辅助函数声明 */
static bool get_current_pose(Pose2D_t *pose);
static void restore_motion_params(void);
static void pid_reset(PIDController_t *pid);
static float pid_update(PIDController_t *pid, float error, float dt);
static float calculate_distance(Pose2D_t a, Pose2D_t b);
static bool check_arrival(Pose2D_t current, TargetPoint_t *target);
static void compute_position_velocity(float dx, float dy, float distance, float current_yaw,
                                      const TargetPoint_t *target, float dt, float *out_vx, float *out_vy);
static float compute_yaw_speed(float yaw_error, float distance, float dt);
static void world_to_body(float dx_world, float dy_world, float yaw_deg, float *body_dx, float *body_dy);
static void apply_accel_limit(
    float desired_vx, float desired_vy, float *out_vx, float *out_vy, float acceleration, float deceleration, float dt);
static void update_measured_velocity(Pose2D_t pose, uint32_t now_tick);
static void reset_measured_velocity(void);
static void nav_debug_commit(
    Pose2D_t pose, TargetPoint_t *target, float distance, float yaw_error, float out_vx, float out_vy, float yaw_speed);

/**
 * @brief 获取当前位姿
 */
static bool get_current_pose(Pose2D_t *pose) {
    PoseTimestamp_t pt;
    if (!Loc_Get(&pt)) return false;
    pose->x = pt.pose.x;
    pose->y = pt.pose.y;
    pose->yaw = pt.pose.yaw;
    return true;
}

/**
 * @brief 恢复运动参数
 */
static void restore_motion_params(void) {
    MotionControl_SetMotionParams(
        g_nav_core->saved_linear_speed, g_nav_core->saved_yaw_speed, g_nav_core->saved_acc, g_nav_core->saved_dec);
}

/**
 * @brief 重置PID控制器
 */
static void pid_reset(PIDController_t *pid) {
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->d_filtered = 0.0f;
    pid->output = 0.0f;
}

/**
 * @brief 更新PID控制器输出（条件积分抗饱和 + 微分一阶低通）
 */
static float pid_update(PIDController_t *pid, float error, float dt) {
    if (dt <= 0.0f) dt = 1e-3f;

    float p_term = pid->kp * error;

    /* 条件积分(anti-windup)：输出已饱和且误差还在往同方向推时不继续积分，
     * 避免大角度初始误差把积分顶到限幅、yaw 冲到目标后来不及退积分造成过冲。 */
    bool saturated_high = (pid->output >= pid->output_limit) && (error > 0.0f);
    bool saturated_low = (pid->output <= -pid->output_limit) && (error < 0.0f);
    if (!saturated_high && !saturated_low) {
        pid->integral += error * dt;
    }

    if (pid->ki > 1e-9f) {
        float integral_limit = pid->output_limit / pid->ki;
        pid->integral = clamp(pid->integral, -integral_limit, integral_limit);
    } else {
        pid->integral = 0.0f;
    }
    float i_term = pid->ki * pid->integral;

    /* 微分项一阶低通: 5ms 周期下直接微分会把定位噪声放大成 yaw 抖动，滤波保留阻尼去掉噪声。 */
    float derivative = (error - pid->last_error) / dt;
    float d_tau = (g_nav_core != NULL) ? g_nav_core->yaw_d_filter_tau_s : NAV_YAW_PID_D_FILTER_TAU_S;
    if (d_tau > 0.0f) {
        float alpha = dt / (d_tau + dt);
        pid->d_filtered += (derivative - pid->d_filtered) * alpha;
    } else {
        pid->d_filtered = derivative;
    }
    float d_term = pid->kd * pid->d_filtered;

    pid->last_error = error;

    pid->output = p_term + i_term + d_term;
    pid->output = clamp(pid->output, -pid->output_limit, pid->output_limit);

    return pid->output;
}

/**
 * @brief 计算两个点之间的距离
 */
static float calculate_distance(Pose2D_t a, Pose2D_t b) {
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    return sqrtf(dx * dx + dy * dy);
}

/**
 * @brief 检查是否到达目标点（连续多次确认 + 指令/实测速度门控）
 */
static bool check_arrival(Pose2D_t current, TargetPoint_t *target) {
    bool distance_ok = false;
    bool yaw_ok = false;

    if (target->arrive.check_mode == ARRIVE_CHECK_DISTANCE || target->arrive.check_mode == ARRIVE_CHECK_BOTH) {
        float dist = calculate_distance(current, target->pose);
        distance_ok = (dist <= target->arrive.distance_threshold);
    } else {
        distance_ok = true;
    }

    /* 速度过大时不判定到达（防止高速穿堂而过）；指令速度与实测速度都要降到近目标上限以下，
     * 避免"指令已减到 0 但车体还在滑行"提前报到位。 */
    float current_speed = sqrtf(g_nav_core->last_vx * g_nav_core->last_vx + g_nav_core->last_vy * g_nav_core->last_vy);
    bool measured_ok = true;
    if (g_nav_core->est_valid && g_nav_core->est_age_ms <= NAV_MEAS_VEL_STALE_MS) {
        measured_ok = (g_nav_core->est_speed <= g_nav_core->traj_speed_cap);
    }
    bool speed_not_too_fast = (current_speed <= g_nav_core->traj_speed_cap) && measured_ok;

    if (target->arrive.check_mode == ARRIVE_CHECK_YAW || target->arrive.check_mode == ARRIVE_CHECK_BOTH) {
        float target_yaw_norm = normalize_angle(target->pose.yaw);
        float yaw_diff = fabsf(normalize_angle(target_yaw_norm - current.yaw));
        yaw_ok = (yaw_diff <= target->arrive.yaw_threshold);
    } else {
        yaw_ok = true;
    }

    if (distance_ok && yaw_ok && speed_not_too_fast) {
        g_nav_core->arrive_check_count++;
        if (g_nav_core->arrive_check_count >= NAV_ARRIVE_CHECK_COUNT) {
            return true;
        }
    } else {
        g_nav_core->arrive_check_count = 0;
    }

    return false;
}

/**
 * @brief 世界系到车身系坐标变换
 */
static void world_to_body(float dx_world, float dy_world, float yaw_deg, float *body_dx, float *body_dy) {
    float cos_yaw = cosf(yaw_deg * DEG_TO_RAD);
    float sin_yaw = sinf(yaw_deg * DEG_TO_RAD);
    *body_dx = dx_world * cos_yaw + dy_world * sin_yaw;
    *body_dy = -dx_world * sin_yaw + dy_world * cos_yaw;
}

/**
 * @brief 应用加减速限制（按分量限制速度变化量）
 */
static void apply_accel_limit(float desired_vx,
                              float desired_vy,
                              float *out_vx,
                              float *out_vy,
                              float acceleration,
                              float deceleration,
                              float dt) {
    float dvx = desired_vx - g_nav_core->last_vx;
    float dvy = desired_vy - g_nav_core->last_vy;
    float max_accel_change = acceleration * dt;
    float max_decel_change = deceleration * dt;

    if (dvx > 0) {
        dvx = clamp(dvx, -INFINITY, max_accel_change);
    } else {
        dvx = clamp(dvx, -max_decel_change, INFINITY);
    }
    if (dvy > 0) {
        dvy = clamp(dvy, -INFINITY, max_accel_change);
    } else {
        dvy = clamp(dvy, -max_decel_change, INFINITY);
    }

    g_nav_core->last_vx += dvx;
    g_nav_core->last_vy += dvy;
    *out_vx = g_nav_core->last_vx;
    *out_vy = g_nav_core->last_vy;
}

/**
 * @brief 统一位置环：滞后补偿制动上限 + 全误差向量 P
 *
 * 速度幅值 = min(kp*距离, 制动上限)，方向指向目标。制动上限解 v²/(2a)+v*lag=距离，
 * 把命令链/轮速滞后计入减速距离；实测速度高于指令时收紧制动距离（提前刹车）。
 * 远处制动上限主导（梯形巡航/减速），近处 P 主导（指数收敛），全程连续。
 */
static void compute_position_velocity(float dx, float dy, float distance, float current_yaw,
                                      const TargetPoint_t *target, float dt, float *out_vx, float *out_vy) {
    float kp = g_nav_core->xy_kp * g_nav_core->near_gain;
    float acc = target->motion.acceleration;
    float dec = target->motion.deceleration;
    float max_speed = target->motion.target_speed;
    dec = clamp(dec, 1.0f, INFINITY);
    acc = clamp(acc, 1.0f, INFINITY);

    float near_radius = target->arrive.distance_threshold * g_nav_core->near_factor;

    /* 滞后补偿制动上限 */
    float lag = NAV_VEL_LAG_S;
    float al = dec * lag;
    float d_brake = distance;

    /* 实测速度参与减速：实测高于指令的部分会在滞后期内多滑一段，从制动距离里扣除 */
    float cmd_speed = sqrtf(g_nav_core->last_vx * g_nav_core->last_vx + g_nav_core->last_vy * g_nav_core->last_vy);
    if (g_nav_core->use_meas_speed_decel && g_nav_core->est_valid &&
        g_nav_core->est_age_ms <= NAV_MEAS_VEL_STALE_MS) {
        float over = g_nav_core->est_speed - cmd_speed;
        if (over > 0.0f) d_brake -= over * lag;
    }
    d_brake = clamp(d_brake, 0.0f, INFINITY);

    float v_cap = sqrtf(al * al + 2.0f * dec * d_brake) - al;
    v_cap = clamp(v_cap, -INFINITY, max_speed);
    /* 复合门限，非单边界限幅，保持原比较顺序（R2-P1-02 不替换） */
    if (distance <= near_radius && v_cap > g_nav_core->traj_speed_cap) {
        v_cap = g_nav_core->traj_speed_cap;
    }

    /* 全误差向量 P + 制动上限 */
    float v_des = kp * distance;
    if (distance < NAV_POS_DEADBAND_CM) v_des = 0.0f;
    v_des = clamp(v_des, -INFINITY, v_cap);

    /* 起步下限：克服静摩擦/整数 RPM 量化，误差大于死区才生效 */
    if (v_des > 0.0f && v_des < g_nav_core->startup_speed && distance > NAV_POS_DEADBAND_CM) {
        v_des = g_nav_core->startup_speed;
        v_des = clamp(v_des, -INFINITY, v_cap);
    }

    float ux = (distance > 1e-3f) ? dx / distance : 0.0f;
    float uy = (distance > 1e-3f) ? dy / distance : 0.0f;
    float desired_vx = 0.0f, desired_vy = 0.0f;
    world_to_body(ux * v_des, uy * v_des, current_yaw, &desired_vx, &desired_vy);

    apply_accel_limit(desired_vx, desired_vy, out_vx, out_vy, acc, dec, dt);

    float speed = sqrtf(*out_vx * *out_vx + *out_vy * *out_vy);
    /* 多副作用缩放块，非纯 clamp，禁止替换（R2-P1-02 不替换） */
    if (speed > max_speed) {
        *out_vx = *out_vx / speed * max_speed;
        *out_vy = *out_vy / speed * max_speed;
        g_nav_core->last_vx = *out_vx;
        g_nav_core->last_vy = *out_vy;
        speed = max_speed;
    }
    g_nav_core->current_speed_magnitude = speed;
}

/**
 * @brief 航向环：sqrt 减速剖面馈 + PID 反馈修正
 *
 * 前馈 sign(err)*min(yaw_max, sqrt(2*a_yaw*|err|)) 远处饱和快转、近处平滑减速到 0；
 * PID 修正残差并提供阻尼。最小角速度钳位只在 |err| 较大时生效，近目标平滑收敛，无零交叉锁存。
 */
static float compute_yaw_speed(float yaw_error, float distance, float dt) {
    float far_gain = g_nav_core->yaw_fast ? g_nav_core->yaw_gain_fast : g_nav_core->yaw_gain_far;
    float gain_factor = (distance > g_nav_core->align_dist) ? far_gain : 1.0f;

    float abs_err = fabsf(yaw_error);
    float error_sign = (yaw_error > 0.0f) ? 1.0f : -1.0f;

    /* sqrt 减速剖面馈 */
    float w_profile = sqrtf(2.0f * g_nav_core->yaw_accel_limit * abs_err);
    w_profile = clamp(w_profile, -INFINITY, g_nav_core->yaw_max_speed);
    w_profile *= error_sign;

    /* PID 反馈修正 */
    float w_pid = pid_update(&g_nav_core->yaw_pid, yaw_error, dt);

    float yaw_speed = (w_profile * g_nav_core->ff_weight + w_pid * g_nav_core->fb_weight) * gain_factor;
    yaw_speed = clamp(yaw_speed, -g_nav_core->yaw_max_speed, g_nav_core->yaw_max_speed);

    if (abs_err < NAV_YAW_DEADBAND) {
        /* 已在航向死区（=到达门限）：直接停转并清积分/上一拍角速度，避免残余 momentum 冲过门限 */
        g_nav_core->yaw_pid.integral = 0.0f;
        g_nav_core->last_yaw_speed = 0.0f;
        return 0.0f;
    }

    /* 静摩擦下限只在大误差时生效，近目标允许平滑收敛到 0 */
    if (abs_err > NAV_YAW_MIN_SPEED_ERR_THRESH && fabsf(yaw_speed) < g_nav_core->yaw_min_speed) {
        yaw_speed = error_sign * g_nav_core->yaw_min_speed;
    }

    float max_yaw_accel = g_nav_core->yaw_accel_limit * dt;
    float dyaw_speed = clamp(yaw_speed - g_nav_core->last_yaw_speed, -max_yaw_accel, max_yaw_accel);
    g_nav_core->last_yaw_speed += dyaw_speed;
    return g_nav_core->last_yaw_speed;
}

/**
 * @brief 重置实测速度估计器（开始新一段导航/停车时调用）
 */
static void reset_measured_velocity(void) {
    if (g_nav_core == NULL) return;
    g_nav_core->est_valid = false;
    g_nav_core->est_last_pose.x = 0.0f;
    g_nav_core->est_last_pose.y = 0.0f;
    g_nav_core->est_last_pose.yaw = 0.0f;
    g_nav_core->est_last_pose_tick = 0;
    g_nav_core->est_last_change_tick = 0;
    g_nav_core->est_vx = 0.0f;
    g_nav_core->est_vy = 0.0f;
    g_nav_core->est_speed = 0.0f;
    g_nav_core->est_age_ms = NAV_MEAS_VEL_STALE_MS + 1;
}

/**
 * @brief 用 OPS 位姿差分更新实测速度估计（机体系，cm/s）
 *
 * 只有检测到位姿确实变化（超过量化噪声阈值）才更新：OPS 数据率低于控制周期时，
 * 绝大多数控制拍位姿相同，直接差分会得到 0，导致速度估计被拉低。
 * 位姿不变超过 NAV_MEAS_VEL_STALE_MS 时认为已静止，速度清零。
 */
static void update_measured_velocity(Pose2D_t pose, uint32_t now_tick) {
    if (g_nav_core == NULL) return;

    if (!g_nav_core->est_valid) {
        g_nav_core->est_last_pose = pose;
        g_nav_core->est_last_pose_tick = now_tick;
        g_nav_core->est_last_change_tick = now_tick;
        g_nav_core->est_valid = true;
        g_nav_core->est_age_ms = 0;
        return;
    }

    float dx = pose.x - g_nav_core->est_last_pose.x;
    float dy = pose.y - g_nav_core->est_last_pose.y;
    float dyaw = normalize_angle(pose.yaw - g_nav_core->est_last_pose.yaw);

    if (fabsf(dx) > NAV_MEAS_POS_EPS_CM || fabsf(dy) > NAV_MEAS_POS_EPS_CM || fabsf(dyaw) > NAV_MEAS_YAW_EPS_DEG) {
        uint32_t dt_ms = now_tick - g_nav_core->est_last_pose_tick;
        if (dt_ms >= 1 && dt_ms <= NAV_MEAS_VEL_STALE_MS) {
            float dt = (float)dt_ms * 0.001f;
            float yaw_rad = pose.yaw * DEG_TO_RAD;
            float c = cosf(yaw_rad);
            float s = sinf(yaw_rad);
            float vx_world = dx / dt;
            float vy_world = dy / dt;
            float vx_body = vx_world * c + vy_world * s;
            float vy_body = -vx_world * s + vy_world * c;

            float alpha = dt / (NAV_MEAS_VEL_TAU_S + dt);
            g_nav_core->est_vx += (vx_body - g_nav_core->est_vx) * alpha;
            g_nav_core->est_vy += (vy_body - g_nav_core->est_vy) * alpha;
        }
        g_nav_core->est_last_pose = pose;
        g_nav_core->est_last_pose_tick = now_tick;
        g_nav_core->est_last_change_tick = now_tick;
    }

    uint32_t age = now_tick - g_nav_core->est_last_change_tick;
    g_nav_core->est_age_ms = age;
    if (age > NAV_MEAS_VEL_STALE_MS) {
        g_nav_core->est_vx = 0.0f;
        g_nav_core->est_vy = 0.0f;
    }
    g_nav_core->est_speed = sqrtf(g_nav_core->est_vx * g_nav_core->est_vx + g_nav_core->est_vy * g_nav_core->est_vy);
}

/**
 * @brief 刷新调试快照（`nav trace` 读取；只在控制任务里写）
 */
static void nav_debug_commit(
    Pose2D_t pose, TargetPoint_t *target, float distance, float yaw_error, float out_vx, float out_vy, float yaw_speed) {
    if (g_nav_core == NULL) return;
    NavDebug_t *d = &g_nav_core->debug;
    d->state = (uint8_t)g_nav_core->state;
    d->running = (g_nav_core->state == NAV_STATE_RUNNING && g_nav_core->cached_target != NULL) ? 1 : 0;
    d->pose = pose;
    if (target != NULL) {
        d->target = *target;
    } else {
        memset(&d->target, 0, sizeof(d->target));
    }
    d->distance = distance;
    d->yaw_error = yaw_error;
    d->plan_speed = g_nav_core->current_speed_magnitude;
    d->cmd_vx = out_vx;
    d->cmd_vy = out_vy;
    d->cmd_w = yaw_speed;
    d->meas_vx = g_nav_core->est_vx;
    d->meas_vy = g_nav_core->est_vy;
    d->meas_speed = g_nav_core->est_speed;
    d->meas_age_ms = g_nav_core->est_age_ms;
    d->arrive_count = g_nav_core->arrive_check_count;
    d->task_elapsed_ms = osKernelGetTickCount() - g_nav_core->task_start_tick;
}

/**
 * @brief 读取导航调试快照
 */
void Nav_GetDebug(NavDebug_t *out) {
    if (out == NULL) return;
    memset(out, 0, sizeof(*out));
    if (g_nav_core == NULL) return;
    taskENTER_CRITICAL();
    *out = g_nav_core->debug;
    taskEXIT_CRITICAL();
}

/**
 * @brief 初始化导航核心控制器
 */
bool Nav_Core_Init(void) {
    g_nav_core = pvPortMalloc(sizeof(NavCore_t));
    if (g_nav_core == NULL) {
        return false;
    }
    memset(g_nav_core, 0, sizeof(NavCore_t));

    g_nav_core->state = NAV_STATE_IDLE;
    g_nav_core->yaw_pid.kp = NAV_YAW_PID_KP;
    g_nav_core->yaw_pid.ki = NAV_YAW_PID_KI;
    g_nav_core->yaw_pid.kd = NAV_YAW_PID_KD;
    g_nav_core->yaw_pid.output_limit = NAV_YAW_PID_OUTPUT_LIMIT;

    /* 运行时整定参数默认全部取 nav_config.h 的宏，现场用 `nav par <name> <value>` 在线修改。 */
    g_nav_core->ff_weight = NAV_YAW_FEEDFORWARD_WEIGHT;
    g_nav_core->fb_weight = NAV_YAW_FEEDBACK_WEIGHT;
    g_nav_core->yaw_gain_far = NAV_YAW_GAIN_FAR;
    g_nav_core->yaw_gain_fast = NAV_YAW_GAIN_FAST;
    g_nav_core->align_dist = NAV_ALIGN_DIST;
    g_nav_core->xy_kp = NAV_ALIGN_XY_KP;
    g_nav_core->xy_ki = NAV_ALIGN_XY_KI;
    g_nav_core->near_gain = NAV_NEAR_TARGET_GAIN;
    g_nav_core->near_factor = NAV_NEAR_TARGET_FACTOR;
    g_nav_core->traj_speed_cap = NAV_TRAJ_SPEED_CAP;
    g_nav_core->startup_speed = NAV_STARTUP_SPEED;
    g_nav_core->min_speed = NAV_MIN_SPEED;
    g_nav_core->yaw_max_speed = NAV_ALIGN_YAW_MAX;
    g_nav_core->yaw_min_speed = NAV_ALIGN_YAW_MIN;
    g_nav_core->yaw_accel_limit = NAV_YAW_ACCEL_LIMIT;
    g_nav_core->yaw_d_filter_tau_s = NAV_YAW_PID_D_FILTER_TAU_S;
    g_nav_core->stop_settle_ms = NAV_STOP_SETTLE_MS;
    g_nav_core->use_meas_speed_decel = (NAV_USE_MEAS_SPEED_DECEL != 0);
    reset_measured_velocity();

    return true;
}

/**
 * @brief 读取当前整定参数
 */
void Nav_GetTunables(NavTunables_t *out) {
    if (out == NULL) return;
    memset(out, 0, sizeof(*out));
    if (g_nav_core == NULL) return;

    out->yaw_kp = g_nav_core->yaw_pid.kp;
    out->yaw_ki = g_nav_core->yaw_pid.ki;
    out->yaw_kd = g_nav_core->yaw_pid.kd;
    out->yaw_d_filter_tau_s = g_nav_core->yaw_d_filter_tau_s;
    out->ff_weight = g_nav_core->ff_weight;
    out->fb_weight = g_nav_core->fb_weight;
    out->yaw_gain_far = g_nav_core->yaw_gain_far;
    out->yaw_gain_fast = g_nav_core->yaw_gain_fast;
    out->align_dist = g_nav_core->align_dist;
    out->xy_kp = g_nav_core->xy_kp;
    out->xy_ki = g_nav_core->xy_ki;
    out->near_gain = g_nav_core->near_gain;
    out->near_factor = g_nav_core->near_factor;
    out->traj_speed_cap = g_nav_core->traj_speed_cap;
    out->startup_speed = g_nav_core->startup_speed;
    out->min_speed = g_nav_core->min_speed;
    out->yaw_max_speed = g_nav_core->yaw_max_speed;
    out->yaw_min_speed = g_nav_core->yaw_min_speed;
    out->yaw_accel_limit = g_nav_core->yaw_accel_limit;
    out->stop_settle_ms = g_nav_core->stop_settle_ms;
}

/**
 * @brief 运行时修改一个整定参数（带安全范围检查）
 */
bool Nav_SetTunable(const char *name, float value) {
    if (g_nav_core == NULL || name == NULL) return false;
    if (!(value == value) || value > 1.0e6f || value < -1.0e6f) return false; // NaN/Inf/离谱值

    if (strcmp(name, "yaw_kp") == 0) {
        if (value < 0.0f || value > 50.0f) return false;
        g_nav_core->yaw_pid.kp = value;
    } else if (strcmp(name, "yaw_ki") == 0) {
        if (value < 0.0f || value > 20.0f) return false;
        g_nav_core->yaw_pid.ki = value;
    } else if (strcmp(name, "yaw_kd") == 0) {
        if (value < 0.0f || value > 50.0f) return false;
        g_nav_core->yaw_pid.kd = value;
    } else if (strcmp(name, "yaw_d_filter_tau_s") == 0) {
        if (value < 0.0f || value > 0.5f) return false;
        g_nav_core->yaw_d_filter_tau_s = value;
    } else if (strcmp(name, "ff_weight") == 0) {
        if (value < 0.0f || value > 1.0f) return false;
        g_nav_core->ff_weight = value;
    } else if (strcmp(name, "fb_weight") == 0) {
        if (value < 0.0f || value > 2.0f) return false;
        g_nav_core->fb_weight = value;
    } else if (strcmp(name, "yaw_gain_far") == 0) {
        if (value < 0.05f || value > 2.0f) return false;
        g_nav_core->yaw_gain_far = value;
    } else if (strcmp(name, "yaw_gain_fast") == 0) {
        if (value < 0.05f || value > 2.0f) return false;
        g_nav_core->yaw_gain_fast = value;
    } else if (strcmp(name, "align_dist") == 0) {
        if (value < 0.5f || value > 100.0f) return false;
        g_nav_core->align_dist = value;
    } else if (strcmp(name, "xy_kp") == 0) {
        if (value < 0.0f || value > 20.0f) return false;
        g_nav_core->xy_kp = value;
    } else if (strcmp(name, "xy_ki") == 0) {
        if (value < 0.0f || value > 5.0f) return false;
        g_nav_core->xy_ki = value;
    } else if (strcmp(name, "near_gain") == 0) {
        if (value < 0.1f || value > 20.0f) return false;
        g_nav_core->near_gain = value;
    } else if (strcmp(name, "near_factor") == 0) {
        if (value < 1.0f || value > 100.0f) return false;
        g_nav_core->near_factor = value;
    } else if (strcmp(name, "traj_speed_cap") == 0) {
        if (value < 5.0f || value > 200.0f) return false;
        g_nav_core->traj_speed_cap = value;
    } else if (strcmp(name, "startup_speed") == 0) {
        if (value < 0.1f || value > 20.0f) return false;
        g_nav_core->startup_speed = value;
    } else if (strcmp(name, "min_speed") == 0) {
        if (value < 0.0f || value > 20.0f) return false;
        g_nav_core->min_speed = value;
    } else if (strcmp(name, "yaw_max_speed") == 0) {
        if (value < 20.0f || value > 300.0f) return false;
        g_nav_core->yaw_max_speed = value;
    } else if (strcmp(name, "yaw_min_speed") == 0) {
        if (value < 0.5f || value > 30.0f) return false;
        g_nav_core->yaw_min_speed = value;
    } else if (strcmp(name, "yaw_accel_limit") == 0) {
        if (value < 10.0f || value > 5000.0f) return false;
        g_nav_core->yaw_accel_limit = value;
    } else if (strcmp(name, "stop_settle_ms") == 0) {
        if (value < 20.0f || value > 2000.0f) return false;
        g_nav_core->stop_settle_ms = (uint32_t)(value + 0.5f);
    } else {
        return false;
    }
    return true;
}

/**
 * @brief 导航到指定ID点
 */
bool Nav_GoTo(uint8_t target_id) {
    if (g_nav_core == NULL) return false;
    TargetPoint_t *target = Map_GetPoint(target_id);
    if (target == NULL) return false;

    return Nav_GoToDirect(target);
}

/**
 * @brief 导航到指定名称点
 */
bool Nav_GoTo_fromName(const char *name) {
    if (g_nav_core == NULL) return false;
    TargetPoint_t *target = Map_GetPointByName(name);
    if (target == NULL) return false;

    return Nav_GoToDirect(target);
}

/**
 * @brief 导航到指定目标点
 */
bool Nav_GoToDirect(TargetPoint_t *target) {
    if (g_nav_core == NULL || target == NULL) return false;

    MotionControl_GetMotionParams(
        &g_nav_core->saved_linear_speed, &g_nav_core->saved_yaw_speed, &g_nav_core->saved_acc, &g_nav_core->saved_dec);

    /* 加速度/减速度一并下发给底盘速度环: 速度模式下底盘加速度参数换算成电机加速度档位，
     * 传 0 会让电机用最慢档加速、速度环跟不上导航指令。 */
    MotionControl_SetMotionParams(target->motion.target_speed,
                                  target->motion.target_angular_speed,
                                  target->motion.acceleration,
                                  target->motion.deceleration);

    g_nav_core->cached_target = target;
    g_nav_core->state = NAV_STATE_RUNNING;
    g_nav_core->task_start_tick = osKernelGetTickCount();
    g_nav_core->last_control_tick = osKernelGetTickCount();
    g_nav_core->arrive_timeout_tick = g_nav_core->task_start_tick + target->arrive.timeout_ms;
    g_nav_core->last_vx = 0.0f;
    g_nav_core->last_vy = 0.0f;
    g_nav_core->last_yaw_speed = 0.0f;
    g_nav_core->current_speed_magnitude = 0.0f;
    g_nav_core->arrive_check_count = 0;
    g_nav_core->yaw_fast = false; // 普通导航段复位快速跟随（过弯由任务层按需置位）
    g_nav_core->ops_stale_reported = false;
    memset(&g_nav_core->stop_win_pose, 0, sizeof(g_nav_core->stop_win_pose));
    g_nav_core->stop_win_tick = 0;
    g_nav_core->stop_sliding = false;
    g_nav_core->stop_resume_count = 0;
    memset(&g_nav_core->debug, 0, sizeof(g_nav_core->debug));
    reset_measured_velocity();

    logInfo("Direct navigation to: x: %.2f, y: %.2f, yaw: %.2f, name: %s",
            target->pose.x,
            target->pose.y,
            target->pose.yaw,
            target->name);
    pid_reset(&g_nav_core->yaw_pid);
    return true;
}

/**
 * @brief 运行中直接切换目标点（不停车、不打断行驶）
 * @note  保留速度状态（last_vx/last_vy/last_yaw_speed）与加减速限制的连续性；
 *        只重置与"本段终点"相关的状态：到达计数、yaw PID。
 */
bool Nav_Retarget(TargetPoint_t *target) {
    if (g_nav_core == NULL || target == NULL) return false;
    if (g_nav_core->state != NAV_STATE_RUNNING) return false;

    /* 不停车换目标: 同步更新底盘速度环参数 */
    MotionControl_SetMotionParams(target->motion.target_speed,
                                  target->motion.target_angular_speed,
                                  target->motion.acceleration,
                                  target->motion.deceleration);

    g_nav_core->cached_target = target;
    g_nav_core->arrive_timeout_tick = osKernelGetTickCount() + target->arrive.timeout_ms;
    g_nav_core->arrive_check_count = 0;
    g_nav_core->current_speed_magnitude = 0.0f;

    /* yaw PID：清积分并把 last_error 预置为切换瞬间的 yaw 误差，
     * 避免目标 yaw 跳变（如 90° -> 0°）时微分项打出尖峰。 */
    pid_reset(&g_nav_core->yaw_pid);
    {
        Pose2D_t pose;
        if (get_current_pose(&pose)) {
            float yaw_error = normalize_angle(normalize_angle(target->pose.yaw) - pose.yaw);
            g_nav_core->yaw_pid.last_error = yaw_error;
        }
    }

    logInfo("Nav retarget (keep moving): x: %.2f, y: %.2f, yaw: %.2f, name: %s",
            target->pose.x,
            target->pose.y,
            target->pose.yaw,
            target->name);
    return true;
}

/**
 * @brief 设置/清除"yaw 快速跟随"标志
 */
void Nav_SetYawFast(bool enable) {
    if (g_nav_core == NULL) return;
    g_nav_core->yaw_fast = enable;
}

/**
 * @brief 停止导航
 */
void Nav_Stop(void) {
    if (g_nav_core == NULL) return;

    MotionControl_Stop();
    g_nav_core->state = NAV_STATE_IDLE;
    g_nav_core->cached_target = NULL;
    g_nav_core->last_vx = 0.0f;
    g_nav_core->last_vy = 0.0f;
    g_nav_core->yaw_fast = false;
    g_nav_core->ops_stale_reported = false;
    g_nav_core->debug.state = (uint8_t)NAV_STATE_IDLE;
    g_nav_core->debug.running = 0;
    reset_measured_velocity();
    restore_motion_params();
}

/**
 * @brief 获取导航状态
 */
NavState_t Nav_GetState(void) {
    if (g_nav_core == NULL) return NAV_STATE_IDLE;
    return g_nav_core->state;
}

/**
 * @brief 导航任务（独立 RTOS 线程，每 NAV_UPDATE_TIME 调用一次 Nav_Update）
 */
void Nav_Task(void *argument) {
    (void)argument;

    osEventFlagsWait(System_StatusHandle, SYS_INIT_COMPLETE, osFlagsNoClear, osWaitForever);
    if (g_nav_core == NULL) osThreadExit();

    for (;;) {
        osDelay(NAV_UPDATE_TIME);
        Nav_Update();
    }
}

/**
 * @brief 执行一个导航控制周期（非阻塞；由 Nav_Task 每 5ms 调用，主机仿真可直接调用）
 */
void Nav_Update(void) {
    if (g_nav_core == NULL) return;

    Pose2D_t current_pose;
    float out_vx = 0.0f, out_vy = 0.0f, yaw_speed = 0.0f;

    if (g_nav_core->state == NAV_STATE_IDLE) {
        g_nav_core->last_control_tick = osKernelGetTickCount();
        return;
    }

    /* OPS 数据新鲜度保护：RUNNING/STOPPING 期间 OPS 断线/丢帧立即停车报错。 */
    if ((g_nav_core->state == NAV_STATE_RUNNING || g_nav_core->state == NAV_STATE_STOPPING) &&
        OPS_Is_Stale(NAV_OPS_STALE_TIMEOUT_MS)) {
        if (!g_nav_core->ops_stale_reported) {
            logError("Navigation aborted: OPS frame stale (>%u ms)", (unsigned)NAV_OPS_STALE_TIMEOUT_MS);
            g_nav_core->ops_stale_reported = true;
        }
        MotionControl_Stop();
        restore_motion_params();
        g_nav_core->state = NAV_STATE_ERROR;
        g_nav_core->last_vx = 0.0f;
        g_nav_core->last_vy = 0.0f;
        g_nav_core->last_yaw_speed = 0.0f;
        g_nav_core->current_speed_magnitude = 0.0f;
        g_nav_core->debug.state = (uint8_t)NAV_STATE_ERROR;
        g_nav_core->debug.running = 0;
        osEventFlagsSet(System_StatusHandle, NAV_EVT_ERROR);
        return;
    }

    if (!get_current_pose(&current_pose)) {
        logError("Failed to get current pose");
        return;
    }

    uint32_t current_tick = osKernelGetTickCount();
    float dt = (current_tick - g_nav_core->last_control_tick) / 1000.0f;
    g_nav_core->last_control_tick = current_tick;

    /* 实测速度估计：OPS 位姿差分（供到达门控/减速规划/诊断使用），STOPPING 阶段也更新。 */
    update_measured_velocity(current_pose, current_tick);

    if (g_nav_core->state == NAV_STATE_RUNNING && g_nav_core->cached_target != NULL) {
        TargetPoint_t *target = g_nav_core->cached_target;
        float dx = target->pose.x - current_pose.x;
        float dy = target->pose.y - current_pose.y;
        float distance = calculate_distance(current_pose, target->pose);

        if (check_arrival(current_pose, target)) {
            MotionControl_Stop();
            g_nav_core->state = NAV_STATE_STOPPING;
            g_nav_core->stop_start_tick = current_tick;
            g_nav_core->stop_win_pose = current_pose;
            g_nav_core->stop_win_tick = current_tick;
            g_nav_core->stop_sliding = false;
            g_nav_core->stop_resume_count = 0;
            g_nav_core->last_vx = 0.0f;
            g_nav_core->last_vy = 0.0f;
            g_nav_core->current_speed_magnitude = 0.0f;
            return;
        }

        if (current_tick > g_nav_core->arrive_timeout_tick) {
            MotionControl_Stop();
            /* 先恢复运动参数再切状态：调用方看到 ERROR 会立刻发起下一次导航 */
            restore_motion_params();
            g_nav_core->state = NAV_STATE_ERROR;
            g_nav_core->last_vx = 0.0f;
            g_nav_core->last_vy = 0.0f;
            g_nav_core->current_speed_magnitude = 0.0f;
            logError("Navigation timeout");
            osEventFlagsSet(System_StatusHandle, NAV_EVT_ERROR);
            return;
        }

        /* 统一位置环 + 航向环 */
        compute_position_velocity(dx, dy, distance, current_pose.yaw, target, dt, &out_vx, &out_vy);

        float target_yaw_norm = normalize_angle(target->pose.yaw);
        float yaw_error = normalize_angle(target_yaw_norm - current_pose.yaw);
        yaw_speed = compute_yaw_speed(yaw_error, distance, dt);

        if (g_nav_core->state != NAV_STATE_RUNNING) {
            out_vx = out_vy = yaw_speed = 0.0f;
            MotionControl_Stop();
        }

        MotionControl_SetVelocity(out_vx, out_vy, yaw_speed);

        /* 刷新调试快照（nav trace 读取）：放在下发指令之后，拿到的是本拍真实输出 */
        nav_debug_commit(current_pose, target, distance, yaw_error, out_vx, out_vy, yaw_speed);
    }

    if (g_nav_core->state == NAV_STATE_STOPPING && g_nav_core->cached_target != NULL) {
        TargetPoint_t *stop_target = g_nav_core->cached_target;
        uint32_t elapsed_ms = current_tick - g_nav_core->stop_start_tick;

        /* 停稳检测（滑动窗口）：每 NAV_STOP_WINDOW_MS 采一次位姿，只比较窗口内位移/转角。
         * 不能用"进入 STOPPING 以来的累计位移"：OPS 延迟会把减速正常滑行误判成"还在动"。 */
        if ((current_tick - g_nav_core->stop_win_tick) >= NAV_STOP_WINDOW_MS) {
            float win_moved = calculate_distance(current_pose, g_nav_core->stop_win_pose);
            float win_yaw = fabsf(normalize_angle(current_pose.yaw - g_nav_core->stop_win_pose.yaw));
            g_nav_core->stop_sliding =
                (win_moved > NAV_STOP_WINDOW_MOVE_CM) || (win_yaw > NAV_STOP_WINDOW_YAW_DEG);
            g_nav_core->stop_win_pose = current_pose;
            g_nav_core->stop_win_tick = current_tick;
        }

        /* 终判前要求 OPS 位姿足够新，避免用停车前的旧帧判"已停稳/已到位"。 */
        bool pose_fresh = (OPS_GetFrameAgeMs() <= (uint32_t)NAV_STOP_FRESH_OPS_MS);
        bool settled = (!g_nav_core->stop_sliding) && pose_fresh;
        bool extra_wait_done = (elapsed_ms >= g_nav_core->stop_settle_ms + NAV_STOP_EXTRA_WAIT_MS);

        if (elapsed_ms < g_nav_core->stop_settle_ms || (!settled && !extra_wait_done)) {
            return;
        }

        float final_distance = calculate_distance(current_pose, stop_target->pose);
        float final_yaw_error = 0.0f;
        bool yaw_ok = true;
        if (stop_target->arrive.check_mode == ARRIVE_CHECK_YAW ||
            stop_target->arrive.check_mode == ARRIVE_CHECK_BOTH) {
            final_yaw_error = normalize_angle(normalize_angle(stop_target->pose.yaw) - current_pose.yaw);
            yaw_ok = (fabsf(final_yaw_error) <= stop_target->arrive.yaw_threshold);
        }
        bool dist_ok = (final_distance <= stop_target->arrive.distance_threshold);

        /* 反复补位保护：用尽 NAV_STOP_MAX_RESUME 次后，距离达标就按到位处理（航向只告警）。 */
        bool give_up_align = (g_nav_core->stop_resume_count >= NAV_STOP_MAX_RESUME);

        if (dist_ok && yaw_ok) {
            logInfo("Arrived at target");
            /* 判到达瞬间那次 Stop 可能因电机命令队列满被丢弃；停稳后补发一次停止。 */
            MotionControl_Stop();
            restore_motion_params();
            g_nav_core->state = NAV_STATE_COMPLETE;
            g_nav_core->last_vx = 0.0f;
            g_nav_core->last_vy = 0.0f;
            g_nav_core->last_yaw_speed = 0.0f;
            g_nav_core->current_speed_magnitude = 0.0f;
            nav_debug_commit(current_pose, stop_target, final_distance, final_yaw_error, 0, 0, 0);
            osEventFlagsSet(System_StatusHandle, NAV_EVT_DONE);
        } else if (dist_ok && give_up_align) {
            logWarning("Navigation: yaw still off after %u resume(s) (dist %.2fcm, yaw %.2fdeg), accept",
                       (unsigned)g_nav_core->stop_resume_count,
                       (double)final_distance,
                       (double)final_yaw_error);
            MotionControl_Stop();
            restore_motion_params();
            g_nav_core->state = NAV_STATE_COMPLETE;
            g_nav_core->last_vx = 0.0f;
            g_nav_core->last_vy = 0.0f;
            g_nav_core->last_yaw_speed = 0.0f;
            g_nav_core->current_speed_magnitude = 0.0f;
            nav_debug_commit(current_pose, stop_target, final_distance, final_yaw_error, 0, 0, 0);
            osEventFlagsSet(System_StatusHandle, NAV_EVT_DONE);
        } else {
            /* 位置/航向没到位（或仍在滑移）：恢复 RUNNING 闭环补位，收敛不了由导航超时兜底。 */
            g_nav_core->stop_resume_count++;
            logWarning("Navigation: stop check failed (dist %.2fcm, yaw %.2fdeg%s), resume %u/%u",
                       (double)final_distance,
                       (double)final_yaw_error,
                       settled ? "" : ", not settled",
                       (unsigned)g_nav_core->stop_resume_count,
                       (unsigned)NAV_STOP_MAX_RESUME);
            MotionControl_Stop();
            g_nav_core->state = NAV_STATE_RUNNING;
            g_nav_core->current_speed_magnitude = 0.0f;
            g_nav_core->arrive_check_count = 0;
            g_nav_core->arrive_timeout_tick = current_tick + stop_target->arrive.timeout_ms;
            nav_debug_commit(current_pose, stop_target, final_distance, final_yaw_error, 0, 0, 0);
        }
    }
}