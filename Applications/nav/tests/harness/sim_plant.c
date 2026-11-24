/**
 * @file sim_plant.c
 * @brief 主机端 nav 仿真：麦克纳姆轮底盘 + 地面里程被控对象模型
 *
 * 模型链路（由内向外）：
 *   1) 逆运动学：直接调用真实 Applications/chassis/kinematics/kinematics.c，保证
 *      "导航下发机体系速度 -> 四轮角速度" 与固件完全一致（含旋转中心偏移）。
 *   2) 整数 RPM 量化：复刻真实 MotionControl 的行为——速度模式命令按
 *      `(uint16_t)(|rad/s| * 60 / 2π)` 截断成整数 RPM 下发（低速度分辨率约 0.38cm/s 一档）。
 *   3) 电机加速度斜坡：Emm 固件速度模式的 acc 档位折算成电机 RPM/s，对应轮角加速度上限
 *      acc_car/R（R=轮半径）。由 nav 当拍下发的 car_acc 决定（与真实命令链一致）。
 *   4) 轮速一阶滞后 tau：机械传动 + 驱动器速度环的等效滞后。
 *   5) 正运动学还原机体系速度（本文件推导，与 kinematics.c 的逆解严格互逆）。
 *   6) 轮地打滑：地面速度 = 轮速推算速度 * (1 - slip)。
 *   7) 位姿积分：世界系 x/y 与连续航向角（不归一化，便于统计 180° 旋转过冲）。
 */

#include "sim_plant.h"
#include "kinematics.h"
#include "chassis_config.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define PLANT_HIST 2048 /* 真实位姿历史长度 (ms)，需覆盖 OPS 最大测量延迟 */

typedef struct {
    float x;
    float y;
    float yaw;
} PlantPose_t;

static const SimPlantCfg *s_cfg = NULL;
static uint32_t s_start_tick = 0;
static uint32_t s_tick = 0;

static PlantPose_t s_hist[PLANT_HIST];

/* 地面真实状态 */
static float s_x, s_y, s_yaw; /* 连续 yaw（可超出 ±180） */
static float s_vx_body, s_vy_body, s_w_dps;
static float s_wheel_act[4]; /* 实际轮角速度 rad/s，下标 fl/fr/rl/rr */
static float s_wheel_ramp[4]; /* 电机斜坡后的目标轮角速度 */

static float deg2rad_f(float d) {
    return d * (float)M_PI / 180.0f;
}

static float rad2deg_f(float r) {
    return r * 180.0f / (float)M_PI;
}

/* 复刻 MotionControl: 轮角速度 (rad/s) -> 整数 RPM（截断），再换算回 rad/s。
 * 真实代码为 uint16_t 截断，这里保留大小与符号。 */
static float quantize_wheel_rpm(float rad_s) {
    float rpm = fabsf(rad_s) * 60.0f / (2.0f * (float)M_PI);
    float rpm_i = (float)((unsigned int)rpm); /* 截断，与 (uint16_t) 强转一致（转速远小于 65535） */
    float out = rpm_i * 2.0f * (float)M_PI / 60.0f;
    return (rad_s >= 0.0f) ? out : -out;
}

void SimCfg_Default(SimPlantCfg *cfg) {
    memset(cfg, 0, sizeof(*cfg));
    /* 电机命令链：默认与"修好总线后"的估计一致（<10ms），delay 由套件覆写 */
    cfg->cmd_delay_ms = 0.0f;
    cfg->wheel_tau_s = 0.060f; /* 轮速一阶滞后 60ms（机械+速度环，任务要求默认 40~80ms） */
    cfg->rpm_quantize = 1; /* 复刻真实整数 RPM 命令量化 */
    cfg->motor_acc_limit = 1; /* 电机驱动器加速度斜坡 */
    cfg->motor_acc_scale = 1.0f;
    /* STOP 命令（0xFE 立即停止）：默认等效 4000cm/s² + 20ms 残余滑移，约 1cm 停车距离 */
    cfg->stop_decel_cm_s2 = 4000.0f;
    cfg->stop_tau_s = 0.020f;
    /* 地面 */
    cfg->slip = 0.0f;
    /* OPS 测量 */
    cfg->ops_rate_hz = 50.0f;
    cfg->ops_delay_ms = 20.0f;
    cfg->ops_pos_quant_cm = 0.01f; /* 0.1mm */
    cfg->ops_yaw_quant_deg = 0.01f;
    cfg->ops_pos_noise_cm = 0.0f;
    cfg->ops_yaw_noise_deg = 0.05f;
    cfg->seed = 20261113u;
    /* 控制回路时序 */
    cfg->nav_period_ms = 5;
    cfg->jitter_max_ms = 0;
    cfg->jitter_prob = 0.0f;
    cfg->max_time_ms = 15000;
    cfg->hold_after_complete_ms = 500;
}

void Plant_Reset(const SimPlantCfg *cfg, uint32_t start_tick, float x0, float y0, float yaw0) {
    int i;

    s_cfg = cfg;
    s_start_tick = start_tick;
    s_tick = start_tick;
    s_x = x0;
    s_y = y0;
    s_yaw = yaw0;
    s_vx_body = 0.0f;
    s_vy_body = 0.0f;
    s_w_dps = 0.0f;
    for (i = 0; i < 4; i++) {
        s_wheel_act[i] = 0.0f;
        s_wheel_ramp[i] = 0.0f;
    }

    /* 整个历史环都填成初始位姿：延迟采样在场景开始前不会读到上一个场景的残留 */
    for (i = 0; i < PLANT_HIST; i++) {
        s_hist[i].x = x0;
        s_hist[i].y = y0;
        s_hist[i].yaw = yaw0;
    }
    s_hist[start_tick % PLANT_HIST].x = x0;
    s_hist[start_tick % PLANT_HIST].y = y0;
    s_hist[start_tick % PLANT_HIST].yaw = yaw0;
}

/**
 * @brief 正运动学：四轮角速度 -> 机体系速度（kinematics.c 逆解的严格逆）
 *
 * kinematics.c 逆解（旋转中心偏移 cx=CHASSIS_ROTATION_CENTER_OFFSET_X 等）：
 *   FL 轮 = (vx - vy - l_fl*w)/R, FR 轮 = (vx + vy + l_fr*w)/R
 *   RL 轮 = (vx + vy - l_rl*w)/R, RR 轮 = (vx - vy + l_rr*w)/R
 * 由 (RR - FL)、(FR - RL) 两组差分解出 w，再平均解出 vx/vy，对四轮噪声更稳。
 */
static void forward_kinematics(const float wheel[4], float *vx, float *vy, float *w_rad) {
    const float L = WHEEL_BASE_LENGTH / 2.0f;
    const float W = WHEEL_BASE_WIDTH / 2.0f;
    const float R = WHEEL_RADIUS;
    const float cx = CHASSIS_ROTATION_CENTER_OFFSET_X;
    const float cy = CHASSIS_ROTATION_CENTER_OFFSET_Y;
    const float r0 = L + W;

    const float l_fl = r0 - cx - cy;
    const float l_fr = r0 - cx + cy;
    const float l_rl = r0 + cx - cy;
    const float l_rr = r0 + cx + cy;

    float w1 = R * (wheel[3] - wheel[0]) / (l_fl + l_rr); /* (RR - FL) */
    float w2 = R * (wheel[1] - wheel[2]) / (l_fr + l_rl); /* (FR - RL) */
    float omega = 0.5f * (w1 + w2);

    float a1 = wheel[0] * R + l_fl * omega;
    float a2 = wheel[3] * R - l_rr * omega;
    float a = 0.5f * (a1 + a2); /* a = vx - vy */

    float b1 = wheel[1] * R - l_fr * omega;
    float b2 = wheel[2] * R + l_rl * omega;
    float b = 0.5f * (b1 + b2); /* b = vx + vy */

    *vx = 0.5f * (a + b);
    *vy = 0.5f * (b - a);
    *w_rad = omega;
}

void Plant_Step1ms(const SimCommand *cmd) {
    Wheel_t wheels;
    float target[4];
    float a_max_rad; /* 轮角加速度上限 (rad/s²) */
    float alpha; /* 一阶滞后系数 */
    float tau_used; /* 本拍使用的轮速滞后时间常数 (s) */
    float vx, vy, omega;
    const float dt = 0.001f;
    int i;

    if (s_cfg == NULL || cmd == NULL) return;

    /* 1) 真实逆运动学（STOP 命令为立即停止，目标轮速 0，见步骤 3/4 的制动模型） */
    if (cmd->stop) {
        wheels.fl = 0.0f;
        wheels.fr = 0.0f;
        wheels.rl = 0.0f;
        wheels.rr = 0.0f;
    } else {
        Kinematics_Inverse(cmd->vx, cmd->vy, cmd->w, &wheels);
    }
    target[0] = wheels.fl;
    target[1] = wheels.fr;
    target[2] = wheels.rl;
    target[3] = wheels.rr;

    /* 2) 整数 RPM 量化（真实命令链的量化，低速度时每档约 0.38cm/s） */
    if (s_cfg->rpm_quantize) {
        for (i = 0; i < 4; i++) target[i] = quantize_wheel_rpm(target[i]);
    }

    /* 3) 电机加速度斜坡：轮角加速度上限 = 车体加速度 / 轮半径（与固件 acc 档位一致）。
     *    STOP 命令(0xFE)是驱动器主动立即停止，与普通速度斜坡不同：用 stop_decel 高减速度
     *    + stop_tau 短滞后建模（默认停车距离约 1cm；该参数是模型不确定项，见 uncertainty 套件）。 */
    if (cmd->stop) {
        a_max_rad = s_cfg->stop_decel_cm_s2 / WHEEL_RADIUS;
        tau_used = s_cfg->stop_tau_s;
    } else if (s_cfg->motor_acc_limit && cmd->acc > 0.0f) {
        a_max_rad = cmd->acc * s_cfg->motor_acc_scale / WHEEL_RADIUS;
        tau_used = s_cfg->wheel_tau_s;
    } else {
        a_max_rad = 1.0e9f; /* 不限幅 */
        tau_used = s_cfg->wheel_tau_s;
    }
    if (a_max_rad <= 0.0f) a_max_rad = 1.0e9f;
    if (tau_used < 0.0f) tau_used = 0.0f;

    /* 4) 轮速一阶滞后（tau=0 时等效无滞后） */
    if (tau_used > 1.0e-6f) {
        alpha = 1.0f - expf(-dt / tau_used);
    } else {
        alpha = 1.0f;
    }

    for (i = 0; i < 4; i++) {
        float d = target[i] - s_wheel_ramp[i];
        float lim = a_max_rad * dt;
        if (d > lim) d = lim;
        if (d < -lim) d = -lim;
        s_wheel_ramp[i] += d;
        s_wheel_act[i] += (s_wheel_ramp[i] - s_wheel_act[i]) * alpha;
    }

    /* 5) 正运动学 + 6) 打滑 */
    forward_kinematics(s_wheel_act, &vx, &vy, &omega);
    {
        float slip = s_cfg->slip;
        if (slip < 0.0f) slip = 0.0f;
        if (slip > 0.95f) slip = 0.95f;
        vx *= (1.0f - slip);
        vy *= (1.0f - slip);
        omega *= (1.0f - slip);
    }
    s_vx_body = vx;
    s_vy_body = vy;
    s_w_dps = rad2deg_f(omega);

    /* 7) 位姿积分（用半拍航向提高纯旋转/斜向精度） */
    {
        float c = cosf(deg2rad_f(s_yaw + 0.5f * s_w_dps * dt));
        float s = sinf(deg2rad_f(s_yaw + 0.5f * s_w_dps * dt));
        s_x += (vx * c - vy * s) * dt;
        s_y += (vx * s + vy * c) * dt;
        s_yaw += s_w_dps * dt;
    }

    s_tick++;
    s_hist[s_tick % PLANT_HIST].x = s_x;
    s_hist[s_tick % PLANT_HIST].y = s_y;
    s_hist[s_tick % PLANT_HIST].yaw = s_yaw;
}

void Plant_GetPose(float *x, float *y, float *yaw) {
    if (x) *x = s_x;
    if (y) *y = s_y;
    if (yaw) *yaw = s_yaw;
}

void Plant_GetTwist(float *vx_body, float *vy_body, float *w_dps) {
    if (vx_body) *vx_body = s_vx_body;
    if (vy_body) *vy_body = s_vy_body;
    if (w_dps) *w_dps = s_w_dps;
}

void Plant_GetWheelSpeed(float wheel_rad_s[4]) {
    int i;
    if (wheel_rad_s == NULL) return;
    for (i = 0; i < 4; i++) wheel_rad_s[i] = s_wheel_act[i];
}

void Plant_GetPoseAt(uint32_t tick, float *x, float *y, float *yaw) {
    uint32_t idx;
    if (tick < s_start_tick) tick = s_start_tick;
    if (tick > s_tick) tick = s_tick;
    idx = tick % PLANT_HIST;
    if (x) *x = s_hist[idx].x;
    if (y) *y = s_hist[idx].y;
    if (yaw) *yaw = s_hist[idx].yaw;
}
