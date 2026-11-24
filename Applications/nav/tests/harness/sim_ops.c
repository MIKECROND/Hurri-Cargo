/**
 * @file sim_ops.c
 * @brief 主机端 nav 仿真：OPS 平面定位测量 + Loc_Get 实现
 *
 * 复刻固件的定位语义（Applications/nav/localization/nav_local.c，单 OPS 源时）：
 *   - OPS 按固定更新率输出一帧位姿，经过测量/传输延迟后才可用；
 *   - Loc_Get 返回最近一帧 OPS 的位置（帧间保持）与一阶低通后的航向
 *     （nav_local.c 每 1ms 以 alpha=0.5 按最短弧低通 yaw，等效时间常数约 2ms）。
 * 测量误差：位置/航向量化 + 可配高斯噪声（固定种子，结果可复现）。
 */

#include "sim_ops.h"
#include "sim_plant.h"
#include "sim_platform.h"
#include "nav_local.h"
#include "nav_math.h"
#include "ops.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static const SimPlantCfg *s_cfg = NULL;
static uint32_t s_start_tick = 0;
static uint32_t s_period_ms = 20;
static uint32_t s_delay_ms = 0;

static int s_has_raw = 0;
static float s_raw_x = 0.0f;
static float s_raw_y = 0.0f;
static float s_raw_yaw = 0.0f;
static float s_filtered_yaw = 0.0f;

/* OPS 完整帧新鲜度（供 nav_core 的 OPS_Is_Stale 判定）：
 * 上电以来从未收到完整帧 -> 年龄返回 0xFFFFFFFF；否则返回距最近一帧的毫秒数。
 * 场景复位时视为"导航开始前 OPS 已经稳定输出"（真实系统初始化早于导航启动）。 */
static int s_frame_seen = 0;
static uint32_t s_last_frame_tick = 0;

static uint32_t s_rng_state = 1u;

static uint32_t rng_next(void) {
    uint32_t x = s_rng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    s_rng_state = x;
    return x;
}

static float rng_uniform(void) {
    return (float)(rng_next() >> 8) / 16777216.0f; /* [0,1) */
}

/* Box-Muller 高斯噪声（sigma=1） */
static float rng_gauss(void) {
    float u1 = rng_uniform();
    float u2 = rng_uniform();
    if (u1 < 1.0e-7f) u1 = 1.0e-7f;
    return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * (float)M_PI * u2);
}

static float quantize(float v, float q) {
    if (q <= 0.0f) return v;
    return floorf(v / q + 0.5f) * q;
}

void SimOps_Reset(const SimPlantCfg *cfg, uint32_t start_tick) {
    float x0, y0, yaw0;
    uint32_t period;

    s_cfg = cfg;
    s_start_tick = start_tick;

    period = (cfg->ops_rate_hz > 1.0f) ? (uint32_t)(1000.0f / cfg->ops_rate_hz + 0.5f) : 20u;
    if (period < 1u) period = 1u;
    s_period_ms = period;
    s_delay_ms = (uint32_t)(cfg->ops_delay_ms + 0.5f);

    s_rng_state = cfg->seed ^ 0xA5A5A5A5u;
    if (s_rng_state == 0u) s_rng_state = 1u;

    /* 首帧 = 场景起点位姿（真实系统导航开始前 OPS 已经稳定输出） */
    Plant_GetPoseAt(start_tick, &x0, &y0, &yaw0);
    s_raw_x = quantize(x0, cfg->ops_pos_quant_cm);
    s_raw_y = quantize(y0, cfg->ops_pos_quant_cm);
    s_raw_yaw = quantize(yaw0, cfg->ops_yaw_quant_deg);
    s_filtered_yaw = s_raw_yaw;
    s_has_raw = 1;
    s_frame_seen = 1;
    s_last_frame_tick = start_tick;
}

void SimOps_Tick(uint32_t now_tick) {
    if (s_cfg == NULL) return;

    if (now_tick >= s_delay_ms) {
        uint32_t sample_t = now_tick - s_delay_ms;
        if (sample_t >= s_start_tick && ((sample_t - s_start_tick) % s_period_ms) == 0) {
            float x, y, yaw;
            Plant_GetPoseAt(sample_t, &x, &y, &yaw);
            s_raw_x = quantize(x, s_cfg->ops_pos_quant_cm) + s_cfg->ops_pos_noise_cm * rng_gauss();
            s_raw_y = quantize(y, s_cfg->ops_pos_quant_cm) + s_cfg->ops_pos_noise_cm * rng_gauss();
            s_raw_yaw = normalize_angle(quantize(yaw, s_cfg->ops_yaw_quant_deg) + s_cfg->ops_yaw_noise_deg * rng_gauss());
            s_has_raw = 1;
            s_frame_seen = 1;
            s_last_frame_tick = now_tick;
        }
    }

    /* 复刻 nav_local.c 的 yaw 一阶低通：每 1ms 以 alpha=0.5 按最短弧趋近 */
    if (s_has_raw) {
        s_filtered_yaw = normalize_angle(s_filtered_yaw + 0.5f * normalize_angle(s_raw_yaw - s_filtered_yaw));
    }
}

void SimOps_GetMeasPose(float *x, float *y, float *yaw) {
    if (x) *x = s_has_raw ? s_raw_x : 0.0f;
    if (y) *y = s_has_raw ? s_raw_y : 0.0f;
    if (yaw) *yaw = s_filtered_yaw;
}

bool SimOps_HasSample(void) {
    return s_has_raw != 0;
}

/* ===================== OPS 帧新鲜度（ops.h API 的仿真实现） ===================== */

uint32_t OPS_GetFrameAgeMs(void) {
    if (!s_frame_seen) return 0xFFFFFFFFu;
    return Sim_Tick() - s_last_frame_tick;
}

bool OPS_Is_Stale(uint32_t max_age_ms) {
    uint32_t age = OPS_GetFrameAgeMs();
    if (age == 0xFFFFFFFFu) return true;
    return age > max_age_ms;
}

/* ===================== 固件 Loc_Get 的仿真实现 ===================== */

bool Loc_Get(PoseTimestamp_t *pose) {
    if (pose == NULL) return false;
    pose->pose.x = s_has_raw ? s_raw_x : 0.0f;
    pose->pose.y = s_has_raw ? s_raw_y : 0.0f;
    pose->pose.yaw = s_filtered_yaw;
    pose->timestamp = Sim_Tick();
    return true;
}
