/**
 * @file sim_platform.c
 * @brief 主机端 nav 仿真：CMSIS-RTOS2 / FreeRTOS / MotionControl / log / 地图 打桩 + 电机命令链模型
 *
 * 命令链模型（模拟电机总线/驱动器）：
 *   MotionControl_SetVelocity() 把"本拍机体系速度命令"写入 latest；
 *   仿真每 1ms 把 latest 采样进环形缓冲（sample-and-hold）；
 *   被控对象在 t 时刻执行的是 t - cmd_delay_ms 时刻缓冲里的命令，
 *   即端到端纯延迟（含总线传输、命令队列、驱动器响应）。
 *
 * 这与真实固件 motion_control.c -> zdt_v5 命令队列 -> 电机驱动器 的区别：
 *   - 真实命令队列会阻塞 Nav_Task（控制周期抖动），这里用 jitter_max_ms 单独建模；
 *   - 真实总线是帧级离散时延，这里是毫秒级纯延迟。
 */

#include "sim_platform.h"
#include "sim_ops.h"
#include "sim_plant.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "log.h"
#include "motion_control.h"
#include "nav_map.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CMD_RING_SIZE 512
#define SIM_MALLOC_ARENA (64 * 1024)

static const SimPlantCfg *s_cfg = NULL;
static uint32_t s_tick = 0;
static SimCommand s_latest; /* 最新的机体系速度命令 */
static SimCommand s_ring[CMD_RING_SIZE]; /* 每 1ms 的 sample-and-hold 命令历史 */
static int s_verbose = 0;
static uint32_t s_events = 0;
static int s_abort = 0; /* 0=未中止，1=正常完成，2=场景超时，3=导航 ERROR，4=osThreadExit */
static SimTickFn s_hook = NULL;
static void (*s_abort_jump)(void) = NULL;

/* 电机命令链延迟，单位 ms（四舍五入到 1ms） */
static uint32_t s_cmd_delay_ms = 0;

/* 控制周期统计 */
static uint32_t s_ctl_min = 0xFFFFFFFFu;
static uint32_t s_ctl_max = 0;
static uint32_t s_ctl_sum = 0;
static uint32_t s_ctl_count = 0;

/* 独立随机源（场景复位时重置，保证可复现）：xorshift32 */
static uint32_t s_rng_state = 0x12345678u;

/* 动态内存桩：nav_core 只用一次 Nav_Core_Init，给个简单 bump arena 即可 */
static unsigned char s_arena[SIM_MALLOC_ARENA];
static size_t s_arena_used = 0;

/* 底盘运动参数（默认值与真实 MotionControl_Init 一致） */
static float s_motion_linear = 80.0f;
static float s_motion_yaw = 140.0f;
static float s_motion_acc = 70.0f;
static float s_motion_dec = 100.0f;

/* 事件标志句柄（Events.h 里 System_StatusHandle 的仿真实例） */
static int s_status_flag_obj = 0;
osEventFlagsId_t System_StatusHandle = (osEventFlagsId_t)&s_status_flag_obj;

static uint32_t rng_next(void) {
    uint32_t x = s_rng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    s_rng_state = x;
    return x;
}

static void rng_reset(uint32_t seed) {
    s_rng_state = (seed == 0u) ? 0x12345678u : seed;
}

void Sim_PlatformInit(const SimPlantCfg *cfg) {
    s_cfg = cfg;
    s_tick = 0;
    memset(&s_latest, 0, sizeof(s_latest));
    memset(s_ring, 0, sizeof(s_ring));
    s_cmd_delay_ms = (uint32_t)(cfg->cmd_delay_ms + 0.5f);
    s_events = 0;
    s_abort = 0;
    s_hook = NULL;
    s_abort_jump = NULL;
    s_arena_used = 0;
    s_ctl_min = 0xFFFFFFFFu;
    s_ctl_max = 0;
    s_ctl_sum = 0;
    s_ctl_count = 0;
    s_motion_linear = 80.0f;
    s_motion_yaw = 140.0f;
    s_motion_acc = 70.0f;
    s_motion_dec = 100.0f;
    rng_reset(cfg->seed);
}

void Sim_ResetBus(void) {
    memset(&s_latest, 0, sizeof(s_latest));
    memset(s_ring, 0, sizeof(s_ring));
    s_events = 0;
    s_abort = 0;
    s_ctl_min = 0xFFFFFFFFu;
    s_ctl_max = 0;
    s_ctl_sum = 0;
    s_ctl_count = 0;
    rng_reset(s_cfg->seed ^ 0x9E3779B9u);
}

uint32_t Sim_Tick(void) {
    return s_tick;
}

const SimPlantCfg *Sim_Cfg(void) {
    return s_cfg;
}

void Sim_SetTickHook(SimTickFn fn) {
    s_hook = fn;
}

void Sim_SetAbortJump(void (*jump_fn)(void)) {
    s_abort_jump = jump_fn;
}

void Sim_RequestAbort(int reason) {
    if (s_abort == 0) s_abort = reason;
}

int Sim_AbortReason(void) {
    return s_abort;
}

void Sim_ClearAbort(void) {
    s_abort = 0;
}

void Sim_SetVerbose(int on) {
    s_verbose = on;
}

int Sim_GetVerbose(void) {
    return s_verbose;
}

uint32_t Sim_GetEventFlags(void) {
    return s_events;
}

void Sim_GetControlTiming(uint32_t *min_ms, uint32_t *max_ms, uint32_t *mean_ms, uint32_t *count) {
    if (min_ms) *min_ms = (s_ctl_count == 0) ? 0 : s_ctl_min;
    if (max_ms) *max_ms = s_ctl_max;
    if (mean_ms) *mean_ms = (s_ctl_count == 0) ? 0 : (s_ctl_sum / s_ctl_count);
    if (count) *count = s_ctl_count;
}

SimCommand Sim_CurrentCommand(void) {
    return s_latest;
}

/**
 * @brief 计算下一个控制周期长度：周期 + （可选）随机抖动
 * 抖动用于模拟真实系统里 Motor_Send_Cmd 队列满阻塞 Nav_Task 造成的唤醒延迟。
 */
uint32_t Sim_NextPeriodMs(void) {
    uint32_t base = (uint32_t)s_cfg->nav_period_ms;
    if (base == 0) base = 5;
    if (s_cfg->jitter_max_ms > 0 && s_cfg->jitter_prob > 0.0f) {
        float r = (float)(rng_next() & 0xFFFFFF) / 16777216.0f;
        if (r < s_cfg->jitter_prob) {
            uint32_t extra = (uint32_t)(1 + (rng_next() % (uint32_t)s_cfg->jitter_max_ms));
            base += extra;
        }
    }
    return base;
}

void Sim_Advance(uint32_t ms) {
    uint32_t i;

    if (s_cfg == NULL) return;
    if (ms > 1000) ms = 1000;

    s_ctl_sum += ms;
    s_ctl_count++;
    if (ms < s_ctl_min) s_ctl_min = ms;
    if (ms > s_ctl_max) s_ctl_max = ms;

    for (i = 0; i < ms; i++) {
        SimCommand eff;
        SimCommand zero_cmd;

        if (s_abort != 0) return;

        /* 1) 本毫秒的 sample-and-hold 命令 */
        s_ring[s_tick % CMD_RING_SIZE] = s_latest;

        /* 2) 取延迟后的命令（tick < delay 时车还没收到任何命令，保持静止） */
        memset(&zero_cmd, 0, sizeof(zero_cmd));
        if (s_tick >= s_cmd_delay_ms) {
            eff = s_ring[(s_tick - s_cmd_delay_ms) % CMD_RING_SIZE];
        } else {
            eff = zero_cmd;
        }

        /* 3) 被控对象步进 1ms */
        Plant_Step1ms(&eff);
        s_tick++;

        /* 4) OPS 测量 + 1ms 钩子 */
        SimOps_Tick(s_tick);
        if (s_hook != NULL) s_hook(s_tick);
    }
}

/* ======================== CMSIS-RTOS2 桩 ======================== */

uint32_t osKernelGetTickCount(void) {
    return s_tick;
}

osStatus_t osDelay(uint32_t ticks) {
    Sim_Advance(ticks);
    if (s_abort != 0 && s_abort_jump != NULL) {
        s_abort_jump(); /* 不返回：跳回场景驱动 */
    }
    return osOK;
}

uint32_t osEventFlagsWait(osEventFlagsId_t ef_id, uint32_t flags, uint32_t options, uint32_t timeout) {
    (void)ef_id;
    (void)options;
    (void)timeout;
    return flags; /* 系统初始化完成等事件在仿真里视为已就绪 */
}

uint32_t osEventFlagsSet(osEventFlagsId_t ef_id, uint32_t flags) {
    (void)ef_id;
    s_events |= flags;
    return 0;
}

void osThreadExit(void) {
    Sim_RequestAbort(4);
    if (s_abort_jump != NULL) s_abort_jump();
    fprintf(stderr, "[sim] osThreadExit called, scenario aborted\n");
    exit(4);
}

osStatus_t osMutexAcquire(osMutexId_t mutex_id, uint32_t timeout) {
    (void)mutex_id;
    (void)timeout;
    return osOK;
}

osStatus_t osMutexRelease(osMutexId_t mutex_id) {
    (void)mutex_id;
    return osOK;
}

/* ======================== FreeRTOS 桩 ======================== */

void *pvPortMalloc(size_t size) {
    size_t aligned = (size + 7u) & ~(size_t)7u;
    void *p;
    if (s_arena_used + aligned > SIM_MALLOC_ARENA) return NULL;
    p = &s_arena[s_arena_used];
    s_arena_used += aligned;
    return p;
}

void vPortFree(void *ptr) {
    (void)ptr; /* bump arena：不回收（nav 只在 Init 分配一次） */
}

/* ======================== log 桩 ======================== */

void Sim_LogWrite(char level, const char *fmt, ...) {
    va_list ap;
    if (!s_verbose) return;
    va_start(ap, fmt);
    fprintf(stdout, "[nav:%c] ", level);
    vfprintf(stdout, fmt, ap);
    fprintf(stdout, "\n");
    va_end(ap);
    fflush(stdout);
}

/* ======================== 地图桩（nav_core 的 Nav_GoTo 未在仿真中使用） ======================== */

TargetPoint_t *Map_GetPoint(uint8_t id) {
    (void)id;
    return NULL;
}

TargetPoint_t *Map_GetPointByName(const char *name) {
    (void)name;
    return NULL;
}

/* ======================== MotionControl 桩（命令采样进电机总线模型） ======================== */

bool MotionControl_Init(void) {
    return true;
}

void MotionControl_GetMotionParams(float *linear_speed, float *yaw_speed, float *acc, float *dec) {
    if (linear_speed) *linear_speed = s_motion_linear;
    if (yaw_speed) *yaw_speed = s_motion_yaw;
    if (acc) *acc = s_motion_acc;
    if (dec) *dec = s_motion_dec;
}

void MotionControl_SetMotionParams(float linear_speed, float yaw_speed, float acc, float dec) {
    s_motion_linear = linear_speed;
    s_motion_yaw = yaw_speed;
    s_motion_acc = acc;
    s_motion_dec = dec;
}

void MotionControl_SetVelocity(float x_component, float y_component, float yaw_component) {
    s_latest.vx = x_component;
    s_latest.vy = y_component;
    s_latest.w = yaw_component;
    s_latest.acc = s_motion_acc;
    s_latest.stop = 0;
}

void MotionControl_Stop(void) {
    s_latest.vx = 0.0f;
    s_latest.vy = 0.0f;
    s_latest.w = 0.0f;
    s_latest.acc = s_motion_acc;
    s_latest.stop = 1;
}

bool MotionControl_SetPosition(float x_offset, float y_offset, float yaw_offset) {
    (void)x_offset;
    (void)y_offset;
    (void)yaw_offset;
    return true;
}

bool MotionControl_PositionArrived(void) {
    return true;
}
