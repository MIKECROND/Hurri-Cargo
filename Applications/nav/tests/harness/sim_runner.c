/**
 * @file sim_runner.c
 * @brief 主机端 nav 仿真：场景编排 / 指标统计 / CSV 与 trace 输出
 *
 * 驱动方式：
 *   - 固件旧版（无 Nav_Update，如基线快照）：用 osDelay 桩推进仿真时钟，用 setjmp/longjmp
 *     从 Nav_Task 的无限循环里退出，控制代码 100% 为真实 nav_core.c；
 *   - 新版（nav_core.h 暴露 void Nav_Update(void)）：构建时定义 SIM_HAVE_NAV_UPDATE，
 *     每个控制周期先推进 5ms（被控对象/定位），再调用一次 Nav_Update()。
 *
 * 指标定义（每个 leg 从导航指令下发开始统计）：
 *   arrive_ms         到达（NavState COMPLETE）耗时；PASS 腿为触发换目标的耗时
 *   pos_err_cm        到达瞬间真实位姿与目标点的距离误差
 *   yaw_err_deg       到达瞬间真实航向与目标航向误差
 *   true_dist_cm      到达瞬间真实剩余距离（明显大于到达阈值 = 提前判到/滑移）
 *   settle_tail_ms    从"真实进入到达阈值"到 NavState COMPLETE 的尾延迟（含 STOPPING 等待）
 *   overshoot_cm      沿起点->目标方向越过目标的最大距离（超调）
 *   yaw_over_deg      朝目标方向转过头的最大角度（航向超调）
 *   lat_max/rms_cm    相对起点->目标直线的最大/均方根横向偏差
 *   near_zone_ms      进入目标附近（距离<=18cm 或 角度<=15°）后到到达的时间
 *   near_osc          目标附近误差符号翻转次数（来回振荡）
 *   cmd_rev           机体系速度指令分量符号翻转总次数（指令抖动）
 *   pass_min_speed    不停车换目标窗口内真实速度最小值（越小越接近停车）
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <setjmp.h>

#include "nav_core.h"
#include "nav_math.h"
#include "nav_map.h"
#include "nav_local.h"

#include "sim_config.h"
#include "sim_plant.h"
#include "sim_platform.h"
#include "sim_ops.h"

/* nav_core.h 没有声明 Nav_Core_Init / Nav_Task（固件里由 freertos.c/初始化代码引用） */
bool Nav_Core_Init(void);
#ifndef SIM_HAVE_NAV_UPDATE
void Nav_Task(void *argument);
#endif

#define MAX_LEGS 3
#define MAX_TUNE 8

/* ======================= 场景定义 ======================= */

typedef struct {
    Pose2D_t pose; /* 目标位姿（世界系 cm/deg） */
    float speed; /* target_speed (cm/s) */
    float acc; /* acceleration (cm/s^2) */
    float dec; /* deceleration (cm/s^2) */
    float arrive_dist; /* distance_threshold (cm) */
    float arrive_yaw; /* yaw_threshold (deg) */
    uint32_t timeout_ms; /* 导航超时 (ms) */
    float pass_trigger_cm; /* >0: 本腿是 PASS 不停车通过点，距目标该距离内切下一腿 */
    int yaw_fast_after; /* PASS 切换后置位 Nav_SetYawFast */
} SimLeg;

typedef struct {
    const char *name;
    int n_legs;
    SimLeg legs[MAX_LEGS];
} SimScenario;

#define LEG(x_, y_, yaw_, spd_)                                                                                        \
    {                                                                                                                  \
        .pose = {.x = (x_), .y = (y_), .yaw = (yaw_)},                                                                 \
        .speed = (spd_),                                                                                               \
        .acc = 110.0f,                                                                                                 \
        .dec = 70.0f,                                                                                                  \
        .arrive_dist = 1.5f,                                                                                           \
        .arrive_yaw = 1.0f,                                                                                            \
        .timeout_ms = 10000,                                                                                           \
        .pass_trigger_cm = 0.0f,                                                                                       \
        .yaw_fast_after = 0,                                                                                           \
    }

/* PASS 腿：到达门限收小到 0.3cm/30°（与 mission_config.h 的 PASS 守卫一致），
 * 在距目标 trigger 距离内由 mission 层换目标，所以导航层不应先判到达。 */
#define LEG_PASS(x_, y_, yaw_, spd_, trig_, fast_)                                                                     \
    {                                                                                                                  \
        .pose = {.x = (x_), .y = (y_), .yaw = (yaw_)},                                                                 \
        .speed = (spd_),                                                                                               \
        .acc = 110.0f,                                                                                                 \
        .dec = 80.0f,                                                                                                  \
        .arrive_dist = 0.3f,                                                                                           \
        .arrive_yaw = 30.0f,                                                                                           \
        .timeout_ms = 8000,                                                                                            \
        .pass_trigger_cm = (trig_),                                                                                    \
        .yaw_fast_after = (fast_),                                                                                     \
    }

static const SimScenario s_scenarios[] = {
    {"line100_30", 1, {LEG(100.0f, 0.0f, 0.0f, 30.0f)}},
    {"line100_50", 1, {LEG(100.0f, 0.0f, 0.0f, 50.0f)}},
    {"line100_70", 1, {LEG(100.0f, 0.0f, 0.0f, 70.0f)}},
    {"line100_100", 1, {LEG(100.0f, 0.0f, 0.0f, 100.0f)}},
    {"short5", 1, {LEG(5.0f, 0.0f, 0.0f, 70.0f)}},
    {"line100_yaw90", 1, {LEG(100.0f, 0.0f, 90.0f, 70.0f)}},
    {"diag100", 1, {LEG(70.7107f, 70.7107f, 0.0f, 70.0f)}},
    {"rot90", 1, {LEG(0.0f, 0.0f, 90.0f, 70.0f)}},
    {"rot180", 1, {LEG(0.0f, 0.0f, 180.0f, 70.0f)}},
    {"pass_straight", 2, {LEG_PASS(40.0f, 0.0f, 0.0f, 100.0f, 4.0f, 0), LEG(100.0f, 0.0f, 0.0f, 70.0f)}},
    {"pass_turn", 2, {LEG_PASS(40.0f, 0.0f, 0.0f, 100.0f, 4.0f, 1), LEG(40.0f, 60.0f, 90.0f, 70.0f)}},
    {"seq_3pt", 3, {LEG(30.0f, 0.0f, 0.0f, 70.0f), LEG(60.0f, 40.0f, 0.0f, 70.0f), LEG(60.0f, 40.0f, 90.0f, 70.0f)}},
    {"line30_100", 1, {LEG(30.0f, 0.0f, 0.0f, 100.0f)}},
};
#define SCENARIO_COUNT ((int)(sizeof(s_scenarios) / sizeof(s_scenarios[0])))

static int scenario_in_set(const SimScenario *sc, const char *const *set, int n) {
    int i;
    for (i = 0; i < n; i++) {
        if (strcmp(sc->name, set[i]) == 0) return 1;
    }
    return 0;
}

static int scenario_in_quick_set(const SimScenario *sc) {
    static const char *quick[] = {"line100_30", "line100_70", "line100_100", "short5",
                                  "diag100",   "rot90",     "rot180",      "pass_straight",
                                  "seq_3pt"};
    return scenario_in_set(sc, quick, (int)(sizeof(quick) / sizeof(quick[0])));
}

static int scenario_trace_wanted(const SimScenario *sc) {
    static const char *want[] = {"line100_70", "line100_100", "short5", "rot90", "pass_straight", "pass_turn"};
    return scenario_in_set(sc, want, (int)(sizeof(want) / sizeof(want[0])));
}

/* ======================= 指标 ======================= */

#define NEAR_FACTOR 12.0f /* 与 NAV_NEAR_TARGET_FACTOR 一致（固定值保证跨版本可比） */
#define NEAR_MIN_CM 10.0f
#define NEAR_YAW_DEG 15.0f

typedef struct {
    uint32_t t_begin;
    int has_dist; /* 1=有平移目标（用距离方向统计），0=原地旋转腿 */
    float start_x, start_y, start_yaw;
    float target_x, target_y;
    float target_yaw_norm; /* 目标航向（归一化） */
    float target_yaw_cont; /* 目标连续航向 */
    float dir_x, dir_y; /* 起点->目标单位方向 */
    float start_dist; /* 起点到目标距离 */
    float arrive_dist, arrive_yaw;
    float yaw_dir; /* 有航向变化时 = sign(目标连续航向-起始连续航向)，否则 0 */

    float max_overshoot;
    float max_yaw_over;
    float max_yaw_dev;
    float lat_max;
    double lat_sq_sum;
    int lat_n;
    float max_speed;
    double path_len;
    int near_entered;
    uint32_t t_near;
    uint32_t near_ms;
    int near_osc;
    int last_err_sign;
    int cmd_rev;
    int sx_vx, sx_vy, sx_w;
    int settle_seen;
    uint32_t t_settle;
} LegAccum;

typedef struct {
    int kind; /* 0=未完成，1=COMPLETE 到达，2=PASS 切换 */
    uint32_t t_begin, t_end;
    uint32_t arrive_ms;
    float pos_err_cm;
    float yaw_err_deg;
    float true_dist_cm;
    long settle_tail_ms; /* -1 = 未观察到 */
    float overshoot_cm, overshoot_ratio;
    float yaw_over_deg, yaw_dev_deg;
    float lat_max_cm, lat_rms_cm;
    uint32_t near_zone_ms;
    int near_osc, cmd_rev;
    float max_speed, path_len_cm, mean_speed;
} LegResult;

/* ======================= 运行期全局状态 ======================= */

static const SimPlantCfg *g_cfg = NULL;
static FILE *g_csv = NULL;
static char g_tag[64] = "run";
static char g_suite[32] = "single";
static char g_out_dir[512] = ".";
static int g_verbose = 0;
static int g_trace_enabled = 0;

static const SimScenario *g_scen = NULL;
static TargetPoint_t g_targets[MAX_LEGS];
static int g_leg_idx = 0;
static int g_leg_active = 0;
static LegAccum g_acc;
static LegResult g_results[MAX_LEGS];
static uint32_t g_scen_start_tick = 0;
static uint32_t g_done_tick = 0;
static int g_scenario_done = 0;
static int g_pending_next_leg = -1; /* PASS 换目标失败时延后到 COMPLETE 再切 */
static uint32_t g_pass_window_end = 0;
static float g_pass_min_speed = -1.0f;
static int g_pass_checked = 0;
static int g_nav_error = 0;
static FILE *g_trace = NULL;
static uint32_t g_trace_period = 5;

#ifndef SIM_HAVE_NAV_UPDATE
static jmp_buf g_jmp;
static int g_jmp_ready = 0;
static void sim_abort_jump(void) {
    if (g_jmp_ready) {
        g_jmp_ready = 0;
        longjmp(g_jmp, 1);
    }
}
#else
static NavTunables_t g_default_tunables;
static int g_tunables_saved = 0;
#endif

struct CliTune {
    char name[32];
    float value;
};
static struct CliTune g_tunes[MAX_TUNE];
static int g_tune_count = 0;

/* ======================= 工具函数 ======================= */

static float dist2d(float x1, float y1, float x2, float y2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    return sqrtf(dx * dx + dy * dy);
}

static float unwrap_target_yaw(float start_yaw_cont, float target_yaw_norm) {
    return start_yaw_cont + normalize_angle(target_yaw_norm - start_yaw_cont);
}

static void update_sign(int *sign, float value, float deadband) {
    if (value > deadband) {
        *sign = 1;
    } else if (value < -deadband) {
        *sign = -1;
    }
}

static void count_reversal(int prev_sign, int new_sign, int *counter) {
    if (prev_sign != 0 && new_sign != 0 && prev_sign != new_sign) (*counter)++;
}

static void make_target(TargetPoint_t *t, const SimLeg *leg, int id) {
    memset(t, 0, sizeof(*t));
    t->id = (uint8_t)id;
    snprintf(t->name, sizeof(t->name), "sim%d", id);
    t->pose = leg->pose;
    t->motion.target_speed = leg->speed;
    t->motion.target_angular_speed = 220.0f;
    t->motion.acceleration = leg->acc;
    t->motion.deceleration = leg->dec;
    t->motion.angular_acceleration = 0.0f;
    t->arrive.check_mode = ARRIVE_CHECK_BOTH;
    t->arrive.distance_threshold = leg->arrive_dist;
    t->arrive.yaw_threshold = leg->arrive_yaw;
    t->arrive.timeout_ms = (uint16_t)leg->timeout_ms;
}

/* ======================= 指标累积 ======================= */

static void leg_begin(uint32_t now, const SimLeg *leg) {
    float x, y, yaw;

    Plant_GetPose(&x, &y, &yaw);

    memset(&g_acc, 0, sizeof(g_acc));
    g_acc.t_begin = now;
    g_acc.start_x = x;
    g_acc.start_y = y;
    g_acc.start_yaw = yaw;
    g_acc.target_x = leg->pose.x;
    g_acc.target_y = leg->pose.y;
    g_acc.target_yaw_norm = normalize_angle(leg->pose.yaw);
    g_acc.target_yaw_cont = unwrap_target_yaw(yaw, g_acc.target_yaw_norm);
    g_acc.start_dist = dist2d(x, y, leg->pose.x, leg->pose.y);
    g_acc.arrive_dist = leg->arrive_dist;
    g_acc.arrive_yaw = leg->arrive_yaw;
    g_acc.has_dist = (g_acc.start_dist > 1.0f) ? 1 : 0;
    if (g_acc.has_dist) {
        g_acc.dir_x = (leg->pose.x - x) / g_acc.start_dist;
        g_acc.dir_y = (leg->pose.y - y) / g_acc.start_dist;
    }
    {
        float dyaw = g_acc.target_yaw_cont - yaw;
        g_acc.yaw_dir = (fabsf(dyaw) > 5.0f) ? ((dyaw > 0.0f) ? 1.0f : -1.0f) : 0.0f;
    }
    g_acc.t_settle = now;
    g_leg_active = 1;
}

static void leg_update(uint32_t now) {
    float x, y, yaw, vx, vy, w;
    float ex, ey, dist, err_along, yaw_err_abs, speed, lat;
    SimCommand cmd;

    if (!g_leg_active || g_scen == NULL) return;

    Plant_GetPose(&x, &y, &yaw);
    Plant_GetTwist(&vx, &vy, &w);
    cmd = Sim_CurrentCommand();

    ex = x - g_acc.target_x;
    ey = y - g_acc.target_y;
    dist = sqrtf(ex * ex + ey * ey);
    speed = sqrtf(vx * vx + vy * vy);

    if (speed > g_acc.max_speed) g_acc.max_speed = speed;
    g_acc.path_len += (double)speed * 0.001;

    yaw_err_abs = fabsf(normalize_angle(g_acc.target_yaw_norm - yaw));
    if (yaw_err_abs > g_acc.max_yaw_dev) g_acc.max_yaw_dev = yaw_err_abs;
    if (g_acc.yaw_dir != 0.0f) {
        float over = (yaw - g_acc.target_yaw_cont) * g_acc.yaw_dir;
        if (over > g_acc.max_yaw_over) g_acc.max_yaw_over = over;
    }

    if (g_acc.has_dist) {
        float proj = ex * g_acc.dir_x + ey * g_acc.dir_y;
        float near_radius = NEAR_FACTOR * g_acc.arrive_dist;
        if (near_radius < NEAR_MIN_CM) near_radius = NEAR_MIN_CM;

        if (proj > g_acc.max_overshoot) g_acc.max_overshoot = proj;
        lat = -g_acc.dir_y * ex + g_acc.dir_x * ey;
        if (fabsf(lat) > g_acc.lat_max) g_acc.lat_max = fabsf(lat);
        g_acc.lat_sq_sum += (double)lat * (double)lat;
        g_acc.lat_n++;

        err_along = proj;

        if (dist <= near_radius) {
            if (!g_acc.near_entered) {
                g_acc.near_entered = 1;
                g_acc.t_near = now;
            }
            g_acc.near_ms = now - g_acc.t_near;
            {
                int sign = (proj > 0.05f) ? 1 : ((proj < -0.05f) ? -1 : 0);
                count_reversal(g_acc.last_err_sign, sign, &g_acc.near_osc);
                if (sign != 0) g_acc.last_err_sign = sign;
            }
        }
    } else {
        err_along = normalize_angle(g_acc.target_yaw_cont - yaw);
        if (fabsf(err_along) <= NEAR_YAW_DEG) {
            if (!g_acc.near_entered) {
                g_acc.near_entered = 1;
                g_acc.t_near = now;
            }
            g_acc.near_ms = now - g_acc.t_near;
            {
                int sign = (err_along > 0.1f) ? 1 : ((err_along < -0.1f) ? -1 : 0);
                count_reversal(g_acc.last_err_sign, sign, &g_acc.near_osc);
                if (sign != 0) g_acc.last_err_sign = sign;
            }
        }
    }

    /* 指令分量翻转计数 */
    {
        int s = 0;
        update_sign(&s, cmd.vx, 0.4f);
        count_reversal(g_acc.sx_vx, s, &g_acc.cmd_rev);
        g_acc.sx_vx = (s != 0) ? s : g_acc.sx_vx;
        update_sign(&s, cmd.vy, 0.4f);
        count_reversal(g_acc.sx_vy, s, &g_acc.cmd_rev);
        g_acc.sx_vy = (s != 0) ? s : g_acc.sx_vy;
        update_sign(&s, cmd.w, 1.0f);
        count_reversal(g_acc.sx_w, s, &g_acc.cmd_rev);
        g_acc.sx_w = (s != 0) ? s : g_acc.sx_w;
    }

    /* 首次真实进入到达门限（用于统计 STOPPING 尾延迟） */
    if (!g_acc.settle_seen) {
        int ok;
        if (g_acc.has_dist) {
            ok = (dist <= g_acc.arrive_dist);
        } else {
            ok = (yaw_err_abs <= g_acc.arrive_yaw);
        }
        if (ok) {
            g_acc.settle_seen = 1;
            g_acc.t_settle = now;
        }
    }

    /* PASS 不停车窗口内真实速度最小值 */
    if (g_pass_window_end != 0 && now <= g_pass_window_end) {
        if (g_pass_min_speed < 0.0f || speed < g_pass_min_speed) g_pass_min_speed = speed;
    }
}

static void leg_finalize(uint32_t now, int kind) {
    float x, y, yaw;
    LegResult *r;
    if (!g_leg_active) return;
    r = &g_results[g_leg_idx];

    Plant_GetPose(&x, &y, &yaw);
    memset(r, 0, sizeof(*r));
    r->kind = kind;
    r->t_begin = g_acc.t_begin;
    r->t_end = now;
    r->arrive_ms = now - g_acc.t_begin;
    r->pos_err_cm = dist2d(x, y, g_acc.target_x, g_acc.target_y);
    r->yaw_err_deg = fabsf(normalize_angle(g_acc.target_yaw_norm - yaw));
    r->true_dist_cm = r->pos_err_cm;
    r->settle_tail_ms = g_acc.settle_seen ? (long)(now - g_acc.t_settle) : -1;
    r->overshoot_cm = g_acc.max_overshoot;
    r->overshoot_ratio = (g_acc.start_dist > 1.0f) ? (g_acc.max_overshoot / g_acc.start_dist) : 0.0f;
    r->yaw_over_deg = g_acc.max_yaw_over;
    r->yaw_dev_deg = g_acc.max_yaw_dev;
    r->lat_max_cm = g_acc.lat_max;
    r->lat_rms_cm = (g_acc.lat_n > 0) ? sqrtf((float)(g_acc.lat_sq_sum / g_acc.lat_n)) : 0.0f;
    r->near_zone_ms = g_acc.near_ms;
    r->near_osc = g_acc.near_osc;
    r->cmd_rev = g_acc.cmd_rev;
    r->max_speed = g_acc.max_speed;
    r->path_len_cm = (float)g_acc.path_len;
    r->mean_speed = (now > g_acc.t_begin) ? (float)(g_acc.path_len / ((double)(now - g_acc.t_begin) / 1000.0)) : 0.0f;
    g_leg_active = 0;
}

static float true_dist_to_current_target(void) {
    float x, y;
    Plant_GetPose(&x, &y, NULL);
    return dist2d(x, y, g_acc.target_x, g_acc.target_y);
}

/* ======================= 场景状态机（1ms tick 钩子） ======================= */

static void start_leg(uint32_t now, int idx) {
    g_leg_idx = idx;
    leg_begin(now, &g_scen->legs[idx]);
}

static void tick_hook(uint32_t now) {
    NavState_t st;
    const SimLeg *cur;

    if (g_scen == NULL) return;

    leg_update(now);

    st = Nav_GetState();
    cur = &g_scen->legs[g_leg_idx];

    if (g_leg_active) {
        if (st == NAV_STATE_COMPLETE) {
            leg_finalize(now, 1);

            if (g_pending_next_leg >= 0) {
                int next = g_pending_next_leg;
                g_pending_next_leg = -1;
                if (next < g_scen->n_legs) {
                    start_leg(now, next);
                    Nav_GoToDirect(&g_targets[next]);
                } else {
                    g_scenario_done = 1;
                    g_done_tick = now;
                }
            } else if (g_leg_idx + 1 < g_scen->n_legs) {
                int next = g_leg_idx + 1;
                start_leg(now, next);
                Nav_GoToDirect(&g_targets[next]);
            } else {
                g_scenario_done = 1;
                g_done_tick = now;
            }
        } else if (st == NAV_STATE_ERROR) {
            Sim_RequestAbort(3);
            return;
        } else if (st == NAV_STATE_RUNNING && cur->pass_trigger_cm > 0.0f && g_leg_idx + 1 < g_scen->n_legs) {
            /* PASS：按导航位姿（OPS 测量）判断距通过点距离，与 mission 层语义一致 */
            PoseTimestamp_t pt;
            float d = 1.0e9f;
            if (Loc_Get(&pt)) {
                d = dist2d(pt.pose.x, pt.pose.y, cur->pose.x, cur->pose.y);
            }
            if (d <= cur->pass_trigger_cm) {
                int next = g_leg_idx + 1;
                int ok;
                if (cur->yaw_fast_after) Nav_SetYawFast(true);
                ok = Nav_Retarget(&g_targets[next]);
                if (ok) {
                    leg_finalize(now, 2);
                    g_results[next - 1].true_dist_cm = true_dist_to_current_target();
                    if (!g_pass_checked) {
                        g_pass_checked = 1;
                        g_pass_min_speed = -1.0f;
                        g_pass_window_end = now + 1500;
                    }
                    start_leg(now, next);
                } else {
                    /* 换目标失败（例如已进入 STOPPING）：等 COMPLETE 后再用 GoToDirect 起下一腿 */
                    g_pending_next_leg = next;
                }
            }
        }
    }

    if (g_scenario_done) {
        if ((now - g_done_tick) >= (uint32_t)g_cfg->hold_after_complete_ms) {
            Sim_RequestAbort(1);
            return;
        }
    }
    if ((now - g_scen_start_tick) > (uint32_t)g_cfg->max_time_ms) {
        Sim_RequestAbort(2);
        return;
    }

    if (g_trace != NULL && ((now - g_scen_start_tick) % g_trace_period) == 0) {
        float x, y, yaw, vx, vy, w, mx, my, myaw;
        PoseTimestamp_t pt;
        SimCommand cmd = Sim_CurrentCommand();
        float dist = -1.0f, yaw_err = 0.0f, plan_speed = 0.0f, meas_speed = 0.0f;
        int state = (int)st;
        Plant_GetPose(&x, &y, &yaw);
        Plant_GetTwist(&vx, &vy, &w);
        SimOps_GetMeasPose(&mx, &my, &myaw);
        if (Loc_Get(&pt)) {
            dist = dist2d(pt.pose.x, pt.pose.y, g_targets[g_leg_idx].pose.x, g_targets[g_leg_idx].pose.y);
            yaw_err = normalize_angle(g_targets[g_leg_idx].pose.yaw - pt.pose.yaw);
        }
#ifdef SIM_HAVE_NAV_UPDATE
        {
            NavDebug_t dbg;
            Nav_GetDebug(&dbg);
            plan_speed = dbg.plan_speed;
            meas_speed = dbg.meas_speed;
            state = (int)dbg.state;
        }
#endif
        fprintf(g_trace,
                "%u,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
                "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                (unsigned)(now - g_scen_start_tick),
                state,
                g_targets[g_leg_idx].pose.x,
                g_targets[g_leg_idx].pose.y,
                g_targets[g_leg_idx].pose.yaw,
                x,
                y,
                yaw,
                cmd.vx,
                cmd.vy,
                cmd.w,
                mx,
                my,
                vx,
                vy,
                w,
                dist,
                yaw_err,
                plan_speed,
                meas_speed,
                myaw);
        fflush(g_trace);
    }
}

/* ======================= 场景执行 ======================= */

static void reset_scenario_state(void) {
    int i;
    memset(g_results, 0, sizeof(g_results));
    memset(&g_acc, 0, sizeof(g_acc));
    for (i = 0; i < MAX_LEGS; i++) {
        g_results[i].settle_tail_ms = -1;
        make_target(&g_targets[i], &g_scen->legs[i], i);
    }
    g_leg_idx = 0;
    g_leg_active = 0;
    g_scenario_done = 0;
    g_done_tick = 0;
    g_pending_next_leg = -1;
    g_pass_window_end = 0;
    g_pass_min_speed = -1.0f;
    g_pass_checked = 0;
    g_nav_error = 0;
}

static void apply_tunables_to_nav(void) {
#ifdef SIM_HAVE_NAV_UPDATE
    int t;
    if (g_tunables_saved) {
        Nav_SetTunable("yaw_kp", g_default_tunables.yaw_kp);
        Nav_SetTunable("yaw_ki", g_default_tunables.yaw_ki);
        Nav_SetTunable("yaw_kd", g_default_tunables.yaw_kd);
        Nav_SetTunable("yaw_d_filter_tau_s", g_default_tunables.yaw_d_filter_tau_s);
        Nav_SetTunable("ff_weight", g_default_tunables.ff_weight);
        Nav_SetTunable("fb_weight", g_default_tunables.fb_weight);
        Nav_SetTunable("yaw_gain_far", g_default_tunables.yaw_gain_far);
        Nav_SetTunable("yaw_gain_fast", g_default_tunables.yaw_gain_fast);
        Nav_SetTunable("align_dist", g_default_tunables.align_dist);
        Nav_SetTunable("xy_kp", g_default_tunables.xy_kp);
        Nav_SetTunable("xy_ki", g_default_tunables.xy_ki);
        Nav_SetTunable("near_gain", g_default_tunables.near_gain);
        Nav_SetTunable("near_factor", g_default_tunables.near_factor);
        Nav_SetTunable("traj_speed_cap", g_default_tunables.traj_speed_cap);
        Nav_SetTunable("startup_speed", g_default_tunables.startup_speed);
        Nav_SetTunable("min_speed", g_default_tunables.min_speed);
        Nav_SetTunable("yaw_max_speed", g_default_tunables.yaw_max_speed);
        Nav_SetTunable("yaw_min_speed", g_default_tunables.yaw_min_speed);
        Nav_SetTunable("yaw_accel_limit", g_default_tunables.yaw_accel_limit);
        Nav_SetTunable("stop_settle_ms", (float)g_default_tunables.stop_settle_ms);
    }
    for (t = 0; t < g_tune_count; t++) {
        if (!Nav_SetTunable(g_tunes[t].name, g_tunes[t].value)) {
            fprintf(stderr, "WARN: Nav_SetTunable('%s', %g) rejected\n", g_tunes[t].name, (double)g_tunes[t].value);
        }
    }
#endif
}

static void run_one(const SimScenario *scen, uint32_t delay_ms, SimPlantCfg *cfg) {
    uint32_t nav_error;
    uint32_t tmin, tmax, tmean, tcount;
    char trace_path[512];

    g_scen = scen;
    g_cfg = cfg;
    reset_scenario_state();

    /* 平台/总线复位：仿真时钟归零（每场景独立，导航内部只用相对时间） */
    Sim_PlatformInit(cfg);
    Plant_Reset(cfg, Sim_Tick(), 0.0f, 0.0f, 0.0f);
    SimOps_Reset(cfg, Sim_Tick());
    Sim_ResetBus();
    Sim_SetTickHook(tick_hook);

    apply_tunables_to_nav();
    Nav_Stop();
    g_scen_start_tick = Sim_Tick();

    /* trace：只对代表性场景 + 默认模型参数输出，避免产生大量文件 */
    if (g_trace_enabled && scenario_trace_wanted(scen) && (delay_ms == 0 || delay_ms == 50 || delay_ms == 100) &&
        fabsf(cfg->wheel_tau_s - 0.06f) < 1.0e-6f && cfg->slip == 0.0f && cfg->ops_rate_hz == 50.0f) {
        snprintf(trace_path, sizeof(trace_path), "%s/%s_%s_d%d.csv", g_out_dir, g_tag, scen->name, (int)delay_ms);
        g_trace = fopen(trace_path, "w");
        if (g_trace != NULL) {
            fprintf(g_trace,
                    "t_ms,state,ref_x,ref_y,ref_yaw,true_x,true_y,true_yaw,"
                    "cmd_vx,cmd_vy,cmd_w,meas_x,meas_y,true_vx,true_vy,true_w,"
                    "dist,yaw_err,plan_speed,meas_speed,meas_yaw\n");
        }
        g_trace_period = (cfg->nav_period_ms > 0) ? (uint32_t)cfg->nav_period_ms : 5;
    } else {
        g_trace = NULL;
    }

    start_leg(Sim_Tick(), 0);
    g_nav_error = 0;
    Nav_GoToDirect(&g_targets[0]);

#ifdef SIM_HAVE_NAV_UPDATE
    while (Sim_AbortReason() == 0) {
        uint32_t ms = Sim_NextPeriodMs();
        Sim_Advance(ms);
        if (Sim_AbortReason() != 0) break;
        Nav_Update();
    }
#else
    Sim_SetAbortJump(sim_abort_jump);
    g_jmp_ready = 1;
    if (setjmp(g_jmp) == 0) {
        Nav_Task(NULL); /* 真实旧版导航任务：osDelay 桩推进仿真并在场景结束时 longjmp 回来 */
    }
    g_jmp_ready = 0;
    Sim_SetAbortJump(NULL);
#endif

    if (g_trace != NULL) {
        fclose(g_trace);
        g_trace = NULL;
    }

    /* 未完成最后一条腿：把已发生的指标也落盘，便于诊断 */
    if (g_leg_active) {
        leg_finalize(Sim_Tick(), 0);
        g_results[g_leg_idx].arrive_ms = 0;
    }

    nav_error = (uint32_t)Sim_AbortReason();
    if (nav_error == 1 && g_scenario_done) nav_error = 0;
    g_nav_error = (int)nav_error;

    Sim_GetControlTiming(&tmin, &tmax, &tmean, &tcount);
    (void)tcount;

    /* 汇总指标：最后一条腿 + 场景结束真实位姿 */
    {
        const LegResult *last = &g_results[scen->n_legs - 1];
        float x, y, yaw;
        float end_pos_err, end_yaw_err;
        char s_leg_arrive[128], s_leg_pos[128], s_leg_yaw[128], s_leg_dist[128], s_leg_tail[128];
        char s_leg_over[160], s_leg_yover[160], s_leg_lat[160], s_leg_osc[160], s_leg_rev[160];
        char s_leg_mspeed[160], s_leg_plen[160];
        int k;

        Plant_GetPose(&x, &y, &yaw);
        end_pos_err = dist2d(x, y, scen->legs[scen->n_legs - 1].pose.x, scen->legs[scen->n_legs - 1].pose.y);
        end_yaw_err = fabsf(normalize_angle(scen->legs[scen->n_legs - 1].pose.yaw - yaw));

        s_leg_arrive[0] = s_leg_pos[0] = s_leg_yaw[0] = s_leg_dist[0] = 0;
        s_leg_tail[0] = s_leg_over[0] = s_leg_yover[0] = s_leg_lat[0] = 0;
        s_leg_osc[0] = s_leg_rev[0] = s_leg_mspeed[0] = s_leg_plen[0] = 0;
        for (k = 0; k < scen->n_legs; k++) {
            const LegResult *r = &g_results[k];
            size_t o;
            o = strlen(s_leg_arrive);
            if (o) s_leg_arrive[o++] = ';';
            snprintf(s_leg_arrive + o, sizeof(s_leg_arrive) - o, "%u:%u", (unsigned)r->kind, (unsigned)r->arrive_ms);

            o = strlen(s_leg_pos);
            if (o) s_leg_pos[o++] = ';';
            snprintf(s_leg_pos + o, sizeof(s_leg_pos) - o, "%.3f", r->pos_err_cm);

            o = strlen(s_leg_yaw);
            if (o) s_leg_yaw[o++] = ';';
            snprintf(s_leg_yaw + o, sizeof(s_leg_yaw) - o, "%.3f", r->yaw_err_deg);

            o = strlen(s_leg_dist);
            if (o) s_leg_dist[o++] = ';';
            snprintf(s_leg_dist + o, sizeof(s_leg_dist) - o, "%.3f", r->true_dist_cm);

            o = strlen(s_leg_tail);
            if (o) s_leg_tail[o++] = ';';
            snprintf(s_leg_tail + o, sizeof(s_leg_tail) - o, "%ld", r->settle_tail_ms);

            o = strlen(s_leg_over);
            if (o) s_leg_over[o++] = ';';
            snprintf(s_leg_over + o, sizeof(s_leg_over) - o, "%.3f", r->overshoot_cm);

            o = strlen(s_leg_yover);
            if (o) s_leg_yover[o++] = ';';
            snprintf(s_leg_yover + o, sizeof(s_leg_yover) - o, "%.3f", r->yaw_over_deg);

            o = strlen(s_leg_lat);
            if (o) s_leg_lat[o++] = ';';
            snprintf(s_leg_lat + o, sizeof(s_leg_lat) - o, "%.3f", r->lat_max_cm);

            o = strlen(s_leg_osc);
            if (o) s_leg_osc[o++] = ';';
            snprintf(s_leg_osc + o, sizeof(s_leg_osc) - o, "%d", r->near_osc);

            o = strlen(s_leg_rev);
            if (o) s_leg_rev[o++] = ';';
            snprintf(s_leg_rev + o, sizeof(s_leg_rev) - o, "%d", r->cmd_rev);

            o = strlen(s_leg_mspeed);
            if (o) s_leg_mspeed[o++] = ';';
            snprintf(s_leg_mspeed + o, sizeof(s_leg_mspeed) - o, "%.2f", r->max_speed);

            o = strlen(s_leg_plen);
            if (o) s_leg_plen[o++] = ';';
            snprintf(s_leg_plen + o, sizeof(s_leg_plen) - o, "%.1f", r->path_len_cm);
        }

        if (g_csv != NULL) {
            fprintf(g_csv,
                    "%s,%s,%s,%d,%d,%.1f,%.3f,%.1f,%.1f,%d,%d,%.0f,%.1f,%d,%u,%u,"
                    "%u,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,"
                    "%.3f,%.3f,%.3f,%.3f,"
                    "%.3f,%.3f,%.3f,%.3f,"
                    "%.3f,%.3f,%u,%d,%d,"
                    "%.2f,%.1f,%.2f,%.2f,"
                    "%u,%u,%u,%u\n",
                    g_tag,
                    g_suite,
                    scen->name,
                    scen->n_legs,
                    (int)delay_ms,
                    cfg->wheel_tau_s * 1000.0f,
                    cfg->slip,
                    cfg->ops_rate_hz,
                    cfg->ops_delay_ms,
                    cfg->rpm_quantize,
                    cfg->motor_acc_limit,
                    cfg->stop_decel_cm_s2,
                    cfg->stop_tau_s * 1000.0f,
                    cfg->jitter_max_ms,
                    (unsigned)cfg->seed,
                    (unsigned)nav_error,
                    (unsigned)last->arrive_ms,
                    s_leg_arrive,
                    s_leg_pos,
                    s_leg_yaw,
                    s_leg_dist,
                    s_leg_tail,
                    s_leg_over,
                    s_leg_yover,
                    s_leg_lat,
                    s_leg_osc,
                    s_leg_rev,
                    s_leg_mspeed,
                    s_leg_plen,
                    last->pos_err_cm,
                    last->yaw_err_deg,
                    end_pos_err,
                    end_yaw_err,
                    last->overshoot_cm,
                    last->overshoot_ratio,
                    last->yaw_over_deg,
                    last->yaw_dev_deg,
                    last->lat_max_cm,
                    last->lat_rms_cm,
                    (unsigned)last->near_zone_ms,
                    last->near_osc,
                    last->cmd_rev,
                    last->max_speed,
                    last->path_len_cm,
                    last->mean_speed,
                    g_pass_min_speed,
                    (unsigned)tmin,
                    (unsigned)tmax,
                    (unsigned)tmean,
                    (unsigned)(g_scenario_done ? (g_done_tick - g_scen_start_tick)
                                               : (Sim_Tick() - g_scen_start_tick)));
        }

        if (g_verbose) {
            printf("  %-16s d=%3dms tau=%.0fms slip=%.1f%% ops=%.0fHz err=%u arr=%4ums pos_err=%.2fcm "
                   "yaw_err=%.2fdeg over=%.2fcm osc=%d tail=%ldms\n",
                   scen->name,
                   (int)delay_ms,
                   cfg->wheel_tau_s * 1000.0f,
                   cfg->slip * 100.0f,
                   cfg->ops_rate_hz,
                   (unsigned)nav_error,
                   (unsigned)last->arrive_ms,
                   last->pos_err_cm,
                   last->yaw_err_deg,
                   last->overshoot_cm,
                   last->near_osc,
                   last->settle_tail_ms);
        }
    }
}

/* ======================= CSV ======================= */

static void csv_open(const char *out_dir) {
    char path[512];
    int need_header = 1;
    FILE *probe;

    snprintf(path, sizeof(path), "%s/runs_%s.csv", out_dir, g_tag);
    probe = fopen(path, "r");
    if (probe != NULL) {
        need_header = 0;
        fclose(probe);
    }
    g_csv = fopen(path, "a");
    if (g_csv == NULL) {
        fprintf(stderr, "ERROR: cannot open CSV %s\n", path);
        exit(2);
    }
    if (need_header) {
        fprintf(g_csv,
                "tag,suite,scenario,n_legs,delay_ms,tau_ms,slip,ops_rate_hz,ops_delay_ms,rpm_quant,motor_acc,"
                "stop_decel_cm_s2,stop_tau_ms,jitter_max_ms,seed,nav_error,"
                "arrive_total_ms,leg_arrive_ms,leg_pos_err_cm,leg_yaw_err_deg,leg_true_dist_cm,"
                "leg_settle_tail_ms,leg_overshoot_cm,leg_yaw_over_deg,leg_lat_max_cm,leg_osc,leg_cmd_rev,"
                "leg_max_speed,leg_path_len_cm,"
                "final_pos_err_cm,final_yaw_err_deg,end_pos_err_cm,end_yaw_err_deg,"
                "max_overshoot_cm,overshoot_ratio,max_yaw_overshoot_deg,max_yaw_dev_deg,"
                "lat_max_cm,lat_rms_cm,near_zone_ms,near_osc_count,cmd_reversals,"
                "max_speed_cm_s,path_len_cm,mean_speed_cm_s,pass_min_speed_cm_s,"
                "nav_dt_min_ms,nav_dt_max_ms,nav_dt_mean_ms,sim_time_ms\n");
    }
}

/* ======================= 套件 ======================= */

static void set_cfg_defaults(SimPlantCfg *cfg, int delay_ms) {
    SimCfg_Default(cfg);
    cfg->cmd_delay_ms = (float)delay_ms;
}

/* 延迟套件：全部（或 quick 子集）场景 x 延迟序列 */
static void suite_delay(const int *delays, int n_delays, int quick_only) {
    int si, di;
    SimPlantCfg cfg;

    for (si = 0; si < SCENARIO_COUNT; si++) {
        const SimScenario *sc = &s_scenarios[si];
        if (quick_only && !scenario_in_quick_set(sc)) continue;
        for (di = 0; di < n_delays; di++) {
            set_cfg_defaults(&cfg, delays[di]);
            run_one(sc, (uint32_t)delays[di], &cfg);
        }
    }
}

/* 不确定度套件：tau x slip x OPS 更新率 + 控制周期抖动 */
static void suite_uncertainty(void) {
    static const float taus[] = {0.040f, 0.060f, 0.080f, 0.120f};
    static const float slips[] = {0.0f, 0.05f, 0.10f};
    static const float ops_rates[] = {20.0f, 50.0f, 100.0f};
    static const int delays[] = {0, 50};
    static const char *subset[] = {"line100_70", "short5", "rot90", "pass_straight"};
    int ti, ki, ri, di, si, idx;
    SimPlantCfg cfg;

    for (ti = 0; ti < (int)(sizeof(taus) / sizeof(taus[0])); ti++) {
        for (ki = 0; ki < (int)(sizeof(slips) / sizeof(slips[0])); ki++) {
            for (ri = 0; ri < (int)(sizeof(ops_rates) / sizeof(ops_rates[0])); ri++) {
                for (di = 0; di < (int)(sizeof(delays) / sizeof(delays[0])); di++) {
                    for (si = 0; si < (int)(sizeof(subset) / sizeof(subset[0])); si++) {
                        for (idx = 0; idx < SCENARIO_COUNT; idx++) {
                            if (strcmp(s_scenarios[idx].name, subset[si]) == 0) break;
                        }
                        if (idx >= SCENARIO_COUNT) continue;
                        SimCfg_Default(&cfg);
                        cfg.cmd_delay_ms = (float)delays[di];
                        cfg.wheel_tau_s = taus[ti];
                        cfg.slip = slips[ki];
                        cfg.ops_rate_hz = ops_rates[ri];
                        run_one(&s_scenarios[idx], (uint32_t)delays[di], &cfg);
                    }
                }
            }
        }
    }

    /* 控制周期抖动（模拟命令队列阻塞 Nav_Task 造成的唤醒延迟） */
    {
        static const int jdelays[] = {0, 20, 50};
        int ji;
        for (ji = 0; ji < 3; ji++) {
            for (si = 0; si < (int)(sizeof(subset) / sizeof(subset[0])); si++) {
                for (idx = 0; idx < SCENARIO_COUNT; idx++) {
                    if (strcmp(s_scenarios[idx].name, subset[si]) == 0) break;
                }
                if (idx >= SCENARIO_COUNT) continue;
                SimCfg_Default(&cfg);
                cfg.cmd_delay_ms = (float)jdelays[ji];
                cfg.jitter_max_ms = 8;
                cfg.jitter_prob = 0.35f;
                run_one(&s_scenarios[idx], (uint32_t)jdelays[ji], &cfg);
            }
        }
    }

    /* STOP 制动模型不确定度（0xFE 立即停止的实际减速度/残余滑移未知） */
    {
        static const float stop_decels[] = {800.0f, 4000.0f, 8000.0f};
        static const float stop_taus[] = {0.02f, 0.06f};
        int a, b, dd;
        for (a = 0; a < (int)(sizeof(stop_decels) / sizeof(stop_decels[0])); a++) {
            for (b = 0; b < (int)(sizeof(stop_taus) / sizeof(stop_taus[0])); b++) {
                for (dd = 0; dd < 2; dd++) {
                    for (si = 0; si < (int)(sizeof(subset) / sizeof(subset[0])); si++) {
                        for (idx = 0; idx < SCENARIO_COUNT; idx++) {
                            if (strcmp(s_scenarios[idx].name, subset[si]) == 0) break;
                        }
                        if (idx >= SCENARIO_COUNT) continue;
                        SimCfg_Default(&cfg);
                        cfg.cmd_delay_ms = (float)delays[dd];
                        cfg.stop_decel_cm_s2 = stop_decels[a];
                        cfg.stop_tau_s = stop_taus[b];
                        run_one(&s_scenarios[idx], (uint32_t)delays[dd], &cfg);
                    }
                }
            }
        }
    }

    /* OPS 传输/测量延迟不确定度 */
    {
        static const float ops_delays[] = {5.0f, 20.0f, 50.0f};
        int c, dd;
        for (c = 0; c < (int)(sizeof(ops_delays) / sizeof(ops_delays[0])); c++) {
            for (dd = 0; dd < 2; dd++) {
                for (si = 0; si < (int)(sizeof(subset) / sizeof(subset[0])); si++) {
                    for (idx = 0; idx < SCENARIO_COUNT; idx++) {
                        if (strcmp(s_scenarios[idx].name, subset[si]) == 0) break;
                    }
                    if (idx >= SCENARIO_COUNT) continue;
                    SimCfg_Default(&cfg);
                    cfg.cmd_delay_ms = (float)delays[dd];
                    cfg.ops_delay_ms = ops_delays[c];
                    run_one(&s_scenarios[idx], (uint32_t)delays[dd], &cfg);
                }
            }
        }
    }
}

/* ======================= CLI ======================= */

static int find_scenario(const char *name) {
    int i;
    for (i = 0; i < SCENARIO_COUNT; i++) {
        if (strcmp(s_scenarios[i].name, name) == 0) return i;
    }
    return -1;
}

static void print_usage(const char *exe) {
    printf("Usage: %s --out-dir DIR --tag NAME --suite SUITE [options]\n", exe);
    printf("  --suite baseline   all scenarios x delays {0,10,20,50,100}\n");
    printf("  --suite delay      all scenarios x delays {0,20,50,100}\n");
    printf("  --suite quick      quick scenarios x delays {0,50}\n");
    printf("  --suite uncertainty tau/slip/OPS-rate/jitter matrix\n");
    printf("  --suite single     one scenario (--scenario NAME) x delay (--delay MS)\n");
    printf("  --scenario NAME    scenario name (see --list)\n");
    printf("  --delay MS         command path delay override\n");
    printf("  --tune name=value  runtime tunable (new nav only; repeatable up to %d)\n", MAX_TUNE);
    printf("  --trace            write one CSV trace per run (representative scenarios only)\n");
    printf("  --verbose          print per-run summary lines and nav logs\n");
    printf("  --list             list scenarios and exit\n");
}

int main(int argc, char **argv) {
    const char *out_dir = ".";
    const char *suite = "baseline";
    const char *scenario = NULL;
    int delay_override = -1;
    int i;
    int rc = 0;

    for (i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--out-dir") == 0 && i + 1 < argc) {
            out_dir = argv[++i];
        } else if (strcmp(argv[i], "--tag") == 0 && i + 1 < argc) {
            snprintf(g_tag, sizeof(g_tag), "%s", argv[++i]);
        } else if (strcmp(argv[i], "--suite") == 0 && i + 1 < argc) {
            suite = argv[++i];
        } else if (strcmp(argv[i], "--scenario") == 0 && i + 1 < argc) {
            scenario = argv[++i];
        } else if (strcmp(argv[i], "--delay") == 0 && i + 1 < argc) {
            delay_override = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--tune") == 0 && i + 1 < argc) {
            char *eq;
            if (g_tune_count >= MAX_TUNE) {
                fprintf(stderr, "too many --tune (max %d)\n", MAX_TUNE);
                return 2;
            }
            eq = strchr(argv[i + 1], '=');
            if (eq == NULL) {
                fprintf(stderr, "bad --tune '%s' (expect name=value)\n", argv[i + 1]);
                return 2;
            }
            *eq = '\0';
            snprintf(g_tunes[g_tune_count].name, sizeof(g_tunes[g_tune_count].name), "%s", argv[i + 1]);
            g_tunes[g_tune_count].value = (float)atof(eq + 1);
            g_tune_count++;
            i++;
        } else if (strcmp(argv[i], "--trace") == 0) {
            g_trace_enabled = 1;
        } else if (strcmp(argv[i], "--verbose") == 0) {
            g_verbose = 1;
        } else if (strcmp(argv[i], "--list") == 0) {
            int k;
            for (k = 0; k < SCENARIO_COUNT; k++) printf("%s (%d legs)\n", s_scenarios[k].name, s_scenarios[k].n_legs);
            return 0;
        } else if (strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        } else {
            fprintf(stderr, "unknown argument: %s\n", argv[i]);
            print_usage(argv[0]);
            return 2;
        }
    }

    g_out_dir[0] = '\0';
    snprintf(g_out_dir, sizeof(g_out_dir), "%s", out_dir);
    snprintf(g_suite, sizeof(g_suite), "%s", suite);
    Sim_SetVerbose(g_verbose);

    /* 初始化真实导航控制器（只初始化一次） */
    if (!Nav_Core_Init()) {
        fprintf(stderr, "ERROR: Nav_Core_Init failed\n");
        return 2;
    }

#ifdef SIM_HAVE_NAV_UPDATE
    Nav_GetTunables(&g_default_tunables);
    g_tunables_saved = 1;
    if (g_tune_count > 0) {
        int t;
        for (t = 0; t < g_tune_count; t++) {
            if (!Nav_SetTunable(g_tunes[t].name, g_tunes[t].value)) {
                fprintf(stderr, "WARN: Nav_SetTunable('%s', %g) rejected\n", g_tunes[t].name, (double)g_tunes[t].value);
            }
        }
    }
#else
    if (g_tune_count > 0) {
        fprintf(stderr, "WARN: --tune ignored (baseline build has no Nav_SetTunable)\n");
    }
#endif

    csv_open(out_dir);

    if (strcmp(suite, "baseline") == 0) {
        static const int delays[] = {0, 10, 20, 50, 100};
        suite_delay(delays, 5, 0);
    } else if (strcmp(suite, "delay") == 0) {
        static const int delays[] = {0, 20, 50, 100};
        suite_delay(delays, 4, 0);
    } else if (strcmp(suite, "quick") == 0) {
        static const int delays[] = {0, 50};
        suite_delay(delays, 2, 1);
    } else if (strcmp(suite, "uncertainty") == 0) {
        suite_uncertainty();
    } else if (strcmp(suite, "single") == 0) {
        SimPlantCfg cfg;
        int idx;
        int dly = (delay_override >= 0) ? delay_override : 0;
        if (scenario == NULL) {
            fprintf(stderr, "--suite single requires --scenario NAME\n");
            rc = 2;
        } else {
            idx = find_scenario(scenario);
            if (idx < 0) {
                fprintf(stderr, "unknown scenario '%s'\n", scenario);
                rc = 2;
            } else {
                set_cfg_defaults(&cfg, dly);
                run_one(&s_scenarios[idx], (uint32_t)dly, &cfg);
            }
        }
    } else {
        fprintf(stderr, "unknown suite '%s'\n", suite);
        rc = 2;
    }

    if (g_csv != NULL) {
        fclose(g_csv);
        g_csv = NULL;
    }

    if (rc == 0) {
        printf("[nav-sim] tag=%s suite=%s done\n", g_tag, g_suite);
    }
    return rc;
}
