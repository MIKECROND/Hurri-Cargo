/**
 * @file mission.c
 * @author T-Car Team
 * @brief 总任务流程编排
 * 可调参数见 mission_config.h / Applications/mission/README.md。
 */

#include "mission.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "Events.h"
#include "Key.h"
#include "log.h"
#include "mission_arm.h"
#include "mission_config.h"
#include "mission_nav.h"
#include "mission_route.h"
#include "mission_timing.h"
#include "mission_ui.h"
#include "mission_code.h"
#include "ops.h"
#include "rpi_port.h"
#include "arm_ctrl.h"
#include "arm_config.h"
#include "scan_driver.h"
#include "task.h"
#include <stdint.h>
#include <string.h>

/* ===== 子任务失败/超时软处理 =====
 * MISSION_STEP_SOFT_FAIL=1: 单段导航/二维码等待/一轮流程/回家等"小步骤"失败时只记录并继续
 * （不 goto fail），整场任务继续跑未完成的部分；0=旧行为(任一失败立即中止整场)。
 * 注意：真正的中止(mission stop)仍会立即返回 false，不参与软失败。 */
static uint16_t s_soft_step_failures = 0;

#if MISSION_STEP_SOFT_FAIL
static void mission_soft_step_failed(const char *name) {
    s_soft_step_failures++;
    logError("Mission: step '%s' failed/timeout, continue remaining steps (soft-fail #%u)",
             name,
             (unsigned)s_soft_step_failures);
}
/* 软失败宏: 表达式失败时, 若任务已被用户 stop/中止 则立即走 aborted 结束本轮, 不按软失败继续 */
#define MISSION_STEP(expr, name)                                                                                       \
    do {                                                                                                               \
        if (!(expr)) {                                                                                                 \
            if (!mission_is_running()) goto aborted;                                                                   \
            mission_soft_step_failed(name);                                                                            \
        }                                                                                                              \
    } while (0)
#else
#define MISSION_STEP(expr, name)                                                                                       \
    do {                                                                                                               \
        if (!(expr)) goto fail;                                                                                        \
    } while (0)
#endif

/* 扫码结果缓冲区长度：足够容纳最大条码及显示时的临时截断 */
#define MISSION_BARCODE_BUF_SIZE 32

static bool s_mission_running = false; /* 总任务是否正在运行 */

/* ===== 任务运行记时 (run -> stop/完成/失败) ===== */
static uint32_t s_mission_run_start_tick = 0; /* 本轮运行开始时刻(tick) */
static uint32_t s_mission_run_elapsed_ms = 0; /* 最近一次运行时长(ms) */
static bool s_mission_run_timing = false; /* 是否正在计时 */

/**
 * @brief 打印时长(ms)为 h:mm:ss.mmm
 * @param tag 日志前缀(如 "done"/"failed"/"stopped")
 * @param elapsed_ms 时长(ms)
 */
static void mission_log_run_time(const char *tag, uint32_t elapsed_ms) {
    uint32_t rest = elapsed_ms;
    uint32_t hours = rest / 3600000u;
    rest %= 3600000u;
    uint32_t minutes = rest / 60000u;
    rest %= 60000u;
    uint32_t seconds = rest / 1000u;
    uint32_t millis = rest % 1000u;
    logInfo("Mission: %s, run time %u:%02u:%02u.%03u (%u ms)",
            tag,
            (unsigned)hours,
            (unsigned)minutes,
            (unsigned)seconds,
            (unsigned)millis,
            (unsigned)elapsed_ms);
}

/**
 * @brief 启动任务记时(仅在实际开始新一轮任务时调用, 已在计时中不重置)
 */
static void mission_run_timer_start(void) {
    if (s_mission_run_timing) return;
    s_mission_run_start_tick = osKernelGetTickCount();
    s_mission_run_elapsed_ms = 0;
    s_mission_run_timing = true;
    MissionTiming_Reset(); /* 新一轮任务: 清空上一轮的分阶段耗时统计 */
}

/**
 * @brief 停止任务记时并记录时长(重复调用无副作用)
 * @param tag 日志前缀("done"/"failed"/"stopped")
 * @return 本次运行时长(ms); 未计时过则返回最近一次的时长
 */
static uint32_t mission_run_timer_stop(const char *tag) {
    if (s_mission_run_timing) {
        s_mission_run_elapsed_ms = osKernelGetTickCount() - s_mission_run_start_tick;
        s_mission_run_timing = false;
    }
    mission_log_run_time(tag, s_mission_run_elapsed_ms);
    return s_mission_run_elapsed_ms;
}

/**
 * @brief 结束本轮计时并打印耗时汇总表(只在真正结束本轮时调用一次)
 * @param tag 日志前缀("done"/"failed"/"stopped")
 * @return 本轮运行时长(ms)
 */
static uint32_t mission_run_timer_finish(const char *tag) {
    uint32_t elapsed_ms = mission_run_timer_stop(tag);

    MissionTiming_SetPhase(MISSION_PHASE_NONE); /* 把最后一个阶段的时间结账 */
#if MISSION_TIMING_SUMMARY_ENABLE
    logInfo("Mission timing breakdown:");
    MissionTiming_PrintSummary(elapsed_ms);
#endif
    return elapsed_ms;
}

/**
 * @brief 取当前计时值: 运行中返回实时时长, 未运行返回最近一次时长
 * @return 时长(ms)
 */
static uint32_t mission_run_timer_elapsed(void) {
    if (s_mission_run_timing) {
        return osKernelGetTickCount() - s_mission_run_start_tick;
    }
    return s_mission_run_elapsed_ms;
}

/* ==========================================================================
 * 分阶段耗时仪表实现（对外接口与统计口径见 utils/mission_timing.h）
 *
 * 设计取舍：
 *   - 只在任务线程里调用 Begin/End/Count，全部是整数加减，无动态内存/无关中断/无锁；
 *   - shell 线程查询时可能读到"上一拍"的值，只影响显示精度，不影响任务；
 *   - 日志只在"每段结束一行"与"任务结束/手动查询的汇总表"打印，不在 5ms 控制路径打印。
 * ========================================================================== */
#if MISSION_TIMING_ENABLE

/** 单个计时类别的累计统计 */
typedef struct {
    uint32_t total_ms; // 累计耗时(ms)
    uint32_t max_ms; // 单次最长(ms)
    uint32_t last_ms; // 最近一次(ms)
    uint16_t count; // 次数
} MissionTimingStat_t;

/* 互斥阶段的短名（用于汇总表），顺序必须与 MissionTimingId_t 的前 MISSION_TM_EXCLUSIVE_NUM 项一致 */
static const char *const s_timing_exclusive_name[MISSION_TM_EXCLUSIVE_NUM] = {
    "nav", // MISSION_TM_NAV
    "premove", // MISSION_TM_ARM_PREMOVE
    "arm-action", // MISSION_TM_ARM_ACTION
    "cal", // MISSION_TM_CAL
    "qr-wait", // MISSION_TM_QR_WAIT
    "radar", // MISSION_TM_RADAR
    "settle", // MISSION_TM_SETTLE
    "finish", // MISSION_TM_FINISH
};

/* 阶段短名（用于日志前缀/阶段小计） */
static const char *const s_timing_phase_name[MISSION_PHASE_COUNT] = {
    "none", // MISSION_PHASE_NONE
    "qr", // MISSION_PHASE_QR
    "r1", // MISSION_PHASE_ROUND1
    "r2", // MISSION_PHASE_ROUND2
    "home", // MISSION_PHASE_HOME
};

static MissionTimingStat_t s_timing_stat[MISSION_TM_COUNT]; // 分类耗时统计
static uint32_t s_timing_start_tick[MISSION_TM_COUNT]; // Begin 时刻(tick)
static bool s_timing_active[MISSION_TM_COUNT]; // 该类别是否正在计时
static uint16_t s_timing_counter[MISSION_TC_COUNT]; // 次数型统计
static uint32_t s_timing_phase_ms[MISSION_PHASE_COUNT]; // 各阶段小计(ms)
static MissionPhase_t s_timing_phase = MISSION_PHASE_NONE; // 当前阶段
static uint32_t s_timing_phase_start_tick = 0; // 当前阶段开始时刻(tick)

/* 汇总表百分比(千分比, 如 294 表示 29.4%)：任务总时长都在分钟级，ms*1000 不会溢出 uint32。
 * 用 pct_base 而不是直接除 total_ms，避免总时长为 0(极端情况下刚启动就停)时除零。 */
#define MISSION_TIMING_PCT10(ms_, base_) (uint32_t)(((uint32_t)(ms_)) * 1000u / (base_))

/* 汇总表基础行格式（6 处内联格式的唯一来源；相邻字符串字面量拼接后字节完全一致） */
#define MISSION_TIMING_LINE_FMT "  %-11s %7u ms  %3u.%u%%"

/**
 * @brief 打印一行"类别 耗时 占比"（共享基础行，后缀字段由调用点自行拼接）
 */
static void mission_timing_print_line(const char *name, uint32_t ms,
                                      uint32_t pct_hi, uint32_t pct_lo) {
    logPrintln(MISSION_TIMING_LINE_FMT, name, (unsigned)ms, (unsigned)pct_hi, (unsigned)pct_lo);
}

void MissionTiming_Reset(void) {
    static const MissionTimingStat_t k_zero_stat = {0, 0, 0, 0};

    for (uint8_t i = 0; i < (uint8_t)MISSION_TM_COUNT; i++) {
        s_timing_stat[i] = k_zero_stat;
        s_timing_start_tick[i] = 0;
        s_timing_active[i] = false;
    }
    for (uint8_t i = 0; i < (uint8_t)MISSION_TC_COUNT; i++) {
        s_timing_counter[i] = 0;
    }
    for (uint8_t i = 0; i < (uint8_t)MISSION_PHASE_COUNT; i++) {
        s_timing_phase_ms[i] = 0;
    }
    s_timing_phase = MISSION_PHASE_NONE;
    s_timing_phase_start_tick = osKernelGetTickCount();
}

void MissionTiming_SetPhase(MissionPhase_t phase) {
    uint32_t now = osKernelGetTickCount();

    if ((uint32_t)phase >= (uint32_t)MISSION_PHASE_COUNT) {
        return;
    }
    /* 先把上一阶段经过的时间结掉，再切到新阶段（NONE 表示只结账不进入新阶段） */
    if (s_timing_phase != MISSION_PHASE_NONE) {
        s_timing_phase_ms[s_timing_phase] += now - s_timing_phase_start_tick;
    }
    s_timing_phase = phase;
    s_timing_phase_start_tick = now;
}

const char *MissionTiming_PhaseName(void) {
    if ((uint32_t)s_timing_phase >= (uint32_t)MISSION_PHASE_COUNT) {
        return s_timing_phase_name[MISSION_PHASE_NONE];
    }
    return s_timing_phase_name[s_timing_phase];
}

void MissionTiming_Begin(MissionTimingId_t id) {
    if ((uint32_t)id >= (uint32_t)MISSION_TM_COUNT || s_timing_active[id]) {
        return; /* 越界或重复 Begin(嵌套)直接忽略，保证每个类别同一时刻只计一次 */
    }
    s_timing_start_tick[id] = osKernelGetTickCount();
    s_timing_active[id] = true;
}

uint32_t MissionTiming_End(MissionTimingId_t id) {
    uint32_t elapsed_ms;

    if ((uint32_t)id >= (uint32_t)MISSION_TM_COUNT || !s_timing_active[id]) {
        return 0;
    }
    s_timing_active[id] = false;
    elapsed_ms = osKernelGetTickCount() - s_timing_start_tick[id];
    MissionTiming_Add(id, elapsed_ms);
    return elapsed_ms;
}

void MissionTiming_Add(MissionTimingId_t id, uint32_t elapsed_ms) {
    MissionTimingStat_t *stat;

    if ((uint32_t)id >= (uint32_t)MISSION_TM_COUNT) {
        return;
    }
    stat = &s_timing_stat[id];
    stat->total_ms += elapsed_ms;
    stat->last_ms = elapsed_ms;
    if (elapsed_ms > stat->max_ms) {
        stat->max_ms = elapsed_ms;
    }
    if (stat->count < 0xFFFFu) {
        stat->count++;
    }
}

uint32_t MissionTiming_Last(MissionTimingId_t id) {
    if ((uint32_t)id >= (uint32_t)MISSION_TM_COUNT) {
        return 0;
    }
    return s_timing_stat[id].last_ms;
}

void MissionTiming_Count(MissionTimingCounter_t counter, uint16_t n) {
    if ((uint32_t)counter >= (uint32_t)MISSION_TC_COUNT) {
        return;
    }
    if ((uint32_t)s_timing_counter[counter] + (uint32_t)n > 0xFFFFu) {
        s_timing_counter[counter] = 0xFFFFu;
    } else {
        s_timing_counter[counter] = (uint16_t)(s_timing_counter[counter] + n);
    }
}

/**
 * @brief 取某类别当前累计耗时(运行中把正在计时的这一段也算进去, 便于 shell 随时查询)
 */
static uint32_t mission_timing_elapsed_ms(MissionTimingId_t id) {
    uint32_t ms = s_timing_stat[id].total_ms;

    if (s_timing_active[id]) {
        ms += osKernelGetTickCount() - s_timing_start_tick[id];
    }
    return ms;
}

/**
 * @brief 打印一行"类别 耗时 占比"
 */
static void mission_timing_print_exclusive(MissionTimingId_t id, uint32_t total_ms) {
    uint32_t pct_base = (total_ms != 0) ? total_ms : 1u;
    uint32_t ms = mission_timing_elapsed_ms(id);

    mission_timing_print_line(s_timing_exclusive_name[id], ms,
                              MISSION_TIMING_PCT10(ms, pct_base) / 10u,
                              MISSION_TIMING_PCT10(ms, pct_base) % 10u);
}

void MissionTiming_PrintSummary(uint32_t total_ms) {
    uint32_t now = osKernelGetTickCount();
    uint32_t pct_base = (total_ms != 0) ? total_ms : 1u;
    uint32_t exclusive_sum = 0;
    uint32_t stop_cnt;
    uint32_t stop_est_ms;
    uint32_t other_ms;
    uint16_t nav_leg = s_timing_counter[MISSION_TC_NAV_LEG];
    uint16_t nav_pass = s_timing_counter[MISSION_TC_NAV_PASS_LEG];
    uint16_t nav_stop = s_timing_counter[MISSION_TC_NAV_STOP];
    uint16_t nav_retry = s_timing_counter[MISSION_TC_NAV_RETRY];
    uint16_t premove_cnt = s_timing_counter[MISSION_TC_PREMOVE];
    uint16_t rotate_cnt = s_timing_counter[MISSION_TC_PREROTATE_STOP];
    uint16_t arm_action = s_timing_counter[MISSION_TC_ARM_ACTION];
    uint16_t cal_cnt = s_timing_counter[MISSION_TC_CAL];

    /* 从未跑过任务(上电后直接查询)时不做无意义的全 0 打印 */
    if (total_ms == 0 && nav_leg == 0 && premove_cnt == 0 && arm_action == 0 && cal_cnt == 0) {
        return;
    }

    logPrintln("=== Mission timing: total %u.%us (%u ms) ===",
               (unsigned)(total_ms / 1000u),
               (unsigned)((total_ms % 1000u) / 100u),
               (unsigned)total_ms);

    /* 阶段小计：把当前阶段的实时耗时也算进去（shell 运行中查询时同样有效） */
    {
        uint32_t live = (s_timing_phase != MISSION_PHASE_NONE) ? (now - s_timing_phase_start_tick) : 0;
        uint32_t qr_ms = s_timing_phase_ms[MISSION_PHASE_QR] + ((s_timing_phase == MISSION_PHASE_QR) ? live : 0);
        uint32_t r1_ms =
            s_timing_phase_ms[MISSION_PHASE_ROUND1] + ((s_timing_phase == MISSION_PHASE_ROUND1) ? live : 0);
        uint32_t r2_ms =
            s_timing_phase_ms[MISSION_PHASE_ROUND2] + ((s_timing_phase == MISSION_PHASE_ROUND2) ? live : 0);
        uint32_t home_ms = s_timing_phase_ms[MISSION_PHASE_HOME] + ((s_timing_phase == MISSION_PHASE_HOME) ? live : 0);

        logPrintln("  phase       qr %u.%us | r1 %u.%us | r2 %u.%us | home %u.%us",
                   (unsigned)(qr_ms / 1000u),
                   (unsigned)((qr_ms % 1000u) / 100u),
                   (unsigned)(r1_ms / 1000u),
                   (unsigned)((r1_ms % 1000u) / 100u),
                   (unsigned)(r2_ms / 1000u),
                   (unsigned)((r2_ms % 1000u) / 100u),
                   (unsigned)(home_ms / 1000u),
                   (unsigned)((home_ms % 1000u) / 100u));
    }

    /* 互斥阶段分类（各项之和 + other ≈ 总时长） */
    for (uint8_t i = 0; i < (uint8_t)MISSION_TM_EXCLUSIVE_NUM; i++) {
        exclusive_sum += mission_timing_elapsed_ms((MissionTimingId_t)i);
    }

    /* 导航：段数 / PASS 段数 / 停车次数 / 重试次数，是本轮调"快"的主要观测项 */
    {
        uint32_t ms = mission_timing_elapsed_ms(MISSION_TM_NAV);
        logPrintln(MISSION_TIMING_LINE_FMT "  legs %u (pass %u, stop %u, retry %u)",
                   s_timing_exclusive_name[MISSION_TM_NAV],
                   (unsigned)ms,
                   (unsigned)(MISSION_TIMING_PCT10(ms, pct_base) / 10u),
                   (unsigned)(MISSION_TIMING_PCT10(ms, pct_base) % 10u),
                   (unsigned)nav_leg,
                   (unsigned)nav_pass,
                   (unsigned)nav_stop,
                   (unsigned)nav_retry);
    }
    /* 每段导航前的机械臂准备：总次数 + 内部"物料盘归位等待/预旋转"分解 */
    {
        uint32_t ms = mission_timing_elapsed_ms(MISSION_TM_ARM_PREMOVE);
        uint32_t pad_ms = mission_timing_elapsed_ms(MISSION_TM_PREMOVE_PAD);
        uint32_t rot_ms = mission_timing_elapsed_ms(MISSION_TM_PREMOVE_ROTATE);
        logPrintln(MISSION_TIMING_LINE_FMT "  x%u (pad-wait %u, prerotate %u x%u overlap)",
                   s_timing_exclusive_name[MISSION_TM_ARM_PREMOVE],
                   (unsigned)ms,
                   (unsigned)(MISSION_TIMING_PCT10(ms, pct_base) / 10u),
                   (unsigned)(MISSION_TIMING_PCT10(ms, pct_base) % 10u),
                   (unsigned)premove_cnt,
                   (unsigned)pad_ms,
                   (unsigned)rot_ms,
                   (unsigned)rotate_cnt);
    }
    /* 机械臂动作批次：批次耗时 + 单个物料动作平均耗时 */
    {
        uint32_t ms = mission_timing_elapsed_ms(MISSION_TM_ARM_ACTION);
        uint32_t per = (arm_action != 0) ? (ms / arm_action) : 0;
        logPrintln(MISSION_TIMING_LINE_FMT "  actions %u (%u ms/action)",
                   s_timing_exclusive_name[MISSION_TM_ARM_ACTION],
                   (unsigned)ms,
                   (unsigned)(MISSION_TIMING_PCT10(ms, pct_base) / 10u),
                   (unsigned)(MISSION_TIMING_PCT10(ms, pct_base) % 10u),
                   (unsigned)arm_action,
                   (unsigned)per);
    }
    /* 二次标定：总次数 + "机械臂预备/树莓派应答/位置精修"三分解 */
    {
        uint32_t ms = mission_timing_elapsed_ms(MISSION_TM_CAL);
        uint32_t arm_ms = mission_timing_elapsed_ms(MISSION_TM_CAL_ARM);
        uint32_t rpi_ms = mission_timing_elapsed_ms(MISSION_TM_CAL_RPI);
        uint32_t pos_ms = mission_timing_elapsed_ms(MISSION_TM_CAL_POS);
        logPrintln(MISSION_TIMING_LINE_FMT "  x%u (arm-prep %u, rpi %u, pos-fix %u)",
                   s_timing_exclusive_name[MISSION_TM_CAL],
                   (unsigned)ms,
                   (unsigned)(MISSION_TIMING_PCT10(ms, pct_base) / 10u),
                   (unsigned)(MISSION_TIMING_PCT10(ms, pct_base) % 10u),
                   (unsigned)cal_cnt,
                   (unsigned)arm_ms,
                   (unsigned)rpi_ms,
                   (unsigned)pos_ms);
    }
    /* 其余互斥阶段：qr-wait / radar / settle / finish */
    mission_timing_print_exclusive(MISSION_TM_QR_WAIT, total_ms);
    mission_timing_print_exclusive(MISSION_TM_RADAR, total_ms);
    mission_timing_print_exclusive(MISSION_TM_SETTLE, total_ms);
    mission_timing_print_exclusive(MISSION_TM_FINISH, total_ms);

    /* 未归类时间（启动/停止收尾、阶段切换、日志打印等） */
    other_ms = (total_ms > exclusive_sum) ? (total_ms - exclusive_sum) : 0;
    mission_timing_print_line("other", other_ms,
                              MISSION_TIMING_PCT10(other_ms, pct_base) / 10u,
                              MISSION_TIMING_PCT10(other_ms, pct_base) % 10u);

    /* 停车耗时提示：导航层每段到点有一次 STOPPING 停车确认（含准备期预旋转），
     * 该时间已包含在 nav/premove 段内，这里只把次数与估算耗时单列，便于评估"减少停车"的收益。 */
    stop_cnt = (uint32_t)nav_stop + (uint32_t)rotate_cnt;
    stop_est_ms = stop_cnt * (uint32_t)MISSION_TIMING_NAV_STOP_EST_MS;
    logPrintln("  nav-stop    %u x %ums = %u ms (included in nav/premove)",
               (unsigned)stop_cnt,
               (unsigned)MISSION_TIMING_NAV_STOP_EST_MS,
               (unsigned)stop_est_ms);
}

#endif /* MISSION_TIMING_ENABLE */

#if RPI_RADAR_AVAILABLE == 1
static bool Radar_Scan(void);
#endif
static bool Run_Round(MissionTaskRow_t grab_row, MissionTaskRow_t storage_row, uint8_t process_layer);
static void Mission_Finish(void);
static bool Wait_Qr(void);
static bool HomeSweetHome(void);
static bool NavRouteAfterArmPreMove(const MissionRoute_t *route, const char *tag);

/**
 * @brief 导航出发前先做机械臂准备(回初始状态 + 物料盘归位), 再执行一段路线
 * @param route 路线
 * @param tag   阶段名(日志)
 * @return true=准备并导航完成; false=机械臂回初始位/物料盘归位失败或导航失败
 * @note 物料盘换格伺服在非基准格时会向车体两侧伸出, 带着它行驶/过弯容易刮碰;
 *       机械臂带长臂/低姿态行驶也不安全。因此每次小车要移动(每段导航)前:
 *       先让机械臂回初始状态(收短+抬升), 再把物料盘归到基准格, 最后才出发。
 *       机械臂关闭(MISSION_ARM_ENABLE=0)的台架流程不做这些准备, 只导航。
 *       物料盘归位期间车体允许原地旋转: 若本段路线首段是原地旋转段(MISSION_NAV_MODE_ROTATE),
 *       把这段旋转提前到归位期间并行执行(收臂动作执行完才启动), 归位完成时旋转已基本转好,
 *       路线首段不再重复旋转(见 mission_config.h 的 MISSION_PAD_PREPOSE_ROTATE_ENABLE);
 *       预旋转失败只告警并回退为按原路线执行, 不影响任务正确性。
 */
static bool NavRouteAfterArmPreMove(const MissionRoute_t *route, const char *tag) {
#if MISSION_ARM_ENABLE
    bool pre_rotate = false;
    bool skip_first_leg = false;
    bool ok = false;
    bool premove_ok = true; /* 导航前机械臂准备是否成功(失败时按 MISSION_PREMOVE_SOFT_FAIL 决定是否继续) */
    MissionRoute_t rest = {.legs = NULL, .count = 0};

    MissionTiming_Count(MISSION_TC_PREMOVE, 1);
    /* 每段导航前的准备整体计入 ARM_PREMOVE（内部再分解"物料盘归位等待"与"预旋转"） */
    MissionTiming_Begin(MISSION_TM_ARM_PREMOVE);

#if MISSION_PAD_PREPOSE_ROTATE_ENABLE && MISSION_PAD_PREPOSE_ENABLE
    /* 只有路线首段是"原地旋转"时才预旋转: 提前的只是路线本来就要做的转体, 不改变平移路径;
     * 机械臂未回零时不提前转(未回零的臂姿态不可信, 可能带着长臂/低姿态转体) */
    pre_rotate = (Arm_IsHomeDone() && route != NULL && route->legs != NULL && route->count > 0 &&
                  route->legs[0].mode == MISSION_NAV_MODE_ROTATE);
#endif

    if (!Mission_ArmPreMoveStart()) {
#if MISSION_PREMOVE_SOFT_FAIL
        /* 软处理(不安全): 机械臂/物料盘可能没收回, 继续导航有刮碰风险, 仅现场确认净空足够才开 */
        logError("Mission[%s] arm pre-move start failed/timeout, continue navigation (arm/pad may not be stowed!)",
                 tag);
        premove_ok = false;
        pre_rotate = false;
#else
        goto out;
#endif
    }

#if MISSION_PAD_PREPOSE_ROTATE_ENABLE && MISSION_PAD_PREPOSE_ENABLE
    if (pre_rotate && !mission_is_running()) {
        goto out; /* 启动前任务已被中止: 不再动车 */
    }
    if (pre_rotate) {
        /* 收臂回初始位的动作已执行完, 车体才允许转: 旋转与物料盘换格动作并行,
         * 物料盘是否到位由下面的 Mission_ArmPreMoveWait 统一确认 */
        MissionTiming_Count(MISSION_TC_PREROTATE, 1);
        if (!Mission_NavStartRotateInPlace(route->legs[0].yaw_deg)) {
            logWarning("Mission[%s] pre-rotate start failed, fall back to route", tag);
            pre_rotate = false;
        } else {
            bool rotate_ok;
            /* 预旋转与物料盘归位并行, 这段耗时已包含在 ARM_PREMOVE 里, 这里单独记录便于看重叠是否划算 */
            MissionTiming_Begin(MISSION_TM_PREMOVE_ROTATE);
            /* 先等旋转(内部按中止事件及时停车), 再等物料盘归位: 若先等归位(最长 5s、不检查中止),
             * 任务中止时车体可能还在转 */
            rotate_ok = Mission_NavWaitRotateInPlace();
            MissionTiming_End(MISSION_TM_PREMOVE_ROTATE);
            if (!rotate_ok) {
                if (!mission_is_running()) {
                    goto out; /* 任务被中止: Wait_Arrive 已停车, 不再继续 */
                }
                /* 预旋转未按时到位: 停车后按原路线重跑, 首段旋转由路线自己完成 */
                logWarning("Mission[%s] pre-rotate failed, fall back to full route", tag);
                Mission_NavStop();
                pre_rotate = false;
            } else {
                MissionTiming_Count(MISSION_TC_PREROTATE_STOP, 1); /* 到位也会触发一次停车确认 */
            }
        }
    }
#endif

    /* 物料盘归位等待单独计时（在预旋转之后才等，所以这里通常是"剩余等待"） */
    MissionTiming_Begin(MISSION_TM_PREMOVE_PAD);
    ok = premove_ok ? Mission_ArmPreMoveWait() : true;
    MissionTiming_End(MISSION_TM_PREMOVE_PAD);
    if (!ok) {
        /* 预旋转可能已启动: 归位失败时不要留下运动中的车体 */
        Mission_NavStop();
#if MISSION_PREMOVE_SOFT_FAIL
        logError("Mission[%s] arm pre-move wait failed/timeout, continue navigation (arm/pad may not be stowed!)", tag);
        ok = true;
#else
        goto out;
#endif
    }

    skip_first_leg = pre_rotate;
    ok = true;

out:
    MissionTiming_End(MISSION_TM_ARM_PREMOVE);
    if (!ok) {
        return false;
    }

    if (skip_first_leg) {
        /* 首段旋转已提前执行完: 从第二段开始执行, 避免重复旋转(空转也会多等一次到位/停车) */
        rest.legs = route->legs + 1;
        rest.count = (uint8_t)(route->count - 1);
        logInfo("Mission[%s] leg 1/%u rotate done during pad prepose, skip it", tag, (unsigned int)route->count);
        return Mission_NavGoToRoute(&rest, tag);
    }
#endif /* MISSION_ARM_ENABLE */

    return Mission_NavGoToRoute(route, tag);
}

/**
 * @brief 设置总任务运行状态
 * @note  停止时会置任务中止标志，并中止阻塞中的规划调用
 * @param running 运行状态
 */
void mission_set_running(bool running) {
    s_mission_running = running;

    if (running) {
        /* 新一轮任务开始：清掉上一次的中止标志，并开始记时(run -> stop) */
        osEventFlagsClear(System_StatusHandle, MISSION_ABORT);
        mission_run_timer_start();
    } else {
        /* 主动停止：置中止标志唤醒正在等待导航/二维码的任务，并中止阻塞中的规划 */
        osEventFlagsSet(System_StatusHandle, MISSION_ABORT);
        Mission_NavAbort();
        /* stop 命令即本次运行的计时终点(任务线程随后走到 fail 分支时不再重复记录) */
        if (s_mission_run_timing) {
            mission_run_timer_finish("stopped");
        }
    }
}

/**
 * @brief 查询总任务是否正在运行
 * @return true=正在运行; false=未运行或已被请求停止
 */
bool mission_is_running(void) {
    return s_mission_running;
}

/**
 * @brief 查询任务运行记时(从 run 到 stop/完成/失败)
 * @param elapsed_ms 输出: 运行时长(ms), 可为 NULL
 * @return true=任务正在运行(计时中); false=任务未运行(elapsed_ms 为最近一次运行时长)
 */
bool mission_get_run_time(uint32_t *elapsed_ms) {
    if (elapsed_ms != NULL) {
        *elapsed_ms = mission_run_timer_elapsed();
    }
    return s_mission_run_timing;
}

/**
 * @brief 总任务执行函数
 */
void Mission_Task(void *argument) {
    (void)argument;

    osEventFlagsWait(System_StatusHandle, SYS_INIT_COMPLETE, osFlagsNoClear, osWaitForever);

#if MISSION_DEBUG_MODE
    logInfo("Mission: DEBUG mode (MISSION_DEBUG_MODE=1), RPI calibration DISABLED");
#elif MISSION_CAL_ENABLE
    logInfo("Mission: RELEASE mode, RPI calibration ENABLED");
#else
    logInfo("Mission: RELEASE mode, RPI calibration DISABLED (MISSION_CAL_ENABLE=0)");
#endif

#if MISSION_ARM_ENABLE
    logInfo("Mission: arm actions ENABLED (task code: grab colors / place slots)");
#else
    logInfo("Mission: arm actions DISABLED (MISSION_ARM_ENABLE=0), navigation only");
#endif

    for (;;) {
        osDelay(1);
        if (Key_Up_Scan()) {
            mission_set_running(true);
        }

        if (!s_mission_running) continue;
        logInfo("Mission Start (stack HWM %u words free)", (unsigned)uxTaskGetStackHighWaterMark(NULL));
        s_soft_step_failures = 0; /* 新一轮: 清空软失败计数 */
        OPS_Zero(); /* 码盘清零 */

        /* 阶段小计: 扫码阶段(雷达扫描 + 导航到扫码点 + 等待任务码) */
        MissionTiming_SetPhase(MISSION_PHASE_QR);

#if RPI_RADAR_AVAILABLE == 1
        {
            bool radar_ok;
            MissionTiming_Begin(MISSION_TM_RADAR);
            radar_ok = Radar_Scan();
            MissionTiming_End(MISSION_TM_RADAR);
            if (!radar_ok) logError("radar ERROR");
        }
#else
        logInfo("radar not enabled");
#endif

        Arm_FlipSetAngle(ARM_FLIP_CAL_ANGLE_DEG);
        /* 导航到扫码点，等待二维码识别出合法任务码。
         * 导航/等待失败按软失败处理（配置见 mission_config.h 的 MISSION_STEP_SOFT_FAIL）：
         * 记录告警后继续后续流程, 用当前任务码(默认/上次)跑, 无效码的物料动作会再按软失败跳过。 */
        MISSION_STEP(NavRouteAfterArmPreMove(Mission_RouteQr(), "qr"), "qr-nav");
        {
            bool qr_ok;
            MissionTiming_Begin(MISSION_TM_QR_WAIT);
            qr_ok = Wait_Qr();
            MissionTiming_End(MISSION_TM_QR_WAIT);
            MISSION_STEP(qr_ok, "qr-wait");
        }

        /* 第一轮：原料抓取 -> 粗加工(第一层) -> 暂存 */
        MissionTiming_SetPhase(MISSION_PHASE_ROUND1);
        MISSION_STEP(Run_Round(MISSION_ROUND1_GRAB, MISSION_ROUND1_STORAGE, 0), "round1");

        /* 第二轮：原料抓取 -> 粗加工(第二层) -> 暂存(叠放) */
        MissionTiming_SetPhase(MISSION_PHASE_ROUND2);
        MISSION_STEP(Run_Round(MISSION_ROUND2_GRAB, MISSION_ROUND2_STORAGE, 1), "round2");

        /* 完成显示 + 回家阶段 */
        MissionTiming_SetPhase(MISSION_PHASE_HOME);
        MissionTiming_Begin(MISSION_TM_FINISH);
        Mission_Finish();
        MissionTiming_End(MISSION_TM_FINISH);

        /* 回到 Home 点 */
        MISSION_STEP(HomeSweetHome(), "home");

        Mission_NavStop();
        s_mission_running = false;
        /* 正常完成才记录(若用户已 stop, 计时已在 stop 时结束, 不重复记录) */
        if (s_mission_run_timing) {
            mission_run_timer_finish("done");
        }
        if (s_soft_step_failures > 0) {
            logError("Mission done with %u soft step failure(s), stack HWM %u words free",
                     (unsigned)s_soft_step_failures,
                     (unsigned)uxTaskGetStackHighWaterMark(NULL));
        } else {
            logInfo("Mission done (stack HWM %u words free)", (unsigned)uxTaskGetStackHighWaterMark(NULL));
        }
        continue;

#if MISSION_STEP_SOFT_FAIL
    aborted:
        /* 软失败流程中任务被用户停止: 立即停车并结束本轮(不继续未完成部分) */
        Mission_NavStop();
        s_mission_running = false;
        if (s_mission_run_timing) {
            mission_run_timer_finish("stopped");
        }
        logWarning("Mission stopped by user during running step (soft-fail path)");
#else
    fail:
        Mission_NavStop();
        s_mission_running = false;
        /* 用户 stop 时计时已结束: 这里只在任务自身失败/完成时记录一次 */
        if (s_mission_run_timing) {
            mission_run_timer_finish("failed");
        }
        logError("Mission Failed (stack HWM %u words free)", (unsigned)uxTaskGetStackHighWaterMark(NULL));
#endif /* MISSION_STEP_SOFT_FAIL */
    }
}

/**
 * @brief 固定稳定延时(带计时): 机械臂动作后的放置稳定等待
 * @note  用函数包一层只是为了把 MISSION_SETTLE_MS 计入耗时统计，行为与原 osDelay 完全一致
 */
static void Mission_DelaySettle(void) {
    MissionTiming_Begin(MISSION_TM_SETTLE);
    osDelay(MISSION_SETTLE_MS);
    MissionTiming_End(MISSION_TM_SETTLE);
}

#if MISSION_ARM_ENABLE
/**
 * @brief 机械臂动作批次结束: 结束计时、累计动作个数、打印一行本批耗时
 * @param ok   该批动作结果
 * @param name 批次名(日志用, 如 "load-pad")
 * @return ok 原样返回, 便于调用点写成 if (!Mission_ArmBatchDone(...)) return false;
 * @note  一个批次 = 一个工位的一轮物料动作(MISSION_MATERIAL_NUM 个), 日志每批一行
 */
static bool Mission_ArmBatchDone(bool ok, const char *name) {
    uint32_t elapsed_ms = MissionTiming_End(MISSION_TM_ARM_ACTION);

    if (ok) {
        MissionTiming_Count(MISSION_TC_ARM_ACTION, MISSION_MATERIAL_NUM);
    }
#if MISSION_TIMING_ENABLE && MISSION_TIMING_LOG_LEG
    logInfo("MissionTiming: [%s] arm %s %u ms%s",
            MissionTiming_PhaseName(),
            name,
            (unsigned)elapsed_ms,
            ok ? "" : " FAILED");
#else
    (void)name;
    (void)elapsed_ms;
#endif
    return ok;
}
#endif /* MISSION_ARM_ENABLE */

/**
 * @brief 执行一轮"原料抓取 -> 粗加工 -> 暂存"流程
 * @param grab_row      原料区抓取颜色序列所在任务码行
 * @param storage_row   暂存区/粗加工区放置格位序列所在任务码行
 * @param process_layer 粗加工区叠放层：0=第一层, 1=第二层
 * @return true 本轮全部完成
 * @note  各点位树莓派二次标定类型、via 点、导航方式均由 mission_route.c 的路线表决定；
 *        本函数只负责按阶段取路线、执行与机械臂动作衔接。
 */
static bool Run_Round(MissionTaskRow_t grab_row, MissionTaskRow_t storage_row, uint8_t process_layer) {
    bool ok = true;
    uint8_t round_no = (uint8_t)(process_layer + 1);

#if !MISSION_ARM_ENABLE
    /* 机械臂动作关闭（调试/台架）：任务码参数不参与流程 */
    (void)grab_row;
    (void)storage_row;
#endif

    /* 1) 按启动区/轮次走固定路线进入原料区（到位后按路线表做圆盘标定）。
     *    导航失败/超时: 记录告警, 跳过本阶段的机械臂动作(车不在正确位置), 继续下一阶段。 */
    if (!NavRouteAfterArmPreMove(Mission_RouteGrab(process_layer), "grab")) {
        logError("Mission: round%u grab navigation failed/timeout, skip grab arm actions (continue)",
                 (unsigned)round_no);
        ok = false;
    } else {
        /* 2) 原料抓取上盘：按任务码颜色序列依次抓取放到物料盘（第 id 个放第 id+1 格） */
#if MISSION_ARM_ENABLE
        MissionTiming_Begin(MISSION_TM_ARM_ACTION);
        if (!Mission_ArmBatchDone(Mission_ArmLoadPad(grab_row), "load-pad")) {
            ok = false;
        }
#endif
    }

    /* 3) 粗加工区：从物料盘按任务码格位序列取料，放到粗加工区对应格位（到位后圆环第一次标定） */
    if (!NavRouteAfterArmPreMove(Mission_RouteProcessing(), "processing")) {
        logError("Mission: round%u processing navigation failed/timeout, skip processing arm actions (continue)",
                 (unsigned)round_no);
        ok = false;
    } else {
#if MISSION_ARM_ENABLE
        MissionTiming_Begin(MISSION_TM_ARM_ACTION);
        if (!Mission_ArmBatchDone(Mission_ArmUnloadToProcessing(storage_row), "unload-process")) {
            ok = false;
        }
#endif
        Mission_DelaySettle(); /* 放置稳定/粗加工节拍 */

        /* 4) 粗加工取回：物料仍留在粗加工区格位上，先按同样格位序列抓回物料盘，
         *    再带物料去暂存区（否则暂存区会从空物料盘取料，物料永远留在粗加工区） */
#if MISSION_ARM_ENABLE
        MissionTiming_Begin(MISSION_TM_ARM_ACTION);
        if (!Mission_ArmBatchDone(Mission_ArmLoadPadFromProcessing(storage_row), "pick-process")) {
            ok = false;
        }
#endif
    }

    /* 5) 暂存区：先经锁路线 via 点/OA2，入位后按轮次标定圆环（RING_FIRST/RING_SECOND），
     *    再从物料盘按任务码格位序列取料放到暂存区对应格位（第二轮叠放） */
    if (!NavRouteAfterArmPreMove(Mission_RouteStorage(process_layer), "storage")) {
        logError("Mission: round%u storage navigation failed/timeout, skip storage arm actions (continue)",
                 (unsigned)round_no);
        ok = false;
    } else {
#if MISSION_ARM_ENABLE
        MissionTiming_Begin(MISSION_TM_ARM_ACTION);
        if (!Mission_ArmBatchDone(Mission_ArmUnloadToStorage(storage_row, process_layer), "unload-storage")) {
            ok = false;
        }
#endif
    }
    Mission_DelaySettle();

    return ok;
}

/**
 * @brief 等待二维码识别
 * @return true 识别到合法任务码
 * @return false 超时或任务被中止
 */
static bool Wait_Qr(void) {
    /* 静态缓冲: 避免在 Mission 任务栈上再占 100B(二维码解析期间任务栈还要向下调用导航/显示) */
    static uint8_t barcode[MISSION_BARCODE_BUF_SIZE];
    uint32_t start_tick = osKernelGetTickCount();

    for (;;) {
        if (!s_mission_running) return false;

        /* 先尝试消费已暂存的条码；非法条码显示后继续等待，只有合法任务码才结束等待 */
        if (Scan_GetLatestBarcode(barcode, sizeof(barcode))) {
            bool valid = Mission_TaskCodeParse(barcode);
            Mission_UiShowBarcode(barcode, valid);
            if (valid) return true;
        }

        uint32_t elapsed = osKernelGetTickCount() - start_tick;
        if (elapsed >= MISSION_QR_TIMEOUT_MS) {
            logWarning("qr timeout");
            return false;
        }

        /* 阻塞等待新条码或任务中止事件，不再 10ms 轮询 */
        osEventFlagsWait(
            System_StatusHandle, SCAN_EVT_BARCODE | MISSION_ABORT, osFlagsWaitAny, MISSION_QR_TIMEOUT_MS - elapsed);
    }
}

/**
 * @brief 沿固定回家路线返回 Home
 * @return true 已回到 Home
 */
static bool HomeSweetHome(void) {
    return NavRouteAfterArmPreMove(Mission_RouteHome(), "home");
}

#if RPI_RADAR_AVAILABLE == 1
/**
 * @brief 扫描雷达
 * @return 扫描状态
 */
static bool Radar_Scan(void) {
    RPI_ScanStatus_t st;
    uint32_t retry = 0;
    do {
        st = RPI_SetGridVal();
        if (st == RPI_SCAN_ERROR && ++retry > MISSION_RADAR_RETRY_MAX) {
            logError("radar scan failed");
            return false;
        }
    } while (st != RPI_SCAN_DONE);
    return true;
}
#endif /* RPI_RADAR_AVAILABLE */

/**
 * @brief 完成任务：获取完成计数并在 LCD 显示
 */
static void Mission_Finish(void) {
    int num_1 = 6, num_2 = 6;
    const char *mission_msg = "Mission Finish without complete num";

#if RPI_COMPLETENUM_ENABLED == 1
    if (RPI_SetCompleteNum(&num_1, &num_2)) {
        mission_msg = "Mission Finish";
    } else {
        logError("RPI_SetCompleteNum failed");
        num_1 = num_2 = 6; // 失败时回退
    }
#endif /* RPI_COMPLETENUM_ENABLED */

    Mission_UiShowComplete(num_1, num_2);
    logInfo("%s", mission_msg);
}
