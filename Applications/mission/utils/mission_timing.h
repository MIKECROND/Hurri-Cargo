/**
 * @file mission_timing.h
 * @author T-Car Team
 * @brief 任务分阶段耗时仪表（现场调参用的轻量统计）
 *
 * 目的：把整场任务拆成"导航段 / 每段导航前的机械臂准备(ArmPreMove) / 机械臂取放料动作批次 /
 * 二次标定 / 状态码等待 / 雷达扫描 / 固定稳定延时 / 完成显示"等类别分别累计耗时，
 * 让"车到底慢在哪"能够被量化，而不是靠猜。
 *
 * 用法（只在任务线程里调用，实现见 mission.c，不额外新增编译单元）：
 *   MissionTiming_Reset()          ：新一轮任务开始时清零（mission.c 计时启动处已调用）；
 *   MissionTiming_SetPhase()       ：标记当前阶段（扫码/第一轮/第二轮/回家），用于阶段小计；
 *   MissionTiming_Begin(id)        ：开始计时某个类别（同一类别不允许嵌套，重复 Begin 被忽略）；
 *   MissionTiming_End(id)          ：结束计时并把本次耗时累加进该类别的 total/max/last/count；
 *   MissionTiming_Count(counter, n)：累加次数型统计（导航段数/停车次数/重试/动作数等）；
 *   MissionTiming_PrintSummary()   ：打印汇总表（任务结束 done/failed/stopped 时打印，shell
 *                                    `mission time` 也可随时打印）。
 *
 * 统计口径（重要）：
 *   - MISSION_TM_NAV / ARM_PREMOVE / ARM_ACTION / CAL / QR_WAIT / RADAR / SETTLE / FINISH
 *     是"互斥阶段"：同一时刻只会有一个在计时，因此它们之和 + other ≈ 总时长，汇总表按总时长算占比；
 *   - MISSION_TM_PREMOVE_PAD / PREMOVE_ROTATE / CAL_ARM / CAL_RPI / CAL_POS / NAV_RETRY
 *     是"明细分解"：它们已经包含在对应互斥阶段里，只用于看阶段内部构成，不参与占比合计；
 *   - 所有读写都是 32 位对齐标量，任务线程写、shell 线程读（可能读到"上一拍"的值），
 *     不做关中断/互斥，避免在任务控制路径引入额外开销；日志只在段结束/任务结束打印。
 * 可调参数见 mission_config.h / Applications/mission/README.md。
 */

#ifndef __MISSION_TIMING_H__
#define __MISSION_TIMING_H__

#include "mission_config.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 计时类别
 *
 * 前 MISSION_TM_EXCLUSIVE_NUM 项是互斥阶段（之和≈总时长），后面的都是明细分解。
 */
typedef enum {
    /* ===== 互斥阶段（参与占比合计） ===== */
    MISSION_TM_NAV = 0, // 导航段（含导航层到位停车确认/失败重试等待，PASS 链整体算一次）
    MISSION_TM_ARM_PREMOVE, // 每段导航前的机械臂准备（收臂回初始位 + 物料盘归位，含预旋转重叠时间）
    MISSION_TM_ARM_ACTION, // 机械臂取放料动作批次（一批 = 每轮每个工位的 MISSION_MATERIAL_NUM 个物料动作）
    MISSION_TM_CAL, // 树莓派二次标定（机械臂预备 + 视觉应答 + 位置模式精修）
    MISSION_TM_QR_WAIT, // 等待合法任务码
    MISSION_TM_RADAR, // 雷达扫描（RPI_RADAR_AVAILABLE=1 时才有实际耗时）
    MISSION_TM_SETTLE, // 固定稳定延时 MISSION_SETTLE_MS
    MISSION_TM_FINISH, // 完成显示 / 完赛数量交互

    MISSION_TM_EXCLUSIVE_NUM, // = 互斥阶段数量（内部用）

    /* ===== 明细分解（已包含在上面某项里，不参与占比合计） ===== */
    MISSION_TM_PREMOVE_PAD = MISSION_TM_EXCLUSIVE_NUM, // 准备期"物料盘归位等待"（ArmPreMove 内）
    MISSION_TM_PREMOVE_ROTATE, // 准备期"车体原地预旋转"（与物料盘归位并行，ArmPreMove 内）
    MISSION_TM_CAL_ARM, // 标定前机械臂预备位姿（Cal 内）
    MISSION_TM_CAL_RPI, // 视觉标定等待树莓派应答（Cal 内）
    MISSION_TM_CAL_POS, // 标定后底盘位置模式精修（Cal 内）
    MISSION_TM_NAV_RETRY, // 导航失败后的重试等待（Nav 内，仅失败路径）

    MISSION_TM_COUNT // 类别总数
} MissionTimingId_t;

/**
 * @brief 次数型统计（不按时间累计）
 */
typedef enum {
    MISSION_TC_NAV_LEG = 0, // 执行的导航段数（含 PASS 段，含 ROTATE 段）
    MISSION_TC_NAV_PASS_LEG, // 其中不停车通过(PASS)段数
    MISSION_TC_NAV_STOP, // 其中会进入导航层 STOPPING 停车的段数（普通到点段）
    MISSION_TC_NAV_RETRY, // 导航失败重试次数
    MISSION_TC_PREROTATE, // 准备期预旋转启动次数
    MISSION_TC_PREROTATE_STOP, // 准备期预旋转实际到位次数（每次也会触发一次 STOPPING 停车确认）
    MISSION_TC_ARM_ACTION, // 机械臂物料动作个数（单次抓/放）
    MISSION_TC_CAL, // 二次标定次数
    MISSION_TC_PREMOVE, // 每段导航前准备次数

    MISSION_TC_COUNT
} MissionTimingCounter_t;

/**
 * @brief 任务阶段（用于"按轮次/阶段"小计）
 */
typedef enum {
    MISSION_PHASE_NONE = 0, // 未进入任何阶段（雷达/启动准备之前）
    MISSION_PHASE_QR, // 扫码阶段（雷达扫描 + 导航到扫码点 + 等待任务码）
    MISSION_PHASE_ROUND1, // 第一轮（原料抓取 -> 粗加工 -> 暂存）
    MISSION_PHASE_ROUND2, // 第二轮
    MISSION_PHASE_HOME, // 完成显示 + 回家

    MISSION_PHASE_COUNT
} MissionPhase_t;

#if MISSION_TIMING_ENABLE

/**
 * @brief 清零全部计时统计（新一轮任务开始时调用；shell 查询不受影响）
 */
void MissionTiming_Reset(void);

/**
 * @brief 切换当前任务阶段（把上一阶段经过的时间计入小计）
 * @param phase 新阶段；MISSION_PHASE_NONE 表示结束当前阶段小计
 * @note 只用于阶段小计，不影响互斥类别统计
 */
void MissionTiming_SetPhase(MissionPhase_t phase);

/**
 * @brief 取当前阶段的短名（日志用，如 "qr"/"r1"/"r2"/"home"）
 */
const char *MissionTiming_PhaseName(void);

/**
 * @brief 开始计时一个类别
 * @param id 类别；越界或该类别已在计时中时忽略（防止嵌套重复计时）
 */
void MissionTiming_Begin(MissionTimingId_t id);

/**
 * @brief 结束计时并把本次耗时累加
 * @param id 类别
 * @return 本次耗时(ms)；该类别未在计时时返回 0
 */
uint32_t MissionTiming_End(MissionTimingId_t id);

/**
 * @brief 直接累加一次已知耗时（用于外部已经量好时间、或不要 Begin/End 配对的场合）
 * @param id 类别
 * @param elapsed_ms 本次耗时(ms)
 */
void MissionTiming_Add(MissionTimingId_t id, uint32_t elapsed_ms);

/**
 * @brief 累加次数型统计
 * @param counter 统计项
 * @param n 本次增量（0 表示不变）
 */
void MissionTiming_Count(MissionTimingCounter_t counter, uint16_t n);

/**
 * @brief 打印汇总表（任务结束 done/failed/stopped 时打印；shell `mission time` 也可随时打印）
 * @param total_ms 任务总时长(ms)，用于算各项占比
 * @note 日志量：约 10 行，只在任务结束/手动查询时打印，不在分段执行路径里打印
 */
void MissionTiming_PrintSummary(uint32_t total_ms);

#else /* MISSION_TIMING_ENABLE */

/* 编译期关闭：全部变成空操作，调用点代码不变（便于整机省 Flash/完全排除计时影响）。
 * 用 static inline 空函数而不是 ((void)0)/0u：调用点既有"当语句用"也有"取返回值"的写法，
 * 空函数两种写法都不会触发 armcc "expression has no effect / unused value" 告警。 */
static inline void mission_timing_noop_void(void) {
}

static inline uint32_t mission_timing_noop_u32(uint32_t value) {
    (void)value;
    return 0u;
}

#define MissionTiming_Reset() mission_timing_noop_void()
#define MissionTiming_SetPhase(phase) mission_timing_noop_u32((uint32_t)(phase))
#define MissionTiming_PhaseName() ("?")
#define MissionTiming_Begin(id) mission_timing_noop_u32((uint32_t)(id))
#define MissionTiming_End(id) mission_timing_noop_u32((uint32_t)(id))
#define MissionTiming_Add(id, ms) mission_timing_noop_u32((uint32_t)(id) + (uint32_t)(ms))
#define MissionTiming_Last(id) mission_timing_noop_u32((uint32_t)(id))
#define MissionTiming_Count(counter, n) mission_timing_noop_u32((uint32_t)(counter) + (uint32_t)(n))
#define MissionTiming_PrintSummary(total_ms) mission_timing_noop_u32((uint32_t)(total_ms))

#endif /* MISSION_TIMING_ENABLE */

#ifdef __cplusplus
}
#endif

#endif /* __MISSION_TIMING_H__ */
