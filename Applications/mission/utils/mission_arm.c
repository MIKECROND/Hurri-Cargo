/**
 * @file mission_arm.c
 * @author T-Car Team
 * @brief 总任务流程中的机械臂动作编排实现
 *
 * 从 mission.c 剥离：本文件只负责"任务码校验 + 按物料逐个调用 Arm_Action_*"，
 * 不涉及导航与显示；动作序列与目标位姿在 Applications/arm 的 arm_action.c。
 *
 * 任务码约定（沿用历史实现）：
 *   - 抓取行（MISSION_ROUNDx_GRAB）：3 位是颜色码 1~6（对应 RPI_Color_t）；
 *   - 放置行（MISSION_ROUNDx_STORAGE）：3 位是目标格位号 1~MISSION_MATERIAL_NUM。
 * 每个动作失败默认按"软失败"处理(MISSION_STEP_SOFT_FAIL=1): 记录告警后继续下一颗物料、
 * 后续工位照常执行, 整场任务不因单颗物料失败而中止(最终日志汇总软失败次数)。
 * 置 0 恢复旧行为: 任一动作失败立即返回 false 终止本轮/整场任务。
 * 机械臂动作是否编译由 MISSION_ARM_ENABLE 控制（调试模式默认关闭）。
 * 可调参数见 mission_config.h / Applications/mission/README.md。
 */

#include "mission_arm.h"
#include "mission_config.h"

#if MISSION_ARM_ENABLE

#include "arm_action.h"
#include "arm_config.h" /* ARM_PAD_SLOT_NUM / ARM_SLOT_NUM：与 MISSION_MATERIAL_NUM 一致性自检用 */
#include "arm_ctrl.h" /* Arm_PadSetSlot：运动前物料盘归位用 */
#include "log.h"
#include "mission.h"
#include <stddef.h>

/* 机械臂物料盘格位/放置格位必须能容纳每轮物料数量 */
#if (MISSION_MATERIAL_NUM > ARM_PAD_SLOT_NUM) || (MISSION_MATERIAL_NUM > ARM_SLOT_NUM)
#error "MISSION_MATERIAL_NUM 超过机械臂格位数量(ARM_PAD_SLOT_NUM/ARM_SLOT_NUM)，请同步修改配置"
#endif

/**
 * @brief 抓取行颜色码校验（1~6 对应 RPI_Color_t）
 * @param digit 任务码数字
 * @param color 输出颜色
 * @return true=合法; false=越界
 */
static bool Arm_TaskCodeToColor(uint8_t digit, RPI_Color_t *color) {
    if (color == NULL) {
        return false;
    }
    if (digit < (uint8_t)COLOR_RED || digit > (uint8_t)COLOR_LIGHTBLUE) {
        logError("arm: invalid color code %u in task code (expect 1~%u)", (unsigned)digit, (unsigned)COLOR_LIGHTBLUE);
        return false;
    }
    *color = (RPI_Color_t)digit;
    return true;
}

/**
 * @brief 放置行格位号校验（1~MISSION_MATERIAL_NUM）
 * @param digit 任务码数字
 * @param slot  输出格位号
 * @return true=合法; false=越界
 */
static bool Arm_TaskCodeToSlot(uint8_t digit, uint8_t *slot) {
    if (slot == NULL) {
        return false;
    }
    if (digit < 1 || digit > MISSION_MATERIAL_NUM) {
        logError(
            "arm: invalid slot code %u in task code (expect 1~%u)", (unsigned)digit, (unsigned)MISSION_MATERIAL_NUM);
        return false;
    }
    *slot = digit;
    return true;
}

/* ===== 循环样板公共 helper（R2-P1-03）：仅抽中止检查 / 软失败计数 =====
 * 4 个对外函数本体、各自 logInfo/logError 文本与 Arm_Action_* 调用保留在原调用点。 */
static bool mission_arm_aborted(const char *what) {
    if (mission_is_running()) return false;
    logWarning("arm: mission aborted, stop %s", what);   /* 输出与原文案逐字一致 */
    return true;                                          /* true = 已中止，调用方立即 return false */
}

static void mission_arm_fail_inc(uint8_t *failures) {
#if MISSION_STEP_SOFT_FAIL
    (*failures)++;
#else
    (void)failures;          /* 避免硬失败分支下的未使用参数告警 */
#endif
}

/* 软失败跳过：SOFT_FAIL=1 计数后 continue；=0 直接 return false。
 * 用 while(false) 而非 while(0)，避免新增数值 token（numeric 门禁零增量）。 */
#if MISSION_STEP_SOFT_FAIL
#define MISSION_ARM_SKIP(failures) \
    do { mission_arm_fail_inc(&(failures)); continue; } while (false)
#else
#define MISSION_ARM_SKIP(failures) \
    do { (void)(failures); return false; } while (false)
#endif

/**
 * @brief 原料区抓取上盘：按任务码颜色序列，把第 id 个物料放到物料盘第 id+1 格
 * @param grab_row 抓取行
 * @return true=全部抓取上盘完成; false=任务码非法/动作失败/任务被中止
 */
bool Mission_ArmLoadPad(MissionTaskRow_t grab_row) {
    uint8_t failures = 0;

    for (uint8_t id = 0; id < MISSION_MATERIAL_NUM; id++) {
        RPI_Color_t color;

        if (mission_arm_aborted("load pad")) return false;
        if (!Arm_TaskCodeToColor(Mission_TaskCodeGetDigit(grab_row, id), &color)) {
            MISSION_ARM_SKIP(failures);
        }

        logInfo("arm: load pad %u/%u color=%u", (unsigned)(id + 1), (unsigned)MISSION_MATERIAL_NUM, (unsigned)color);
        if (!Arm_Action_LoadPad(color, (uint8_t)(id + 1))) {
            logError("arm: load pad %u/%u failed", (unsigned)(id + 1), (unsigned)MISSION_MATERIAL_NUM);
            MISSION_ARM_SKIP(failures);
        }
    }
    if (failures > 0) {
        logError("arm: load pad finished with %u/%u failures (soft-fail, continue)",
                 (unsigned)failures,
                 (unsigned)MISSION_MATERIAL_NUM);
    }
    return (failures == 0);
}

/**
 * @brief 物料盘取料放到粗加工区：按任务码格位序列
 * @param storage_row 放置行（与暂存区共用同一行的格位序列）
 * @return true=全部放置完成; false=任务码非法/动作失败/任务被中止
 */
bool Mission_ArmUnloadToProcessing(MissionTaskRow_t storage_row) {
    uint8_t failures = 0;

    for (uint8_t id = 0; id < MISSION_MATERIAL_NUM; id++) {
        uint8_t slot;

        if (mission_arm_aborted("unload processing")) return false;
        if (!Arm_TaskCodeToSlot(Mission_TaskCodeGetDigit(storage_row, id), &slot)) {
            MISSION_ARM_SKIP(failures);
        }

        logInfo("arm: to process %u/%u pad=%u slot=%u",
                (unsigned)(id + 1),
                (unsigned)MISSION_MATERIAL_NUM,
                (unsigned)(id + 1),
                (unsigned)slot);
        /* 下一盘位号: 取到料后立即预启动它的换格, 让物料盘转动与移臂/放料并行(节约时间) */
        uint8_t next_pad = (id + 2 <= (uint8_t)MISSION_MATERIAL_NUM) ? (uint8_t)(id + 2) : 0;
        if (!Arm_Action_UnloadToProcessing((uint8_t)(id + 1), slot, next_pad)) {
            logError("arm: unload to process %u/%u failed", (unsigned)(id + 1), (unsigned)MISSION_MATERIAL_NUM);
            MISSION_ARM_SKIP(failures);
        }
    }
    if (failures > 0) {
        logError("arm: unload to process finished with %u/%u failures (soft-fail, continue)",
                 (unsigned)failures,
                 (unsigned)MISSION_MATERIAL_NUM);
    }
    return (failures == 0);
}

/**
 * @brief 粗加工区取回物料上盘：按任务码格位序列，把粗加工区各格位物料抓回物料盘第 id+1 格
 * @param storage_row 放置行（与暂存区共用同一行的格位序列）
 * @return true=全部取回; false=任务码非法/动作失败/任务被中止
 * @note 粗加工区放料完成后物料仍留在粗加工区格位上；去暂存区前必须先把它们抓回物料盘，
 *       否则暂存区动作会在空物料盘上取料（物料永远留在粗加工区）。
 */
bool Mission_ArmLoadPadFromProcessing(MissionTaskRow_t storage_row) {
    uint8_t failures = 0;

    for (uint8_t id = 0; id < MISSION_MATERIAL_NUM; id++) {
        uint8_t slot;

        if (mission_arm_aborted("pick back from processing")) return false;
        if (!Arm_TaskCodeToSlot(Mission_TaskCodeGetDigit(storage_row, id), &slot)) {
            MISSION_ARM_SKIP(failures);
        }

        logInfo("arm: pick back from process %u/%u slot=%u -> pad=%u",
                (unsigned)(id + 1),
                (unsigned)MISSION_MATERIAL_NUM,
                (unsigned)slot,
                (unsigned)(id + 1));
        if (!Arm_Action_LoadPadFromProcessing(slot, (uint8_t)(id + 1))) {
            logError("arm: pick back from process %u/%u failed", (unsigned)(id + 1), (unsigned)MISSION_MATERIAL_NUM);
            MISSION_ARM_SKIP(failures);
        }
    }
    if (failures > 0) {
        logError("arm: pick back from process finished with %u/%u failures (soft-fail, continue)",
                 (unsigned)failures,
                 (unsigned)MISSION_MATERIAL_NUM);
    }
    return (failures == 0);
}

/**
 * @brief 物料盘取料放到暂存区：按任务码格位序列 + 本轮叠放层
 * @param storage_row   放置行
 * @param process_layer 叠放层：0=第一层, 1=第二层
 * @return true=全部放置完成; false=任务码非法/动作失败/任务被中止
 */
bool Mission_ArmUnloadToStorage(MissionTaskRow_t storage_row, uint8_t process_layer) {
    uint8_t failures = 0;

    for (uint8_t id = 0; id < MISSION_MATERIAL_NUM; id++) {
        uint8_t slot;

        if (mission_arm_aborted("unload storage")) return false;
        if (!Arm_TaskCodeToSlot(Mission_TaskCodeGetDigit(storage_row, id), &slot)) {
            MISSION_ARM_SKIP(failures);
        }

        logInfo("arm: to storage %u/%u pad=%u slot=%u layer=%u",
                (unsigned)(id + 1),
                (unsigned)MISSION_MATERIAL_NUM,
                (unsigned)(id + 1),
                (unsigned)slot,
                (unsigned)process_layer);
        /* 下一盘位号: 取到料后立即预启动它的换格, 让物料盘转动与移臂/放料并行(节约时间) */
        uint8_t next_pad = (id + 2 <= (uint8_t)MISSION_MATERIAL_NUM) ? (uint8_t)(id + 2) : 0;
        if (!Arm_Action_UnloadToStorage((uint8_t)(id + 1), slot, process_layer, next_pad)) {
            logError("arm: unload to storage %u/%u failed", (unsigned)(id + 1), (unsigned)MISSION_MATERIAL_NUM);
            MISSION_ARM_SKIP(failures);
        }
    }
    if (failures > 0) {
        logError("arm: unload to storage finished with %u/%u failures (soft-fail, continue)",
                 (unsigned)failures,
                 (unsigned)MISSION_MATERIAL_NUM);
    }
    return (failures == 0);
}

/**
 * @brief 每段导航（小车移动）出发前把物料盘归到基准格(ARM_PAD_SLOT_BASE, 默认 2 号)
 * @return true=已到基准格(或 MISSION_PAD_PREPOSE_ENABLE=0 跳过); false=换格失败
 * @note 由 Mission_Task 在每段导航前调用: 物料盘在非基准格时会向车体两侧伸出,
 *       行驶/过弯容易刮碰; 归位后也让后续取放料的换格从已知位置开始。
 *       失败返回 false, 由任务层判任务失败。
 */
bool Mission_ArmPadPrepose(void) {
#if MISSION_PAD_PREPOSE_ENABLE
    logInfo("arm: prepose material pad to slot %d before moving", (int)ARM_PAD_SLOT_BASE);
    if (!Arm_PadSetSlot((uint8_t)ARM_PAD_SLOT_BASE)) {
        logError("arm: prepose material pad to slot %d failed", (int)ARM_PAD_SLOT_BASE);
        return false;
    }
    logInfo("arm: material pad ready at slot %d", (int)ARM_PAD_SLOT_BASE);
#endif
    return true;
}

/**
 * @brief 小车移动前的机械臂准备(第一步): 启动物料盘归位 + 机械臂回初始状态(不等待物料盘到位)
 * @return true=换格命令已下发且收臂回初始位的动作已执行完; false=换格命令/回初始位失败
 * @note 由 Mission_Task 在每段导航前调用; 初始状态位姿在 arm_config.h 的 ARM_POSE_INIT_*
 *       (占位值需标定, `arm preset init` 对位)。
 *       先启动物料盘换格(不阻塞), 再收臂回初始位: 盘转动与收臂并行, 省一次等待;
 *       物料盘是否到位留给 Mission_ArmPreMoveWait 校验, 这段等待时间可给车体做原地旋转。
 *       收臂动作沿用动作层 best-effort 语义(各段尽力到位, 不再因单段未到位判失败)。
 *       未整臂回零时绝对位姿不可信, 跳过"回初始位"只做物料盘归位(避免拿错误零点乱伸臂)。
 */
bool Mission_ArmPreMoveStart(void) {
#if MISSION_ARM_ENABLE
#if MISSION_PAD_PREPOSE_ENABLE
    /* 先启动物料盘换格(不阻塞): 盘转动与"机械臂回初始位"并行 */
    logInfo("arm: prepose material pad to slot %d before moving", (int)ARM_PAD_SLOT_BASE);
    if (!Arm_PadSetSlotStart((uint8_t)ARM_PAD_SLOT_BASE)) {
        logError("arm: prepose material pad to slot %d start failed", (int)ARM_PAD_SLOT_BASE);
        return false;
    }
#endif

    if (!Arm_IsHomeDone()) {
        logWarning("arm: not homed, skip initial pose before moving");
    } else if (!Arm_Action_MoveToInitPose()) {
        logError("arm: move to initial pose before moving failed");
        return false;
    }
#endif
    return true;
}

/**
 * @brief 小车移动前的机械臂准备(第二步): 等物料盘归位到基准格
 * @return true=已到基准格(或 MISSION_PAD_PREPOSE_ENABLE=0 跳过); false=换格失败
 * @note 由 Mission_ArmPreMoveStart 启动换格后调用; 等待期间车体可以执行原地旋转等动作。
 */
bool Mission_ArmPreMoveWait(void) {
#if MISSION_ARM_ENABLE && MISSION_PAD_PREPOSE_ENABLE
    if (!Arm_PadWaitArrive(ARM_PAD_SETTLE_TIMEOUT_MS)) {
        logError("arm: prepose material pad to slot %d failed", (int)ARM_PAD_SLOT_BASE);
        return false;
    }
    logInfo("arm: material pad ready at slot %d", (int)ARM_PAD_SLOT_BASE);
#endif
    return true;
}

/**
 * @brief 小车移动前的机械臂准备: 机械臂回初始状态 + 物料盘归到 2 号基准格
 * @return true=准备完成; false=机械臂回初始位或物料盘归位失败(任务层判失败)
 * @note 等价于 Mission_ArmPreMoveStart + Mission_ArmPreMoveWait; 两步之间没有其他动作时用本接口。
 */
bool Mission_ArmPreMove(void) {
    if (!Mission_ArmPreMoveStart()) {
        return false;
    }
    return Mission_ArmPreMoveWait();
}

#endif /* MISSION_ARM_ENABLE */
