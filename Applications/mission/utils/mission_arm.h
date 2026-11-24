/**
 * @file mission_arm.h
 * @author T-Car Team
 * @brief 总任务流程中的机械臂动作编排
 *
 * 从 mission.c 剥离：只负责"任务码校验 + 按物料逐个执行机械臂动作"的编排，
 * 不涉及导航与显示；动作序列与目标位姿在 Applications/arm 的 arm_action.c。
 *
 * 任务码约定（沿用历史实现）：
 *   - 抓取行（MISSION_ROUNDx_GRAB）：3 位是颜色码 1~6（对应 RPI_Color_t）；
 *   - 放置行（MISSION_ROUNDx_STORAGE）：3 位是目标格位号 1~MISSION_MATERIAL_NUM。
 * 每个动作失败都会返回 false 并终止本轮（机械臂可能停在中间位姿，不再继续后续动作）。
 * 机械臂动作是否编译由 mission_config.h 的 MISSION_ARM_ENABLE 控制（调试模式默认关闭）。
 * 可调参数见 mission_config.h / Applications/mission/README.md。
 */

#ifndef __MISSION_ARM_H__
#define __MISSION_ARM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "mission_code.h"

/**
 * @brief 原料区抓取上盘：按任务码颜色序列，把第 id 个物料放到物料盘第 id+1 格
 * @param grab_row 抓取行
 * @return true=全部抓取上盘完成; false=任务码非法/动作失败/任务被中止
 */
bool Mission_ArmLoadPad(MissionTaskRow_t grab_row);

/**
 * @brief 物料盘取料放到粗加工区：按任务码格位序列
 * @param storage_row 放置行（与暂存区共用同一行的格位序列）
 * @return true=全部放置完成; false=任务码非法/动作失败/任务被中止
 */
bool Mission_ArmUnloadToProcessing(MissionTaskRow_t storage_row);

/**
 * @brief 粗加工区取回物料上盘：按任务码格位序列，把粗加工区各格位物料抓回物料盘第 id+1 格
 * @param storage_row 放置行（与暂存区共用同一行的格位序列）
 * @return true=全部取回; false=任务码非法/动作失败/任务被中止
 * @note  粗加工区放料完成后物料仍在粗加工区格位上；去暂存区前必须先抓回物料盘，
 *        否则暂存区会从空物料盘取料。由 Run_Round 在粗加工区放置/加工等待之后调用。
 */
bool Mission_ArmLoadPadFromProcessing(MissionTaskRow_t storage_row);

/**
 * @brief 物料盘取料放到暂存区：按任务码格位序列 + 本轮叠放层
 * @param storage_row   放置行
 * @param process_layer 叠放层：0=第一层, 1=第二层
 * @return true=全部放置完成; false=任务码非法/动作失败/任务被中止
 */
bool Mission_ArmUnloadToStorage(MissionTaskRow_t storage_row, uint8_t process_layer);

/**
 * @brief 每段导航（小车移动）出发前把物料盘归到基准格（ARM_PAD_SLOT_BASE，默认 2 号）
 * @return true=已到基准格（或 MISSION_PAD_PREPOSE_ENABLE=0 跳过）; false=换格失败
 * @note  由 Mission_Task 在每段导航前调用：物料盘在非基准格时会向车体两侧伸出，行驶/过弯容易刮碰；
 *        失败按任务失败处理。仅在 MISSION_ARM_ENABLE=1 时编译/调用。
 */
bool Mission_ArmPadPrepose(void);

/**
 * @brief 小车移动前的机械臂准备（第一步）：启动物料盘归位 + 机械臂回初始状态
 * @return true=物料盘换格命令已下发且收臂回初始位的动作已执行完; false=归位/回初始位失败
 * @note  供任务层在"每段导航出发前"调用；只做到"换格命令已下发、收臂动作已执行完"就返回，
 *        不等待物料盘到位，以便调用方在这段等待时间里安排其他动作（例如路线首段的原地旋转）。
 *        收臂动作沿用动作层 best-effort 语义（各段尽力到位，不再因单段未到位判失败）。
 *        与 Mission_ArmPreMoveWait 配对使用；不需要重叠动作时直接用 Mission_ArmPreMove。
 *        先启动物料盘换格（非阻塞）再收臂：盘转动与"机械臂回初始位"并行，省一次等待。
 *        未整臂回零时绝对位姿不可信，跳过"回初始位"只做物料盘归位（避免拿错误零点乱伸臂）。
 */
bool Mission_ArmPreMoveStart(void);

/**
 * @brief 小车移动前的机械臂准备（第二步）：等待物料盘归位到基准格
 * @return true=已到基准格（或 MISSION_PAD_PREPOSE_ENABLE=0 跳过）; false=换格失败
 * @note  由 Mission_ArmPreMoveStart 启动换格后调用；等待期间车体可以执行其他动作（如原地旋转）。
 */
bool Mission_ArmPreMoveWait(void);

/**
 * @brief 小车移动前的机械臂准备：先回机械臂初始状态（收短 + 抬升），再把物料盘归到 2 号基准格
 * @return true=准备完成; false=机械臂回初始位或物料盘归位失败（任务层判失败）
 * @note  由 Mission_Task 在每段导航（小车出发）前调用；初始状态位姿在 arm_config.h 的 ARM_POSE_INIT_*
 *        （占位值需标定，`arm preset init` 对位）。未整臂回零时跳过"回初始位"、只做物料盘归位。
 *        等价于 Mission_ArmPreMoveStart + Mission_ArmPreMoveWait（两步间不放其他动作时用本接口）。
 *        仅在 MISSION_ARM_ENABLE=1 时编译/调用。
 */
bool Mission_ArmPreMove(void);

#ifdef __cplusplus
}
#endif

#endif /* __MISSION_ARM_H__ */
