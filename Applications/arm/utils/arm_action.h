/**
 * @file arm_action.h
 * @author T-Car Team
 * @brief 机械臂任务动作: 原料抓取上盘、物料盘取料放到暂存区/粗加工区
 *
 * 供 mission 任务流程调用。每个动作按"移动到位 -> 夹爪动作 -> 抬升撤离"的顺序执行,
 * 全部使用 Arm_MoveToAndWait 保证到位后再开合夹爪, 避免抓空/撞料。
 *
 * 返回值约定: 每个动作函数返回 bool, true=全部步骤确认到位; false=参数非法/
 * 运动未到位/颜色识别超时/物料盘换格失败。动作中途失败时机械臂可能停在中间位姿,
 * 调用方应把本轮任务判失败, 不要继续执行后续动作。
 *
 * 物料盘约定:
 *   - 物料盘由总线舵机驱动做直线运动, 夹爪到物料盘的取放位置固定;
 *   - 取放时由物料盘把目标格位送到夹爪下方(arm_ctrl 的 Arm_PadSetSlot),
 *     机械臂不需要为物料盘格位做水平偏移;
 *   - 第 k 个抓取的物料放在物料盘第 k 格, 取料时按同样顺序取第 k 格。
 *
 * 各工位目标位姿在 arm_config.h 的 "动作目标位姿(待标定)" 区统一配置
 * (arm_config.h 给初始化宏, 本模块 .c 内用 static const ArmPose_t 绑定)。
 * 可调参数见 arm_config.h / Applications/arm。
 */

#ifndef __ARM_ACTION_H__
#define __ARM_ACTION_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <stdbool.h>
#include <stdint.h>
#include "arm_config.h"
#include "rpi_port.h"

/**
 * @brief 从原料区抓取指定颜色的物料, 并放置到物料盘指定格位
 * @param color    需要抓取的颜色(COLOR_RED ~ COLOR_LIGHTBLUE)
 * @param pad_slot 物料盘格位号(1 ~ ARM_PAD_SLOT_NUM)
 * @return true=抓取+上盘完成; false=颜色非法/颜色识别超时/运动或换格失败
 * @note 抓料前先启动物料盘换格, 物料盘转动与"去原料区抓取"并行, 抓完回到盘上时盘已到位(节约时间)。
 */
bool Arm_Action_LoadPad(RPI_Color_t color, uint8_t pad_slot);

/**
 * @brief 从物料盘指定格位取料, 放置到粗加工区指定格位
 * @param pad_slot      物料盘格位号(1 ~ ARM_PAD_SLOT_NUM)
 * @param process_slot  粗加工区格位号(1 ~ ARM_SLOT_NUM)
 * @param next_pad_slot 下一个要取的物料盘格位号(1 ~ ARM_PAD_SLOT_NUM); 0=没有下一格(不预启动)
 * @return true=取料+放料完成; false=格位非法/运动失败
 * @note 取到料后立即预启动 next_pad_slot 的换格, 让物料盘转动与"移臂到场地放料"并行(节约时间);
 *       下一次取料时盘内已到位/接近到位。
 */
bool Arm_Action_UnloadToProcessing(uint8_t pad_slot, uint8_t process_slot, uint8_t next_pad_slot);

/**
 * @brief 从物料盘指定格位取料, 放置到暂存区指定格位
 * @param pad_slot      物料盘格位号(1 ~ ARM_PAD_SLOT_NUM)
 * @param storage_slot  暂存区格位号(1 ~ ARM_SLOT_NUM)
 * @param layer         叠放层号: 0=第一层, 1=第二层(叠放在第一层上)
 * @param next_pad_slot 下一个要取的物料盘格位号(1 ~ ARM_PAD_SLOT_NUM); 0=没有下一格(不预启动)
 * @return true=取料+放料完成; false=格位/层号非法/运动失败
 * @note 取到料后立即预启动 next_pad_slot 的换格, 与"移臂到场地放料"并行(节约时间)。
 */
bool Arm_Action_UnloadToStorage(uint8_t pad_slot, uint8_t storage_slot, uint8_t layer, uint8_t next_pad_slot);

/**
 * @brief 从粗加工区指定格位抓取物料, 放回物料盘指定格位(放置动作的反向)
 * @param process_slot 粗加工区格位号(1 ~ ARM_SLOT_NUM)
 * @param pad_slot     物料盘格位号(1 ~ ARM_PAD_SLOT_NUM)
 * @return true=取料+上盘完成; false=格位非法/运动失败
 * @note 取料点与 Arm_Action_UnloadToProcessing 的放料点一致(同一套格位位姿计算);
 *       取到料后不再单独抬升一次: 由上盘移动的"先升到盘安全高度再水平"一次性完成撤离,
 *       可反向回退联调时已放到场地的物料; 到场位取料前先启动物料盘换格, 换格与取料/移臂并行(节约时间)。
 */
bool Arm_Action_LoadPadFromProcessing(uint8_t process_slot, uint8_t pad_slot);

/**
 * @brief 从暂存区指定格位/叠层抓取物料, 放回物料盘指定格位(放置动作的反向)
 * @param storage_slot 暂存区格位号(1 ~ ARM_SLOT_NUM)
 * @param layer        叠放层号: 0=第一层, 1=第二层
 * @param pad_slot     物料盘格位号(1 ~ ARM_PAD_SLOT_NUM)
 * @return true=取料+上盘完成; false=格位/层号非法/运动失败
 * @note 与 Arm_Action_UnloadToStorage 共用同一套格位/层位姿计算;
 *       到场位取料前先启动物料盘换格, 换格与取料/移臂并行(节约时间)。
 */
bool Arm_Action_LoadPadFromStorage(uint8_t storage_slot, uint8_t layer, uint8_t pad_slot);

/**
 * @brief 放置区域(计算放置目标位姿用)
 */
typedef enum {
    ARM_PLACE_AREA_PROCESSING = 0, // 粗加工区(只用格位, 不用叠放层)
    ARM_PLACE_AREA_STORAGE, // 暂存区(格位 + 叠放层)
} ArmPlaceArea_t;

/**
 * @brief 计算放置目标位姿(逐点预设表)
 * @param area  放置区域(粗加工区/暂存区)
 * @param slot  格位号(1 ~ ARM_SLOT_NUM)
 * @param layer 叠放层号: 0=第一层, 1=第二层(仅暂存区使用, 粗加工区忽略)
 * @param pose  输出: 目标极坐标位姿(伸展长度/基座角度/抬升高度)
 * @return true=参数合法并已填充; false=area/slot/layer 非法或 pose 为空
 * @note 不再用"中心位姿 + 格距位移"换算, 而是直接查逐点实车标定的预设表:
 *       粗加工区每格一个位姿, 暂存区每格每层一个位姿(arm_config.h 的
 *       ARM_POSE_PROCESS_SLOTn_INIT / ARM_POSE_STORAGE_SLOTn_Lm_INIT),
 *       与任务动作、`arm preset process|storage`、`arm act pose` 共用同一份数据。
 */
bool Arm_Action_CalcPlacePose(ArmPlaceArea_t area, uint8_t slot, uint8_t layer, ArmPose_t *pose);

/**
 * @brief 计算"场地固定"放置位姿并叠加视觉 yaw 补偿(调试/标定核对用)
 * @param area  放置区域(粗加工区/暂存区)
 * @param slot  格位号(1 ~ ARM_SLOT_NUM)
 * @param layer 叠放层号: 0=第一层, 1=第二层(仅暂存区)
 * @param pose  输出: 已叠加 yaw 补偿的目标位姿
 * @return true=参数合法并已填充; false=area/slot/layer 非法或 pose 为空
 * @note 与 Arm_Action_UnloadToProcessing/UnloadToStorage 实际放置位一致(含 yaw 补偿),
 *       供 `arm act pose` 等调试命令直接核对补偿后的基座角度。
 */
bool Arm_Action_CalcFieldPose(ArmPlaceArea_t area, uint8_t slot, uint8_t layer, ArmPose_t *pose);

/**
 * @brief 回到机械臂初始状态位(行驶/待机位: 臂收短 + 抬升)
 * @return true=已到位; false=运动失败
 * @note 位姿在 arm_config.h 的 ARM_POSE_INIT_* (占位值需实测标定, `arm preset init` 对位);
 *       整臂回零完成后由回零任务走到该位; 每次小车移动前由任务层调用, 避免带长臂/低姿态行驶。
 *       只动三轴, 不动夹爪/物料盘。
 */
bool Arm_Action_MoveToInitPose(void);

/**
 * @brief 设置/清除视觉标定 yaw 误差的机械臂补偿角
 * @param yaw_err_deg 视觉标定 yaw 误差(°); 传 0 表示恢复不补偿
 * @note 车身旋转修正精度差时, 把视觉 yaw 误差改由基座旋转补偿: 抓取/放置时基座角度 += 补偿角。
 *       只作用于场地固定工位(原料区/粗加工区/暂存区), 物料盘随车体转动不补偿;
 *       补偿角限幅/方向见 arm_config.h 的 ARM_YAW_CORRECTION_MAX_DEG / ARM_YAW_CORRECTION_SIGN。
 *       任务层在标定成功后调用, 每个工位开始时清零(见 mission_config.h 的
 *       MISSION_CAL_YAW_FIX_WITH_ARM)。
 */
void Arm_Action_SetYawCorrection(float yaw_err_deg);

/**
 * @brief 读取当前视觉 yaw 补偿角
 * @return 当前实际叠加到场地固定工位基座角上的补偿角(°)(已按 ARM_YAW_CORRECTION_SIGN 换算并限幅)
 */
float Arm_Action_GetYawCorrection(void);

/**
 * @brief 视觉标定预备位姿: 基座旋转 / 伸展 / 抬升的目标值
 * @note 由任务层按目标区域（原料区/粗加工区/暂存区）配置后传入，坐标约定与 arm_motion 一致。
 */
typedef struct {
    int32_t base_angle_deg; // 基座目标角度(°), 相对机械零位
    float extend_mm; // 伸展长度(mm), 0=完全收回
    float lift_mm; // 抬升高度(mm), 0=最底部
} ArmCalPose_t;

/**
 * @brief 视觉标定预备动作: 先把水平两轴(基座旋转+伸展)移动到指定位姿, 再升降到位;
 *        然后张开夹爪, 最后把相机翻转舵机翻到标定角度
 * @param pose 预备位姿(基座旋转/伸展/抬升), 受各轴软限位约束; 为空时返回 false
 * @return true=三轴已到位(或 ARM_CAL_PREP_ENABLE=0 跳过移动)且相机已翻到标定角;
 *         false=参数为空/下发失败/等待超时/未到位(此时相机不会翻转, 可安全放弃本次标定)
 * @note  供树莓派视觉标定前调用, 让相机/标定板进入视野;
 *        运动顺序固定为"先旋转/伸展到位, 再升降到位", **不一边旋转一边升降**;
 *        三个区域的预备位姿在 mission_config.h 的 MISSION_CAL_ARM_* 配置,
 *        翻转舵机 ID/角度在 arm_config.h 的"相机翻转舵机"区;
 *        预备位姿到位后会把夹爪张开到 ARM_CAL_GRIPPER_OPEN_DEG(默认 -110°), 避免夹爪挡住标定目标,
 *        夹爪动作失败只告警、不阻断标定。
 */
bool Arm_Action_CalibrationReady(const ArmCalPose_t *pose);

/**
 * @brief 视觉标定结束收尾: 保持相机翻转角(不再恢复默认角)
 * @note  每次标定结束后都调用(成功/失败/被中止都要), 但只做收尾/日志:
 *        标定角同时作为后续颜色识别的视角, 不再自动翻回;
 *        默认角只在整臂回零/伸展轴回零前(Arm_CtrlInit / Arm_HomeAxis)或 `arm flip restore` 时恢复。
 */
void Arm_Action_CalibrationFinish(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __ARM_ACTION_H__ */
