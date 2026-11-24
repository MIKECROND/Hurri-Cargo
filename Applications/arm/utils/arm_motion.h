/**
 * @file arm_motion.h
 * @author T-Car Team
 * @brief 机械臂运动接口: 模块初始化、笛卡尔/极坐标移动、到位等待、当前位置读取
 *
 * 分层: arm_action(动作流程) -> arm_motion(多轴联动/坐标) -> arm_ctrl(单轴/电机)
 *
 * 坐标约定(与 arm_config.h 一致):
 *   - x_mm / y_mm : 水平面内目标坐标 (mm), 原点在基座回转中心, x 为基座零位方向;
 *   - z_mm        : 升降抬升高度 (mm), 0=最底部(逻辑零点, 向上增大), 最大 ARM_LIFT_MAX_MM;
 *                   机械回零在顶部顶点, 由 ARM_LIFT_TOP_HOME_OFFSET_MM 换算成从底部计的高度。
 * 可调参数见 arm_config.h / Applications/arm。
 */

#ifndef __ARM_MOTION_H__
#define __ARM_MOTION_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <stdbool.h>
#include <stdint.h>
#include "arm_config.h"

/**
 * @brief 初始化机械臂模块并发起异步回零
 * @note 非阻塞, 回零在后台任务中执行; 运动接口会自动等待回零就绪。
 */
void Arm_Init(void);

/**
 * @brief 设置当前动作使用的速度/加速度(三轴 VEL/ACC)
 * @param profile 动作 profile(arm_config.h 的 ARM_PROFILE_xxx_INIT); 传 NULL 恢复默认 profile
 * @note 动作层在每个对外动作开始时调用; 该动作内部的极坐标移动/标定预备等全部按此 profile 执行,
 *       实现"每个机械臂动作的速度/加速度互相解耦"。调试命令未设置时使用默认 profile。
 */
void Arm_SetMotionProfile(const ArmMotionProfile_t *profile);

/**
 * @brief 读取当前动作使用的速度/加速度配置
 * @return 当前 profile 指针(只读, 指向模块内静态数据, 不要修改/释放)
 */
const ArmMotionProfile_t *Arm_GetMotionProfile(void);

/**
 * @brief 下发笛卡尔目标位置(异步, 不等待到位)
 * @param x_mm 目标 X 坐标 (mm)
 * @param y_mm 目标 Y 坐标 (mm)
 * @param z_mm 目标抬升高度 (mm), 0=最底部
 * @return true 三条轴命令均下发成功; false 目标不可达或下发失败
 * @note 夹爪等动作前应使用 Arm_MoveToAndWait, 否则可能在未到位时开合(抓空/撞料)。
 */
bool Arm_MoveTo(float x_mm, float y_mm, float z_mm);

/**
 * @brief 下发笛卡尔目标并等待机械臂到位
 * @param x_mm/y_mm/z_mm 目标位置 (mm), 含义同 Arm_MoveTo
 * @param timeout_ms     到位等待超时 (ms)
 * @return true=已到位; false=下发失败或等待超时/位置不可用
 * @note 分两步走, 顺序按升降方向决定: 下降动作先旋转+伸展到位再下降;
 *       上升动作先升到目标高度再旋转+伸展(水平动作始终在较高高度上做)。
 */
bool Arm_MoveToAndWait(float x_mm, float y_mm, float z_mm, uint32_t timeout_ms);

/**
 * @brief 下发极坐标目标(不走逆解, 直接给定基座角度; 异步, 不等待到位)
 * @param extend_mm      同步带伸展长度 (mm), 0=完全收回; 与 Arm_ExtendTo / `arm pos` 的 extend 一致,
 *                       不要传水平半径(水平半径 = ARM_BASE_TO_EXTEND_MM + 伸展长度, 仅内部换算用)
 * @param base_angle_deg 基座目标角度 (°), 0=机械零位, 正/负为两侧
 * @param lift_mm        目标抬升高度 (mm), 0=最底部
 * @return true 三条轴命令均下发成功; false 目标不可达或下发失败
 * @note 所有对外位姿/极坐标参数统一填伸展长度, 不需要再做 "+ARM_BASE_TO_EXTEND_MM" 换算。
 */
bool Arm_MovePolarTo(float extend_mm, float base_angle_deg, float lift_mm);

/**
 * @brief 下发极坐标目标并等待机械臂到位
 * @param extend_mm 同步带伸展长度 (mm), 含义同 Arm_MovePolarTo
 * @param base_angle_deg 基座目标角度 (°), 0=机械零位, 正/负为两侧
 * @param lift_mm 目标抬升高度 (mm), 0=最底部
 * @param timeout_ms     到位等待超时 (ms)
 * @return true=已执行完(某段未到位时按实际可达位置继续, 仍返回 true); false=电机离线/位姿不可用
 * @note 分两步走, 顺序按升降方向决定: 下降动作先旋转+伸展到位再下降;
 *       上升动作先升到目标高度再旋转+伸展(水平动作始终在较高高度上做)。
 *       目标超软限位时钳到限位值执行; 某段重试用尽仍未到位只告警并继续后续段/动作(best effort),
 *       不再因为"没到位"让整轮任务失败。
 */
bool Arm_MovePolarToAndWait(float extend_mm, float base_angle_deg, float lift_mm, uint32_t timeout_ms);
/*============================ 当前位置读取(三种数据) ============================*/

/**
 * @brief 笛卡尔坐标 (mm): z 为抬升高度
 */
typedef struct {
    float x_mm; // X 坐标 (mm)
    float y_mm; // Y 坐标 (mm)
    float z_mm; // 抬升高度 (mm), 0=最底部
} ArmCartesianPose_t;

/**
 * @brief 极坐标: 伸展长度 / 旋转角度 / 抬升高度
 */
typedef struct {
    float extend_mm; // 同步带当前伸展长度 (mm), 0=完全收回
    float base_angle_deg; // 基座当前旋转角度 (°), 0=机械零位
    float lift_mm; // 当前抬升高度 (mm), 0=最底部
} ArmPolarPose_t;

/**
 * @brief 机械臂当前位姿(三种数据表示)
 *
 * 读取失败/电机离线时对应 valid 标志为 false, 数值字段无效。
 */
typedef struct {
    /** 1) 机械臂相关电机当前角度: 下标顺序与 Arm_AxisCount 一致(基座/升降/伸展) */
    float motor_angle_deg[ARM_MOTOR_NUM];
    bool motor_angle_valid[ARM_MOTOR_NUM];
    /** 2) 当前笛卡尔坐标 (mm): x/y 水平位置, z 抬升高度 */
    ArmCartesianPose_t cartesian;
    bool cartesian_valid;
    /** 3) 当前极坐标: 伸展长度/旋转角度/抬升高度 */
    ArmPolarPose_t polar;
    bool polar_valid;
} ArmPos_t;

/**
 * @brief 读取机械臂当前极坐标(伸展长度/旋转角度/抬升高度)
 * @param pose 输出: 极坐标位姿
 * @return true=三个轴均读取成功; false=参数为空或存在电机离线/位置不可用
 * @note 返回 false 时输出参数内容未定义。
 */
bool Arm_GetPolarPose(ArmPolarPose_t *pose);

/**
 * @brief 读取机械臂当前笛卡尔坐标(x/y/z, z=抬升高度)
 * @param pose 输出: 笛卡尔位姿
 * @return true=读取并换算成功; false=参数为空或位置不可用
 * @note 返回 false 时输出参数内容未定义。
 */
bool Arm_GetCartesianPose(ArmCartesianPose_t *pose);

/**
 * @brief 读取机械臂当前位姿的三种数据(电机角度 / 笛卡尔坐标 / 极坐标)
 * @param pos 输出: 位姿(各分量是否有效见对应 *_valid 标志)
 * @return true=极坐标(及依赖它的笛卡尔坐标)读取成功; false=参数为空或位置不可用
 * @note 第 1 组电机角度与第 2/3 组位姿是先后两次采样, 机械臂运动过程中读取可能略有差异;
 *       第 2/3 组由同一次极坐标采样换算, 彼此一致。
 */
bool Arm_GetPos(ArmPos_t *pos);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __ARM_MOTION_H__ */
