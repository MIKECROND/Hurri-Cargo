/**
 * @file arm_kinematics.h
 * @author T-Car Team
 * @brief 机械臂运动学: 笛卡尔/极坐标 -> 基座角度 + 同步带伸长量(逆解) 及 正解
 *
 * 机械臂构型: 旋转基座 + 同步带伸缩 + 升降。水平面内为极坐标,
 * 因此逆解只需把目标点换算成(基座角度, 水平半径), 再由水平半径得到同步带伸长量;
 * 正解用于位置回读(arm pos), 由伸长量/角度反算末端笛卡尔坐标。
 * 注意: "水平半径 <-> 伸长量"的偏移换算(ARM_BASE_TO_EXTEND_MM)只用于笛卡尔接口;
 *       对外位姿/伸展配置统一直接给伸展长度(extend_mm), 不需要手工加偏移。
 * 本文件为纯数学计算, 不访问硬件, 便于单独测试。
 * 可调参数见 arm_config.h / Applications/arm。
 */

#ifndef __ARM_KINEMATICS_H__
#define __ARM_KINEMATICS_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <stdbool.h>

/**
 * @brief 水平半径(mm)换算为同步带伸长量, 并做行程检查
 * @param radius_mm  末端到基座回转中心的水平半径 (mm)
 * @param extend_mm  输出: 同步带伸长量 (mm)
 * @return true 目标在行程内; false 超出伸展轴软限位(不可达)
 * @note 仅供笛卡尔逆解(Arm_Kin_CartesianToPolar)内部使用;
 *       对外极坐标/位姿接口统一直接使用伸展长度, 不要再手工"加/减偏移"。
 */
bool Arm_Kin_RadiusToExtend(float radius_mm, float *extend_mm);

/**
 * @brief 笛卡尔坐标逆解, 输出基座角度与同步带伸长量
 * @param x_mm             目标 X 坐标 (mm)
 * @param y_mm             目标 Y 坐标 (mm)
 * @param base_angle_deg   输出: 基座目标角度 (°), 0=机械零位
 * @param extend_mm        输出: 同步带伸长量 (mm)
 * @return true 目标可达; false 超出伸展轴软限位(不可达)
 */
bool Arm_Kin_CartesianToPolar(float x_mm, float y_mm, float *base_angle_deg, float *extend_mm);

/**
 * @brief 极坐标正解: 同步带伸长量 + 基座角度 -> 笛卡尔坐标
 * @param extend_mm      同步带伸长量 (mm)
 * @param base_angle_deg 基座角度 (°), 0=机械零位
 * @param x_mm           输出: X 坐标 (mm)
 * @param y_mm           输出: Y 坐标 (mm)
 * @return true 换算成功(指针有效)
 * @note 用于位置回读(arm pos), 不做行程检查; 与 Arm_Kin_CartesianToPolar 互逆
 *       (水平半径 = ARM_BASE_TO_EXTEND_MM + 伸长量)。
 */
bool Arm_Kin_PolarToCartesian(float extend_mm, float base_angle_deg, float *x_mm, float *y_mm);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __ARM_KINEMATICS_H__ */
