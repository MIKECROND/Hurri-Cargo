/**
 * @file kinematics.h
 * @author Pcb-yun (pcbyinyun@163.com)
 * @brief 麦克纳姆轮运动学解算模块头文件
 * 可调参数见 chassis_config.h。
 */

#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "chassis.h"

/**
 * @brief 位姿增量
 */
typedef struct {
    float dx; // X方向位移增量 (cm)
    float dy; // Y方向位移增量 (cm)
    float dyaw; // 航向角增量 (deg)
} PoseDelta_t;

/**
 * @brief 逆运动学解算（旋转中心可偏移的麦轮模型）
 * @param vx    旋转中心 C 的机体系 X 速度 (cm/s)，+ 向前
 * @param vy    旋转中心 C 的机体系 Y 速度 (cm/s)，+ 向左
 * @param w     绕 C 点的角速度 (deg/s)，俯视逆时针为正
 * @param wheels 四个轮子的输出角速度 (rad/s)
 *
 * @note C = 四轮几何中心 + (CHASSIS_ROTATION_CENTER_OFFSET_X, CHASSIS_ROTATION_CENTER_OFFSET_Y)，
 *       偏移为 0 时等价于原来的"绕四轮几何中心"模型。
 *       vx = vy = 0 时车体严格绕 C 点原地旋转（该点应与导航定位点一致）。
 */
void Kinematics_Inverse(float vx, float vy, float w, Wheel_t *wheels);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __KINEMATICS_H__ */
