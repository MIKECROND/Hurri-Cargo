/**
 * @file kinematics.c
 * @author Pcb-yun (pcbyinyun@163.com)
 * @brief 麦克纳姆轮运动学解算模块源文件
 *
 * 逆运动学模型（机体系：+X 前、+Y 左、w 逆时针为正，单位 cm / deg）：
 *   旋转中心 C 相对四轮几何中心的偏移为 (cx, cy) = CHASSIS_ROTATION_CENTER_OFFSET_X/Y。
 *   四轮中心相对 C 的坐标：
 *     FL(+L-cx, +W-cy)   FR(+L-cx, -W-cy)
 *     RL(-L-cx, +W-cy)   RR(-L-cx, -W-cy)
 *   其中 L = WHEEL_BASE_LENGTH/2，W = WHEEL_BASE_WIDTH/2。
 *
 *   轮心速度 v_i = v_C + w × r_i，代入麦轮轮速公式：
 *     FL = (vx_FL - vy_FL)/R   FR = (vx_FR + vy_FR)/R
 *     RL = (vx_RL + vy_RL)/R   RR = (vx_RR - vy_RR)/R
 *   得到各轮的"角速度力臂"（vx=vy=0 时轮速 = -/+ 力臂*w/R）：
 *     FL = r0 - cx - cy   FR = r0 - cx + cy
 *     RL = r0 + cx - cy   RR = r0 + cx + cy
 *   其中 r0 = L + W。偏移为 0 时退化为原实现（四轮力臂都是 L+W，绕轮组中心转）。
 * 可调参数见 chassis_config.h。
 */

#include "kinematics.h"
#include "chassis_config.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
 * @brief 逆运动学解算
 * @param vx X方向速度 (cm/s)，旋转中心 C 的机体系 X 速度
 * @param vy Y方向速度 (cm/s)，旋转中心 C 的机体系 Y 速度
 * @param w 绕旋转中心 C 的角速度 (deg/s)，逆时针为正
 * @param wheels 四个轮子的输出角速度 (rad/s)
 * @note 纯旋转指令 (vx=vy=0, w!=0) 时车体绕 C 点原地旋转，
 *       C = 四轮几何中心 + (CHASSIS_ROTATION_CENTER_OFFSET_X, CHASSIS_ROTATION_CENTER_OFFSET_Y)。
 */
void Kinematics_Inverse(float vx, float vy, float w, Wheel_t *wheels) {
    float omega = w * M_PI / 180.0f;

    float L = WHEEL_BASE_LENGTH / 2.0f;
    float W = WHEEL_BASE_WIDTH / 2.0f;
    float R = WHEEL_RADIUS;

    /* 旋转中心相对四轮几何中心的偏移 (cm) */
    const float cx = CHASSIS_ROTATION_CENTER_OFFSET_X;
    const float cy = CHASSIS_ROTATION_CENTER_OFFSET_Y;

    /* 各轮角速度力臂：旋转中心偏移后，前后轮 ±cx、左右轮 ±cy */
    const float r0 = L + W;
    const float lever_fl = r0 - cx - cy; // 前左轮
    const float lever_fr = r0 - cx + cy; // 前右轮
    const float lever_rl = r0 + cx - cy; // 后左轮
    const float lever_rr = r0 + cx + cy; // 后右轮

    wheels->fl = (vx - vy - lever_fl * omega) / R;
    wheels->fr = (vx + vy + lever_fr * omega) / R;
    wheels->rl = (vx + vy - lever_rl * omega) / R;
    wheels->rr = (vx - vy + lever_rr * omega) / R;
}
