/**
 * @file arm_kinematics.c
 * @author T-Car Team
 * @brief 机械臂运动学: 逆解(笛卡尔/极坐标 -> 基座角度 + 伸长量) 与 正解(回读用)
 * 可调参数见 arm_config.h / Applications/arm。
 */

#include "arm_kinematics.h"
#include "arm_config.h"
#include <math.h>
#include <stddef.h>

/** 圆周率(仅本文件使用, 不污染全局命名) */
#define ARM_KIN_PI 3.14159265358979f

bool Arm_Kin_RadiusToExtend(float radius_mm, float *extend_mm) {
    if (extend_mm == NULL) return false;

    // 1. 水平半径减去基座到同步带的固定偏移, 得到同步带需要伸出的长度
    float d = radius_mm - ARM_BASE_TO_EXTEND_MM;

    // 2. 行程检查(防止撞机): 太近无法收回, 太远超出同步带行程
    if (d < ARM_EXTEND_MIN_MM || d > ARM_EXTEND_MAX_MM) return false;

    *extend_mm = d;
    return true;
}

bool Arm_Kin_CartesianToPolar(float x_mm, float y_mm, float *base_angle_deg, float *extend_mm) {
    if (base_angle_deg == NULL || extend_mm == NULL) return false;

    // 1. 目标点到基座回转中心的水平半径
    float radius_mm = sqrtf(x_mm * x_mm + y_mm * y_mm);

    // 2. 半径换算同步带伸长量并做行程检查
    if (!Arm_Kin_RadiusToExtend(radius_mm, extend_mm)) return false;

    // 3. 基座角度(极坐标 theta), 弧度转角度
    *base_angle_deg = atan2f(y_mm, x_mm) * (180.0f / ARM_KIN_PI);
    return true;
}

bool Arm_Kin_PolarToCartesian(float extend_mm, float base_angle_deg, float *x_mm, float *y_mm) {
    if (x_mm == NULL || y_mm == NULL) return false;

    // 1. 水平半径 = 基座到同步带的固定偏移 + 同步带当前伸长量
    float radius_mm = ARM_BASE_TO_EXTEND_MM + extend_mm;

    // 2. 极坐标转直角坐标(角度: 弧度)
    float angle_rad = base_angle_deg * (ARM_KIN_PI / 180.0f);
    *x_mm = radius_mm * cosf(angle_rad);
    *y_mm = radius_mm * sinf(angle_rad);
    return true;
}
