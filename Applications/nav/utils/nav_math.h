/**
 * @file nav_math.h
 * @author Pcb-yun (pcbyinyun@163.com)
 * @brief 导航数学库头文件
 * 可调参数见 nav_config.h / Applications/nav/README.md。
 */

#ifndef __NAV_MATH_H__
#define __NAV_MATH_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <stdint.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define DEG_TO_RAD (M_PI / 180.0f)

/**
 * @brief 角度转弧度
 */
static inline float deg2rad(float deg) {
    return deg * DEG_TO_RAD;
}

/**
 * @brief 位姿参考点换算：把位置点 P 换算为车体参考点 G = P + R(yaw) * offset
 * @param x,y      输入输出位置 (cm)
 * @param yaw_deg  航向角 (deg)
 * @param dx_cm    G 相对 P 的机体系 X 偏移 (cm)，+ 向前
 * @param dy_cm    G 相对 P 的机体系 Y 偏移 (cm)，+ 向左
 * @note  偏移为 0 时不做任何运算（保持旧行为）；用于把 OPS 安装点位姿换算成车体参考点位姿。
 */
static inline void pose_apply_body_ref_offset(float *x, float *y, float yaw_deg, float dx_cm, float dy_cm) {
    if (dx_cm == 0.0f && dy_cm == 0.0f) return;
    float c = cosf(yaw_deg * DEG_TO_RAD);
    float s = sinf(yaw_deg * DEG_TO_RAD);
    float px = *x;
    float py = *y;
    *x = px + c * dx_cm - s * dy_cm;
    *y = py + s * dx_cm + c * dy_cm;
}

float normalize_angle(float angle);

// pose.x / pose.y 单位是厘米（cm），pose.yaw 单位是度（deg）
typedef struct {
    float x;
    float y;
    float yaw;
} Pose2D_t;

/**
 * @brief 限制值在指定范围内
 */
static inline float clamp(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __NAV_MATH_H__ */
