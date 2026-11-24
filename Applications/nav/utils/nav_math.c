/**
 * @file nav_math.c
 * @author Pcb-yun (pcbyinyun@163.com)
 * @brief 导航数学库源文件
 * 可调参数见 nav_config.h / Applications/nav/README.md。
 */

#include "nav_math.h"

/**
 * @brief 角度归一化
 * @param angle 角度值(度)
 * @return 归一化后的角度值(度)，范围[-180, 180]
 */
float normalize_angle(float angle) {
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}
