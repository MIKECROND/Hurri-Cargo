/**
 * @file zdt_v5_cfg.h
 * @brief 主机端仿真用张大头 V5 驱动配置最小桩
 *
 * nav_core.c 只用到 MOTOR_VELOCITY_MODE 的编译期检查；真配置还依赖机械臂/协议头，
 * 主机端无需引入。数值与固件当前配置保持一致（速度模式 = 1）。
 */

#ifndef __ZDT_V5_CFG_H__
#define __ZDT_V5_CFG_H__

#define MOTOR_VELOCITY_MODE 1
#define MOTOR_POS_MODE 1
#define CURRENT_FIRMWARE 0

#endif /* __ZDT_V5_CFG_H__ */
