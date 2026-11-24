/**
 * @file sim_plant.h
 * @brief 主机端 nav 仿真：麦轮底盘 + 地面里程被控对象模型
 */

#ifndef SIM_PLANT_H
#define SIM_PLANT_H

#include <stdint.h>
#include <stdbool.h>
#include "sim_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 一个控制周期内被控对象接收到的机体系速度命令（含该时刻底盘的加速度参数） */
typedef struct {
    float vx; /* 机体系 X 速度 (cm/s)，+ 向前 */
    float vy; /* 机体系 Y 速度 (cm/s)，+ 向左 */
    float w; /* 绕旋转中心的角速度 (deg/s)，逆时针为正 */
    float acc; /* 当拍底盘运动参数里的加速度 (cm/s²)，用于折算电机加速度斜坡 */
    int stop; /* 1 = 本拍是停止命令（MotionControl_Stop） */
} SimCommand;

void Plant_Reset(const SimPlantCfg *cfg, uint32_t start_tick, float x0, float y0, float yaw0);
void Plant_Step1ms(const SimCommand *cmd);

void Plant_GetPose(float *x, float *y, float *yaw);
void Plant_GetTwist(float *vx_body, float *vy_body, float *w_dps);
void Plant_GetWheelSpeed(float wheel_rad_s[4]);
/* 读取历史上某一 tick 的真实位姿（用于 OPS 测量延迟）；范围外的 tick 返回最近值 */
void Plant_GetPoseAt(uint32_t tick, float *x, float *y, float *yaw);

#ifdef __cplusplus
}
#endif

#endif /* SIM_PLANT_H */
