/**
 * @file sim_ops.h
 * @brief 主机端 nav 仿真：OPS 平面定位测量源（Loc_Get 的仿真实现）
 */

#ifndef SIM_OPS_H
#define SIM_OPS_H

#include <stdint.h>
#include <stdbool.h>
#include "sim_config.h"

#ifdef __cplusplus
extern "C" {
#endif

void SimOps_Reset(const SimPlantCfg *cfg, uint32_t start_tick);
void SimOps_Tick(uint32_t now_tick);

/* 最近一次 OPS 测量（位置为原始测量值，航向为复刻 nav_local.c 低通后的值） */
void SimOps_GetMeasPose(float *x, float *y, float *yaw);
bool SimOps_HasSample(void);

#ifdef __cplusplus
}
#endif

#endif /* SIM_OPS_H */
