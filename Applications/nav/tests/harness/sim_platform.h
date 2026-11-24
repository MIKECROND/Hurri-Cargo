/**
 * @file sim_platform.h
 * @brief 主机端 nav 仿真：实时钟 / 电机命令链 / CMSIS-RTOS 桩 的仿真侧接口
 */

#ifndef SIM_PLATFORM_H
#define SIM_PLATFORM_H

#include <stdint.h>
#include <stdbool.h>
#include "sim_config.h"
#include "sim_plant.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*SimTickFn)(uint32_t now_ms);

/* 初始化/复位电机总线模型（命令环清空；仿真时钟保持单调递增，不归零） */
void Sim_PlatformInit(const SimPlantCfg *cfg);
void Sim_ResetBus(void);

/* 推进仿真：每 1ms 一次被控对象步进 + OPS 更新 + tick 钩子 */
void Sim_Advance(uint32_t ms);

/* 一个导航控制周期应该推进多久（nav_period + 可选随机抖动，模拟命令队列阻塞） */
uint32_t Sim_NextPeriodMs(void);

void Sim_SetTickHook(SimTickFn fn);

void Sim_RequestAbort(int reason);
int Sim_AbortReason(void);
void Sim_ClearAbort(void);

/* 旧版 Nav_Task 驱动：osDelay 里检测到 abort 后调用该函数跳回场景驱动（longjmp） */
void Sim_SetAbortJump(void (*jump_fn)(void));

uint32_t Sim_Tick(void);
const SimPlantCfg *Sim_Cfg(void);

/* 本拍导航实际下发的机体系速度命令（未含延迟） */
SimCommand Sim_CurrentCommand(void);

void Sim_SetVerbose(int on);
int Sim_GetVerbose(void);
uint32_t Sim_GetEventFlags(void);

/* 控制周期统计（由 Sim_Advance 的调用长度得到） */
void Sim_GetControlTiming(uint32_t *min_ms, uint32_t *max_ms, uint32_t *mean_ms, uint32_t *count);

#ifdef __cplusplus
}
#endif

#endif /* SIM_PLATFORM_H */
