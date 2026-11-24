/**
 * @file motion_control.h
 * @author Pcb-yun (pcbyinyun@163.com)
 * @brief 运动控制模块头文件
 * 可调参数见 chassis_config.h。
 */

#ifndef __MOTION_CONTROL_H__
#define __MOTION_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <stdint.h>
#include <stdbool.h>
#include "chassis.h"

bool MotionControl_Init(void);
void MotionControl_GetMotionParams(float *linear_speed, float *yaw_speed, float *acc, float *dec);
void MotionControl_SetMotionParams(float linear_speed, float yaw_speed, float acc, float dec);

/**
 * @brief 位置模式移动：相对当前位姿的机体系偏移（轮位置闭环）
 * @param x_offset   机体系 X 方向偏移 (cm)，+ 向前
 * @param y_offset   机体系 Y 方向偏移 (cm)，+ 向左
 * @param yaw_offset 航向角偏移 (deg)，俯视逆时针为正
 * @return true=位置命令已下发(或偏移小于 1 个脉冲、无需运动); false=未初始化/参数非法/命令发送失败
 * @note  非阻塞：下发后立即返回，需要确认到位时用 MotionControl_PositionArrived() 轮询。
 *        运动模式固定为"相对当前实时位置"，所以调用时应保证底盘静止(不要与导航/速度命令同时调用)，
 *        否则下发瞬间的"当前位置"会被后续速度运动带走，剩下的相对位移就不准了。
 *        位置模式只保证"四个轮子各转过目标脉冲"，不保证地面实际位移(打滑时偏小)；
 *        x/y/yaw 同时非零时等价于一个恒定机体系运动(路径为圆弧)，不是"先平移后旋转"，
 *        且四轮速度上限相同、目标位移不等时会先后到位——因此小距离修正(标定精修)最合适，
 *        大转角/长距离仍应使用速度模式导航闭环。
 *        速度/加速度沿用 MotionControl_SetMotionParams 的当前值(线速度 + 旋转分量折算轮速上限)。
 */
bool MotionControl_SetPosition(float x_offset, float y_offset, float yaw_offset);

/**
 * @brief 查询最近一次位置模式移动是否到位（轮子实时位置到达目标并停稳）
 * @return true=已到位/没有进行中的位置移动; false=仍在运动
 * @note  到位判定用各轮电机实时位置(S_CPOS 状态轮询，Emm 固件单位 65536/圈)相对下发前位置的变化量，
 *        把目标命令脉冲换算成同样的回读单位后比较（两者相差 20.48 倍，不能直接比）；
 *        到达目标容差(MOTOR_POS_ARRIVE_TOL_READ)并保持 MOTOR_POS_SETTLE_MS 不动才算到位。
 *        若读不到任何电机位置反馈，则按 MOTOR_POS_FALLBACK_DONE_MS 固定时间兜底返回 true
 *        （只表示命令已下发，不保证实际到位）。任务层等待时应自己加超时/中止检查。
 */
bool MotionControl_PositionArrived(void);

void MotionControl_SetVelocity(float x_component, float y_component, float yaw_component);

/**
 * @brief 强制下一次速度命令一定下发（作废"重复指令去重"状态）
 * @note  仅供 shell 手动调试（`car vel`）使用：正常导航/任务流程不需要调用。
 *        速度 setpoint 有限频/去重（见 chassis_config.h 的 CHASSIS_MOTION_SETPOINT_*），
 *        手动重复下发同一速度时默认会被跳过；调试时先调用本函数可保持"每次命令都生效"。
 *        仅在 MOTOR_VELOCITY_MODE=1 时可用。
 */
void MotionControl_ResetSetpointFilter(void);
void MotionControl_Stop(void);

/**
 * @brief 获取运动 setpoint 限频统计（供 `car bus` 查看总线带宽节省效果）
 * @param posted  实际投递到电机总线的 setpoint 次数（输出，可为 NULL）
 * @param skipped 因"与上次指令几乎相同且未到保持刷新时间"而跳过的次数（输出，可为 NULL）
 * @note  跳过比例高说明稳态巡航去重生效；加减速/接近目标时 posted 速率会升到约 100Hz。
 *        Emm 速度模式为绝对值指令，跳过重复值不影响电机保持速度；心跳由状态轮询维持。
 */
void MotionControl_GetSetpointStats(uint32_t *posted, uint32_t *skipped);

#ifdef __cplusplus
}
#endif

#endif /* __MOTION_CONTROL_H__ */
