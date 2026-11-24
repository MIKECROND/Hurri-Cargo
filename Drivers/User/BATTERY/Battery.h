/**
 * @file Battery.h
 * @author T-Car Team
 * @brief 电池电压监控模块头文件
 *
 * 电压来源改为**总线舵机回传的供电电压**(Servo_Monitor 的 voltage 字段), 不使用 ADC:
 *   - 舵机总线与电池同源供电(或经固定稳压), 用某一颗总线舵机(ID 可配)的电压作为电池参考;
 *   - 数值为舵机自身供电电压(mV), 可用 BATTERY_SERVO_SCALE / BATTERY_SERVO_OFFSET_MV
 *     按万用表实测校准到电池端电压;
 *   - 舵机总线读命令已由协议层事务互斥串行化, 本模块的周期采样可与机械臂动作并发。
 * 可调参数见 Battery.h。
 */

#ifndef __BATTERY_H__
#define __BATTERY_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <stdbool.h>
#include <stdint.h>

#ifndef BATTERY_ENABLED
#define BATTERY_ENABLED 1 // 1=启用(舵机回传电压检测); 0=关闭(空实现, 不占用任务)
#endif

#if BATTERY_ENABLED == 1

/* ===== 电压来源(总线舵机) ===== */
#define BATTERY_SERVO_ID 2 // 用于取电压的舵机 ID(总线舵机: 夹爪=0/相机翻转=1/物料盘=2)
#define BATTERY_SERVO_SCALE 1.0f // 舵机电压 -> 电池电压换算比例(按实测校准, 1.0=直读)
#define BATTERY_SERVO_OFFSET_MV 0 // 电压偏置(mV), 用于实测修正

/* ===== 采样与告警 ===== */
#define BATTERY_SAMPLE_AVG 4 // 每次采样读取次数(取平均抑制读数抖动)
#define BATTERY_SAMPLE_INTERVAL_MS 1000 // 采样周期(ms)
#define BATTERY_MIN_VALID_MV 5000 // 有效电压下限(mV), 低于此视为读取失败/未接电池
#define BATTERY_THRESHOLD 11300 // 低电压告警阈值(mV)
#define BATTERY_WARN_INTERVAL_MS 30000 // 读取失败/低电压告警的最小间隔(ms)

bool Battery_Init(void);
bool Battery_GetVoltage(uint16_t *voltage);

#endif /* BATTERY_ENABLED */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __BATTERY_H__ */
