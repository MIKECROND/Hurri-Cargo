/**
 * @file zdt_v5_port.h
 * @brief 张大头V5步进电机移植接口
 *
 * zdt-v5-driver - 张大头V5步进电机通用驱动
 * Copyright (C) 2024-2026  Pcb-yun
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * 将本文件复制到工程中，并改名为 zdt_v5_port.h
 */

#ifndef __ZDT_V5_PORT_H__
#define __ZDT_V5_PORT_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <stdint.h>
#include <stdbool.h>
#include "zdt_v5_cmd.h"
#include "log.h"

#define USE_VIEW 0

/* 应答超时(ms): 超过该时间没收到该电机任何应答, 视为不在线, 其状态数据不可信 */
#define MOTOR_ONLINE_TIMEOUT_MS 3000

/**
 * 心跳保护余量(ms): 固件心跳保护时间 = 状态轮询周期(step_update_time) + 该余量。
 * 余量太小的话, 系统繁忙把状态轮询任务拖过保护时间时, 电机会被心跳保护误停转
 * (表现为机械臂/底盘偶发走到一半停住, 再发一次命令又能走)。取 500ms 留足抖动余量,
 * 同时保证上位机真的掉线时电机仍能在 1s 内停转。
 */
#define MOTOR_HEARTBEAT_MARGIN_MS 500

#define USART6_RX_BUF_SIZE 40

typedef struct {
    uint8_t data[USART6_RX_BUF_SIZE];
    uint8_t len;
} Usart6_RxBuf_t;

#ifndef MOTOR_CMD_GAP_MS
#define MOTOR_CMD_GAP_MS 1 // 普通/紧急命令之间的最小帧间隔(ms): 0=只靠 TX 完成回调(需台架确认)
#endif
#ifndef MOTOR_MOTION_SYNC_GAP_MS
#define MOTOR_MOTION_SYNC_GAP_MS 1 // 多机打包帧 → 同步触发帧之间的间隔(ms), 让驱动器先缓存命令
#endif
#ifndef MOTOR_CMD_IDLE_POLL_MS
#define MOTOR_CMD_IDLE_POLL_MS 1 // 控制任务空闲时等普通队列的超时(ms): 高优先级通道的最坏唤醒延迟
#endif
#define MOTOR_MOTION_WAIT_SLOW_MS 20 // 运动 setpoint 排队超过该值(ms)计入 motion_wait_slow(验收: 长期 0)
#if MOTOR_CMD_IDLE_POLL_MS < 1
#error "MOTOR_CMD_IDLE_POLL_MS must be >= 1 (0 会让控制任务空转占用 CPU)"
#endif
#ifndef MOTOR_CMD_LEGACY_CHAIN
/* 现场 A/B 开关: 1=旧命令链(全部 FIFO + 每条命令 osDelay(2)), 与实车验证过的旧固件完全一致。
 * 由于新链路在实车上出现"机械臂动作/回零经常到不了(不可达)"的疑似应答丢帧问题,
 * 先默认回退旧链路定位问题; 确认新链路安全后再改回 0(或用 -DMOTOR_CMD_LEGACY_CHAIN=0 构建)。 */
#define MOTOR_CMD_LEGACY_CHAIN 1
#endif
#ifndef MOTOR_CMD_LEGACY_GAP_MS
#define MOTOR_CMD_LEGACY_GAP_MS 2 // 旧链路每条命令后的 osDelay 值(ms), 仅 LEGACY_CHAIN=1 时生效
#endif
#ifndef MOTOR_URGENT_QUEUE_LEN
#define MOTOR_URGENT_QUEUE_LEN 4 // 紧急通道环形深度(停止/使能), 满时丢最旧一条
#endif
#ifndef MOTOR_TX_TIMEOUT_MS
#define MOTOR_TX_TIMEOUT_MS 50 // 等待 UART6 TX 完成的超时(ms): 超时则中止发送, 避免控制任务永久阻塞
#endif
#ifndef MOTOR_BUS_STATS
#define MOTOR_BUS_STATS 1 // 1=统计排队时延/发送耗时/队列峰值(供 step bus 查看), 0=关闭计数
#endif

#if MOTOR_MULTI_CMD
/* 多机打包生效时, 一个运动 setpoint = 1 条 0xAA 帧(+1 条同步触发) */
#define MOTOR_MOTION_BATCH_MAX 1
#else
/* 未启用多机打包时, 一个运动 setpoint = 4 条单机速度帧(+1 条同步触发) */
#define MOTOR_MOTION_BATCH_MAX 4
#endif
#if MOTOR_MOTION_BATCH_MAX < 1
#error "MOTOR_MOTION_BATCH_MAX must be >= 1"
#endif

/**
 * @brief 电机总线命令链统计（只读快照, 由 Motor_Bus_Stats_Get() 返回）
 * @note  所有时延单位 ms; 发送耗时单位 us。计数在 MOTOR_BUS_STATS=0 时保持 0。
 */
typedef struct {
    /* —— 运动 setpoint 邮箱（高优先级通道） —— */
    uint32_t motion_posted; // 投递的 setpoint 数（≈导航控制周期数）
    uint32_t motion_sent; // 真正发到总线的 setpoint 数
    uint32_t motion_dropped; // 投递时发现上一件还没发出 ⇒ 旧 setpoint 被覆盖丢弃
    uint32_t motion_wait_last_ms; // 最近一次"投递→开始发送"的等待
    uint32_t motion_wait_max_ms; // 最大等待（修复前等效 64×2.1ms≈134ms，目标 <5ms）
    uint32_t motion_wait_sum_ms; // 累计等待（均值 = sum/motion_sent）
    uint32_t motion_wait_slow; // 等待超过 20ms 的次数（验收指标：应长期为 0）
    uint32_t motion_rate_1s; // 最近 1s 内实际发送的 setpoint 数（≈Hz，现场判断限频是否生效）
    /* —— 紧急通道（停止/使能） —— */
    uint32_t urgent_posted; // 投递次数
    uint32_t urgent_sent; // 发送次数
    uint32_t urgent_dropped; // 环满时被丢弃的条目（理论上只会发生在连续重复停止时）
    uint32_t urgent_latency_max_ms; // 投递→开始发送的最大等待（目标 <2ms）
    /* —— 普通 FIFO 队列（状态轮询/机械臂/工具） —— */
    uint32_t queue_put_count; // 成功入队次数
    uint32_t queue_put_timeout; // 入队等待超时(失败)次数
    uint8_t queue_usage; // 最近一次入队后的占用
    uint8_t queue_peak; // 占用峰值
    uint16_t queue_depth; // 队列深度(配置值, 64)
    /* —— 帧发送 —— */
    uint32_t tx_frames; // 实际发送帧数
    uint32_t tx_time_last_us; // 最近一帧 DMA启动→TX完成 耗时
    uint32_t tx_time_max_us; // 峰值
    uint32_t tx_rate_1s; // 最近 1s 内发送的帧数（帧/s，总线占用估算用）
    uint32_t tx_timeout; // 等待 TX 完成超时次数(异常保护)
    /* —— 状态轮询 —— */
    uint32_t poll_frames; // 状态轮询帧数(OP_PARAM_READ)
} MotorBusStats_t;

void zdt_v5_port_send(uint8_t *cmd, uint8_t len);
bool Motor_Send_Cmd(MotorCmd_t *cmd);

/**
 * @brief 投递一个运动 setpoint（走"最新值覆盖"邮箱, 永不阻塞）
 * @param cmds        本批运动命令（多机打包生效时 1 条 0xAA 帧; 否则 4 条单机速度帧）
 * @param count       命令条数, 1~MOTOR_MOTION_BATCH_MAX
 * @param sync_after  发完本批后是否补一帧 0xFF66 同步触发
 * @return true=已投递（旧 setpoint 会被覆盖丢弃）; false=参数非法/驱动未初始化
 * @note  只能在任务上下文调用（内部用临界区拷贝 ~100 字节）。
 *        旧实现把运动命令塞进 64 深 FIFO 队列, 队列满时 Motor_Send_Cmd 会阻塞 100ms,
 *        这才是 Nav 5ms 周期被拉长、速度指令 100ms 级延迟的根因。
 */
bool Motor_Send_Motion_Setpoint(const MotorCmd_t *cmds, uint8_t count, bool sync_after);

/**
 * @brief 作废尚未发出的运动 setpoint（停止/位置模式前调用, 避免"停完又执行一条更早的速度"）
 */
void Motor_Cancel_Motion_Setpoint(void);

/**
 * @brief 投递紧急命令（停止/使能）：非阻塞, 先于运动 setpoint 与普通队列发送
 * @return true=已投递; false=参数为空/驱动未初始化
 */
bool Motor_Send_Cmd_Urgent(const MotorCmd_t *cmd);

/**
 * @brief 当前普通队列占用条数（未初始化时返回 0）
 */
uint8_t Motor_Get_Queue_Usage(void);

/**
 * @brief 取命令链统计快照（未初始化也返回有效指针; 计数器在 MOTOR_BUS_STATS=0 时全 0）
 */
const MotorBusStats_t *Motor_Bus_Stats_Get(void);

/**
 * @brief 清零命令链统计（峰值/最大值一并清零）
 */
void Motor_Bus_Stats_Reset(void);

/**
 * @brief 打印命令链统计（`step bus` / `car bus` 共用；只在 shell 上下文调用，不影响控制路径）
 * @note  判读方法见实现处注释；MOTOR_BUS_STATS=0 时只打印一行 "statistics disabled"。
 */
void Motor_Bus_Stats_Print(void);

/**
 * @brief 查询电机距上次应答已经过的毫秒数
 * @param motor_id 电机总线 ID
 * @return 毫秒数; 从未收到应答或 ID 非法时返回 UINT32_MAX
 */
uint32_t Motor_Rx_Age_Ms(uint8_t motor_id);

/**
 * @brief 日志输出宏（可选）
 */
#define ZDT_V5_LOG(fmt, ...) logPrintln(fmt, ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __ZDT_V5_PORT_H__ */
