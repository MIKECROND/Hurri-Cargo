/**
 * @file rpi_config.h
 * @brief 树莓派(RPi)链路配置：功能开关 / 链路参数 / 命令字 / 应答帧规格
 *
 * 本文件只放"配置与协议常量"，不包含任何驱动实现：
 *   - 功能开关：RPI_RADAR_ENABLED / RPI_COMPLETENUM_ENABLED / RPI_RADAR_AVAILABLE
 *   - 链路参数：串口接收缓冲大小、应答超时
 *   - 命令字  ：MCU -> 树莓派 的单字节命令
 *   - 应答规格：树莓派 -> MCU 的帧格式、负载长度与字段下标
 *
 * 帧格式约定：
 *   - 通用应答帧 = [0xFE, 负载..., 0xEF]
 *   - 颜色检测应答 = [0xFE, color, 0xEF]（树莓派实际脚本; 兼容早期相反的 [0xEF, color, 0xFE]）
 *   - 树莓派每 100ms 重复发送同一应答直到收到新命令(0x11 停止发送); MCU 收到应答后补发 0x11,
 *     并在下一条命令前确认树莓派已停发(停发确认延迟, 见 rpi_port.c 的 RPI_LinkPrepare)
 */

#ifndef __RPI_CONFIG_H__
#define __RPI_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "planner_config.h" /* PLANNER_ENABLED / PLANNER_PLATFORM */

/* ==========================================================================
 * 1. 功能开关
 * ========================================================================== */

/* 默认关闭(非核心功能); 允许构建系统 -D 覆盖, 现场切换无需改源码 */
#ifndef RPI_RADAR_ENABLED
#define RPI_RADAR_ENABLED 0 // 激光雷达探测是否启用
#endif
#ifndef RPI_COMPLETENUM_ENABLED
#define RPI_COMPLETENUM_ENABLED 0 // 获取完成数量是否启用
#endif

/* 雷达链路只有在"规划器跑在 MCU 上"时才有意义 */
#if RPI_RADAR_ENABLED == 1 && PLANNER_ENABLED == 1 && PLANNER_PLATFORM == PLATFORM_MCU
#define RPI_RADAR_AVAILABLE 1
#else
#define RPI_RADAR_AVAILABLE 0
#endif

/* ==========================================================================
 * 2. 链路参数
 * ========================================================================== */

/**
 * 命令应答超时(ms): 树莓派要等检测"稳定"才发帧(颜色锁定约 2s, 圆环标定 1~3s),
 * **这是首帧等待的颜色/圆环锁定时间, 不能为了快改小**(改小会在视觉正常时误判超时、任务失败);
 * 只在树莓派离线/脚本异常时才是纯等待, 由上层超时(ARM_GRAB_COLOR_TIMEOUT_MS 等)兜底。
 * 允许构建系统 -D 覆盖(现场如确认某类命令更快可单独调小, 但必须先实测)。
 */
#ifndef RPI_RSP_TIMEOUT_MS
#define RPI_RSP_TIMEOUT_MS 6000
#endif
/**
 * 颜色识别会话中等待下一帧的超时(ms):
 *   树莓派收到识别命令后每个主循环重复上报一帧(约 100ms 一帧), 会话内采样不需要重新下发命令,
 *   因此后续帧只需短超时; 首帧仍用 RPI_RSP_TIMEOUT_MS(等颜色锁定)。
 */
#ifndef RPI_COLOR_SAMPLE_TIMEOUT_MS
#define RPI_COLOR_SAMPLE_TIMEOUT_MS 1000
#endif
#ifndef RPI_TX_TIMEOUT_MS
#define RPI_TX_TIMEOUT_MS 100 // DMA 发送完成等待超时(ms): 单字节在 115200 下约 0.1ms, 超时视为链路异常
#endif
#define RPI_TX_POLL_MS 10 // 发送完成事件延迟/丢失时的 HAL 状态查询间隔(ms)
#define RPI_RX_BUF_SIZE 32 // 原始/调试接收缓冲大小(仅 rpi_tools 使用; 命令解析用 RPI_RX_ACCUM_SIZE)
#define RPI_RAW_IDLE_DELAY_MS 20 // `rpi send` 只发不收(rsp=NULL)时, 补发空闲命令前的短延时(ms)
/**
 * 应答累积缓冲: 一次命令等待期间收到的所有字节先累积到这里再解析。
 * 允许"帧被拆成多次中断交付"以及"应答前有少量垃圾", 解析在缓冲里按帧头/帧尾对齐定位,
 * 命中后从帧头偏移处拷出整帧(允许前导垃圾)。
 */
#define RPI_RX_ACCUM_SIZE 96
#ifndef RPI_RX_QUIET_MS
#define RPI_RX_QUIET_MS 5 // 已收到应答后的帧间静默窗口(ms): 窗口内无新数据视为一帧结束
#endif
/**
 * 静默判定事件 UART2_RX_ACT 的置位节流(ms): 只在任务等待静默判定时置位, 且最多每这么久一次。
 * 原因: ISR 里 osEventFlagsSet 在 CMSIS-RTOS2 中经 FreeRTOS 定时器命令队列投递
 * (configTIMER_QUEUE_LENGTH=10, 守护任务优先级 2); 树莓派异常持续发送时若每字节都置位,
 * 队列可能被打满, 进而挤掉其它模块的 ISR 事件(如 UART6_TX_IDLE), 让它们卡死。
 * 静默判定的真正判据是"最后一次收到字节的时间戳", 事件只用于快速唤醒, 因此可以放心节流。
 */
#define RPI_RX_EVT_THROTTLE_MS 20
/**
 * 命令结束后立刻补发 0x11 让树莓派停止重复发送, 但"是否真的停发"的静默判定
 * 推迟到下一条命令前(见 rpi_port.c 的 RPI_LinkPrepare): 调用方在两条命令之间通常要运动/等待,
 * 这段时间足够树莓派处理 0x11, 停发等待多数情况下没有额外开销。
 * 判定方式: 连续 RPI_IDLE_QUIET_MS 没有收到任何字节 -> 认为已停发; 最多等 RPI_IDLE_MAX_WAIT_MS,
 * 超时则判定链路未就绪、本条命令直接失败(下一条命令前会重新补发 0x11 再等, 自愈式重试)。
 * 注意: 1.py 的 SEND_INTERVAL_S(100ms) 只是"最小发送间隔", 实际重复周期 = 主循环耗时
 * (相机采集+检测+绘制, 数十~数百 ms), 所以静默窗口必须明显大于 100ms, 默认 300ms。
 * 时序审计结论: 该窗口只在"下一条命令前仍有最近一帧到达"时才产生等待; 任务流程里两条 RPI 命令
 * 之间都隔着数秒的机械臂动作/导航, 正常路径上是零等待, **不要为了提速改小**(改小会让
 * 树莓派仍在重复发送时被误判为已停发, 上一条命令的重复帧可能污染下一条同格式命令的应答)。
 * 允许构建系统 -D 覆盖(现场若确认树莓派主循环更快可调小, 但需先用 `rpi stat` 验证无脏帧)。
 */
#ifndef RPI_IDLE_QUIET_MS
#define RPI_IDLE_QUIET_MS 300
#endif
#ifndef RPI_IDLE_MAX_WAIT_MS
#define RPI_IDLE_MAX_WAIT_MS 1200
#endif
/** 链路互斥等待(ms): 命令是"发送-等应答-发空闲"的完整事务, 任务/shell 并发调用时串行化 */
#ifndef RPI_LOCK_TIMEOUT_MS
#define RPI_LOCK_TIMEOUT_MS 12000
#endif

/* 应答帧定界符 */
#define RPI_FRAME_HEAD 0xFE // 帧头(通用命令)
#define RPI_FRAME_TAIL 0xEF // 帧尾(通用命令)
/**
 * 颜色应答帧: 树莓派实际脚本(1.py)发送 [0xFE, color, 0xEF](与通用帧相同);
 * 早期协议版本曾用相反的 [0xEF, color, 0xFE], 解析保持兼容两种顺序。
 */
#define RPI_COLOR_FRAME_HEAD RPI_FRAME_HEAD // 0xFE
#define RPI_COLOR_FRAME_TAIL RPI_FRAME_TAIL // 0xEF
#define RPI_COLOR_FRAME_HEAD_ALT 0xEF // 兼容旧顺序: 帧头
#define RPI_COLOR_FRAME_TAIL_ALT 0xFE // 兼容旧顺序: 帧尾

/* ==========================================================================
 * 3. 命令字(MCU -> 树莓派, 单字节)
 *    注释括号内为树莓派侧 Python 脚本中的变量名, 便于两端对照
 * ========================================================================== */

#define RPI_CMD_CAL_RING_FIRST 0xA1 // 圆环定位-第一次(粗加工区) (yuanhuan_start)
#define RPI_CMD_DETECT_COLOR 0xB1 // 颜色检测 (yanse_start)
#define RPI_CMD_CAL_DISK 0xC1 // 圆盘定位-原料区 (zhuanpan_start)
#define RPI_CMD_CAL_RING_SECOND 0xD1 // 圆环定位-第二次(保留: 旧暂存区第二轮命令)
#define RPI_CMD_CAL_STORAGE 0xE1 // 暂存区定位/矫正(暂存区专用命令, 应答格式同圆环: 带朝向误差)
#define RPI_CMD_COMPLETE_NUM 0x0F // 获取已完成数量
#define RPI_CMD_DETECT_NUM 0x10 // 圆环内数字检测 (shuzi_start)
#define RPI_CMD_IDLE 0x11 // 空闲(单次命令结束后让树莓派复位) (xiuxian_start)

#if RPI_RADAR_AVAILABLE == 1
#define RPI_CMD_RADAR_SCAN 0x1F // 激光雷达扫描取一格
#define RPI_CMD_RADAR_DONE 0xEF // 扫描完成通知(单字节; 0xEF 兼作帧尾, 不允许出现在负载中)
#endif

/* ==========================================================================
 * 4. 应答帧规格(树莓派 -> MCU)
 *    帧布局: [0xFE, 负载..., 0xEF]；颜色检测: [0xFE, color, 0xEF](实际脚本)
 *
 *    命令              负载长度  负载字段
 *    0x0B 颜色            1       [color]                                      (RPI_Color_t; 兼容旧 [0xEF,color,0xFE])
 *    0x10 数字            1       [num]                                        (注意: 当前树莓派脚本未实现, 无应答)
 *    0x0C 圆盘            4       [dir_x, dir_y, abs_dx, abs_dy]
 *    0x0A/0x0D 圆环       6       [dir_x, dir_y, abs_dx, abs_dy, dir_angle, abs_angle]
 *    0x0F 完成数量        4       [dir_1, dir_2, abs_1, abs_2]                  (脚本未实现, 功能默认关闭)
 *    0x1F 激光雷达        4       [dir_x, dir_y, abs_x, abs_y]                  (脚本未实现, 功能默认关闭)
 *
 *    符号位约定(必须按树莓派实际脚本 1.py 解码, 旧文档的"0=正"不适用于以下字段):
 *      - 圆盘 0x0C: dir_x / dir_y 为 "1=正, 0=负";
 *      - 圆环 0x0A/0x0D: dir_x / dir_angle 为 "1=正, 0=负", dir_y 为 "0=正, 1=负"。
 *    幅值: 位移 mm(脚本按 uint8 截断, 最大 255), 角度单位 0.1°(同样截断到 255)。
 *
 *    应答发送节奏: 脚本在主循环里每 SEND_INTERVAL_S(100ms) 重复发送同一应答, 直到收到新的命令字节;
 *    收到 0x11(空闲)后停止发送。因此 MCU 侧收到应答后补发 0x11, 并把"等树莓派真正停发"的
 *    静默判定推迟到下一条命令前(RPI_LinkPrepare): 既避免重复帧污染下一条命令, 也保证树莓派
 *    已完成状态切换(检测器复位); 命令之间调用方通常有运动/等待, 这段等待多数情况下是零开销。
 *    等待期间只接受与命令格式匹配的帧。
 *
 *    发送方式: USART2 TX 走 DMA(HAL_UART_Transmit_DMA), 由发送完成(TC)中断置 UART2_TX_IDLE
 *    事件通知; 接收仍为逐字节中断(RXNE), 只有等待应答期间的数据进入解析缓冲。
 *
 *    脏数据防护(见 rpi_port.c 传输层):
 *      - 只有"正在等待应答"期间收到的数据才进入解析缓冲, 其余(重复帧/RPI 调试输出)直接丢弃并计数;
 *      - 每次命令发送前清空接收(中止并重启中断接收), 丢弃残留半帧;
 *      - 解析时在接收缓冲里扫描帧头/帧尾对齐(允许前导垃圾), 不再要求 rsp[0] 就是帧头。
 * ========================================================================== */

#define RPI_PAYLOAD_LEN_COLOR 1
#define RPI_PAYLOAD_LEN_NUM 1
#define RPI_PAYLOAD_LEN_CAL_DISK 4
#define RPI_PAYLOAD_LEN_CAL_RING 6
#define RPI_PAYLOAD_LEN_COMPLETE 4
#define RPI_PAYLOAD_LEN_RADAR 4

/* 校准应答负载字段下标(相对负载首字节; 圆盘只使用前 4 个) */
#define RPI_CAL_IDX_DIR_X 0
#define RPI_CAL_IDX_DIR_Y 1
#define RPI_CAL_IDX_ABS_DX 2
#define RPI_CAL_IDX_ABS_DY 3
#define RPI_CAL_IDX_DIR_ANGLE 4
#define RPI_CAL_IDX_ABS_ANGLE 5

/* 0x0F/0x1F 应答负载字段下标(相对负载首字节; 两命令同布局) */
#define RPI_PAIR_IDX_DIR_1 0
#define RPI_PAIR_IDX_DIR_2 1
#define RPI_PAIR_IDX_ABS_1 2
#define RPI_PAIR_IDX_ABS_2 3

/* 圆环朝向: 上报值 = 角度(°)×10, 解析时除以该系数还原为度 */
#define RPI_CAL_ANGLE_SCALE 10.0f

/* ===== 标定误差方向校准(按实车坐标方向微调, 默认 1.0f) =====
 * 树莓派脚本解码出的 dx/dy/angle 正方向若与车体/机械臂坐标系相反,
 * 把对应宏改成 -1.0f 即可整体翻符号(不必改解码逻辑)。 */
#ifndef RPI_CAL_SIGN_X
#define RPI_CAL_SIGN_X 1.0f // x 误差方向: 1.0=按脚本, -1.0=反向
#endif
#ifndef RPI_CAL_SIGN_Y
#define RPI_CAL_SIGN_Y 1.0f // y 误差方向: 1.0=按脚本, -1.0=反向
#endif
#ifndef RPI_CAL_SIGN_ANGLE
#define RPI_CAL_SIGN_ANGLE 1.0f // yaw 误差方向: 1.0=按脚本, -1.0=反向
#endif

/* ==========================================================================
 * 5. 帧长计算
 * ========================================================================== */

/* 定长帧: 帧头(1) + 负载 + 帧尾(1) */
#define RPI_FRAME_LEN(payload_len) ((payload_len) + 2)
/* 帧尾下标 */
#define RPI_FRAME_TAIL_IDX(payload_len) ((payload_len) + 1)

/* 无固定帧尾帧的最小有效长度: 帧头(1) + 负载 */
#define RPI_FRAME_MIN_LEN(payload_len) ((payload_len) + 1)
/* 无固定帧尾帧允许的前导字节数, 接收长度 = 帧头(1) + 负载 + 前导字节 */
#define RPI_RSP_PREFIX_MAX 2
#define RPI_RSP_LEN_LENIENT(payload_len) (RPI_FRAME_MIN_LEN(payload_len) + RPI_RSP_PREFIX_MAX)

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __RPI_CONFIG_H__ */
