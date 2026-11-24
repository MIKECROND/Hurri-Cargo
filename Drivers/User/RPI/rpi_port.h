/**
 * @file rpi_port.h
 * @author T-Car Team
 * @brief 树莓派(RPi)串口通信接口
 *
 * 接口分三类:
 *   1) 单次命令: 颜色/数字检测、圆盘/圆环标定、完成数量、激光雷达(按需编译);
 *      收到应答后自动补发空闲命令(0x11), 让树莓派停止重复发送;
 *      "树莓派是否真的停发"的静默判定推迟到下一条命令前, 不占用本条命令的关键路径;
 *      若下一条命令前树莓派仍在重复发送, 该条命令直接返回失败, 由上层重试
 *      (雷达链路是长会话轮询, 不补发空闲命令, 见 RPI_SetGridVal);
 *   2) 组合接口: RPI_Cal(标定 + 输出相对当前位置的修正量);
 *   3) 调试接口: RPI_Send_Raw(原始字节收发, 不做帧校验, 收发后同样补发空闲命令)。
 *
 * 所有接口均为阻塞式: 最长等待 RPI_RSP_TIMEOUT_MS(见 rpi_config.h)。
 * 链路层发送使用 USART2 DMA(HAL_UART_Transmit_DMA), 但接口仍同步等待"最后一位已发出"的
 * 发送完成; 接收为逐字节中断。命令事务由模块内部互斥锁串行化, 任务/shell 并发调用安全。
 * 协议(命令字/应答帧格式/功能开关)集中在 rpi_config.h, 本文件只声明接口。
 * 可调参数见 rpi_config.h (禁改)。
 */

#ifndef __RPI_PORT_H__
#define __RPI_PORT_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <stdbool.h>
#include <stdint.h>
#include "rpi_config.h"
#include "nav_map.h" /* TargetPoint_t */

/* ==========================================================================
 * 颜色
 * ========================================================================== */

/**
 * @brief 颜色枚举(顺序即协议值)
 */
typedef enum {
    COLOR_UNKNOWN = 0, // 未知
    COLOR_RED, // 红色
    COLOR_YELLOW, // 黄色
    COLOR_BLUE, // 蓝色
    COLOR_GREEN, // 绿色
    COLOR_BLACK, // 黑色
    COLOR_LIGHTBLUE, // 浅蓝色
} RPI_Color_t;

/* ==========================================================================
 * 标定
 * ========================================================================== */

/**
 * @brief 标定类型
 * @note 原料区用圆盘, 粗加工区/暂存区用圆环; 圆环应答额外携带朝向误差
 */
typedef enum {
    CAL_DISK = 0, // 圆盘(原料区)
    CAL_RING_FIRST, // 圆环-第一次(粗加工区)
    CAL_RING_SECOND, // 圆环-第二次(保留, 旧暂存区第二轮命令 0xD1)
    CAL_STORAGE, // 暂存区标定(命令 0xE1, 应答格式同圆环: 带朝向误差)
    CAL_NONE, // 无效值(不标定)
} RPI_Cal_t;

/* ==========================================================================
 * 初始化
 * ========================================================================== */

/**
 * @brief 初始化树莓派通信(USART2 DMA 发送 + 中断接收 + 回调注册 + 链路互斥)
 * @return true=初始化成功
 * @note 初始化末尾会补发一次空闲命令(0x11): 上电/复位时树莓派可能还停在上一次命令状态。
 */
bool RPI_Init(void);

/* ==========================================================================
 * 单次命令(内部自动补发空闲命令 0x11)
 * ========================================================================== */

/**
 * @brief 检测物料颜色(单色)
 * @param color 输出: 检测到的颜色
 * @return 检测状态
 * @note 树莓派实际脚本(1.py)应答帧: [0xFE, color, 0xEF](与通用帧相同);
 *       兼容早期协议相反的 [0xEF, color, 0xFE]。
 */
bool RPI_Detect_Color(RPI_Color_t *color);

/**
 * @brief 开始颜色识别会话: 下发一次颜色识别命令, 之后暂不补发 0x11, 让树莓派连续重复上报
 * @return true=识别命令已下发(或会话已在进行); false=链路忙/链路准备失败/发送失败
 * @note 用于原料区抓取: 抓取期间用 RPI_ColorSessionRead 连续采样(约 100ms 一帧),
 *       避免每采一次都要"识别命令 -> 补 0x11 -> 重新颜色锁定(约 2s)"。
 */
bool RPI_ColorSessionBegin(void);

/**
 * @brief 颜色识别会话中读取下一帧颜色(不重新下发识别命令, 也不补 0x11)
 * @param color      输出: 检测到的颜色
 * @param timeout_ms 等待下一帧的超时(ms); 首帧等颜色锁定用 RPI_RSP_TIMEOUT_MS, 后续帧用 RPI_COLOR_SAMPLE_TIMEOUT_MS
 * @return true=读到一帧颜色; false=会话未开始/链路忙/超时
 */
bool RPI_ColorSessionRead(RPI_Color_t *color, uint32_t timeout_ms);

/**
 * @brief 结束颜色识别会话: 补发 0x11 让树莓派停止连续上报
 * @note 抓完当前目标颜色后调用; 下一次抓取前用 RPI_ColorSessionBegin 重新下发识别命令。
 */
void RPI_ColorSessionEnd(void);

/**
 * @brief 检测圆环内数字
 * @param num 输出数字
 * @return 检测状态
 * @note 应答帧: [0xFE, num, 0xEF];
 *       注意: 当前树莓派脚本(1.py)的 shuzi_start 为 TODO, 不会返回应答, 本接口会等待到超时。
 */
bool RPI_Detect_NUM(uint8_t *num);

/**
 * @brief 偏移/朝向校准(圆盘/圆环)
 * @param err_x         x 轴误差(mm, 带符号)
 * @param err_y         y 轴误差(mm, 带符号)
 * @param err_angle_deg 朝向误差(deg, 带符号); 仅圆环标定携带, 圆盘标定输出 0
 * @param type          校准类型
 * @return 校准状态
 * @note 圆盘应答: [0xFE, dir_x, dir_y, abs_dx, abs_dy, 0xEF]
 *       圆环应答: [0xFE, dir_x, dir_y, abs_dx, abs_dy, dir_angle, abs_angle, 0xEF]
 *       abs_angle 上报值为 角度(°)×10;
 *       符号位按树莓派实际脚本解码(与旧文档相反): 圆盘 dir_x/dir_y 1=正,
 *       圆环 dir_x/dir_angle 1=正、dir_y 0=正;
 *       若脚本正方向与车体坐标相反, 用 rpi_config.h 的 RPI_CAL_SIGN_X/Y/ANGLE 翻符号。
 */
bool RPI_Calibrate(int16_t *err_x, int16_t *err_y, float *err_angle_deg, RPI_Cal_t type);

/**
 * @brief 查询标定类型对应的命令字(调试/shell 工具打印实际发送的命令用)
 * @param type 标定类型
 * @return 命令字(如 0xC1/0xA1/0xE1); 非法类型返回 0
 */
uint8_t RPI_CalCmd(RPI_Cal_t type);

/* ==========================================================================
 * 标定 + 目标点修正
 * ========================================================================== */

/**
 * @brief 对目标点做视觉二次标定, 输出"相对当前位置的视觉修正量"
 * @param name            目标点名称(按 nav_map 查表, 用于日志/取预设)
 * @param offset_x_mm     目标点机体系 X 固定偏移(mm, +前)
 * @param offset_y_mm     目标点机体系 Y 固定偏移(mm, +左)
 * @param cal_type        校准类型
 * @param out             输出: 相对修正量(机体系): pose.x/pose.y 位置(cm), pose.yaw 角度(deg)
 * @param out_yaw_err_deg 输出: 视觉 yaw 误差(°), 可为 NULL; 圆盘标定无该字段, 输出 0
 * @return 标定状态
 * @note 输出不叠加当前位姿: 位置修正 = (X 固定偏移 + 视觉 x 误差, Y 固定偏移 + 视觉 y 误差) cm,
 *       角度修正 = 视觉角度误差; 由任务层决定用底盘位置模式走完/交给机械臂基座补偿。
 *       任务层可选择不用该 yaw 误差旋转车身, 而把 out_yaw_err_deg 交给机械臂基座补偿
 *       (见 mission_config.h 的 MISSION_CAL_YAW_FIX_WITH_ARM)。
 */
bool RPI_Cal(
    const char *name, int offset_x_mm, int offset_y_mm, RPI_Cal_t cal_type, TargetPoint_t *out, float *out_yaw_err_deg);

/* ==========================================================================
 * 调试: 原始字节收发
 * ========================================================================== */

/**
 * @brief 直接发送原始字节并等待原始应答(调试用)
 * @param data       待发送字节
 * @param len        发送长度
 * @param rsp        应答缓冲区, 传 NULL 表示只发不收
 * @param size       应答缓冲区容量
 * @param timeout_ms 等待应答超时(ms)
 * @return 实际接收到的字节数, 0 表示无应答
 * @note 与单次命令的区别: 不做帧校验, 供 shell 手工调试; 收发后同样补发空闲命令(0x11)
 *       停止树莓派的重复发送; 停发确认同样推迟到下一条命令前, 避免污染后续命令的应答。
 *       链路未就绪(树莓派仍在重复发送)时直接返回 0; rsp 传 NULL 表示只发不收, 此时补发空闲
 *       命令前会等 RPI_RAW_IDLE_DELAY_MS, 避免树莓派把原始命令和 0x11 读进同一批数据。
 */
uint16_t RPI_Send_Raw(const uint8_t *data, uint16_t len, uint8_t *rsp, uint16_t size, uint32_t timeout_ms);

/**
 * @brief 读取接收脏数据统计(非等待应答期间收到并丢弃的数据)
 * @param discarded_frames 输出: 丢弃事件计数(中断接收下约等于丢弃字节数), 可为 NULL
 * @param discarded_bytes  输出: 丢弃字节数, 可为 NULL
 * @note 正常运行时基本不增长; 增长说明树莓派有非请求输出(重复帧/迟到应答), 已被传输层丢弃,
 *       不会污染命令解析。可用 `rpi stat` 查看。
 */
void RPI_GetRxStats(uint32_t *discarded_frames, uint32_t *discarded_bytes);

/* ==========================================================================
 * 可选功能(由 rpi_config.h 的功能开关编译)
 * ========================================================================== */

#if RPI_COMPLETENUM_ENABLED == 1
/**
 * @brief 获取已完成数量
 * @param picknum  输出: 已拾取数量
 * @param placenum 输出: 已放置数量
 * @return 获取状态
 * @note 应答帧: [0xFE, dir_1, dir_2, abs_1, abs_2], 无固定帧尾
 */
bool RPI_SetCompleteNum(int *picknum, int *placenum);
#endif /* RPI_COMPLETENUM_ENABLED */

#if RPI_RADAR_AVAILABLE == 1

/**
 * @brief 激光雷达扫描单次轮询状态
 */
typedef enum {
    RPI_SCAN_ONGOING = 0, // 收到有效帧, 继续扫描
    RPI_SCAN_DONE, // 收到 0xEF, 扫描完成
    RPI_SCAN_ERROR, // 超时/帧错误
} RPI_ScanStatus_t;

/**
 * @brief 激光雷达扫描单次轮询: 取一格并写入地图
 * @return 扫描状态
 * @note 长会话链路: 每次 0x1F 取一格, 直到收到 0xEF 结束通知; 本函数不补发空闲命令
 */
RPI_ScanStatus_t RPI_SetGridVal(void);

#endif /* RPI_RADAR_AVAILABLE */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __RPI_PORT_H__ */
