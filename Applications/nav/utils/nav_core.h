/**
 * @file nav_core.h
 * @author T-Car Team
 * @brief 导航核心控制器头文件
 * 可调参数见 nav_config.h / Applications/nav/README.md。
 */

#ifndef __NAV_CORE_H__
#define __NAV_CORE_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "stdbool.h"
#include "nav_map.h"

/**
 * @brief 导航状态枚举
 */
typedef enum {
    NAV_STATE_IDLE = 0,
    NAV_STATE_RUNNING,
    NAV_STATE_STOPPING,
    NAV_STATE_COMPLETE,
    NAV_STATE_ERROR,
} NavState_t;

bool Nav_GoTo(uint8_t target_id);
bool Nav_GoToDirect(TargetPoint_t *target);
bool Nav_GoTo_fromName(const char *name);
void Nav_Stop(void);
NavState_t Nav_GetState(void);

/**
 * @brief 执行一个导航控制周期（非阻塞）
 *
 * 正常固件由 Nav_Task 每 NAV_UPDATE_TIME(5ms) 调用一次；抽出来是为了：
 *   1) 主机端仿真可以按 5ms 直接步进真实控制代码（免实车整定/回归）；
 *   2) `nav trace` 等调试工具读同一份状态。
 * 时间基准仍用 osKernelGetTickCount()，函数内部不调用 osDelay。
 */
void Nav_Update(void);

/**
 * @brief 导航运行时整定参数（默认值来自 nav_config.h；`nav par` shell 命令可读写）
 * @note  改这些参数只影响运行中的控制律，不改变逻辑/安全判据；复位后恢复宏默认值。
 */
typedef struct {
    float yaw_kp; // 航向 PID 比例
    float yaw_ki; // 航向 PID 积分
    float yaw_kd; // 航向 PID 微分
    float yaw_d_filter_tau_s; // 航向 PID 微分项一阶低通时间常数 (s)，0=不滤波
    float ff_weight; // 航向前馈权重
    float fb_weight; // 航向反馈权重
    float yaw_gain_far; // 远距离航向增益系数
    float yaw_gain_fast; // 过弯快速跟随增益系数
    float align_dist; // 高增益对齐距离 (cm)
    float xy_kp; // 远距离横向位置 P 增益
    float xy_ki; // 远距离横向位置 I 增益
    float near_gain; // 近目标位置增益倍率
    float near_factor; // 近目标阈值倍率（相对到达距离阈值）
    float traj_speed_cap; // 近目标速度上限 (cm/s)
    float startup_speed; // 起步最小速度 (cm/s)
    float min_speed; // 规划最低速度 (cm/s)
    float yaw_max_speed; // 航向环最大角速度 (deg/s)
    float yaw_min_speed; // 航向环最小角速度（克服静摩擦）(deg/s)
    float yaw_accel_limit; // 航向角加速度限幅 (deg/s²)
    uint32_t stop_settle_ms; // 到达后 STOPPING 最少等待 (ms)
} NavTunables_t;

/**
 * @brief 导航控制调试快照（`nav trace` 使用；每 5ms 由 Nav_Update 刷新）
 */
typedef struct {
    uint8_t state; // NavState_t
    uint8_t running; // 1=导航运行中
    Pose2D_t pose; // 当前位姿（cm/deg）
    TargetPoint_t target; // 当前目标（IDLE 时全零）
    float distance; // 剩余距离 (cm)
    float yaw_error; // 航向误差 (deg)
    float plan_speed; // 梯形规划速度幅值 (cm/s)
    float cmd_vx; // 本周期下发的机体系 X 速度 (cm/s)
    float cmd_vy; // 本周期下发的机体系 Y 速度 (cm/s)
    float cmd_w; // 本周期下发的角速度 (deg/s)
    float meas_vx; // OPS 位姿差分估计的机体系 X 速度 (cm/s)
    float meas_vy; // OPS 位姿差分估计的机体系 Y 速度 (cm/s)
    float meas_speed; // 实测速度幅值 (cm/s)
    uint32_t meas_age_ms; // 距最近一次位姿变化的毫秒数
    uint16_t arrive_count; // 连续到达计数
    uint32_t task_elapsed_ms; // 本段导航已运行时间 (ms)
} NavDebug_t;

/**
 * @brief 读取导航调试快照
 * @param out 输出快照，不能为 NULL
 */
void Nav_GetDebug(NavDebug_t *out);

/**
 * @brief 读取当前整定参数
 * @param out 输出参数，不能为 NULL
 */
void Nav_GetTunables(NavTunables_t *out);

/**
 * @brief 运行时修改一个整定参数
 * @param name  参数名（见 NavTunables_t 字段名，如 "yaw_kp"/"stop_settle_ms"）
 * @param value 新值（整型参数按浮点传，内部四舍五入）
 * @return true=已修改；false=未知参数名/值超出安全范围
 * @note  安全范围检查：增益/权重/速度都限制在合理区间，避免现场误输入导致失控。
 */
bool Nav_SetTunable(const char *name, float value);

/**
 * @brief 运行中直接切换目标点（不停车、不打断行驶）
 *
 * 语义与 Nav_GoToDirect 的区别：不重新进入 RUNNING、不清零速度状态（last_vx/last_vy/
 * last_yaw_speed 保持连续），只替换目标位姿并重置本段的位置积分/到达计数/PID 状态，
 * 让车"边开边换目标"，用于任务层的不停车通过点（过弯）：
 *   - 目标点指针由导航层缓存，调用方必须保证其持续有效（可用静态存储）；
 *   - 只在 NAV_STATE_RUNNING 时有效，其它状态返回 false（调用方需回退到停车后重新导航）；
 *   - yaw PID 的 last_error 置为切换瞬间的 yaw 误差，避免微分项因目标 yaw 跳变而冲击。
 * @param target 新目标点（须持续有效）
 * @return true 切换成功；false 未初始化/目标为空/导航不在 RUNNING
 */
bool Nav_Retarget(TargetPoint_t *target);

/**
 * @brief 设置/清除"yaw 快速跟随"标志
 *
 * 置位后，远距离（distance > NAV_ALIGN_DIST）航向环的增益系数由 NAV_YAW_GAIN_FAR
 * 提升到 NAV_YAW_GAIN_FAST，用于过弯时"边走边转"更快把车头转向下一段目标；
 * Nav_GoToDirect / Nav_Stop 会自动复位，避免影响普通导航段。
 * @param enable true=快速跟随；false=恢复默认
 */
void Nav_SetYawFast(bool enable);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __NAV_CORE_H__ */
