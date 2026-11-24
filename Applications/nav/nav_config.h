/**
 * @file nav_config.h
 * @author Pcb-yun (pcbyinyun@163.com)
 * @brief 导航配置头文件
 */

#ifndef __NAV_CONFIG_H__
#define __NAV_CONFIG_H__

#define NAV_MAP_TOOL 1 // 是否启用地图shell工具

/* ===== OPS 定位数据新鲜度保护 =====
 * RUNNING/STOPPING 期间若距最近一次完整 OPS 帧超过该阈值，立即停车并报 NAV_STATE_ERROR
 * （任务层会按既有重试策略处理），避免带错误位姿继续运动。
 * 阈值需大于 OPS 正常帧间隔的 3~5 倍：OPS 帧率按 ≥30Hz 设计，150ms 约 4~5 帧。 */
#ifndef NAV_OPS_STALE_TIMEOUT_MS
#define NAV_OPS_STALE_TIMEOUT_MS 150
#endif

#define NAV_UPDATE_TIME 5 // 导航状态更新时间间隔（毫秒）
#define NAV_TIME_DECAY_FACTOR 5.0f // 时间衰减因子（定位融合）
#define NAV_MAX_SENSOR 2 // 最大传感器数量

/* ===== 位置环（统一"制动感知速度上限 + 全误差向量 P"控制律）=====
 * 不再分远/近两套模式：速度幅值 = min(kp*距离, 制动上限)，方向始终指向目标。
 * 远处由制动上限主导（梯形巡航/减速），近处由 P 主导（指数收敛），全程连续无模式切换。 */
#define NAV_STARTUP_SPEED 1.5f // 起步最小速度（cm/s），克服静摩擦/整数RPM量化；误差大于死区时不低于此值
#define NAV_MIN_SPEED (NAV_STARTUP_SPEED * 0.6f) // 保留兼容（新控制律起步下限统一用 startup_speed）

#define NAV_ALIGN_XY_KP 1.6f // 位置 P 基础增益（与 near_gain 相乘得实际 kp）；仿真整定 1.2->1.6 加速近目标收敛
#define NAV_ALIGN_XY_KI 0.02f // 保留兼容（新控制律位置环无积分，全误差向量 P 天然消除横向偏差）

#define NAV_NEAR_TARGET_FACTOR 12.0f // 近目标区半径倍数（相对 distance_threshold）：区内速度上限取 traj_speed_cap
#define NAV_NEAR_TARGET_GAIN 6.0f // 近目标位置增益倍率（实际 kp = xy_kp * near_gain）；仿真整定 3.1->6.0
#define NAV_TRAJ_SPEED_CAP 50.0f // 近目标区速度上限（cm/s）

/* 位置死区（cm）：误差小于此值不再输出修正速度。旧近目标死区 0.375cm/轴 + 停止区增益减半
 * 会留下 ~1cm 残差（short5 实测）；收紧到 0.15cm 提高到位精度，配合 P 连续收敛避免 chatter。 */
#ifndef NAV_POS_DEADBAND_CM
#define NAV_POS_DEADBAND_CM 0.15f
#endif

/* 速度滞后补偿时间常数 (s)：制动上限按"v²/(2a) + v*lag <= 剩余距离"求解，
 * 把电机命令链延迟 + 轮速一阶滞后的等效滞后计入减速距离，抑制延迟下的终点超调。
 * 默认 0.06s ≈ 修复后命令链(<10ms) + 轮速滞后(40~80ms) 的等效值；实测超调大可适当加大。 */
#ifndef NAV_VEL_LAG_S
#define NAV_VEL_LAG_S 0.06f
#endif

/* ===== 航向环（sqrt 减速剖面馈 + PID 修正）=====
 * 前馈 = sign(err)*min(yaw_max, sqrt(2*a_yaw*|err|))：远处饱和到 yaw_max 快速转，
 * 近处按 sqrt 剖面平滑减速到 0，天然无过冲，取代旧"bang-bang 前馈 + 锥形衰减 + 零交叉锁存"。
 * 反馈 = PID(kp/ki/kd, D 低通, 抗饱和) 修正残差/阻尼。两者按 ff/fb 权重融合。 */
#define NAV_YAW_PID_KP 2.0f // yaw比例系数；仿真整定 1.8->2.0（再高会增大过弯振荡）
#define NAV_YAW_PID_KI 0.05f // yaw积分系数
#define NAV_YAW_PID_KD 0.6f // yaw微分系数（阻尼抑制过冲）
#define NAV_YAW_PID_OUTPUT_LIMIT 150.0f // yaw PID输出限制
#define NAV_YAW_PID_D_FILTER_TAU_S 0.03f // yaw 微分项一阶低通时间常数 (s)，0=不滤波

#define NAV_ALIGN_YAW_MAX 100.0f // 航向环最大角速度 (deg/s)
#define NAV_ALIGN_YAW_MIN 3.0f // 航向环最小角速度（克服静摩擦）(deg/s)
#define NAV_YAW_ACCEL_LIMIT 350.0f // yaw角加速度限幅 (deg/s²)，同时作为 sqrt 减速剖面的减速度
#define NAV_YAW_DEADBAND 1.0f // yaw死区（deg），与到达航向门限一致

/* 最小角速度钳位只在 |err| 大于该阈值时生效：阈值内允许平滑收敛到 0，
 * 消除旧实现"最后几度以 3°/s 爬行 + 8ms 零交叉锁存"造成的航向尾段拖延。 */
#ifndef NAV_YAW_MIN_SPEED_ERR_THRESH
#define NAV_YAW_MIN_SPEED_ERR_THRESH 3.0f
#endif

/* 航向远距离增益系数（distance > NAV_ALIGN_DIST 时乘在 yaw 输出上）：
 * 默认 0.3 —— 普通平移段弱化转向，避免"边走边转"导致打滑/终点偏移；
 * 快速 1.0 —— 任务层"不停车通过点（过弯）"置位 NAV_YAW_FAST 后使用。 */
#define NAV_YAW_GAIN_FAR 0.3f
#define NAV_YAW_GAIN_FAST 1.0f
#define NAV_ALIGN_DIST 5.0f // 航向远/近增益分界距离（cm）

#define NAV_YAW_FEEDFORWARD_WEIGHT 0.4f // yaw 剖面馈权重
#define NAV_YAW_FEEDBACK_WEIGHT 0.6f // yaw PID 反馈权重

#define NAV_ARRIVE_CHECK_COUNT 2 // 连续到达检查次数（防止惯性/打滑误判）

/* ===== 到达后停稳判定（STOPPING 状态）=====
 * "最少等待 + 滑动窗口停稳 + OPS 帧龄门控 + 距离/航向复核"：
 *   - 等待 >= NAV_STOP_SETTLE_MS，且最近 NAV_STOP_WINDOW_MS 窗口内位移/转角足够小，
 *     且 OPS 位姿足够新（NAV_STOP_FRESH_OPS_MS），才做最终距离/航向判定；
 *   - 未停稳/位姿不新鲜时最多再等 NAV_STOP_EXTRA_WAIT_MS，超过则强制终判；
 *   - 复核失败恢复 RUNNING 闭环补位，补位次数上限 NAV_STOP_MAX_RESUME。
 * 默认 150ms 的前提是电机命令链已修复（停止走紧急通道 + 运动 setpoint 邮箱）；
 * 若回退旧命令链（MOTOR_CMD_LEGACY_CHAIN=1）请把本值恢复 300。 */
#ifndef NAV_STOP_SETTLE_MS
#define NAV_STOP_SETTLE_MS 150
#endif
#define NAV_STOP_WINDOW_MS 100 // 停稳检测滑动窗口(ms)
#define NAV_STOP_WINDOW_MOVE_CM 0.4f // 窗口内位移超过该值视为仍在滑移(cm)
#define NAV_STOP_WINDOW_YAW_DEG 1.0f // 窗口内转角超过该值视为仍在滑移(deg)
#define NAV_STOP_FRESH_OPS_MS 50 // 终判前要求 OPS 位姿帧龄上限(ms)
#define NAV_STOP_MAX_RESUME 3 // 停稳/误差复核失败后恢复 RUNNING 补位的最大次数
#define NAV_STOP_EXTRA_WAIT_MS 700 // 仍在滑移/位姿不新鲜时的最长额外等待(ms)

/* ===== OPS 位姿差分实测速度估计 =====
 * 用 OPS 位姿差分估计真实速度（机体系），用于：
 *   - 到达判定：指令已减速但车体仍在动时不提前报到位；
 *   - 减速规划：实测速度参与制动上限收紧（NAV_USE_MEAS_SPEED_DECEL，默认开）；
 *   - `nav trace` 诊断：对比指令速度与实测速度。 */
#define NAV_MEAS_VEL_TAU_S 0.04f // 实测速度一阶低通时间常数(s)
#define NAV_MEAS_VEL_STALE_MS 100 // 超过该时间无位姿变化视为静止(ms)
#define NAV_MEAS_POS_EPS_CM 0.005f // 认为位姿发生变化的位移阈值(cm)
#define NAV_MEAS_YAW_EPS_DEG 0.01f // 认为位姿发生变化的航向阈值(deg)
/* 实测速度参与减速规划：实测速度高于制动上限时进一步收紧上限（提前刹车），
 * 抑制"指令已减但车还在滑行"的超调。仿真验证默认开启；实测不可用时自动回退。 */
#ifndef NAV_USE_MEAS_SPEED_DECEL
#define NAV_USE_MEAS_SPEED_DECEL 1
#endif

#endif /* __NAV_CONFIG_H__ */