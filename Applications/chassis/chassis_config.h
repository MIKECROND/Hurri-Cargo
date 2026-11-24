/**
 * @file chassis_config.h
 * @brief 麦轮小车机械参数配置
 *
 * 坐标与符号约定（机体系）：
 *   +X = 车头方向（前进），+Y = 车体左侧，Z 轴向上；
 *   俯视（从上往下看）绕 Z 轴逆时针为 yaw/角速度正方向。
 *
 * 旋转中心 C 与导航位姿点 G：
 *   逆运动学输入 (vx, vy, w) 描述的是"旋转中心 C"的运动，而不是四轮几何中心：
 *     - vx / vy = C 点在机体系下的线速度 (cm/s)；
 *     - w       = 车体绕 C 点的角速度 (deg/s)；
 *   当 w != 0 且 vx = vy = 0 时，车体严格绕 C 点原地旋转。
 *
 *   为保证导航位置闭环与运动学一致，C 必须等于导航/规划使用的车体位姿点 G。
 *   本工程建议 G = 车体几何中心（这样原地旋转时车体外廓扫掠最小、且不整体平移）：
 *     - G 相对 OPS 安装点 P 的偏移：CHASSIS_BODY_REF_OFFSET_X/Y（导航层换算）；
 *     - C(G) 相对四轮几何中心 O 的偏移：CHASSIS_ROTATION_CENTER_OFFSET_X/Y（运动学）。
 *   关系：CHASSIS_ROTATION_CENTER_OFFSET = (P - O) + (G - P)。
 *
 *   若 G 不在车体几何中心（例如 OPS 点没装在几何中心），原地旋转时车体几何中心会绕 G
 *   画圈，看起来像"车整体往前/后/侧移了一点"；此时按 README §4.3/§4.5 配置 G。
 */

#ifndef __CHASSIS_CONFIG_H__
#define __CHASSIS_CONFIG_H__

// 麦轮小车机械参数
#define WHEEL_RADIUS 3.65f // 轮子半径 (cm)
#define WHEEL_BASE_WIDTH 16.2f // 轮距宽度 (cm)，左右轮中心距
#define WHEEL_BASE_LENGTH 20.5f // 轴距长度 (cm)，前后轮中心距

/* ===== 旋转中心（原地旋转的支点）===== */
/**
 * 旋转中心 C 相对"四轮几何中心"的偏移 (cm)，机体系：+X 向车头、+Y 向车体左侧。
 *   0 / 0 = 绕四轮几何中心旋转（旧行为，与本次修改前完全一致）。
 *   例：车体几何中心在轮组中心后方 2cm、左侧 1cm → X = -2.0，Y = +1.0；
 *       单侧延长 e（只一侧凸出 e，另一侧仍在对称基准上）→ 中心向该侧偏移 e/2；
 *       车体整体相对轮组横移 e（一侧凸 e、另一侧凹 e）→ 偏移 e。
 *
 * 测量方法（二选一，详见 Applications/chassis/README.md）：
 *   1) 尺量法：量导航/OPS 位姿点（通常为车体几何中心）相对四轮几何中心的 X/Y 距离，按方向填；
 *   2) OPS 两点法：车原地转一个足够大的角度（不必精确 180°，150°~200° 均可），记录转前/转后
 *      OPS 位姿 (x0,y0,yaw0) / (x1,y1,yaw1)（OPS 位置单位 mm、角度 deg），用 README §4.2 的
 *      通用公式换算；只有"yaw0=0 且实际正好 180°"时才简化为
 *      C = ((x0 - x1)/20, (y0 - y1)/20)（cm，机体系）。
 *
 * 注意：C 必须与导航/规划使用的车体位姿点（OPS 定位点）一致。否则原地旋转时
 *       定位点会绕 C 画圈，导航位置闭环会与旋转指令互相拉扯，实际轨迹仍会偏离。
 */
#define CHASSIS_ROTATION_CENTER_OFFSET_X 0.0f // 旋转中心 X 偏移 (cm)，+ 向前
#define CHASSIS_ROTATION_CENTER_OFFSET_Y 0.0f // 旋转中心 Y 偏移 (cm)，+ 向左

/* ===== 导航位姿参考点：OPS 安装点 P → 车体参考点 G ===== */
/**
 * 车体参考点 G 相对 OPS 位姿点 P 的偏移 (cm)，机体系：+X 向车头、+Y 向车体左侧。
 *   - 0/0：导航/规划直接使用 OPS 位姿点（旧行为，与修改前完全一致）；
 *   - 非零：定位位姿按 G = P + R(yaw)*偏移 换算，任务点坐标在加载时做同样换算，
 *           整个导航/规划栈以 G 为车体位姿点。推荐把 G 取车体几何中心。
 *
 * 物理含义举例：
 *   - OPS 安装点在车体几何中心前方 2cm、右侧 0cm → 车体几何中心相对 OPS 点为后方
 *     2cm → X = -2.0、Y = 0；
 *   - OPS 在几何中心左侧 1cm → Y = +1.0（几何中心在 OPS 右侧 1cm）。
 *
 * 修改后必须同步把 CHASSIS_ROTATION_CENTER_OFFSET_X/Y 改为"G 相对四轮几何中心的偏移"：
 *   CHASSIS_ROTATION_CENTER_OFFSET = 旧标定值(P - O) + 本偏移(G - P)
 * 否则旋转时导航位姿点 G 会画圈、位置环与旋转指令互相拉扯。
 * 详见 Applications/chassis/README.md §4.3/§4.5。
 */
#define CHASSIS_BODY_REF_OFFSET_X 0.0f // 车体参考点相对 OPS 点 X 偏移 (cm)，+ 向前
#define CHASSIS_BODY_REF_OFFSET_Y 0.0f // 车体参考点相对 OPS 点 Y 偏移 (cm)，+ 向左

// 电机ID定义
#define MOTOR_FRONT_LEFT 1 // 前左轮
#define MOTOR_FRONT_RIGHT 3 // 前右轮
#define MOTOR_BACK_RIGHT 2 // 后右轮
#define MOTOR_BACK_LEFT 4 // 后左轮

// 停止命令发送参数
#define MOTOR_STOP_SEND_RETRY 3 // 停止指令发送重试次数: 命令队列满时防止停止帧被丢弃(偶发"到位仍在动")

/* ===== 运动指令下发策略（2026-11 命令链时延修复，非机械参数） =====
 * 轮速 setpoint 现在走驱动层"最新值覆盖"邮箱（非阻塞、只发最新一件），这里只决定
 * 多机打包帧之后的同步方式：
 *   1 = 打包帧内各轮"缓存等待"，再由驱动层补一帧 0xFF66 同步触发统一启动（当前行为，默认）。
 *       四轮启动时刻严格一致，起步/换向不会产生额外偏摆。
 *   0 = 打包帧内各轮"收到即执行"（snF=false），不再补同步帧：每周期少一帧，端到端时延再降
 *       约 1ms、总线占用减半；代价是四轮启动时刻由各驱动器解析完同一帧的瞬间决定（微秒级差异），
 *       仅在台架/实车确认起步直线性无明显变化后才建议改为 0。
 * 注意：该宏只影响速度模式；位置模式(car pos)走普通队列且本来就不需要同步帧。
 *       用 #ifndef 包住，可用编译选项 -DCHASSIS_MOTION_SYNC_TRIGGER=0 做台架 A/B。
 */
#ifndef CHASSIS_MOTION_SYNC_TRIGGER
#define CHASSIS_MOTION_SYNC_TRIGGER 1
#endif

/* ===== 运动 setpoint 限频/去重（2026-11 命令链时延修复，总线带宽预算） =====
 * 背景：一条运动 setpoint = 多机帧(37B≈403us) + 同步帧(4B≈44us) + 两个 1ms 帧间隔 ≈ 1.44ms。
 *   导航每 5ms 投递一次 ⇒ 200Hz 时电机控制任务约 29% 时间在发运动命令；再叠加状态轮询
 *   （底盘 4 电机×1 帧 + 机械臂 3 电机×2 帧，每 50ms 一批，≈0.53ms/帧）后余量偏紧。
 *   Emm 速度模式是**绝对值指令**，电机收到后会一直保持该速度，所以"指令几乎没变"时重复下发
 *   纯属浪费（心跳由状态轮询维持，见 zdt_v5_port.h 的心跳说明），因此做三级策略：
 *     ① 大幅变化（> URGENT_DV/DW）：立即投递（起步/停车/换向/手动调试不受限，导航下一周期生效）；
 *     ② 常规变化（> DV/DW 且 ≤ URGENT）：最多每 MIN_INTERVAL_MS 投递一次（默认 10ms → ≤100Hz）；
 *     ③ 几乎没变（≤ DV/DW）：最多每 HOLD_MS 刷新一次（默认 20ms → 稳态 50Hz），缓慢漂移不丢。
 *   阈值引起的最大速度误差 = 阈值本身（0.3cm/s / 0.5deg/s），在 20ms 保持窗口内造成的
 *   位置误差 ≤ 0.3×0.02 = 0.006cm，远小于到位判定阈值。
 *   带宽结论：稳态 50Hz、加减速 ≤100Hz；电机控制任务占用 ≈18%/33%（稳态，平均/最坏）、
 *   ≈25%/45%（加减速段），总线线速占用 <6%，普通队列（状态轮询/机械臂）不再被运动命令挤占。
 */
#ifndef CHASSIS_MOTION_SETPOINT_HOLD_MS
#define CHASSIS_MOTION_SETPOINT_HOLD_MS 20 // ③ 指令几乎无变化时的最大刷新周期(ms): 20 → 稳态 50Hz
#endif
#ifndef CHASSIS_MOTION_SETPOINT_MIN_INTERVAL_MS
#define CHASSIS_MOTION_SETPOINT_MIN_INTERVAL_MS 10 // ② 常规变化的最小投递间隔(ms): 10 → ≤100Hz
#endif
#ifndef CHASSIS_MOTION_SETPOINT_DV_CM_S
#define CHASSIS_MOTION_SETPOINT_DV_CM_S 0.3f // ③ 视为"指令未变"的线速度阈值(cm/s)
#endif
#ifndef CHASSIS_MOTION_SETPOINT_DW_DEG_S
#define CHASSIS_MOTION_SETPOINT_DW_DEG_S 0.5f // ③ 视为"指令未变"的角速度阈值(deg/s)
#endif
#ifndef CHASSIS_MOTION_SETPOINT_URGENT_DV_CM_S
#define CHASSIS_MOTION_SETPOINT_URGENT_DV_CM_S 5.0f // ① 立即投递的线速度阶跃阈值(cm/s)
#endif
#ifndef CHASSIS_MOTION_SETPOINT_URGENT_DW_DEG_S
#define CHASSIS_MOTION_SETPOINT_URGENT_DW_DEG_S 10.0f // ① 立即投递的角速度阶跃阈值(deg/s)
#endif

// 电机脉冲参数
#define MOTOR_PULSES_PER_REV 3200 // 电机每转脉冲数(位置命令 0xFD 单位, 由驱动器细分决定)

/**
 * 实时位置回读(S_CPOS / 0x36)的每圈单位数: Emm 固件固定 65536/圈(按电机轴, 与细分无关)。
 * 注意: 位置命令单位与回读单位不同, 两者相差 MOTOR_READ_POS_UNITS_PER_REV/MOTOR_PULSES_PER_REV
 *       = 20.48 倍, 位置模式到位判定必须换算后比较, 不能直接混用。
 *       机械臂模块同样按此约定换算(见 arm_config.h 的 ARM_MOTOR_PULSE_PER_REV/ARM_READ_UNIT_PER_REV)。
 */
#define MOTOR_READ_POS_UNITS_PER_REV 65536

/* ===== 位置模式(轮位置闭环) =====
 * 用于 `car pos` 调试与任务层"视觉标定后的精确入位"：只下发轮子目标脉冲，由电机位置环走完。
 *   - 位置模式是开环轮位置：只保证四个轮子各转过目标脉冲，不保证地面实际位移(打滑会偏小)；
 *   - 到位判定读各轮电机实时位置(S_CPOS 状态轮询，单位 65536/圈)，把目标命令脉冲换算成回读单位后
 *     比较；到达容差内且保持稳定才认为到位；读不到任何位置反馈时按兜底时间返回(只表示已下发)。 */
#define MOTOR_POS_ARRIVE_TOL_READ 150 // 到位容差(回读单位): 150/20.48≈7.3 命令脉冲≈0.05cm 轮位移
#define MOTOR_POS_SETTLE_MS 50 // 到达目标后保持不动的时间(ms), 超过才算停稳到位(约一个状态轮询周期)
#define MOTOR_POS_FALLBACK_DONE_MS 2000 // 读不到电机位置反馈时的兜底"到位"等待(ms), 需覆盖最慢一次位置移动
#define MOTOR_POS_POLL_MS 10 // 等待到位时的轮询间隔(ms), 小于电机状态轮询周期(step_update_time=50ms)的一半

#endif /* __CHASSIS_CONFIG_H__ */
