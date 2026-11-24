/**
 * @file mission_config.h
 * @brief 总任务配置
 *
 * 只放任务流程相关的可调参数，不引入任何模块依赖。
 */

#ifndef __MISSION_CONFIG_H__
#define __MISSION_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* ===== 导航目标点名称（须与 nav_map_data.c 中的点名称一致） ===== */
#define MISSION_TARGET_HOME "HOME" // 出发点
#define MISSION_TARGET_QR "QrCode" // 扫码点
#define MISSION_TARGET_OA1 "OA1" // 避让点1
#define MISSION_TARGET_OA2 "OA2" // 避让点2
#define MISSION_TARGET_MATL_GRAB "MATL_GRAB" // 原料区
#define MISSION_TARGET_MATL_STORAGE "MATL_STORAGE" // 暂存区
#define MISSION_TARGET_MATL_PROCESSING "MATL_PROCESSING" // 粗加工区

/* ===== 任务时序/重试 ===== */
#define MISSION_QR_TIMEOUT_MS 2000 // 二维码识别等待时间(ms)
#define MISSION_RADAR_RETRY_MAX 5 // 雷达扫描最大出错次数

/* ===== 任务分阶段耗时仪表 =====
 * 统计每段导航 / 每段导航前的机械臂准备(ArmPreMove) / 机械臂动作批次 / 二次标定 /
 * 任务码等待 / 雷达 / 固定延时 / 完成显示的累计耗时，任务结束(done/failed/stopped)打印汇总表，
 * 也可随时用 shell `mission time` 查询（实现见 mission.c 的 MissionTiming_* 与 utils/mission_timing.h）。
 * 只做整数累加，不动态分配、不关中断；日志只在"每段结束一行 + 任务结束汇总表"，
 * 不在导航 5ms 控制路径打印，对节拍影响可忽略。 */
#ifndef MISSION_TIMING_ENABLE
#define MISSION_TIMING_ENABLE 1 // 1=启用耗时仪表与汇总表；0=全部空操作(完全排除计时开销)
#endif
#ifndef MISSION_TIMING_LOG_LEG
#define MISSION_TIMING_LOG_LEG 1 // 1=每段导航/每次标定/每批机械臂动作结束时打印一行耗时；0=只打汇总表
#endif
#ifndef MISSION_TIMING_SUMMARY_ENABLE
#define MISSION_TIMING_SUMMARY_ENABLE 1 // 1=任务结束自动打印汇总表；0=只在 `mission time` 查询时打印
#endif
/* 停车耗时估算(ms)：仅用于汇总表把"导航段里的 STOPPING 停车确认次数"折算成时间提示。
 * 该值与导航层当前配置一致(nav_config.h 的 NAV_STOP_SETTLE_MS=150，配合"位姿停稳+距离/航向复核"，
 * 仍在滑移时可能再等最多 NAV_STOP_EXTRA_WAIT_MS)，所以实际停车耗时通常略大于该估算值。
 * 改 nav 停车参数后请同步/用 -D 覆盖本宏。 */
#ifndef MISSION_TIMING_NAV_STOP_EST_MS
#define MISSION_TIMING_NAV_STOP_EST_MS 150
#endif

/* ===== 用 OPS 新鲜度保护任务层的位姿判断 =====
 * 任务层有两处直接用 Loc_Get() 的实测位姿：
 *   - PASS 不停车通过点：按"距通过点距离"判断触发换段；
 *   - 未标定段的到位精修(align)：用当前 yaw + 目标位置做只修位置的精修。
 * OPS 断帧/丢帧时这两处会"距离不再变化"而空等或拿陈旧位姿做目标，虽然导航层自己会先报
 * ERROR 退出，但任务层提前判掉更干净、日志也更能说明问题。
 * 阈值与 nav_config.h 的 NAV_OPS_STALE_TIMEOUT_MS 保持一致(150ms)；OPS 帧率低于该值时按
 * 实际帧率放大(单位 ms)。 */
#ifndef MISSION_NAV_OPS_STALE_TIMEOUT_MS
#define MISSION_NAV_OPS_STALE_TIMEOUT_MS 150
#endif

/* ===== 任务导航容错 =====
 * 规划器依赖实时定位，定位抖动会偶发规划失败或终点精度不达标；任务层对同一
 * 目标点做有限重试，避免一次偶发失败就中止整场任务。 */
#define MISSION_NAV_RETRY_MAX 1 // 单段导航失败后的重试次数（0 = 不重试）
#define MISSION_NAV_RETRY_DELAY_MS 300 // 重试前等待时间(ms)，让定位/车体稳定
#define MISSION_NAV_WAIT_TIMEOUT_MS 15000 // 单次导航等待兜底超时(ms)，须大于导航层自身超时(10s/5s)

/* ===== 子任务超时/失败软处理（不中止整场任务） =====
 * 场景：单段导航超时、树莓派视觉标定超时/失败、机械臂单个物料动作超时/失败、
 *       二维码等待超时、回家导航失败等"小任务"失败时，默认不再让整场任务失败：
 *       记录告警(带步骤名+累计次数)、跳过该小任务，继续执行后面未完成的部分，
 *       最终任务按"完成(带告警)"结束，并在日志里汇总软失败次数。
 * 处置规则：
 *   - 某段导航失败 → 该阶段的机械臂动作跳过（车不在正确位置），后续阶段照常执行；
 *   - 机械臂单个物料动作失败 → 继续做剩下的物料动作；
 *   - 视觉标定失败/超时 → 跳过该点的位置修正，导航与后续动作照常；
 *   - 二维码等待超时 → 用当前任务码（默认值/上一次的码）继续，无效码的物料动作按上面规则跳过。
 * 0 = 恢复旧行为（任一子任务失败立即中止整场任务）。 */
#ifndef MISSION_STEP_SOFT_FAIL
#define MISSION_STEP_SOFT_FAIL 1
#endif

/* 导航前机械臂准备（回初始位+物料盘归位）失败/超时是否也软处理：
 * 1 = 继续导航（不安全：机械臂/物料盘可能未收回，行驶/过弯可能刮碰；确认净空足够再用）；
 * 0 = 旧行为：准备失败即中止整场（默认，安全优先）。 */
#ifndef MISSION_PREMOVE_SOFT_FAIL
#define MISSION_PREMOVE_SOFT_FAIL 0
#endif

/* 放置/动作后的稳定等待(ms)：
 * 机械臂动作内部已做"到位确认 + 放料后抬起撤离"(Arm_GripperSetAngle/arm_action_retract_up 都等到位)，
 * 这里只留一个短稳定时间吸收车体/台面残余振动，不是精度保证环节。
 * 原 400ms 偏保守（每轮 2 次、整场 4 次共 1.6s），改为 200ms（= 2 个 ARM_SETTLE_STABLE_MS）；
 * 若现场出现"放料后物料被碰歪/判断节拍不够"，优先调回 300~400ms。 */
#define MISSION_SETTLE_MS 200

/* ===== 到位精修（只修位置、不转朝向）=====
 * 终点朝向由规划器两段式对齐负责（目标点旋转不安全时规划器会拒绝原地补转）；
 * 任务层精修只在"距离到达"模式下再收一次位置，避免在任务点强制原地转体扫障。 */
#define MISSION_ALIGN_ENABLE 1 // 0 = 关闭到位精修
#define MISSION_ALIGN_DIST_CM 1.0f // 精修位置阈值(cm)
#define MISSION_ALIGN_SPEED_CM_S 60.0f // 精修线速度(cm/s)
#define MISSION_ALIGN_TIMEOUT_MS 5000 // 精修超时(ms)

/* ===== 固定途经点（via）运动参数 ===== */
#define MISSION_VIA_SPEED_CM_S 100.0f // 临时途经点线速度(cm/s)
#define MISSION_VIA_ANGULAR_SPEED_DEG_S 220.0f // 临时途经点角速度(deg/s)
#define MISSION_VIA_ACCEL_CM_S2 110.0f // 临时途经点加速度(cm/s²)
#define MISSION_VIA_DECEL_CM_S2 80.0f // 临时途经点减速度(cm/s²)
#define MISSION_VIA_ARRIVE_DIST_CM 1.0f // 临时途经点位置阈值(cm)
#define MISSION_VIA_ARRIVE_YAW_DEG 2.0f // 临时途经点朝向阈值(deg)
#define MISSION_VIA_TIMEOUT_MS 10000 // 临时途经点导航超时(ms)

/* ===== 不停车通过点（过弯）=====
 * MISSION_NAV_MODE_PASS 段的执行参数：车向通过点直连行驶，距目标 <= 触发距离时
 * 直接切换下一段目标（Nav_Retarget，保留速度状态、不停车），实现"到 OA 点直接拐弯开过去"。
 *
 * 触发距离取值影响过弯速度：直连导航本身会在接近目标时按梯形规划减速，所以触发距离
 * 越小、过点时速度越低、过弯轨迹越贴近折点（拐弯越"紧"）；越大则过弯越顺但越容易切角。
 * 建议先 4cm 实车看拐角轨迹，再按需要 2~6cm 调整。
 *
 * MISSION_PASS_YAW_FAST=1：切换下一段目标后打开导航层 yaw 快速跟随
 * （远距离航向增益 NAV_YAW_GAIN_FAR=0.3 -> NAV_YAW_GAIN_FAST=1.0），
 * 让车在过弯后的下一段上更快把车头转到目标朝向（如 OA2 过弯后 90° -> 0°）；
 * 若实车出现甩尾/打滑，先设 0 或调小 nav_config.h 的 NAV_YAW_GAIN_FAST。 */
#define MISSION_PASS_TRIGGER_CM 4.0f // 距通过点该距离内切换下一段目标(cm)
#define MISSION_PASS_POLL_MS 2 // 等待通过点的轮询周期(ms)
#define MISSION_PASS_WAIT_TIMEOUT_MS 8000 // 等待到达通过点的兜底超时(ms)
#define MISSION_PASS_MISS_MAX_CM 15.0f // 兜底：距通过点最近距离 <= 该值且开始远离时也认为已通过(cm)
#define MISSION_PASS_MISS_BACK_CM 2.0f // 兜底：远离该距离后触发切换(cm)
#define MISSION_PASS_ARRIVE_GUARD_CM 0.3f // PASS 段到达门限守卫(cm)：收小到几乎不可达，防止触发前被导航层判到点停车
#define MISSION_PASS_ARRIVE_GUARD_YAW_DEG 30.0f // PASS 段朝向门限守卫(deg)：过弯朝向由下一段负责，这里放宽
/* 过弯后 yaw 快速跟随开关：1=开启；0=保持默认航向环增益。允许构建系统 -D 覆盖（现场试车用） */
#ifndef MISSION_PASS_YAW_FAST
#define MISSION_PASS_YAW_FAST 1 // 1=过弯后开启 yaw 快速跟随；0=保持默认航向环增益
#endif

/* ===== 调试模式 =====
 * MISSION_DEBUG_MODE = 1：调试模式，不启用树莓派二次校准（无树莓派 / 机械臂未接入 /
 *                       只调导航与流程时使用；导航、路线、重试逻辑不受影响）。
 *                      0：比赛 / 联调模式，按路线表启用二次校准（机械臂预备旋转 + 视觉标定）。
 * 允许构建系统 -D 覆盖，现场切换无需改源码。 */
#ifndef MISSION_DEBUG_MODE
#define MISSION_DEBUG_MODE 0
#endif

/* ===== 树莓派二次标定 =====
 * 到位后调用树莓派视觉校准（圆盘/圆环），拿到相对修正量后用底盘位置模式做一次精确入位
 * （不走 nav 速度闭环，见下方"标定后的精确入位"），
 * 把任务点最终精度从规划器容差收敛到视觉标定精度。
 * 调试模式（MISSION_DEBUG_MODE=1）下强制关闭；正常模式默认启用，
 * 也可构建系统 -DMISSION_CAL_ENABLE=0 单独关闭（台架无树莓派时用）。 */
#if MISSION_DEBUG_MODE
#ifdef MISSION_CAL_ENABLE
#undef MISSION_CAL_ENABLE
#endif
#define MISSION_CAL_ENABLE 1
#else
#ifndef MISSION_CAL_ENABLE
#define MISSION_CAL_ENABLE 1 // 1 = 启用路线表中的树莓派二次标定；0 = 全部关闭
#endif
#endif

/* ===== 视觉 yaw 误差的修正方式 =====
 * 树莓派圆环标定会给出 yaw 误差；原来直接用"车体旋转"(目标 yaw = 当前 yaw + 误差)修正，
 * 麦轮打滑/转角误差使车身修正精度较差。开启本开关后改为:
 *   - 车体不再为这个 yaw 误差旋转(标定后的位置模式精修只走 x/y 位置修正);
 *   - 标定得到的 yaw 误差交给机械臂: 抓取/放置时加到基座旋转角上补偿
 *     (Arm_Action_SetYawCorrection, 补偿方向/限幅见 arm_config.h 的 ARM_YAW_CORRECTION_*)。
 * 关掉(=0)时 yaw 误差也由底盘位置模式随位置一起转车体修正。
 * 圆盘标定(d)无角度字段, 误差恒为 0, 因此只影响 r/r2(粗加工区/暂存区)。 */
#ifndef MISSION_CAL_YAW_FIX_WITH_ARM
#define MISSION_CAL_YAW_FIX_WITH_ARM 0 // 1=用机械臂基座补偿(推荐); 0=由底盘位置模式转车身修正
#endif

/* ===== 标定后的精确入位（底盘位置模式）=====
 * 视觉标定成功后不再走 nav 位置闭环，而是把"视觉修正量"直接交给底盘位置模式：
 *   - RPI_Cal 输出的是相对当前位置的**机体系**修正量: pose.x/pose.y 为位置修正(cm, 含固定偏移),
 *     pose.yaw 为角度修正(deg, 圆盘标定为 0); 方向约定与视觉脚本/RPI_CAL_SIGN_* 一致,
 *     不需要再用定位位姿做世界系换算;
 *   - 任务层直接调用 MotionControl_SetPosition() 下发四个轮子的目标脉冲，并等待
 *     MotionControl_PositionArrived() 到位;
 *   - MISSION_CAL_YAW_FIX_WITH_ARM=1 时 yaw 误差仍交给机械臂基座补偿，本次位置移动只走位置;
 *   - 位置模式是开环轮位置(只保证轮子转过的脉冲)，打滑时地面实际位移会偏小；
 *     相比导航速度闭环更适合"几 cm / 几度"的小修正，长距离移动仍应走导航。
 * 依赖: 该路径要求底盘轮位置模式(MOTOR_POS_MODE=1, zdt_v5_cfg.h)且电机实时位置回读
 *       (MOTOR_STATUS_REAL_POS=1), 否则 MotionControl_SetPosition 直接返回失败/无法确认到位。
 * 位置反馈不可用时底盘层按 MOTOR_POS_FALLBACK_DONE_MS(chassis_config.h)兜底完成。 */
/* 等待位置模式到位的超时(ms), 超时判标定精修失败。依赖车体运动参数(car par): 默认参数下
 * 最慢一次 30cm 级修正约 0.5~0.7s; 若现场把速度/加速度调得很低, 需相应加大。 */
#define MISSION_CAL_POS_TIMEOUT_MS 10000

/* ===== 标定前机械臂预备位姿（按目标区域） =====
 * 发送视觉标定命令前，机械臂需先移动到该区域的预备位姿（基座旋转 / 伸展 / 抬升），
 * 让相机正对标定目标；三个区域的机械臂目标位置不同，按目标区域分开配置：
 *   - 原料区：MISSION_CAL_ARM_GRAB_*
 *   - 粗加工区：MISSION_CAL_ARM_PROCESSING_*
 *   - 暂存区：MISSION_CAL_ARM_STORAGE_*
 * 单位/范围(超出会被机械臂层拒绝, 预备动作直接失败):
 *   - 旋转角度为整度(int，API 按整度下发), 软限位见 arm_config.h 的 ARM_BASE_MIN/MAX_DEG;
 *   - 伸展长度 0~ARM_EXTEND_MAX_MM(125mm): **直接填伸展长度(extend)数值**,
 *     与 `arm pos` 的 extend 读数、arm_config.h 的 ARM_POSE_* 位姿宏口径一致,
 *     不需要再加 ARM_BASE_TO_EXTEND_MM(水平半径 = 偏移 + 伸展长度, 仅内部笛卡尔换算用);
 *   - 抬升高度 0~ARM_LIFT_MAX_MM(170mm), 0=最底部。
 * 占位值必须按实车相机安装实测标定；是否执行预备动作另由 arm_config.h 的 ARM_CAL_PREP_ENABLE 控制。
 * 若填错超出软限位, 默认跳过该点标定并继续任务(见 MISSION_CAL_ARM_PREP_SOFT_FAIL)。 */
#ifndef MISSION_CAL_ARM_PREP_SOFT_FAIL
#define MISSION_CAL_ARM_PREP_SOFT_FAIL 1 // 1=预备位姿超软限位时跳过标定继续任务; 0=判任务失败
#endif
#define MISSION_CAL_ARM_GRAB_BASE_ANGLE_DEG -180 // 原料区：基座目标角度(°)
#define MISSION_CAL_ARM_GRAB_EXTEND_MM 0.0f // 原料区：伸展长度(mm, 0~125, 与 arm pos 的 extend 一致)
#define MISSION_CAL_ARM_GRAB_LIFT_MM 165.0f // 原料区：抬升高度(mm)

#define MISSION_CAL_ARM_PROCESSING_BASE_ANGLE_DEG -180 // 粗加工区：基座目标角度(°)
#define MISSION_CAL_ARM_PROCESSING_EXTEND_MM 30.0f // 粗加工区：伸展长度(mm, 0~125)
#define MISSION_CAL_ARM_PROCESSING_LIFT_MM 5.0f // 粗加工区：抬升高度(mm)

#define MISSION_CAL_ARM_STORAGE_BASE_ANGLE_DEG -180 // 暂存区：基座目标角度(°)
#define MISSION_CAL_ARM_STORAGE_EXTEND_MM 30.0f // 暂存区：伸展长度(mm, 0~125)
#define MISSION_CAL_ARM_STORAGE_LIFT_MM 5.0f // 暂存区：抬升高度(mm)

/* ===== 标定后位置精修的专用运动参数 / 跳过阈值 =====
 * 位置精修通常只有几 cm/几度，若沿用 nav 的车体默认参数(80cm/s、70cm/s²)会"慢慢挪一下"
 * (几厘米要 0.3~0.4s)，在视觉定位完成后看起来像卡顿。这里给这段微调用更快的加速度走完，
 * 命令下发后立即恢复原参数，不影响后续导航。
 *   - 速度上限取 50cm/s: 几厘米的修正主要受加速度限制，速度不必太高，避免打滑；
 *   - 加速度 150cm/s² 约为默认值的 2 倍，3cm 修正的移动时间从 ~0.29s 降到 ~0.2s；
 *   - 修正量小于跳过阈值(位置 2mm / 角度 0.3°)时直接跳过位置模式
 *     (机械臂 yaw 误差补偿仍会设置)，彻底消除该次停顿。
 * 调参建议: 若实车出现"到位后仍有小偏差/打滑"，优先把加速度调回 100~120cm/s²。 */
#define MISSION_CAL_POS_SPEED_CM_S 10.0f
#define MISSION_CAL_POS_YAW_SPEED_DEG_S 100.0f
#define MISSION_CAL_POS_ACC_CM_S2 60.0f
#define MISSION_CAL_POS_DEC_CM_S2 60.0f
#define MISSION_CAL_POS_SKIP_DIST_CM 0.2f // 位置修正小于该值(cm)时跳过位置模式
#define MISSION_CAL_POS_SKIP_YAW_DEG 0.3f // 角度修正小于该值(°)时跳过位置模式

/* ===== 抓取/放置点位校准偏移(mm，机体系，按启动区分两套) =====
 * 每个点位分 X/Y 两个固定偏移(+X = 车头方向/前，+Y = 车体左侧/左，与标定点朝向无关)，
 * 由路线表的 cal_type / cal_offset_x_mm / cal_offset_y_mm 使用：
 *   标定输出的位置修正 = (X 偏移 + 视觉 x 误差, Y 偏移 + 视觉 y 误差)，
 *   作为**相对当前位置的机体系偏移**交给底盘位置模式精修。
 * 两区各一套：启动区1 用 _Z1_ 宏(现场调这一套)，启动区2 的 _Z2_ 宏**默认等于 _Z1_**(两区
 * 场地布局相同)；若二区实测偏移不同，直接把对应的 _Z2_ 宏改成独立数值即可(不改则跟随一区)。
 * 圆环标定(粗加工区/暂存区)额外带视觉朝向误差，开启 MISSION_CAL_YAW_FIX_WITH_ARM 时
 * 该误差交给机械臂基座补偿(车体不转)，否则由底盘位置模式随位置一起转车体；
 * 圆盘标定(原料区)无角度误差。
 * 注意：偏移方向按"车体自身前/左"计，与标定点朝向无关；现场若按旧版(世界系)行为调过符号，
 *       需要在 yaw≠0 的工位(粗加工区 180°/暂存区 90°)重新确认方向。
 * 各点位的标定类型在 mission_route.c 路线表中配置：
 *   原料区 = MISSION_CAL_DISK；第一二轮粗加工区 = MISSION_CAL_RING_FIRST；
 *   暂存区（两轮）= MISSION_CAL_STORAGE（树莓派暂存区专用命令 0xE1，应答格式同圆环、带朝向误差）。 */

/* 启动区1 的标定固定偏移(现场主要调这三组) */
#define MISSION_POINT_GRAB_OFFSET_Z1_X 0.0f // 原料区：机体系 X(前)方向校准偏移(mm)
#define MISSION_POINT_GRAB_OFFSET_Z1_Y 60.0f // 原料区：机体系 Y(左)方向校准偏移(mm)
#define MISSION_POINT_PROCESSING_OFFSET_Z1_X 0.0f // 粗加工区：机体系 X(前)方向校准偏移(mm)
#define MISSION_POINT_PROCESSING_OFFSET_Z1_Y -20.0f // 粗加工区：机体系 Y(左)方向校准偏移(mm)
#define MISSION_POINT_STORAGE_OFFSET_Z1_X -0.0f // 暂存区：机体系 X(前)方向校准偏移(mm)
#define MISSION_POINT_STORAGE_OFFSET_Z1_Y -25.0f // 暂存区：机体系 Y(左)方向校准偏移(mm)

/* 启动区2 的标定固定偏移：默认填一区的值；二区要单独标定时改成独立数值 */
#define MISSION_POINT_GRAB_OFFSET_Z2_X MISSION_POINT_GRAB_OFFSET_Z1_X
#define MISSION_POINT_GRAB_OFFSET_Z2_Y MISSION_POINT_GRAB_OFFSET_Z1_Y
#define MISSION_POINT_PROCESSING_OFFSET_Z2_X 10.0f
#define MISSION_POINT_PROCESSING_OFFSET_Z2_Y -35.0f // 粗加工区：机体系 Y(左)方向校准偏移(mm)
#define MISSION_POINT_STORAGE_OFFSET_Z2_X 0.0f // 暂存区：机体系 X(前)方向校准偏移(mm)
#define MISSION_POINT_STORAGE_OFFSET_Z2_Y -30.0f

/* ===== 机械臂任务动作 =====
 * MISSION_ARM_ENABLE = 1：启用原料抓取上盘 / 物料盘取放（完整任务流程）。
 *                     = 0：只跑导航 + 标定流程，跳过机械臂动作（台架 / 未接机械臂时用）。
 * 默认跟随调试模式：MISSION_DEBUG_MODE=1（调试）→ 关闭；比赛/联调模式 → 开启。
 * 允许构建系统 -D 覆盖；注意机械臂目标位姿必须在 arm_action.c 标定后再开启。 */
#ifndef MISSION_ARM_ENABLE
#if MISSION_DEBUG_MODE
#define MISSION_ARM_ENABLE 1
#else
#define MISSION_ARM_ENABLE 1
#endif
#endif

/* ===== 运动前物料盘归位 =====
 * 每段导航（小车要移动）出发前，先把物料盘换到基准格 ARM_PAD_SLOT_BASE（2 号）：
 *   - 物料盘换格伺服在非基准格时会向车体两侧伸出，带着它行驶/过弯容易刮碰；
 *   - 同时给后续取放料的换格一个已知初始位置（避免上电/上次任务遗留位置未知）。
 * 1 = 开启（默认）；0 = 跳过。换格失败按任务失败处理（不带着非基准盘位起步）。
 * 注意：仅在 MISSION_ARM_ENABLE=1 时执行（机械臂关闭的台架流程不做）。 */
#ifndef MISSION_PAD_PREPOSE_ENABLE
#define MISSION_PAD_PREPOSE_ENABLE 1
#endif

/* ===== 物料盘归位期间车体预旋转 =====
 * 每段导航出发前都要先做"机械臂回初始状态 + 物料盘归位"(Mission_ArmPreMove，约 1s+)：
 *   - 路线首段是原地旋转段(MISSION_NAV_MODE_ROTATE)时，这段转体本来也要做，于是把它提前到
 *     物料盘换格期间并行执行（机械臂回初始位的收臂动作执行完才启动），
 *     归位完成时旋转已基本转好，路线首段不再重复旋转；
 *   - 只对路线**首段的 ROTATE 段**生效，不改变任何平移路径；首段不是旋转时行为与原来完全一致；
 *   - 预旋转未按时到位时只告警，回退为按原路线执行（首段旋转再由路线自己完成），不影响任务正确性；
 *   - 仅在 MISSION_ARM_ENABLE=1 且 MISSION_PAD_PREPOSE_ENABLE=1 时生效（有归位等待才有可重叠的时间）。
 * 1 = 开启（默认）；0 = 关闭（恢复"先归位、再按路线旋转"）。
 * 注意：转体期间物料盘可能还在往基准格移动（物料盘在非基准格会向车体两侧伸出），
 *       只有在现场确认该站位原地转体清障足够、不会刮碰时才开启；某站位转体不安全时，
 *       把该路线首段的 ROTATE 段删掉，或把本开关按构建系统 -D 置 0（整场关闭预旋转）。 */
#ifndef MISSION_PAD_PREPOSE_ROTATE_ENABLE
#define MISSION_PAD_PREPOSE_ROTATE_ENABLE 1
#endif

/* ===== 每轮物料数量（物料盘格位 1 ~ MISSION_MATERIAL_NUM） ===== */
#define MISSION_MATERIAL_NUM 3

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __MISSION_CONFIG_H__ */
