/**
 * @file arm_ctrl.h
 * @author T-Car Team
 * @brief 机械臂控制核心: 电机命令封装、固件回零、单位换算、到位检测
 *
 * 本层只负责"单轴/电机"级动作, 不做坐标解算(见 arm_kinematics.c),
 * 也不做多轴联动(见 arm_motion.c)。
 *
 * @note 需在 FreeRTOS 任务上下文中调用(osDelay/互斥锁)。
 * 可调参数见 arm_config.h / Applications/arm。
 */

#ifndef __ARM_CTRL_H__
#define __ARM_CTRL_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <stdbool.h>
#include <stdint.h>

/*============================ 初始化与回零 ============================*/

/**
 * @brief 初始化机械臂控制模块并发起整臂回零
 * @note 非阻塞: 只创建/唤醒后台回零任务后立即返回, 用 Arm_HomeWaitDone 等待完成;
 *       回零在独立低优先级任务中执行(整臂约需数十秒), 顺序为 升降 -> 伸展 -> 基座旋转;
 *       回零成功后自动移动到机械臂初始状态位(行驶/待机位, 见 ARM_POSE_INIT_*)。
 */
void Arm_CtrlInit(void);

/**
 * @brief 等待整臂回零完成(阻塞)
 * @param timeout_ms 最长等待时间 (ms)
 * @return true=回零已完成或未发起回零; false=回零失败或等待超时
 */
bool Arm_HomeWaitDone(uint32_t timeout_ms);

/**
 * @brief 查询本上电周期内整臂回零是否已完成
 * @return true=回零已完成; false=从未发起/正在回零/回零失败
 * @note 仅用于调试工具提示"先回零再动作"; 手动 `arm zero` 设定零点时可能为 false,
 *       不代表绝对位置不可用, 由调用方决定是否继续。
 */
bool Arm_IsHomeDone(void);

/**
 * @brief 指定单轴执行固件回零(阻塞到完成/失败)
 * @param axis 轴序号(0 ~ Arm_AxisCount()-1, 顺序: 基座/升降/伸展; 名称/ID 见 Arm_AxisName/Arm_AxisMotorId)
 * @return true=回零完成; false=轴序号非法/整臂回零进行中/电机未注册/命令失败/回零失败/超时
 * @note 与整臂异步回零(Arm_CtrlInit)互斥: 整臂回零进行中时拒绝执行;
 *       回零会改变该轴位置基准, 成功后清掉该轴已记录的目标, 其它轴不受影响;
 *       本接口在调用任务上下文阻塞(单轴最长 ARM_HOME_TIMEOUT_MS + 余量), 供 shell/调试单轴重新寻找零位。
 */
bool Arm_HomeAxis(uint8_t axis);

/*============================ 电机使能 ============================*/

/**
 * @brief 使能/失能机械臂全部电机(三轴), 只下发使能控制命令, 不做回零/运动
 * @param enable true=使能(锁轴), false=失能(松轴)
 * @return true=三轴命令均下发成功; false=存在下发失败(具体电机见日志)
 * @note 失能后电机松轴, 机械臂可能因重力下坠, 调用前先确保臂在安全位置或有人扶;
 *       使能后绝对位置仍需回零后可信(见 Arm_CtrlInit / Arm_HomeWaitDone)。
 */
bool Arm_MotorsEnable(bool enable);

/*============================ 零点设置 ============================*/

/**
 * @brief 以机械臂当前位置为各轴零位(只下发位置清零命令, 不做回零/运动)
 * @param timeout_ms 清零前等待三轴静止的最长时间(ms); 机械臂仍在运动/电机离线时拒绝执行
 * @return true=三轴清零命令均下发成功; false=未静止/离线或存在下发失败(具体电机见日志)
 * @note 只建议调试/标定时使用: 会把当前绝对位置定义为各轴 0 点, 后续绝对目标与软限位都相对新零点;
 *       升降轴的逻辑高度仍按"顶部回零偏移(home_offset_mm)"换算, 不随清零改变,
 *       因此升降轴应在顶部机械零位处执行本命令(清零后 lift 仍是 ARM_LIFT_TOP_HOME_OFFSET_MM,
 *       等效于顶部回零); 若在中间位置清零, 逻辑高度会整体偏掉一个偏移量。
 *       清零后可用 `arm pos` 读回确认。
 */
bool Arm_SetCurrentPosAsZero(uint32_t timeout_ms);

/*============================ 轴信息(调试/遍历用) ============================*/

/**
 * @brief 获取机械臂轴数量(与 arm_config.h 的 ARM_MOTOR_NUM 一致)
 */
uint8_t Arm_AxisCount(void);

/**
 * @brief 按轴下标获取电机总线 ID
 * @param axis 轴下标(0 ~ Arm_AxisCount()-1)
 * @return 电机总线 ID; 下标越界时返回 0
 */
uint8_t Arm_AxisMotorId(uint8_t axis);

/**
 * @brief 按轴下标获取轴名(日志/调试用)
 * @param axis 轴下标(0 ~ Arm_AxisCount()-1)
 * @return 轴名字符串; 下标越界时返回 "?"
 */
const char *Arm_AxisName(uint8_t axis);

/*============================ 单轴运动接口 ============================*/

/**
 * @brief 基座旋转到绝对角度(相对机械零位)
 * @param angle_deg 目标角度(°), 超出软限位时自动钳到限位值继续执行(告警)
 * @param vel       速度 (RPM)
 * @param acc       加速度 (Emm 固件为档位 0-255)
 * @return 是否发送成功
 */
bool Arm_BaseRotateAbs(int32_t angle_deg, uint16_t vel, uint16_t acc);

/**
 * @brief 同步带伸展臂伸长到指定长度(绝对位置)
 * @param length_mm 目标伸长量 (mm), 0=完全收回(机械零位)
 * @param vel       速度 (RPM)
 * @param acc       加速度 (Emm 固件为档位 0-255)
 * @return 是否发送成功
 */
bool Arm_ExtendTo(float length_mm, uint16_t vel, uint16_t acc);

/**
 * @brief 升降臂抬升到距底部指定高度(绝对位置)
 * @param lift_mm 抬升高度 (mm), 0=最底部, 数值越大越靠上(逻辑坐标始终"从底部计")
 * @param vel     速度 (RPM)
 * @param acc     加速度 (Emm 固件为档位 0-255)
 * @return 是否发送成功
 * @note 机械回零在顶部顶点(ARM_LIFT_HOME_DIR=0), 内部按 ARM_LIFT_TOP_HOME_OFFSET_MM
 *       换算成相对顶部回零点的位置下发; 对外接口/位姿参数语义不变(从底部计的抬升高度)。
 */
bool Arm_LiftTo(float lift_mm, uint16_t vel, uint16_t acc);

/**
 * @brief 读取指定电机当前角度(线性轴按逻辑零点折算, 即升降从底部、伸展从收回端计)
 * @param motor_id  电机总线 ID (机械臂为 5~7)
 * @param angle_deg 输出: 当前角度(°)
 * @return true 读取成功, false 电机未注册/离线/位置不可用
 * @note 角度按各轴回读方向换算为"物理方向为正"(伸展轴伸出时为正, 与 arm pos 的伸展长度一致);
 *       线性轴再加回零偏移角, 使升降轴角度仍是从底部计的等效角度(顶部回零被抵消);
 *       具体物理长度请用 Arm_LiftGetHeight / Arm_ExtendGetLength 读取。
 */
bool Arm_MotorGetAngle(uint8_t motor_id, float *angle_deg);

/**
 * @brief 读取同步带当前伸展长度
 * @param length_mm 输出: 当前伸长量 (mm), 0=完全收回(机械零位)
 * @return true 读取成功, false 电机未注册/离线/位置不可用
 * @note 返回 false 时输出参数内容未定义。
 */
bool Arm_ExtendGetLength(float *length_mm);

/**
 * @brief 读取升降臂当前抬升高度
 * @param height_mm 输出: 当前抬升高度 (mm), 0=最底部, 顶部=ARM_LIFT_TOP_HOME_OFFSET_MM
 * @return true 读取成功, false 电机未注册/离线/位置不可用
 * @note 机械零位在顶部, 本接口已换算成"从底部计"的抬升高度, 原标定值继续可用;
 *       返回 false 时输出参数内容未定义。
 */
bool Arm_LiftGetHeight(float *height_mm);

/**
 * @brief 等待机械臂三轴运动到位
 * @param timeout_ms 最长等待时间 (ms)
 * @return true=位置已稳定且已下发目标的轴到达目标; false=超时/未到目标/存在电机离线
 * @note 位置命令为异步下发, 应在夹爪等动作前调用, 避免在未到位时开合。
 *       回读来自状态轮询缓存(约 100ms 一次), 判定同时要求"位置稳定"与"到目标"。
 */
bool Arm_WaitSettled(uint32_t timeout_ms);

/**
 * @brief 等待水平方向轴(基座旋转 + 同步带伸缩)到位
 * @param timeout_ms 最长等待时间 (ms)
 * @return true=水平两轴已稳定且到达各自目标; false=超时/未到目标/电机离线
 * @note 用于分步运动: 升降轴不参与判定, 水平两轴到位后再下发升降命令,
 *       避免带高度斜向进入工位。
 */
bool Arm_WaitSettledHorizontal(uint32_t timeout_ms);

/**
 * @brief 只等待升降轴到位
 * @param timeout_ms 最长等待时间 (ms)
 * @return true=升降轴已稳定且到达目标; false=超时/未到目标/电机离线
 * @note 用于"上升动作先升到位, 再旋转+伸缩"的分步运动: 水平两轴不参与判定。
 */
bool Arm_WaitSettledLift(uint32_t timeout_ms);

/*============================ 夹爪(舵机)接口 ============================*/

/**
 * @brief 设置夹爪角度(阻塞到角度回读确认到位)
 * @param angle_deg 目标角度(°)
 * @return true=已到位或已夹稳/被挡停; false=命令下发失败/到位等待超时
 * @note 下发单圈角度命令后轮询舵机角度回读: 到达目标角度(容差 ARM_GRIPPER_SETTLE_TOL_DEG)
 *       或"已开始运动且角度不再变化"(夹住物料)都算到位, 超时 ARM_GRIPPER_SETTLE_TIMEOUT_MS 判失败;
 *       舵机不支持角度回读时回退 ARM_GRIPPER_SETTLE_MS 固定等待。
 */
bool Arm_GripperSetAngle(int angle_deg);

/**
 * @brief 只下发夹爪角度命令(不阻塞到位), 与 Arm_GripperWaitArrive 配对实现"边张/合爪边动臂"
 * @param angle_deg 目标角度(°)
 * @return true=命令已下发; false=命令下发失败
 * @note 用于不需要立即确认夹爪到位的场景(如原料区张爪同时移臂、标定张爪同时翻相机),
 *       之后在需要确认的位置调用 Arm_GripperWaitArrive() 收尾, 可省掉一次串行等待。
 */
bool Arm_GripperStartAngle(int angle_deg);

/**
 * @brief 等待最近一次 Arm_GripperStartAngle 的目标角度到位
 * @return true=已到位或已夹稳/被挡停; false=未调用过 Start/到位等待超时
 */
bool Arm_GripperWaitArrive(void);

/*============================ 物料盘(总线舵机)接口 ============================*/

/**
 * @brief 启动物料盘换格(非阻塞): 只下发角度命令, 到位由 Arm_PadWaitArrive 校验
 * @param slot 格位号(1 ~ ARM_PAD_SLOT_NUM), 越界时钳制到有效范围
 * @return true=换格命令已下发; false=命令下发失败
 * @note 物料盘转动较慢(相邻格 270°), 需要"边转边动臂"时用本接口先启动换格,
 *       让机械臂同时执行后续动作, 最后在下降取放前调用 Arm_PadWaitArrive 校验到位。
 */
bool Arm_PadSetSlotStart(uint8_t slot);

/**
 * @brief 校验物料盘是否已到达最近一次 Arm_PadSetSlotStart 指定的格位(阻塞到到位)
 * @param timeout_ms 最长等待时间(ms)
 * @return true=到位(或回读不可用已回退固定等待); false=未调用 Arm_PadSetSlotStart/等待超时
 */
bool Arm_PadWaitArrive(uint32_t timeout_ms);

/**
 * @brief 物料盘换格: 把指定格位移动到夹爪固定的取放位置
 * @param slot 格位号(1 ~ ARM_PAD_SLOT_NUM), 越界时钳制到有效范围
 * @return true=舵机已到目标格位; false=换格命令下发失败/到位等待超时
 * @note 调用前应保证夹爪已抬离物料盘(在安全高度); 本函数阻塞等待**角度回读确认到位**
 *       (容差 ARM_PAD_SETTLE_TOL_DEG / 超时 ARM_PAD_SETTLE_TIMEOUT_MS), 不再是固定延时;
 *       等价于 Arm_PadSetSlotStart + Arm_PadWaitArrive; 舵机不支持角度回读时自动回退
 *       ARM_PAD_SETTLE_MS 固定等待。
 */
bool Arm_PadSetSlot(uint8_t slot);

/*============================ 相机翻转舵机接口 ============================*/

/**
 * @brief 设置相机翻转舵机角度
 * @param angle_deg 目标角度(°), 单圈角度
 * @note 标定/颜色识别时翻到 ARM_FLIP_CAL_ANGLE_DEG(标定结束保持该角, 不自动恢复);
 *       整臂回零/伸展轴回零前恢复到 ARM_FLIP_RESTORE_ANGLE_DEG, 调试用 `arm flip` 命令。
 *       内部阻塞等待 ARM_FLIP_SETTLE_MS, ARM_FLIP_ENABLE=0 时为空操作。
 */
void Arm_FlipSetAngle(float angle_deg);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __ARM_CTRL_H__ */
