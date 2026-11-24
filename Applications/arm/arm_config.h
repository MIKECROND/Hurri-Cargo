/**
 * @file arm_config.h
 * @brief 机械臂应用参数配置(机械臂电机/换算/限位/回零/动作位姿参数唯一来源)
 *
 * 分层: arm_action -> arm_motion -> arm_ctrl -> zdt_v5 步进电机驱动 / servo 舵机驱动
 *
 * 说明:
 *   - 机械臂使用 3 台张大头 V5 步进电机(总线 ID 5~7) + 夹爪舵机 + 物料盘舵机 + 相机翻转舵机;
 *   - 步进电机由 zdt_v5_port.c 按 ID 1..ZDT_STEP_NUM 统一注册,
 *     其中机械臂的电机数量由本文件的 ARM_MOTOR_NUM 决定(见 zdt_v5_cfg.h);
 *   - 各轴的速度/加速度、回零参数、行程软限位、换算系数统一在本文件配置。
 *
 * 修改指引:
 *   - 电机 ID、减速比、行程限位、回零方向、速度: 本文件
 *   - 动作目标位姿(原料区/物料盘/暂存区/粗加工区): 本文件 "动作目标位姿(待标定)"
 *   - 动作内部偏移量(抓取抬升/盘面下降/格位间距/叠层高度): arm_action.c 顶部
 *   - 夹爪张开/闭合角度: 本文件
 *   - 相机翻转舵机 ID/标定角度/复位角度: 本文件
 */

#ifndef __ARM_CONFIG_H__
#define __ARM_CONFIG_H__

#include <stdint.h>

/*============================ 模块总开关 ============================*/
/**
 * 上电初始化(Sys_Init_Task)时是否自动发起整臂回零:
 *   0 = 不自动回零, 由 shell 命令 `arm init` 手动触发(调试/未接机械臂时使用)
 *   1 = 上电后在后台低优先级任务中自动回零
 */
#define ARM_AUTO_HOME_ON_INIT 0

/*============================ 电机与轴定义 ============================*/
#define ARM_MOTOR_NUM 3 // 机械臂步进电机(轴)数量

// 各轴总线 ID(arm_ctrl.c 的轴表 s_axis[] 按 "基座/升降/伸展" 顺序引用这些 ID)
#define ARM_BASE_MOTOR_ID 5 // 基座旋转轴
#define ARM_LIFT_MOTOR_ID 6 // 升降轴
#define ARM_EXTEND_MOTOR_ID 7 // 同步带伸展轴(命令/回读方向见 arm_ctrl.c 轴表 linear_sign/read_sign)

/*============================ 回零(电机固件内置) ============================*/
/**
 * 回零方式: 统一使用 3 = 多圈有限位开关回零。
 * 应用层先把回零模式/方向/速度/超时写入电机并存储, 再触发固件回零;
 * 电机朝限位开关方向运动, 触发后由固件把该点作为机械零位。
 */
#define ARM_HOME_MODE 3

// 各轴回零方向: 0=CW, 1=CCW, 方向须使各轴朝自己的限位开关端运动
#define ARM_BASE_HOME_DIR 0 // 基座回零方向

#define ARM_LIFT_HOME_DIR 0 // 升降轴向上顶到顶部限位, 顶部即机械零位(逻辑高度仍从底部计)
#define ARM_EXTEND_HOME_DIR 0 // 伸展轴回零方向(朝完全收回端, 0=完全收回)

#define ARM_HOME_VEL_RPM 60 // 回零速度 (RPM), 低速顶限位
#define ARM_HOME_TIMEOUT_MS 8000 // 固件回零超时 (ms), 超时电机自动退出回零
#define ARM_HOME_WAIT_MARGIN_MS 2000 // 应用层等待余量 (ms), 超过 固件超时+余量 判失败
#define ARM_HOME_POLL_MS 20 // 回零状态轮询间隔 (ms)

// 整臂回零串行执行约需数十秒, 放在独立低优先级任务中异步执行
// 回零顺序: 升降(先抬离台面) -> 伸展(收短) -> 基座旋转(最后转, 扫掠最小); 顺序表见 arm_ctrl.c s_home_order
#define ARM_HOME_TASK_STACK 1024 // 整臂回零任务栈大小 (字节)
#define ARM_HOME_STEP_DELAY_MS 200 // 相邻电机回零间隔 (ms), 留出机械动作时间避免干涉
#define ARM_HOME_IDLE_POLL_MS 100 // 等待回零完成的轮询间隔 (ms)
#define ARM_HOME_READY_TIMEOUT_MS 40000 // 运动前等待"回零就绪"的最长时间 (ms)

// 无限位碰撞回零检测参数(仅模式 2 使用, 模式 3 忽略; 写回零参数时需一并下发)
#define ARM_HOME_COLLISION_VEL_RPM 300 // 碰撞检测转速 (RPM)
#define ARM_HOME_COLLISION_CURRENT 800 // 碰撞检测电流 (mA)
#define ARM_HOME_COLLISION_TIME_MS 60 // 碰撞检测时间 (ms)

/*============================ 换算参数 ============================*/
/**
 * 位置命令(0xFD 等)与读取位置(0x36 等)的"每圈"单位不同:
 *   - 位置命令: 脉冲/圈 = 电机每转脉冲数(由细分决定) × 减速比
 *   - 读取位置: Emm 固件固定 65536/圈(按电机轴, 与细分无关) × 减速比
 * 两者相差 65536/3200 = 20.48 倍, 混用会让读出的角度/长度整体放大 20.48 倍。
 */
#define ARM_MOTOR_PULSE_PER_REV 3200 // 电机每转脉冲数(由驱动器细分决定, 默认 16 细分=3200)
#define ARM_READ_UNIT_PER_REV 65536 // Emm 固件读取位置每转单位数(按电机轴)

// 各轴减速比(输出轴转 1 圈 = 电机转 ratio 圈; 直连填 1.0f), 需按实际机械结构标定
#define ARM_BASE_GEAR_RATIO 2.5f // 基座旋转
#define ARM_LIFT_GEAR_RATIO 1.0f // 升降
#define ARM_EXTEND_GEAR_RATIO 1.0f // 同步带伸展

// 直线轴每转对应的直线移动距离 (mm), 与带轮节圆/丝杆导程有关, 需按实际标定
#define ARM_EXTEND_MM_PER_REV 46.0f // 伸展同步带每转移动距离 (mm)
#define ARM_LIFT_MM_PER_REV 60.0f // 升降机构每转移动距离 (mm)

/*============================ 各轴默认速度 ============================*/
/**
 * 各轴运动速度/加速度的**默认值/基准**:
 *   每个动作还可通过下面"各动作速度/加速度(解耦)"的 ARM_PROFILE_xxx_INIT 单独覆盖;
 *   这里主要作为默认 profile 以及各动作 profile 初值的来源。
 *   ARM_*_VEL: 速度, 单位 RPM(电机轴转速), 越大越快;
 *   ARM_*_ACC: 加速度, Emm 固件为"档位 0~255", 越大加速越快(实际 RPM/s ≈ 20000/(256-档位),
 *              例如 220≈555RPM/s、180≈263RPM/s); **不要填 0**(0=无加减速, 直接全速启停,
 *              同步带/丝杆冲击大, 容易过冲/回弹/偶发不到位);
 *   ARM_HOME_VEL_RPM: 回零速度单独控制, 顶限位要慢;
 *   到位判定参数(ARM_SETTLE_STABLE_MS 等)会叠加在每次动作里, 也影响"看起来快不快", 见"到位等待"区。
 *
 * 调参指引:
 *   1) 想更快: 先适当加大 ACC(加速更跟得上指令、到位更干脆), 再逐步加大 VEL;
 *   2) 出现过冲/回弹/丢步/异响: 优先调小 VEL 和 ACC(同步带伸展轴最敏感);
 *   3) 三轴可独立调; 升降/伸展行程短、速度值别一次加太多, 每次改 10%~20% 实车验证;
 *   4) 加减速档位换算见上(如 220→约555RPM/s), 档位越靠近 255 加速越猛, 谨慎使用。
 */
#define ARM_BASE_VEL 200
#define ARM_BASE_ACC 190
#define ARM_LIFT_VEL 650
#define ARM_LIFT_ACC 254
#define ARM_EXTEND_VEL 520
#define ARM_EXTEND_ACC 254

/*============================ 各动作速度/加速度 ============================*/
/**
 * 每个对外机械臂动作可**单独**配置三轴的速度(RPM)与加速度档位(Emm 0~255), 互不影响:
 *   - 动作层在每个动作开始时调用 Arm_SetMotionProfile() 选用下表对应 profile,
 *     该动作内部的所有移动(抓取/放置/盘面升降/标定预备/回初始位等)都按这个 profile 执行;
 *   - 调试命令(`arm pol`/`arm preset`/`arm ctrl` 等)未设置动作 profile 时用默认 profile;
 *   - 初值全部引用"各轴默认速度"的当前值(行为与改前完全一致), 之后可按动作单独改数值;
 *   - Emm 速度模式里"加速/减速"共用同一个档位参数, 因此每个动作只解耦 速度 + 加减速档位;
 *     若将来换 X 固件/位置模式需要单独减速度, 再在这里加 dec 字段。
 *
 * 用法: 想单独调某个动作, 直接把该动作宏里对应项改成具体数值即可, 例如
 *   ARM_PROFILE_LOAD_PAD_INIT 的 .lift_vel 改成 500 → 只有"原料抓取上盘"动作的升降变慢,
 *   其它动作不受影响。改完重新编译烧录生效。
 */
typedef struct {
    uint16_t base_vel; // 基座旋转速度 (RPM)
    uint16_t base_acc; // 基座加减速档位 (Emm 0~255)
    uint16_t lift_vel; // 升降速度 (RPM)
    uint16_t lift_acc; // 升降加减速档位
    uint16_t extend_vel; // 同步带伸展速度 (RPM)
    uint16_t extend_acc; // 伸展加减速档位
} ArmMotionProfile_t;

/* 默认 profile: 调试命令/未指定动作时使用, 等于各轴默认速度 */
#define ARM_PROFILE_DEFAULT_INIT                                                                                       \
    {.base_vel = ARM_BASE_VEL,                                                                                         \
     .base_acc = ARM_BASE_ACC,                                                                                         \
     .lift_vel = ARM_LIFT_VEL,                                                                                         \
     .lift_acc = ARM_LIFT_ACC,                                                                                         \
     .extend_vel = ARM_EXTEND_VEL,                                                                                     \
     .extend_acc = ARM_EXTEND_ACC}

/* 原料区抓取上物料盘: 原材料抓取 + 物料盘放料 */
#define ARM_PROFILE_LOAD_PAD_INIT                                                                                      \
    {.base_vel = ARM_BASE_VEL,                                                                                         \
     .base_acc = ARM_BASE_ACC,                                                                                         \
     .lift_vel = ARM_LIFT_VEL,                                                                                         \
     .lift_acc = ARM_LIFT_ACC,                                                                                         \
     .extend_vel = ARM_EXTEND_VEL,                                                                                     \
     .extend_acc = ARM_EXTEND_ACC}

/* 物料盘取料放到粗加工区 */
#define ARM_PROFILE_UNLOAD_PROCESS_INIT                                                                                \
    {.base_vel = ARM_BASE_VEL,                                                                                         \
     .base_acc = 150,                                                                                                  \
     .lift_vel = ARM_LIFT_VEL,                                                                                         \
     .lift_acc = ARM_LIFT_ACC,                                                                                         \
     .extend_vel = ARM_EXTEND_VEL,                                                                                     \
     .extend_acc = ARM_EXTEND_ACC}

/* 物料盘取料放到暂存区 */
#define ARM_PROFILE_UNLOAD_STORAGE_INIT                                                                                \
    {.base_vel = ARM_BASE_VEL,                                                                                         \
     .base_acc = 150,                                                                                                  \
     .lift_vel = ARM_LIFT_VEL,                                                                                         \
     .lift_acc = ARM_LIFT_ACC,                                                                                         \
     .extend_vel = ARM_EXTEND_VEL,                                                                                     \
     .extend_acc = ARM_EXTEND_ACC}

/* 粗加工区取料回物料盘 */
#define ARM_PROFILE_LOAD_FROM_PROCESS_INIT                                                                             \
    {.base_vel = ARM_BASE_VEL,                                                                                         \
     .base_acc = 150,                                                                                                  \
     .lift_vel = ARM_LIFT_VEL,                                                                                         \
     .lift_acc = ARM_LIFT_ACC,                                                                                         \
     .extend_vel = ARM_EXTEND_VEL,                                                                                     \
     .extend_acc = ARM_EXTEND_ACC}

/* 暂存区取料回物料盘 */
#define ARM_PROFILE_LOAD_FROM_STORAGE_INIT                                                                             \
    {.base_vel = ARM_BASE_VEL,                                                                                         \
     .base_acc = 150,                                                                                                  \
     .lift_vel = ARM_LIFT_VEL,                                                                                         \
     .lift_acc = ARM_LIFT_ACC,                                                                                         \
     .extend_vel = ARM_EXTEND_VEL,                                                                                     \
     .extend_acc = ARM_EXTEND_ACC}

/* 回到机械臂初始状态位(行驶前/回零后) */
#define ARM_PROFILE_INIT_POSE_INIT                                                                                     \
    {.base_vel = ARM_BASE_VEL,                                                                                         \
     .base_acc = ARM_BASE_ACC,                                                                                         \
     .lift_vel = ARM_LIFT_VEL,                                                                                         \
     .lift_acc = ARM_LIFT_ACC,                                                                                         \
     .extend_vel = ARM_EXTEND_VEL,                                                                                     \
     .extend_acc = ARM_EXTEND_ACC}

/* 视觉标定预备位姿 */
#define ARM_PROFILE_CAL_PREP_INIT                                                                                      \
    {.base_vel = ARM_BASE_VEL,                                                                                         \
     .base_acc = ARM_BASE_ACC,                                                                                         \
     .lift_vel = ARM_LIFT_VEL,                                                                                         \
     .lift_acc = ARM_LIFT_ACC,                                                                                         \
     .extend_vel = ARM_EXTEND_VEL,                                                                                     \
     .extend_acc = ARM_EXTEND_ACC}

/*============================ 键盘点动(arm key) ============================*/
/**
 * `arm key` 键盘点动控制: W/S=伸展伸长/收回, A/D=基座左转/右转, Q/E=升降/下降,
 * Z/X=夹爪张开/闭合, ^C=退出。
 * 每次按键下发一次"当前目标 + 步长"的单轴绝对位置命令; 终端长按会自动重复按键,
 * 最小间隔 ARM_KEY_REPEAT_MS 防止重复过快; 软限位由单轴接口检查(超限只提示不运动)。
 */
#define ARM_KEY_EXTEND_STEP_MM 10.0f // 每次按键的伸展步长 (mm)
#define ARM_KEY_ANGLE_STEP_DEG 5.0f // 每次按键的基座旋转步长 (°, 内部按整度累计下发)
#define ARM_KEY_LIFT_STEP_MM 10.0f // 每次按键的升降步长 (mm)
#define ARM_KEY_REPEAT_MS 100 // 连续按键的最小间隔 (ms)

/*============================ 行程软限位 ============================*/
// 目标超限时**自动钳到限位值继续执行**(告警, 不再拒绝): 现场策略"超出可达范围按最大值走";
// 若希望某方向绝对不允许越过, 把对应 MIN/MAX 改成实测机械极限即可(目标会被钳到该值)。
#define ARM_BASE_MIN_DEG -360 // 基座角度下限 (°), 相对机械零位
#define ARM_BASE_MAX_DEG 360 // 基座角度上限 (°)
#define ARM_EXTEND_MIN_MM 0.0f // 伸展最小伸长量 (mm), 0=完全收回
#define ARM_EXTEND_MAX_MM 125.0f // 伸展最大伸长量 (mm)
#define ARM_LIFT_MIN_MM 0.0f // 抬升高度下限 (mm), 0=最底部(逻辑零点; 回零在顶部, 见下方偏移)
#define ARM_LIFT_MAX_MM 170.0f // 抬升高度上限 (mm, 自底部向上), 按实际机械行程标定

/**
 * 升降轴顶部回零点的逻辑高度 (mm) —— 把"顶部机械零位"换算回"从底部计"的坐标:
 *   回零(机械零位)在顶部限位, 但对外数值(arm pos 的 lift / 位姿里的 lift_mm)仍然从底部计:
 *       逻辑抬升高度 = 偏移 - 顶部向下走的距离
 *   偏移 = 底部端到顶部回零限位的总行程; 默认取 ARM_LIFT_MAX_MM。
 *   若顶部限位与 110mm 上限不完全重合, 用卷尺量"底部端 -> 顶部回零触发点"的实际行程后改这里;
 *   **不要改位姿参数**: 参数一直是从底部计的抬升高度, 原有标定值继续有效。
 */
#define ARM_LIFT_TOP_HOME_OFFSET_MM ARM_LIFT_MAX_MM

/*============================ 运动学几何参数 ============================*/
/**
 * 基座回转中心到同步带固定段的水平偏移 (mm): 末端水平半径 = 偏移 + 同步带伸长量。
 *
 * @note 该偏移**只用于内部运动学换算**: 笛卡尔接口(Arm_MoveTo)的逆解和 `arm pos` 的
 *       笛卡尔 x/y 正解; 所有对外位姿/伸展参数(ARM_POSE_*、Arm_MovePolarTo、`arm pol` 等)
 *       统一直接填伸展长度(extend_mm), 不需要、也不应该再加这个偏移。
 */
#define ARM_BASE_TO_EXTEND_MM 133.0f

/*============================ 到位等待 ============================*/
/**
 * 位置命令是异步下发的: 夹爪动作前必须等待机械臂到位, 否则会在未到位时开合(抓空/撞料)。
 * 到位判定基于状态轮询(S_CPOS)的位置: 参与判定的轴位置连续稳定超过判定时长,
 * 且已下发目标的轴到达目标(误差不超过容差)才算到位。
 */
#define ARM_SETTLE_POLL_MS 25 // 到位检测轮询间隔 (ms): 只读状态轮询缓存, 加密轮询可更快判定稳定
#define ARM_SETTLE_STABLE_MS 100 // 位置连续稳定判定时长 (ms), 需大于电机状态更新周期
#define ARM_SETTLE_TOL_RAW 230 // 位置稳定判定容差(读取位置单位, 65536/圈)
#define ARM_MOVE_TIMEOUT_MS 8000 // 单次动作到位等待超时 (ms); 分步运动时每段各用一次
#define ARM_MOVE_LIFT_DIR_TOL_MM 0.5f // 判定"上升/下降"的抬升高度差容差 (mm): 目标高于当前+容差=上升
/**
 * 分步运动"已在目标上"的跳过容差(动作连贯/提速用):
 *   目标与当前位姿差值不超过对应容差时, 跳过该段(不发命令、不等到位等待),
 *   用于只升降或只水平的动作(如盘面下降/抬起、抓取后抬升、回到初始位), 省掉一次空等的到位判定;
 *   基座角度按 ±180° 归算比较。容差不宜大于到位容差太多, 否则本来要动的微小修正会被跳过。
 */
#define ARM_MOVE_SKIP_HORIZONTAL_MM 0.6f // 伸展差值 ≤ 该值(且基座角差 ≤ ARM_MOVE_SKIP_BASE_DEG)时跳过水平段
#define ARM_MOVE_SKIP_BASE_DEG 0.8f // 基座角度差 ≤ 该值(按 ±180° 归算)时视为同角度
#define ARM_MOVE_SKIP_LIFT_MM 0.6f // 抬升差值 ≤ 该值时跳过升降段
/**
 * 分步运动每段(水平段/升降段)等待超时后的自动重试次数:
 * 电机偶发中途停住/保护停转时, 重新下发一次同样的绝对目标即可继续走到位;
 * 0 = 不重试; 1 = 失败后再下发一次(默认)。失败日志会打印目标与实际位姿, 便于定位。
 */
#define ARM_MOVE_RETRY 1

/*============================ 夹爪参数 ============================*/

#define ARM_GRIPPER_SERVO_ID 0  // 夹爪舵机 ID

#define ARM_GRIPPER_OPEN_DEG -81 // 常规张开角度(°): 物料盘取料前张开/物料盘放料张开/场地格位取料前张开
#define ARM_GRIPPER_CLOSE_DEG -60 // 夹爪闭合角度 (°)
/**
 * 夹爪开合到位判定(单圈角度回读确认, 替代固定延时):
 *   - 下发夹爪角度后轮询舵机单圈角度回读, 连续两次在 ARM_GRIPPER_SETTLE_TOL_DEG 内视为到达;
 *   - 夹爪夹住物料时可能到不了目标角度: 检测到"已开始运动且角度不再变化"也视为到位(夹稳/被挡);
 *   - 超时 ARM_GRIPPER_SETTLE_TIMEOUT_MS 判失败; 读不到角度回读时回退固定等待 ARM_GRIPPER_SETTLE_MS。
 */
#define ARM_GRIPPER_SETTLE_TOL_DEG 0.8f // 到达目标角度的判定容差 (°)
#define ARM_GRIPPER_MOVE_START_DEG 1.0f // 判定"已开始动作"的角度变化量 (°)
#define ARM_GRIPPER_STILL_DEG 0.5f // 判定"运动停止"的相邻采样角度变化 (°)
#define ARM_GRIPPER_SETTLE_POLL_MS 20 // 角度回读轮询间隔 (ms)
#define ARM_GRIPPER_SETTLE_TIMEOUT_MS 1500 // 到位等待超时 (ms)
#define ARM_GRIPPER_READ_FAIL_MAX 5 // 连续读不到角度的最大次数, 超过则回退固定等待

/*============================ 树莓派视觉标定预备动作 ============================*/
/**
 * 发送视觉标定命令前, 先把机械臂移动到任务层给出的预备位姿(基座旋转/伸展/抬升)并等待到位
 * (相机装在机械臂末端时, 必须先把相机转到能看到圆盘/圆环的位置)。
 * 运动顺序固定为"**先基座旋转+伸展到位, 再升降到位**"(不一边旋转一边升降, 避免斜向摆入标定视野):
 *   - 1 = 执行预备动作(比赛/联调); 0 = 不动作(相机固定已在视野内时调试用)
 * 速度/加速度用 ARM_PROFILE_CAL_PREP_INIT, 每段到位等待超时用 ARM_CAL_SETTLE_TIMEOUT_MS。
 */
#ifndef ARM_CAL_PREP_ENABLE
#define ARM_CAL_PREP_ENABLE 1
#endif
#ifndef ARM_CAL_SETTLE_TIMEOUT_MS
#define ARM_CAL_SETTLE_TIMEOUT_MS ARM_MOVE_TIMEOUT_MS // 标定预备动作到位等待超时 (ms)
#endif
/* 预备动作后读回基座实际角度的校验: 命令入队成功不等于电机执行到位,
 * 未回零/电机离线时可能出现"假到位", 读回校验可兜住。
 * ARM_CAL_VERIFY_ENABLE=0 关闭校验; ARM_CAL_ANGLE_TOL_DEG 为允许误差(度)。 */
#ifndef ARM_CAL_VERIFY_ENABLE
#define ARM_CAL_VERIFY_ENABLE 1
#endif
#ifndef ARM_CAL_ANGLE_TOL_DEG
#define ARM_CAL_ANGLE_TOL_DEG 2.0f
#endif
/**
 * 视觉标定/原料区抓取时的夹爪大张开角度(°):
 *   - 视觉标定: 相机装在夹爪/末端上时, 闭合的夹爪或夹着的物料会挡住圆盘/圆环视野,
 *     标定前把夹爪张到该角度, 由 Arm_Action_CalibrationReady 执行(失败只告警, 不阻断标定);
 *   - 原料区抓取: 来料位置/姿态不完全确定, 抓取前张到该角度,
 *     避免夹爪手指顶到物料/夹空, 由 arm_action_pick_matl 执行。
 */
#ifndef ARM_CAL_GRIPPER_OPEN_DEG
#define ARM_CAL_GRIPPER_OPEN_DEG -148 // 标定/原料区抓取时的夹爪大张开角度(°)
#endif

/*============================ 相机翻转舵机 ============================*/
/**
 * 机械臂末端 1 路舵机用于翻转相机:
 *   - 标定前: Arm_Action_CalibrationReady 把相机翻到 ARM_FLIP_CAL_ANGLE_DEG,
 *     让相机对准圆盘/圆环标定目标;
 *   - 标定结束: **保持标定角, 不再自动翻回**(后续抓取的颜色识别也用该翻转角视角);
 *   - 恢复默认角 ARM_FLIP_RESTORE_ANGLE_DEG 只在"机械臂伸缩校准/回零"时执行:
 *     整臂回零(`arm init`)与伸展轴单轴回零(`arm home extend`)开始前自动翻回默认角,
 *     避免翻转的相机与回零/伸缩动作干涉; 调试也可用 `arm flip restore` 手动恢复。
 *
 * 当前 ID/角度/等待时间均为占位值, 必须按实际舵机接线/烧录 ID 与相机翻转机构
 * 实测标定后再比赛使用; 未装翻转舵机时可构建系统 -DARM_FLIP_ENABLE=0 关闭。
 */
#ifndef ARM_FLIP_ENABLE
#define ARM_FLIP_ENABLE 1 // 1=标定/颜色识别流程中翻转相机; 0=空操作(未装翻转舵机/角度未标定时用)
#endif
#define ARM_FLIP_SERVO_ID 1 // 翻转舵机 ID
#define ARM_FLIP_CAL_ANGLE_DEG -105.0f // 标定时翻到的角度(°), 标定后保持该角供颜色识别; 占位值需标定
#define ARM_FLIP_RESTORE_ANGLE_DEG ARM_FLIP_CAL_ANGLE_DEG + 90.0f // 回零/伸缩校准前恢复的默认角度(°), 占位值需标定
#define ARM_FLIP_INIT_ANGLE_DEG 71.0f // 初始翻转角度(°)
#define ARM_FLIP_SETTLE_MS 500 // 翻转后等待舵机到位时间(ms), 需保证 RPI_Cal 拍摄前已到位

/*============================ 物料盘参数(总线舵机) ============================*/
/**
 * 物料盘由 1 个总线舵机驱动做直线运动, 夹爪到物料盘的取放位置是固定的:
 * 由物料盘把目标格位送到夹爪下方完成取放, 机械臂不需要为格位做水平偏移。
 *
 * **每个盘位的舵机角度分开填**: 三个盘位的实测角度并不等距, 不再用"基准角 + 格距"固定偏移推算。
 * 标定方法: 把物料盘摆到目标盘位, 用 `servo` 调试命令读舵机多圈角度, 把读数填到对应宏
 * (也可先用下面占位值 `arm pad <slot>` 走位后再微调)。
 * 下面数值是旧"基准 -143.8° + 270°/格"推算的占位值, 必须逐盘位实测替换。
 */
#define ARM_PAD_SERVO_ID 2 // 物料盘舵机 ID (Servo_SetServoAngleMTurn)
#define ARM_PAD_SLOT_NUM 3 // 物料盘盘位数量(与每轮物料数一致)
#define ARM_PAD_SLOT_BASE 2 // 行驶前归位用的默认盘位号(2=中间盘位), 仅作归位目标, 不再参与角度推算
#define ARM_PAD_SLOT1_ANGLE_DEG -405.8f // 1 号盘位舵机角度 (°), 占位值需标定
#define ARM_PAD_SLOT2_ANGLE_DEG -134.0f // 2 号盘位舵机角度 (°), 占位值需标定
#define ARM_PAD_SLOT3_ANGLE_DEG 151.2f // 3 号盘位舵机角度 (°), 占位值需标定
/**
 * 物料盘换格到位判定(角度回读确认, 替代原来的固定延时):
 *   - 下发换格命令后轮询舵机多圈角度, 连续两次在 ARM_PAD_SETTLE_TOL_DEG 内才算稳定到位;
 *   - 盘位间角度差按实测(旧占位值约 270°), 换格约需 1s+, 超过 ARM_PAD_SETTLE_TIMEOUT_MS 判换格失败;
 *   - 若舵机固件不支持角度回读, 回退到固定等待 ARM_PAD_SETTLE_MS(兜底, 仅提示不判失败)。
 */
#define ARM_PAD_SETTLE_TOL_DEG 2.0f // 到位判定容差 (°)
#define ARM_PAD_SETTLE_POLL_MS 30 // 角度回读轮询间隔 (ms)
#define ARM_PAD_SETTLE_TIMEOUT_MS 5000 // 到位等待超时 (ms)
#define ARM_PAD_SETTLE_MS 300 // 角度回读不可用时的兜底固定等待 (ms)
#define ARM_PAD_READ_FAIL_MAX 5 // 连续读不到角度的最大次数, 超过则判定不支持回读并回退固定等待

/*============================ 任务动作公共参数 ============================*/
/**
 * 抓取/放置动作的公共参数与超时:
 *   - ARM_SLOT_NUM: 粗加工区/暂存区放置格位数量(与每轮物料数一致), 用于任务码格位号校验;
 *   - ARM_GRAB_COLOR_TIMEOUT_MS: 原料区等待目标颜色稳定的超时, 超时判抓取失败(不再无限等待);
 *     颜色识别走"会话"方式: 到达抓取位后只下发一次识别命令(首帧等颜色锁定约 2s),
 *     之后树莓派每 ~100ms 连续上报一帧(不补 0x11、不重新锁定), 连续 2 帧命中目标色即抓取;
 *     所以这里仍是整段等待的总超时(默认 24s), 正常情况下几百毫秒~2s 就能完成;
 *   - ARM_GRIPPER_SETTLE_MS: 夹爪角度回读不可用时的兜底固定等待(正常情况按角度回读确认到位);
 *   - ARM_PLACE_LIFT_MM: 放料后垂直向上抬升的抬升高度增量, 避免移臂/转基座时刮碰已放物料
 *     (取回上盘动作不再单独抬升一次, 抬升由后续移动到物料盘的"先升到盘安全高度再水平"完成)。
 */
#ifndef ARM_SLOT_NUM
#define ARM_SLOT_NUM 3
#endif
#ifndef ARM_GRAB_COLOR_TIMEOUT_MS
#define ARM_GRAB_COLOR_TIMEOUT_MS 24000
#endif
#ifndef ARM_GRIPPER_SETTLE_MS
#define ARM_GRIPPER_SETTLE_MS 200
#endif
#ifndef ARM_PLACE_LIFT_MM
#define ARM_PLACE_LIFT_MM 60.0f
#endif

/*============================ 视觉 yaw 误差的机械臂补偿 ============================*/
/**
 * 树莓派视觉标定(rpi cal)会给出 yaw 误差; 靠车身旋转修正精度较差(麦轮打滑/转角误差),
 * 改为在抓取/放置时把该误差加到机械臂基座旋转角上补偿:
 *   - 误差由任务层在标定成功后调用 Arm_Action_SetYawCorrection() 传入(见 mission_config.h
 *     的 MISSION_CAL_YAW_FIX_WITH_ARM), 每个工位开始时由任务层清零;
 *   - 只作用于"场地固定"的工位(原料区/粗加工区/暂存区), 物料盘随车体转动, 不做补偿;
 *   - 超过限幅时按限幅值补偿并告警(误差过大说明车身偏差太大, 应检查导航/标定)。
 */
#ifndef ARM_YAW_CORRECTION_ENABLE
#define ARM_YAW_CORRECTION_ENABLE 1 // 1=启用标定 yaw 误差 -> 基座补偿; 0=忽略
#endif
#ifndef ARM_YAW_CORRECTION_MAX_DEG
#define ARM_YAW_CORRECTION_MAX_DEG 20.0f // 补偿角限幅 (°)
#endif
#ifndef ARM_YAW_CORRECTION_SIGN
#define ARM_YAW_CORRECTION_SIGN 1.0f // 视觉误差 -> 基座补偿方向; 实车方向相反时改成 -1.0f
#endif

/*============================ 动作目标位姿 ============================*/
/**
 * 任务动作工位的目标位姿(伸展长度/基座角度/抬升高度, 单位 mm/deg):
 *   extend_mm      : 同步带伸展长度, 0=完全收回; 与 `arm pos` 的 extend 读数、
 *                    Arm_ExtendTo 的数值一致(不是水平半径);
 *   base_angle_deg : 基座角度, 0=机械零位, 正/负为两侧;
 *   lift_mm        : 抬升高度, 0=最底部(逻辑零点), 数值越大越靠上。
 */
typedef struct {
    float extend_mm;
    float base_angle_deg;
    float lift_mm;
} ArmPose_t;

/* 相对父位姿的偏移量 (mm), 只改 lift(抬升高度), 保持 extend/base_angle 与父位姿一致 */
#define PAD_REACH_MM 62.0f // 物料盘面相对上方安全高度的下降量(lift 减小)

#define ARM_POSE_MATL_GRAB_INIT {.extend_mm = 14.39f, .base_angle_deg = -180.0f, .lift_mm = 80.41f} // 原料区抓取位
#define ARM_POSE_PAD_INIT {.extend_mm = 87.36f, .base_angle_deg = 0.0f, .lift_mm = 167.5f} // 物料盘取放位(上方安全高度)

/* ===== 机械臂初始状态(行驶/待机位) =====
 * 用途: 整臂回零完成后的待机位, 以及每次小车移动前的准备位(臂收短+抬升, 避免带长臂/低姿态行驶)。
 * 占位值需实测标定: `arm preset init` 走到该位后读 `arm pos`, 把实测 extend/rotate/lift 填回下面三个宏;
 * 三项统一约定: 伸展长度/基座角度/抬升高度(见本节开头说明)。 */
#define ARM_POSE_INIT_EXTEND_MM 120.0f // 初始状态: 伸展长度 (mm)
#define ARM_POSE_INIT_BASE_ANGLE_DEG 0 // 初始状态: 基座角度 (°)
#define ARM_POSE_INIT_LIFT_MM 165.0f // 初始状态: 抬升高度 (mm, 从底部计)
#define ARM_POSE_INIT_INIT                                                                                             \
    {.extend_mm = ARM_POSE_INIT_EXTEND_MM,                                                                             \
     .base_angle_deg = (float)ARM_POSE_INIT_BASE_ANGLE_DEG,                                                            \
     .lift_mm = ARM_POSE_INIT_LIFT_MM} // 初始状态位姿

/* ===== 粗加工区放置位预设(逐点标定) =====
 * 下标对应格位号 1~ARM_SLOT_NUM; 初值由旧"中心格 + Y 向 150mm 格距"换算估算,
 * 仅为保证未标定时动作仍大致可用, 使用前必须逐格实测标定(用 `arm preset process <slot>` 对位后读 `arm pos` 填回)。 */
#define ARM_POSE_PROCESS_SLOT1_INIT {.extend_mm = 122.0f, .base_angle_deg = -145.0f, .lift_mm = 3.0f}
#define ARM_POSE_PROCESS_SLOT2_INIT {.extend_mm = 70.85f, .base_angle_deg = -180.0f, .lift_mm = 3.0f}
#define ARM_POSE_PROCESS_SLOT3_INIT {.extend_mm = 122.0f, .base_angle_deg = -215.0f, .lift_mm = 3.0f}

/* ===== 暂存区放置位预设(逐点标定, 每格位每层一个) =====
 * 下标: 格位号 1~ARM_SLOT_NUM × 层号 0/1; 初值同上由旧换算估算, 必须逐点标定。 */
#define ARM_POSE_STORAGE_SLOT1_L0_INIT {.extend_mm = 122.0f, .base_angle_deg = -145.0f, .lift_mm = 3.0f}
#define ARM_POSE_STORAGE_SLOT1_L1_INIT {.extend_mm = 122.0f, .base_angle_deg = -145.0f, .lift_mm = 63.0f}
#define ARM_POSE_STORAGE_SLOT2_L0_INIT {.extend_mm = 70.85f, .base_angle_deg = -180.0f, .lift_mm = 3.0f}
#define ARM_POSE_STORAGE_SLOT2_L1_INIT {.extend_mm = 70.85f, .base_angle_deg = -180.0f, .lift_mm = 63.0f}
#define ARM_POSE_STORAGE_SLOT3_L0_INIT {.extend_mm = 122.0f, .base_angle_deg = -215.0f, .lift_mm = 3.0f}
#define ARM_POSE_STORAGE_SLOT3_L1_INIT {.extend_mm = 122.0f, .base_angle_deg = -215.0f, .lift_mm = 63.0f}

#endif /* __ARM_CONFIG_H__ */
