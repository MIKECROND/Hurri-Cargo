/**
 * @file arm_ctrl.c
 * @author T-Car Team
 * @brief 机械臂控制核心: 电机命令封装、固件回零、单位换算、到位检测
 *
 * 回零实现(电机固件内置回零):
 *  统一使用"多圈有限位开关回零"(模式 3): 应用层先把回零模式/方向/速度/超时写入电机并存储,
 *  再触发固件回零; 电机自行朝限位开关方向运动, 限位触发后由固件把该点作为机械零位。
 *  应用层仅轮询回零状态标志(正在回零 org_sf / 回零失败 org_cf)等待完成或失败。
 *
 * @note 该文件需在 FreeRTOS 任务上下文中调用(osDelay/互斥锁)。
 * 可调参数见 arm_config.h / Applications/arm。
 */

#include "arm_ctrl.h"
#include "arm_config.h"
#include "zdt_v5_cmd.h"
#include "zdt_v5_engine.h"
#include "zdt_v5_port.h"
#include "servo_port.h"
#include "servo_protocol.h"
#include "cmsis_os2.h"
#include "log.h"

/* 电机模块全局资源(由 zdt_v5_port.c / freertos.c 提供) */
extern MotorStatus_t *g_motor_status[ZDT_STEP_NUM];
extern osMutexId_t Motor_MutexHandle;

static bool s_servo_set_origin = false;

/*============================ 轴定义表 ============================*/
/**
 * @brief 单轴静态定义
 * @note 所有"按电机 ID 区分"的参数只在本表出现一次, 增删轴时只改本表与 arm_config.h。
 */
typedef struct {
    uint8_t motor_id; // 电机总线 ID
    const char *name; // 轴名(日志/调试)
    uint8_t home_dir; // 固件回零方向: 0=CW, 1=CCW
    float gear_ratio; // 减速比(输出轴/电机), 直连为 1.0f
    float mm_per_rev; // 直线轴每转移动距离(mm); 旋转轴填 0
    int8_t linear_sign; // 命令方向(直线轴): +1=参数增大时脉冲为正(dir=0), -1=相反
    int8_t read_sign; // 回读方向(直线轴): +1=物理量增大时读数为正, -1=相反
    float home_offset_mm; // 回零点在逻辑坐标里的位置(mm): 逻辑值 = 偏移 + read_sign×回读位移
} ArmAxis_t;

/* 下标: 0=基座, 1=升降, 2=伸展(顺序与 ARM_*_MOTOR_ID 注释一致)
 * 直线轴的"命令方向"/"回读方向"/"回零偏移"分开配置:
 *   升降轴: 回零在顶部机械顶点(home_dir=0), 用 home_offset_mm 换算成"从底部计"的抬升高度,
 *           命令/回读方向都取 +1;
 *   伸展轴: 实车命令方向与"伸出"相反(dir=0 是收回、dir=1 才是伸出) => linear_sign 取 -1;
 *           命令取反后实际伸出对应固件坐标为负、原始回读也为负, 为使"伸出时读数为正"
 *           (含 arm pos 的电机角度/伸展长度) => read_sign 取 -1; 回零在收回端, 偏移 0;
 *   两者共同参与"下发目标 -> 期望回读位置"的换算(见 arm_record_target), 保证到位判定一致。
 * 若重新接线/更换电机后方向又变, 只改本表对应轴的 linear_sign / read_sign, 不要改零位/回零方向。 */
enum { AXIS_BASE = 0, AXIS_LIFT, AXIS_EXTEND };

static const ArmAxis_t s_axis[ARM_MOTOR_NUM] = {
    {ARM_BASE_MOTOR_ID, "base", ARM_BASE_HOME_DIR, ARM_BASE_GEAR_RATIO, 0.0f, 1, 1, 0.0f},
    {ARM_LIFT_MOTOR_ID,
     "lift",
     ARM_LIFT_HOME_DIR,
     ARM_LIFT_GEAR_RATIO,
     ARM_LIFT_MM_PER_REV,
     1,
     1,
     ARM_LIFT_TOP_HOME_OFFSET_MM},
    {ARM_EXTEND_MOTOR_ID, "extend", ARM_EXTEND_HOME_DIR, ARM_EXTEND_GEAR_RATIO, ARM_EXTEND_MM_PER_REV, -1, -1, 0.0f},
};

/**
 * @brief 按轴下标取轴定义
 * @return 轴定义指针; 下标越界返回 NULL
 */
static const ArmAxis_t *arm_axis_at(uint8_t index) {
    return (index < ARM_MOTOR_NUM) ? &s_axis[index] : NULL;
}

/**
 * @brief 按电机总线 ID 查轴定义
 * @return 轴定义指针; 未注册的 ID 返回 NULL
 */
static const ArmAxis_t *arm_axis_find(uint8_t motor_id) {
    for (uint8_t i = 0; i < ARM_MOTOR_NUM; i++) {
        if (s_axis[i].motor_id == motor_id) return &s_axis[i];
    }
    return NULL;
}

/**
 * @brief 各轴最近一次下发的绝对目标位置(读取位置原始单位, 带符号)
 * @note 位置回读来自状态轮询缓存(100ms 一次), 只看"位置连续稳定"会把
 *       还没启动/还没真正到位的轴判成到位, 因此到位判定还要核对本目标。
 *       脉冲与读取位置只差"每圈倍数"(减速比在两者中相同, 约掉)。
 */
static int32_t s_target_read_pos[ARM_MOTOR_NUM];
static bool s_target_read_pos_valid[ARM_MOTOR_NUM];

/**
 * @brief 记录某轴最近一次下发的绝对目标位置
 * @param motor_id 电机总线 ID
 * @param pulses   下发的脉冲幅值(无符号)
 * @param dir      下发方向: 0=正向, 其他=反向
 * @note 期望回读位置 = 命令脉冲 -> 回读位置, 命令方向/回读方向不一致的轴(伸展轴)
 *       需要再乘 linear_sign*read_sign, 否则到位判定会永远对不上目标。
 */
static void arm_record_target(uint8_t motor_id, int32_t pulses, uint8_t dir) {
    const ArmAxis_t *axis = arm_axis_find(motor_id);
    if (axis == NULL) return;

    int32_t signed_pulses = (dir == 0) ? pulses : -pulses;
    float read_pos = (float)signed_pulses * (float)axis->linear_sign * (float)axis->read_sign *
                     ((float)ARM_READ_UNIT_PER_REV / (float)ARM_MOTOR_PULSE_PER_REV);
    uint8_t index = (uint8_t)(axis - s_axis);
    s_target_read_pos[index] = (int32_t)((read_pos >= 0.0f) ? (read_pos + 0.5f) : (read_pos - 0.5f));
    s_target_read_pos_valid[index] = true;
}

/**
 * @brief 清除全部轴的已下发目标(回零后位置基准变化, 旧目标不再有效)
 */
static void arm_clear_targets(void) {
    for (uint8_t i = 0; i < ARM_MOTOR_NUM; i++) {
        s_target_read_pos_valid[i] = false;
    }
}

/**
 * @brief 等待指定轴位置稳定(可选核对已下发目标是否到达), Arm_WaitSettled 的内部实现
 * @param timeout_ms   最长等待时间 (ms)
 * @param axis_mask    参与判定的轴掩码(bit i 对应轴序号 i)
 * @param check_target true=已下发目标的轴必须到达目标; false=只判位置稳定
 */
static bool arm_wait_settled(uint32_t timeout_ms, uint32_t axis_mask, bool check_target);

/**
 * @brief 电机回零状态快照
 * @note 互斥保护下拷贝, 避免与电机接收任务并发访问结构体
 */
typedef struct {
    bool org_sf; // 正在回零
    bool org_cf; // 回零失败
} ArmMotorSnap_t;

/*============================ 底层命令封装 ============================*/

/**
 * @brief 使能/失能电机
 */
static bool arm_cmd_enable(uint8_t motor_id, bool enable) {
    MotorCmd_t cmd;
    cmd.op_type = OP_CONTROL;
    cmd.motor_id = motor_id;

    cmd.type.ctrl.type = CTRL_ENABLE;
    cmd.type.ctrl.p.en.enable = enable;
    cmd.type.ctrl.p.en.sync = false;
    return Motor_Send_Cmd(&cmd);
}

/**
 * @brief 立即停止电机
 */
static bool arm_cmd_stop(uint8_t motor_id) {
    MotorCmd_t cmd;
    cmd.op_type = OP_CONTROL;
    cmd.motor_id = motor_id;

    cmd.type.ctrl.type = CTRL_STOP;
    cmd.type.ctrl.p.stop.sync = false;
    return Motor_Send_Cmd(&cmd);
}

/**
 * @brief 位置模式运动
 * @note 脉冲字段是无符号幅值, 负目标必须由调用方拆成 dir+幅值
 */
static bool arm_cmd_position(uint8_t motor_id, uint8_t dir, uint16_t vel, uint16_t acc, int32_t target, uint8_t mode) {
    if (target < 0) {
        logWarning("arm: motor %d negative target %ld rejected", motor_id, (long)target);
        return false;
    }

    MotorCmd_t cmd;
    cmd.motor_id = motor_id;
    cmd.op_type = OP_CONTROL;
    cmd.type.ctrl.type = CTRL_POS;
    cmd.type.ctrl.p.pos.dir = dir;
    cmd.type.ctrl.p.pos.vel = vel;
    cmd.type.ctrl.p.pos.acc = acc;
    cmd.type.ctrl.p.pos.target = target;
    cmd.type.ctrl.p.pos.mode = mode;
    cmd.type.ctrl.p.pos.sync = false;

    bool sent = Motor_Send_Cmd(&cmd);
    if (sent) {
        /* 记录目标供 Arm_WaitSettled 核对"是否真的到达目标"(不只是"不动了") */
        arm_record_target(motor_id, target, dir);
    }
    return sent;
}

/**
 * @brief 解除堵转/过热/过流保护
 */
static bool arm_cmd_clear_protect(uint8_t motor_id) {
    MotorCmd_t cmd;
    cmd.op_type = OP_TRIGGER;
    cmd.motor_id = motor_id;

    cmd.type.trigger.type = TRIG_CLEAR_PROTECT;
    return Motor_Send_Cmd(&cmd);
}

/**
 * @brief 当前位置清零(以当前位置为该轴零位)
 */
static bool arm_cmd_reset_pos(uint8_t motor_id) {
    MotorCmd_t cmd;
    cmd.op_type = OP_TRIGGER;
    cmd.motor_id = motor_id;

    cmd.type.trigger.type = TRIG_RESET_POS;
    bool sent = Motor_Send_Cmd(&cmd);
    if (sent) {
        /* 清零后当前位置即 0, 目标同步更新为 0 */
        arm_record_target(motor_id, 0, 0);
    }
    return sent;
}

/**
 * @brief 写入电机固件回零参数(模式/方向/速度/超时)并存储
 * @note 无限位碰撞参数一并下发, 仅模式 2 使用
 */
static bool arm_cmd_set_home_params(
    uint8_t motor_id, uint8_t mode, uint8_t dir, uint16_t vel_rpm, uint32_t timeout_ms) {
    MotorCmd_t cmd;
    cmd.op_type = OP_PARAM_WRITE;
    cmd.motor_id = motor_id;

    cmd.type.write.type = PARAM_HOME_PARAMS;
    cmd.type.write.p.home_params.mode = mode;
    cmd.type.write.p.home_params.dir = dir;
    cmd.type.write.p.home_params.vel = vel_rpm;
    cmd.type.write.p.home_params.timeout = timeout_ms;
    cmd.type.write.p.home_params.sl_vel = ARM_HOME_COLLISION_VEL_RPM;
    cmd.type.write.p.home_params.sl_current = ARM_HOME_COLLISION_CURRENT;
    cmd.type.write.p.home_params.sl_time = ARM_HOME_COLLISION_TIME_MS;
    cmd.type.write.p.home_params.auto_home = false; // 由应用层触发, 不用上电自动回零
    cmd.type.write.p.home_params.save = true; // 存储到电机内存
    return Motor_Send_Cmd(&cmd);
}

/**
 * @brief 触发电机固件回零
 */
static bool arm_cmd_home(uint8_t motor_id, uint8_t mode) {
    MotorCmd_t cmd;
    cmd.op_type = OP_TRIGGER;
    cmd.motor_id = motor_id;

    cmd.type.trigger.type = TRIG_HOME_RETURN;
    cmd.type.trigger.p.home.mode = mode;
    cmd.type.trigger.p.home.sync = false;
    bool sent = Motor_Send_Cmd(&cmd);
    if (sent) {
        /* 回零会改变位置基准, 之前记录的目标全部失效 */
        arm_clear_targets();
    }
    return sent;
}

/**
 * @brief 主动读取电机系统状态参数(用于刷新电机状态结构体)
 */
static bool arm_cmd_read_sys(uint8_t motor_id, SysParams_t param) {
    MotorCmd_t cmd;
    cmd.op_type = OP_PARAM_READ;
    cmd.motor_id = motor_id;

    cmd.type.read.type = MP_SYS;
    cmd.type.read.p.sys = param;
    return Motor_Send_Cmd(&cmd);
}

/*============================ 状态读取 ============================*/

/**
 * @brief 按电机总线 ID 查找已注册的状态结构体
 */
static MotorStatus_t *arm_find_motor(uint8_t motor_id) {
    for (uint8_t i = 0; i < ZDT_STEP_NUM; i++) {
        if (g_motor_status[i] != NULL && g_motor_status[i]->motor_id == motor_id) {
            return g_motor_status[i];
        }
    }
    return NULL;
}

/**
 * @brief 获取电机回零状态快照
 */
static void arm_motor_snapshot(MotorStatus_t *motor, ArmMotorSnap_t *snap) {
    if (osMutexAcquire(Motor_MutexHandle, 50) == osOK) {
        snap->org_sf = motor->org_sf;
        snap->org_cf = motor->org_cf;
        osMutexRelease(Motor_MutexHandle);
    }
}

/**
 * @brief 读取电机当前实时位置
 * @note 返回值是"读取位置"的原始单位(Emm 固件为 65536/圈), 与位置命令用的脉冲数不同;
 *       依赖状态轮询刷新(S_CPOS), 需先完成回零;
 *       超过 MOTOR_ONLINE_TIMEOUT_MS 没有该电机的应答(未接/掉线)时返回 false,
 *       避免把状态结构体的上电初始值当成真实位置。
 * @return true 读取成功
 */
static bool arm_motor_read_pos(uint8_t motor_id, int32_t *read_pos) {
    MotorStatus_t *motor = arm_find_motor(motor_id);
    if (motor == NULL) return false;

    if (Motor_Rx_Age_Ms(motor_id) > MOTOR_ONLINE_TIMEOUT_MS) return false;

    if (osMutexAcquire(Motor_MutexHandle, 50) == osOK) {
        *read_pos = motor->pos;
        osMutexRelease(Motor_MutexHandle);
        return true;
    }
    return false;
}

/*============================ 单位换算 ============================*/

/**
 * @brief 输出轴每转对应的位置命令脉冲数 = 电机每转脉冲 × 减速比
 */
static int32_t arm_axis_pulse_per_rev(const ArmAxis_t *axis) {
    return (int32_t)((float)ARM_MOTOR_PULSE_PER_REV * axis->gear_ratio + 0.5f);
}

/**
 * @brief 输出轴每转对应的读取位置单位数 = 65536 × 减速比(Emm 固件按电机轴计)
 */
static float arm_axis_read_unit_per_rev(const ArmAxis_t *axis) {
    return (float)ARM_READ_UNIT_PER_REV * axis->gear_ratio;
}

/**
 * @brief 输出角度(°)换算为位置命令脉冲
 */
static int32_t arm_deg_to_pulse(const ArmAxis_t *axis, int32_t angle_deg) {
    int32_t pulse_per_rev = arm_axis_pulse_per_rev(axis);
    if (pulse_per_rev <= 0) return 0;
    return (int32_t)(((int64_t)angle_deg * pulse_per_rev) / 360);
}

/**
 * @brief 读取位置换算为输出角度(°)
 * @note 读取位置单位为 65536/圈(按电机轴), 与位置命令的脉冲数不同, 不能混用
 */
static float arm_read_pos_to_deg(const ArmAxis_t *axis, int32_t read_pos) {
    return (float)read_pos * 360.0f / arm_axis_read_unit_per_rev(axis);
}

/**
 * @brief 直线距离(mm)换算为位置命令脉冲
 */
static int32_t arm_mm_to_pulse(float length_mm, float pulse_per_mm) {
    float pulses = length_mm * pulse_per_mm;
    return (pulses >= 0.0f) ? (int32_t)(pulses + 0.5f) : (int32_t)(pulses - 0.5f);
}

/*============================ 回零接口 ============================*/

/**
 * @brief 单个轴固件回零(多圈有限位开关回零)
 * @param axis 轴定义
 * @return true 回零成功, false 失败(电机未注册/命令失败/超时/回零失败)
 */
static bool arm_home_axis(const ArmAxis_t *axis) {
    MotorStatus_t *motor = arm_find_motor(axis->motor_id);
    if (motor == NULL) {
        logWarning("arm: motor %d not registered, skip homing", axis->motor_id);
        return false;
    }

    ArmMotorSnap_t snap = {0};
    uint32_t start;
    uint32_t wait_ms = ARM_HOME_TIMEOUT_MS + ARM_HOME_WAIT_MARGIN_MS;
    bool started = false; // 是否已观察到"正在回零"
    bool ok = false;

    logPrintln("[arm] motor %d (%s) firmware home start (mode %d)", axis->motor_id, axis->name, ARM_HOME_MODE);

    /* 1. 清除可能残留的保护, 再使能电机锁轴
     *    命令入队失败必须立即中止, 否则后续只能等超时兜底且无法定位原因 */
    if (!arm_cmd_clear_protect(axis->motor_id)) {
        logWarning("arm: motor %d clear-protect cmd rejected/dropped", axis->motor_id);
        return false;
    }
    osDelay(50);
    if (!arm_cmd_enable(axis->motor_id, true)) {
        logWarning("arm: motor %d enable cmd rejected/dropped", axis->motor_id);
        return false;
    }
    osDelay(100);

    /* 2. 写入回零参数(模式/方向/速度/超时)并存储到电机内存 */
    if (!arm_cmd_set_home_params(
            axis->motor_id, ARM_HOME_MODE, axis->home_dir, ARM_HOME_VEL_RPM, ARM_HOME_TIMEOUT_MS)) {
        logWarning("arm: motor %d set-home-params cmd rejected/dropped", axis->motor_id);
        return false;
    }
    osDelay(100);

    /* 3. 触发固件回零 */
    if (!arm_cmd_home(axis->motor_id, ARM_HOME_MODE)) {
        logWarning("arm: motor %d home trigger cmd rejected/dropped", axis->motor_id);
        arm_cmd_stop(axis->motor_id);
        return false;
    }

    /* 4. 轮询回零状态标志, 等待电机自行完成(顶到限位开关后固件自动清零) */
    start = osKernelGetTickCount();
    while ((osKernelGetTickCount() - start) < wait_ms) {
        arm_cmd_read_sys(axis->motor_id, S_OFLAG);
        osDelay(ARM_HOME_POLL_MS);
        arm_motor_snapshot(motor, &snap);

        if (snap.org_cf) { // 回零失败标志
            logWarning("arm: motor %d home failed (org_cf)", axis->motor_id);
            break;
        }
        if (snap.org_sf) { // 正在回零
            started = true;
            continue;
        }
        if (started) { // 回零中标志已消失 => 回零完成
            ok = true;
            break;
        }
    }

    /* 5. 失败时停止电机, 避免持续转动 */
    if (!ok) {
        logWarning("arm: motor %d home timeout/fail", axis->motor_id);
        arm_cmd_stop(axis->motor_id);
        return false;
    }

    logPrintln("[arm] motor %d (%s) homed", axis->motor_id, axis->name);
    return true;
}

/**
 * @brief 整臂回零顺序(轴下标): 升降 -> 伸展 -> 基座旋转
 * @note 先升到顶部把末端/夹爪抬离台面与物料; 再收回伸展把臂收短, 避免后续旋转扫碰;
 *       最后在"最高 + 最短"姿态下转基座, 扫掠半径最小、最安全。
 */
#if ARM_MOTOR_NUM != 3
#error "整臂回零顺序表按 ARM_MOTOR_NUM=3 定义, 修改电机数量时需同步更新 s_home_order"
#endif
static const uint8_t s_home_order[ARM_MOTOR_NUM] = {AXIS_LIFT, AXIS_EXTEND, AXIS_BASE};

/**
 * @brief 所有机械臂轴按固定顺序依次回零(升降 -> 伸展 -> 旋转)
 * @return true 全部成功, false 存在失败轴
 */
static bool arm_home_all(void) {
    bool ok = true;

    for (uint8_t i = 0; i < ARM_MOTOR_NUM; i++) {
        if (!arm_home_axis(arm_axis_at(s_home_order[i]))) {
            ok = false;
        }
        osDelay(ARM_HOME_STEP_DELAY_MS); // 相邻电机回零留出机械动作时间, 避免干涉
    }
    return ok;
}

/*============================ 异步回零任务 ============================*/
/* 整臂回零串行执行最长约 30s: 放到独立低优先级任务中执行,
 * 避免阻塞系统初始化(Sys_Init_Task)与 shell 命令行 */

#define ARM_HOME_REQ_FLAG (1UL << 0)

/**
 * @brief 整臂回零完成后移动到机械臂初始状态位(行驶/待机位)
 * @return true=三轴已到初始位; false=命令下发失败/到位超时
 * @note 回零结束时臂已升到顶部且完全收回, 此时再走"初始状态位"不会扫碰台面/物料;
 *       初始状态位姿见 arm_config.h 的 ARM_POSE_INIT_*(占位值需标定, `arm preset init` 对位)。
 *       只做三轴运动, 不动夹爪/物料盘/相机翻转。
 */
static bool arm_move_to_init_pose_after_home(void) {
    if (!Arm_BaseRotateAbs(ARM_POSE_INIT_BASE_ANGLE_DEG, ARM_BASE_VEL, ARM_BASE_ACC)) {
        logWarning("arm: init pose: base rotate command failed");
        return false;
    }
    if (!Arm_ExtendTo(ARM_POSE_INIT_EXTEND_MM, ARM_EXTEND_VEL, ARM_EXTEND_ACC)) {
        logWarning("arm: init pose: extend command failed");
        return false;
    }
    if (!Arm_LiftTo(ARM_POSE_INIT_LIFT_MM, ARM_LIFT_VEL, ARM_LIFT_ACC)) {
        logWarning("arm: init pose: lift command failed");
        return false;
    }
    if (!Arm_WaitSettled(ARM_MOVE_TIMEOUT_MS)) {
        logWarning("arm: init pose: wait settled timeout");
        return false;
    }
    Arm_FlipSetAngle(ARM_FLIP_INIT_ANGLE_DEG);
    if (!Arm_GripperStartAngle(ARM_CAL_GRIPPER_OPEN_DEG)) {
        logWarning("arm: init pose: gripper angle command failed");
        return false;
    }

    return true;
}

/**
 * @brief 整臂回零状态
 */
typedef enum {
    ARM_HOME_STATE_IDLE = 0, // 未发起回零
    ARM_HOME_STATE_RUNNING, // 回零进行中
    ARM_HOME_STATE_DONE, // 全部回零成功
    ARM_HOME_STATE_FAILED, // 存在回零失败的电机
} ArmHomeState_t;

static osThreadId_t s_arm_home_task = NULL;
static volatile ArmHomeState_t s_home_state = ARM_HOME_STATE_IDLE;

/**
 * @brief 整臂回零后台任务: 等待请求后执行整臂回零
 */
static void arm_home_task(void *argument) {
    (void)argument;

    for (;;) {
        /* 阻塞等待回零请求(线程标志), 空闲时不占用 CPU */
        osThreadFlagsWait(ARM_HOME_REQ_FLAG, osFlagsWaitAny, osWaitForever);

        s_home_state = ARM_HOME_STATE_RUNNING;
        logPrintln("[arm] async homing start (order: lift -> extend -> base)");

        /* 整臂回零会驱动伸展轴(伸缩校准): 先把翻转相机翻回默认角, 避免翻转机构与回零动作干涉 */
        logInfo("arm: restore camera flip before whole-arm homing");
        Arm_FlipSetAngle(ARM_FLIP_RESTORE_ANGLE_DEG);
        if (!Arm_GripperStartAngle(ARM_GRIPPER_CLOSE_DEG)) {
            logWarning("arm: start gripper angle command failed");
            continue;
        }

        bool ok = arm_home_all();

        /* 回零完成后把机械臂摆到"初始状态"(行驶/待机位): 收短 + 抬升, 供后续动作/出发前保持 */
        if (ok) {
            logPrintln("[arm] homing done, move to initial pose");
            if (!arm_move_to_init_pose_after_home()) {
                logWarning("arm: move to initial pose after homing failed (homing still done)");
            }
        }

        s_home_state = ok ? ARM_HOME_STATE_DONE : ARM_HOME_STATE_FAILED;
        logPrintln("[arm] async homing %s", ok ? "done" : "FAILED");
    }
}

/**
 * @brief 初始化机械臂控制模块并发起整臂回零(非阻塞)
 */
void Arm_CtrlInit(void) {
    if (s_arm_home_task == NULL) {
        const osThreadAttr_t attr = {
            .name = "Arm_Home",
            .stack_size = ARM_HOME_TASK_STACK, // 字节
            .priority = (osPriority_t)osPriorityLow2, // 低优先级, 不抢占系统
        };
        s_arm_home_task = osThreadNew(arm_home_task, NULL, &attr);
        if (s_arm_home_task == NULL) {
            logWarning("arm: failed to create home task");
            return;
        }
    }

    logPrintln("[arm] request homing (async)");
    s_home_state = ARM_HOME_STATE_RUNNING;
    osThreadFlagsSet(s_arm_home_task, ARM_HOME_REQ_FLAG);
}

/**
 * @brief 等待整臂回零完成
 * @return true=回零已完成或未发起回零; false=回零失败或等待超时
 */
bool Arm_HomeWaitDone(uint32_t timeout_ms) {
    if (s_home_state == ARM_HOME_STATE_FAILED) return false;
    if (s_home_state != ARM_HOME_STATE_RUNNING) return true; /* IDLE/DONE 无需等待 */

    uint32_t start = osKernelGetTickCount();
    while (s_home_state == ARM_HOME_STATE_RUNNING) {
        if ((osKernelGetTickCount() - start) >= timeout_ms) {
            logWarning("arm: wait homing done timeout");
            return false;
        }
        osDelay(ARM_HOME_IDLE_POLL_MS);
    }
    return (s_home_state == ARM_HOME_STATE_DONE);
}

/**
 * @brief 查询本上电周期内整臂回零是否已完成
 * @return true=回零已完成(DONE); false=从未发起/正在回零/回零失败
 * @note 用于调试工具提示"先 arm init 再动作"; 手动 arm zero 的流程下可能为 false,
 *       此时绝对位置来自手动零点, 不代表未就绪(由调用方自行判断是否继续)。
 */
bool Arm_IsHomeDone(void) {
    return (s_home_state == ARM_HOME_STATE_DONE);
}

/**
 * @brief 指定单轴执行固件回零(阻塞到完成/失败)
 * @param axis 轴序号(0 ~ Arm_AxisCount()-1, 顺序: 基座/升降/伸展)
 * @return true=回零完成; false=轴序号非法/整臂回零进行中/电机未注册/命令失败/回零失败/超时
 * @note 与整臂异步回零(Arm_CtrlInit)互斥: 整臂回零进行中时拒绝执行;
 *       回零会改变该轴位置基准, 成功后清掉该轴已记录的目标(避免后续到位判定误判), 其它轴不受影响。
 *       本接口在调用任务上下文阻塞(单轴最长 ARM_HOME_TIMEOUT_MS + 余量), 供 shell/调试单轴重新寻找零位。
 */
bool Arm_HomeAxis(uint8_t axis) {
    const ArmAxis_t *def = arm_axis_at(axis);
    if (def == NULL) {
        logError("arm: invalid axis index %u (expect 0~%u)", (unsigned)axis, (unsigned)(ARM_MOTOR_NUM - 1));
        return false;
    }
    if (s_home_state == ARM_HOME_STATE_RUNNING) {
        logWarning("arm: whole-arm homing in progress, refuse single-axis homing");
        return false;
    }

    /* 伸展轴回零(伸缩校准)前先把翻转相机翻回默认角, 避免翻转机构与伸缩/回零动作干涉 */
    if (def == arm_axis_at(AXIS_EXTEND)) {
        logInfo("arm: restore camera flip before extend-axis homing");
        Arm_FlipSetAngle(ARM_FLIP_RESTORE_ANGLE_DEG);
    }

    if (!arm_home_axis(def)) {
        return false;
    }

    /* 该轴位置基准已变: 只清该轴的目标记录, 其它轴等待/到位判定不受影响 */
    uint8_t index = (uint8_t)(def - s_axis);
    if (index < ARM_MOTOR_NUM) {
        s_target_read_pos_valid[index] = false;
    }
    return true;
}

/*============================ 轴信息 ============================*/

uint8_t Arm_AxisCount(void) {
    return ARM_MOTOR_NUM;
}

uint8_t Arm_AxisMotorId(uint8_t axis) {
    const ArmAxis_t *def = arm_axis_at(axis);
    return (def != NULL) ? def->motor_id : 0;
}

const char *Arm_AxisName(uint8_t axis) {
    const ArmAxis_t *def = arm_axis_at(axis);
    return (def != NULL) ? def->name : "?";
}

/*============================ 电机使能 ============================*/

bool Arm_MotorsEnable(bool enable) {
    bool all_ok = true;

    /* 只下发使能/失能命令, 不做回零或运动; 某一轴失败不阻断其它轴, 最终统一返回 */
    for (uint8_t i = 0; i < ARM_MOTOR_NUM; i++) {
        if (!arm_cmd_enable(s_axis[i].motor_id, enable)) {
            logError("arm: motor ID%d %s command failed", (int)s_axis[i].motor_id, enable ? "enable" : "disable");
            all_ok = false;
        }
    }

    return all_ok;
}

/*============================ 零点设置 ============================*/

bool Arm_SetCurrentPosAsZero(uint32_t timeout_ms) {
    /* 运动过程中清零会让位置基准/软限位错乱, 先等三轴静止(电机离线/位置不可读时也会失败);
     * 这里只要求"静止", 不要求到达上一次下发目标(允许人工手动拖动后再清零) */
    if (!arm_wait_settled(timeout_ms, (1u << AXIS_BASE) | (1u << AXIS_LIFT) | (1u << AXIS_EXTEND), false)) {
        logError("arm: not settled/offline, refuse to set current position as zero");
        return false;
    }

    bool all_ok = true;
    for (uint8_t i = 0; i < ARM_MOTOR_NUM; i++) {
        if (!arm_cmd_reset_pos(s_axis[i].motor_id)) {
            logError("arm: motor ID%d reset-zero command failed", (int)s_axis[i].motor_id);
            all_ok = false;
        }
    }

    /* 只清电机位置: 升降轴的"从底部计"坐标由回零偏移(home_offset_mm)换算, 不随清零改变。
     * 因此升降轴应在顶部机械零位处执行本命令(与顶部回零等效, 清零后 lift 仍为
     * ARM_LIFT_TOP_HOME_OFFSET_MM); 若在中间位置清零, 逻辑高度会整体偏掉一个偏移量。 */
    if (all_ok) {
        logPrintln("[arm] position reset to zero; lift logical height = %.1fmm (top-home offset)",
                   (double)s_axis[AXIS_LIFT].home_offset_mm);
    }

    return all_ok;
}

/*============================ 单轴运动接口 ============================*/

/**
 * @brief 轴旋转到绝对角度(相对机械零位)
 * @note 绝对位置模式: 0=机械零位。
 *       Emm 固件 0xFD 的脉冲字段是无符号幅值, 符号由"方向"字节承载,
 *       负角度不能按补码下发(会被当成长度巨大的正数, 电机不停转)。
 */
static bool arm_rotate_abs(const ArmAxis_t *axis, int32_t angle_deg, uint16_t vel, uint16_t acc) {
    int32_t pulses = arm_deg_to_pulse(axis, angle_deg);
    return arm_cmd_position(axis->motor_id, (pulses >= 0) ? 0 : 1, vel, acc, (pulses >= 0) ? pulses : -pulses, 1);
}

/**
 * @brief 基座旋转到绝对角度
 * @note 目标超出软限位时**钳到限位值继续执行**(告警, 不再失败返回), 方便"超出可达范围时按最大值执行"
 */
bool Arm_BaseRotateAbs(int32_t angle_deg, uint16_t vel, uint16_t acc) {
    if (angle_deg < ARM_BASE_MIN_DEG) {
        logWarning("arm: base angle %ld below min %d, clamp to min", (long)angle_deg, ARM_BASE_MIN_DEG);
        angle_deg = ARM_BASE_MIN_DEG;
    } else if (angle_deg > ARM_BASE_MAX_DEG) {
        logWarning("arm: base angle %ld above max %d, clamp to max", (long)angle_deg, ARM_BASE_MAX_DEG);
        angle_deg = ARM_BASE_MAX_DEG;
    }
    return arm_rotate_abs(arm_axis_at(AXIS_BASE), angle_deg, vel, acc);
}

/**
 * @brief 直线轴走到绝对位置
 * @param axis      轴定义
 * @param value_mm  目标位置(mm), 含义由各轴约定(伸展=伸长量, 升降=距底部的抬升高度)
 * @param min_mm    允许的下限(mm)
 * @param max_mm    允许的上限(mm)
 * @return 是否发送成功
 * @note 下发前先减去 home_offset_mm: 升降轴回零在顶部, 逻辑高度(从底部计)换算成
 *       "相对顶部回零点的位移"再转脉冲; 伸展轴偏移为 0, 行为不变。
 */
static bool arm_linear_to_abs(
    const ArmAxis_t *axis, float value_mm, float min_mm, float max_mm, uint16_t vel, uint16_t acc) {
    /* 目标超出软限位时钳到限位值继续执行(告警, 不再拒绝): 配合"超出可达范围按最大值走"的现场策略 */
    if (value_mm < min_mm) {
        logWarning("arm: %s %.1fmm below min %.1f, clamp to min", axis->name, (double)value_mm, (double)min_mm);
        value_mm = min_mm;
    } else if (value_mm > max_mm) {
        logWarning("arm: %s %.1fmm above max %.1f, clamp to max", axis->name, (double)value_mm, (double)max_mm);
        value_mm = max_mm;
    }

    /* 回零点可能不在逻辑 0 位(升降轴顶部回零): 用相对回零点的位移下发 */
    float relative_mm = value_mm - axis->home_offset_mm;
    float pulse_per_mm = (float)arm_axis_pulse_per_rev(axis) / axis->mm_per_rev;
    int32_t pulses = arm_mm_to_pulse(relative_mm, pulse_per_mm) * (int32_t)axis->linear_sign;
    /* 绝对位置模式: 伸展轴 0=完全收回; 升降轴 0=顶部回零点, 向下为负方向(逻辑高度从底部计) */
    return arm_cmd_position(axis->motor_id, (pulses >= 0) ? 0 : 1, vel, acc, (pulses >= 0) ? pulses : -pulses, 1);
}

/**
 * @brief 同步带伸展臂伸长到指定长度(绝对位置)
 */
bool Arm_ExtendTo(float length_mm, uint16_t vel, uint16_t acc) {
    return arm_linear_to_abs(arm_axis_at(AXIS_EXTEND), length_mm, ARM_EXTEND_MIN_MM, ARM_EXTEND_MAX_MM, vel, acc);
}

/**
 * @brief 升降臂抬升到距底部指定高度(绝对位置)
 */
bool Arm_LiftTo(float lift_mm, uint16_t vel, uint16_t acc) {
    return arm_linear_to_abs(arm_axis_at(AXIS_LIFT), lift_mm, ARM_LIFT_MIN_MM, ARM_LIFT_MAX_MM, vel, acc);
}

/**
 * @brief 读取指定电机当前角度(线性轴按逻辑零点折算, 即从底部/收回端计)
 * @note 位置来自状态轮询(S_CPOS), 需电机在线; 回零后才能与机械零位对齐。
 *       角度按 read_sign 换算为"物理方向为正"(伸展轴伸出时为正, 与 arm pos 的伸展长度一致);
 *       线性轴再加上 home_offset_mm 对应的角度, 使升降轴角度仍是"从底部计的等效角度"
 *       (顶部回零的影响被抵消, 与旧底部回零时的读数一致);
 *       物理长度用下面的 Arm_ExtendGetLength / Arm_LiftGetHeight。
 */
bool Arm_MotorGetAngle(uint8_t motor_id, float *angle_deg) {
    if (angle_deg == NULL) return false;

    const ArmAxis_t *axis = arm_axis_find(motor_id);
    if (axis == NULL) return false;

    int32_t read_pos;
    if (!arm_motor_read_pos(motor_id, &read_pos)) return false;

    float angle = (float)axis->read_sign * arm_read_pos_to_deg(axis, read_pos);
    if (axis->mm_per_rev > 0.0f) {
        /* 线性轴: 把回零点在逻辑坐标里的偏移折算成角度, 保持"从底部计"的角度读数 */
        angle += axis->home_offset_mm * (360.0f / axis->mm_per_rev);
    }
    *angle_deg = angle;
    return true;
}

/**
 * @brief 读取直线轴当前位置并换算为物理长度
 * @param axis     直线轴定义
 * @param value_mm 输出: 物理长度(mm)
 * @return true 读取成功
 * @note 读取位置按回读方向换算: value_mm = home_offset + read_sign × 输出角度/360 × mm_per_rev;
 *       升降轴回零在顶部(机械零位), home_offset=ARM_LIFT_TOP_HOME_OFFSET_MM,
 *       read_sign=+1 => 读数仍是从底部计的抬升高度(顶部=偏移、底部=0), 原标定值继续有效;
 *       伸展轴回零在完全收回端、偏移 0、read_sign=-1 => 实际伸出时读数为正(0=完全收回)。
 *       (命令方向见 linear_sign, 两者独立, 见 s_axis[] 说明)
 */
static bool arm_linear_read_mm(const ArmAxis_t *axis, float *value_mm) {
    if (axis == NULL || value_mm == NULL) return false;

    int32_t read_pos;
    if (!arm_motor_read_pos(axis->motor_id, &read_pos)) return false;

    float angle_deg = arm_read_pos_to_deg(axis, read_pos);
    *value_mm = axis->home_offset_mm + (float)axis->read_sign * angle_deg * (axis->mm_per_rev / 360.0f);
    return true;
}

/**
 * @brief 读取同步带当前伸展长度
 */
bool Arm_ExtendGetLength(float *length_mm) {
    return arm_linear_read_mm(arm_axis_at(AXIS_EXTEND), length_mm);
}

/**
 * @brief 读取升降臂当前抬升高度
 */
bool Arm_LiftGetHeight(float *height_mm) {
    return arm_linear_read_mm(arm_axis_at(AXIS_LIFT), height_mm);
}

/**
 * @brief 等待机械臂运动到位(按轴掩码判定)
 * @note 同时满足两个条件才算到位:
 *       1) 参与判定的轴位置连续稳定 ARM_SETTLE_STABLE_MS(位置不再变化);
 *       2) 已下发过目标的轴, 当前位置与目标之差不超过 ARM_SETTLE_TOL_RAW。
 *       只判"位置不再变化"会在轴还没启动(回读来自 100ms 状态轮询缓存)或中途停住时
 *       误判为到位, 导致上层在未到目标位置就开合夹爪/打印到位结果。
 *       任一电机离线/位置不可用时无法判断, 立即返回 false(由调用方决定是否继续)。
 */
static bool arm_wait_settled(uint32_t timeout_ms, uint32_t axis_mask, bool check_target) {
    int32_t last[ARM_MOTOR_NUM];
    int32_t cur[ARM_MOTOR_NUM];

    /* 取一次基准位置; 无法读取则无法判断到位 */
    for (uint8_t i = 0; i < ARM_MOTOR_NUM; i++) {
        if (!(axis_mask & (1u << i))) continue;
        if (!arm_motor_read_pos(arm_axis_at(i)->motor_id, &last[i])) return false;
    }

    uint32_t start = osKernelGetTickCount();
    uint32_t stable_since = start;

    for (;;) {
        osDelay(ARM_SETTLE_POLL_MS);

        uint32_t now = osKernelGetTickCount();
        if ((now - start) >= timeout_ms) {
            logWarning("arm: settle wait timeout (%lu ms)", (unsigned long)timeout_ms);
            return false;
        }

        bool moved = false;
        bool on_target = true;
        for (uint8_t i = 0; i < ARM_MOTOR_NUM; i++) {
            if (!(axis_mask & (1u << i))) continue;
            if (!arm_motor_read_pos(arm_axis_at(i)->motor_id, &cur[i])) return false;
            int32_t delta = cur[i] - last[i];
            if (delta < 0) delta = -delta;
            if (delta > ARM_SETTLE_TOL_RAW) moved = true;
            last[i] = cur[i];

            /* 已下发过目标的轴: 还要核对是否真的到达目标(允许 ARM_SETTLE_TOL_RAW 误差) */
            if (check_target && s_target_read_pos_valid[i]) {
                int32_t err = cur[i] - s_target_read_pos[i];
                if (err < 0) err = -err;
                if (err > ARM_SETTLE_TOL_RAW) on_target = false;
            }
        }

        if (moved) {
            stable_since = now;
        } else if (on_target && (now - stable_since) >= ARM_SETTLE_STABLE_MS) {
            return true;
        }
    }
}

bool Arm_WaitSettled(uint32_t timeout_ms) {
    uint32_t all_mask = (1u << AXIS_BASE) | (1u << AXIS_LIFT) | (1u << AXIS_EXTEND);
    return arm_wait_settled(timeout_ms, all_mask, true);
}

bool Arm_WaitSettledHorizontal(uint32_t timeout_ms) {
    /* 水平方向 = 基座旋转 + 同步带伸缩; 升降轴不参与判定 */
    uint32_t horizontal_mask = (1u << AXIS_BASE) | (1u << AXIS_EXTEND);
    return arm_wait_settled(timeout_ms, horizontal_mask, true);
}

bool Arm_WaitSettledLift(uint32_t timeout_ms) {
    /* 升降轴单独到位判定: 上升动作先走升降, 到位后再旋转+伸展 */
    uint32_t lift_mask = (1u << AXIS_LIFT);
    return arm_wait_settled(timeout_ms, lift_mask, true);
}

/*============================ 夹爪(舵机)接口 ============================*/

/**
 * @brief 等待夹爪舵机动作到位(单圈角度回读判定, 替代固定延时)
 * @param target_deg 目标角度(°), 与 Servo_SetServoAngle 使用同一坐标
 * @return true=已到达目标角度或已夹稳/被挡停; false=到位等待超时
 * @note 与原固定等待的区别:
 *       1) 正常开合: 回读角度连续两次在 ARM_GRIPPER_SETTLE_TOL_DEG 内 => 到达目标;
 *       2) 夹住物料/被挡住时到不了目标角度: 检测到"已开始运动且角度不再变化" => 视为到位(夹稳),
 *          否则会因夹到物料就超时失败;
 *       3) 舵机不支持角度回读时回退到 ARM_GRIPPER_SETTLE_MS 固定等待。
 */
static bool arm_gripper_wait_arrive(float target_deg) {
    uint32_t start_tick = osKernelGetTickCount();
    uint8_t hit_target = 0; // 连续到达目标角度的次数
    uint8_t hit_still = 0; // 连续"角度不再变化"的次数
    uint8_t read_fail = 0; // 连续读取失败次数
    bool got_read = false; // 是否成功读到过角度
    bool moved = false; // 是否检测到夹爪已开始动作
    float start_deg = 0.0f; // 动作前的起始角度
    float last_actual_deg = 0.0f; // 最近一次读到的角度

    /* 先读一次起始角度: 用于区分"夹爪已开始动"与"命令还没生效" */
    if (Servo_QueryServoAngle(ARM_GRIPPER_SERVO_ID, &start_deg) == SERVO_STATUS_SUCCESS) {
        got_read = true;
        last_actual_deg = start_deg;
    }

    while ((osKernelGetTickCount() - start_tick) < ARM_GRIPPER_SETTLE_TIMEOUT_MS) {
        float actual_deg = 0.0f;
        if (Servo_QueryServoAngle(ARM_GRIPPER_SERVO_ID, &actual_deg) == SERVO_STATUS_SUCCESS) {
            got_read = true;
            read_fail = 0;

            float err = actual_deg - target_deg;
            if (err < 0.0f) err = -err;
            if (err <= ARM_GRIPPER_SETTLE_TOL_DEG) {
                if (++hit_target >= 2) {
                    return true;
                }
            } else {
                hit_target = 0;
            }

            /* 夹住物料/被挡时到不了目标: 已开始运动且角度不再变化 => 视为夹稳到位 */
            if (!moved) {
                float moved_err = actual_deg - start_deg;
                if (moved_err < 0.0f) moved_err = -moved_err;
                if (moved_err >= ARM_GRIPPER_MOVE_START_DEG) {
                    moved = true;
                }
            }
            if (moved) {
                float still_err = actual_deg - last_actual_deg;
                if (still_err < 0.0f) still_err = -still_err;
                if (still_err <= ARM_GRIPPER_STILL_DEG) {
                    if (++hit_still >= 2) {
                        return true;
                    }
                } else {
                    hit_still = 0;
                }
            }

            last_actual_deg = actual_deg;
        } else {
            read_fail++;
            hit_target = 0;
            hit_still = 0;
            /* 连续读不到角度 => 舵机不支持角度回读: 回退固定等待, 保证夹爪动作仍能继续 */
            if (!got_read && read_fail >= ARM_GRIPPER_READ_FAIL_MAX) {
                logWarning("arm: gripper angle read unavailable, fallback to fixed delay %ums",
                           (unsigned)ARM_GRIPPER_SETTLE_MS);
                osDelay(ARM_GRIPPER_SETTLE_MS);
                return true;
            }
        }
        osDelay(ARM_GRIPPER_SETTLE_POLL_MS);
    }

    logError(
        "arm: gripper arrive timeout (target %.1f deg, actual %.1f deg)", (double)target_deg, (double)last_actual_deg);
    return false;
}

/**
 * @brief 设置夹爪角度(阻塞到角度回读确认到位)
 * @param angle_deg 目标角度(°)
 * @return true=已到位(或夹稳/被挡停); false=命令下发失败/到位等待超时
 * @note 夹爪为总线舵机(ARM_GRIPPER_SERVO_ID=0), 使用单圈角度命令;
 *       下发后按单圈角度回读确认到位(容差/超时见 arm_config.h), 不再是固定延时。
 */
/** 最近一次 Arm_GripperStartAngle 的目标角度(°)/是否有效: 供"先下张爪命令, 稍后再确认"使用 */
static float s_gripper_target_deg = 0.0f;
static bool s_gripper_target_valid = false;

bool Arm_GripperStartAngle(int angle_deg) {
    float target_deg = (float)angle_deg;

    if (Servo_SetServoAngle(ARM_GRIPPER_SERVO_ID, target_deg, 0, 0) != SERVO_STATUS_SUCCESS) {
        logError("arm: gripper angle %d command failed", angle_deg);
        s_gripper_target_valid = false;
        return false;
    }

    s_gripper_target_deg = target_deg;
    s_gripper_target_valid = true;
    return true;
}

bool Arm_GripperWaitArrive(void) {
    if (!s_gripper_target_valid) {
        logError("arm: gripper wait arrive without start");
        return false;
    }
    return arm_gripper_wait_arrive(s_gripper_target_deg);
}

bool Arm_GripperSetAngle(int angle_deg) {
    if (!Arm_GripperStartAngle(angle_deg)) {
        return false;
    }
    return arm_gripper_wait_arrive((float)angle_deg);
}

/*============================ 物料盘(总线舵机)接口 ============================*/

/**
 * 物料盘各盘位舵机角度表(°): 三个盘位分别实测填写(arm_config.h 的 ARM_PAD_SLOTn_ANGLE_DEG),
 * 不再用"基准角 + 格距"固定偏移推算(实测各盘位角度不等距)。
 */
#if ARM_PAD_SLOT_NUM != 3
#error "物料盘盘位角度表按 ARM_PAD_SLOT_NUM=3 定义, 修改盘位数量时需同步更新本表与 arm_config.h"
#endif
static const float s_pad_slot_angle_deg[ARM_PAD_SLOT_NUM] = {
    ARM_PAD_SLOT1_ANGLE_DEG,
    ARM_PAD_SLOT2_ANGLE_DEG,
    ARM_PAD_SLOT3_ANGLE_DEG,
};

/** 最近一次 Arm_PadSetSlotStart 的目标角度(°)/是否有效: 供"先启动换格, 稍后再校验到位"使用 */
static float s_pad_target_deg = 0.0f;
static bool s_pad_target_valid = false;

/**
 * @brief 等待物料盘舵机移动到目标角度(多圈角度回读判定, 替代固定延时)
 * @param target_deg 目标角度(°), 与 Servo_SetServoAngleMTurnByVelocity 使用同一坐标
 * @param timeout_ms 最长等待时间(ms)
 * @return true=回读角度连续两次在容差内(或舵机不支持回读、已回退固定等待); false=到位等待超时
 * @note 物料盘相邻格位差 270°, 换格实际约需 1s+, 原固定等待 300ms 容易"还没到位就下降夹取";
 *       这里轮询舵机多圈角度: 连续两次在 ARM_PAD_SETTLE_TOL_DEG 内才算稳定, 避免运动途中
 *       恰好经过容差范围被误判到位。
 */
static bool arm_pad_wait_arrive(float target_deg, uint32_t timeout_ms) {
    uint32_t start_tick = osKernelGetTickCount();
    uint8_t hit = 0; // 连续在容差内的次数
    uint8_t read_fail = 0; // 连续读取失败次数
    bool got_read = false; // 是否成功读到过角度
    float last_actual_deg = 0.0f; // 最近一次读到的角度(超时日志用)

    while ((osKernelGetTickCount() - start_tick) < timeout_ms) {
        float actual_deg = 0.0f;
        if (Servo_QueryServoAngleMTurn(ARM_PAD_SERVO_ID, &actual_deg) == SERVO_STATUS_SUCCESS) {
            got_read = true;
            read_fail = 0;
            last_actual_deg = actual_deg;

            float err = actual_deg - target_deg;
            if (err < 0.0f) err = -err;
            if (err <= ARM_PAD_SETTLE_TOL_DEG) {
                if (++hit >= 2) {
                    return true;
                }
            } else {
                hit = 0;
            }
        } else {
            read_fail++;
            hit = 0;
            /* 连续读不到角度 => 舵机固件不支持多圈角度回读: 回退固定等待, 保证换格仍能继续 */
            if (!got_read && read_fail >= ARM_PAD_READ_FAIL_MAX) {
                logWarning("arm: pad angle read unavailable, fallback to fixed delay %ums",
                           (unsigned)ARM_PAD_SETTLE_MS);
                osDelay(ARM_PAD_SETTLE_MS);
                return true;
            }
        }
        osDelay(ARM_PAD_SETTLE_POLL_MS);
    }

    if (got_read) {
        logError(
            "arm: pad arrive timeout (target %.1f deg, actual %.1f deg)", (double)target_deg, (double)last_actual_deg);
    } else {
        logError("arm: pad arrive timeout (target %.1f deg, no angle read)", (double)target_deg);
    }
    return false;
}

/**
 * @brief 启动物料盘换格(非阻塞): 只下发角度命令, 到位由 Arm_PadWaitArrive 校验
 * @param slot 格位号(1 ~ ARM_PAD_SLOT_NUM), 越界时钳制到有效范围
 * @return true=换格命令已下发; false=命令下发失败
 * @note 物料盘转动较慢(盘位间约 270°), 先下发命令让物料盘转动, 同时机械臂可以继续执行
 *       移向物料盘上方等动作, 最后在下降取放前调用 Arm_PadWaitArrive 校验到位, 缩短整段动作时间。
 *       各盘位角度来自 arm_config.h 的 ARM_PAD_SLOTn_ANGLE_DEG(分别实测), 不做固定偏移推算。
 */
bool Arm_PadSetSlotStart(uint8_t slot) {
    if (!s_servo_set_origin) {
        Servo_StopOnControlMode(ARM_PAD_SERVO_ID, 0, 0);
        Servo_ResetServoMTurnAngle(ARM_PAD_SERVO_ID);
        Servo_SetOriginPoint(ARM_PAD_SERVO_ID);
        s_servo_set_origin = true;
    }

    if (slot < 1 || slot > ARM_PAD_SLOT_NUM) {
        logWarning("arm: pad slot %d out of range [1, %d], clamp", (int)slot, ARM_PAD_SLOT_NUM);
        slot = (slot < 1) ? 1 : ARM_PAD_SLOT_NUM;
    }

    /* 每个盘位角度单独配置(实测填入), 不再按基准角+格距推算 */
    float angle_deg = s_pad_slot_angle_deg[slot - 1];
    if (Servo_SetServoAngleMTurnByVelocity(ARM_PAD_SERVO_ID, angle_deg, 255, 0, 0, 0) != SERVO_STATUS_SUCCESS) {
        logError("arm: pad slot %u move command failed", (unsigned)slot);
        s_pad_target_valid = false;
        return false;
    }

    s_pad_target_deg = angle_deg;
    s_pad_target_valid = true;
    return true;
}

/**
 * @brief 校验物料盘是否已到达最近一次 Arm_PadSetSlotStart 指定的格位
 * @param timeout_ms 最长等待时间(ms)
 * @return true=到位(或回读不可用已回退固定等待); false=未调用 Arm_PadSetSlotStart/等待超时
 */
bool Arm_PadWaitArrive(uint32_t timeout_ms) {
    if (!s_pad_target_valid) {
        logError("arm: pad wait arrive without start");
        return false;
    }
    return arm_pad_wait_arrive(s_pad_target_deg, timeout_ms);
}

/**
 * @brief 物料盘换格: 把指定格位移动到夹爪固定的取放位置(阻塞到角度回读确认到位)
 * @param slot 格位号(1 ~ ARM_PAD_SLOT_NUM), 越界时钳制到有效范围
 * @return true=舵机已到目标格位; false=换格命令下发失败/到位等待超时
 * @note 物料盘由总线舵机驱动做直线运动, 各盘位角度在 arm_config.h 中分别实测填写
 *       (ARM_PAD_SLOTn_ANGLE_DEG, 不再用基准角+固定格距推算);
 *       等价于 Arm_PadSetSlotStart + Arm_PadWaitArrive; 动作层想"边转边动臂"时请直接用这两个分离接口。
 */
bool Arm_PadSetSlot(uint8_t slot) {
    if (!Arm_PadSetSlotStart(slot)) {
        return false;
    }
    return Arm_PadWaitArrive(ARM_PAD_SETTLE_TIMEOUT_MS);
}

/*============================ 相机翻转舵机接口 ============================*/

/**
 * @brief 设置相机翻转舵机角度
 * @note 标定/颜色识别时翻到标定角(标定结束保持, 不自动恢复);
 *       整臂/伸展轴回零前与调试命令会翻到恢复角;
 *       下发后阻塞等待 ARM_FLIP_SETTLE_MS, 确保 RPI_Cal 拍摄前相机已到位。
 */
void Arm_FlipSetAngle(float angle_deg) {
#if ARM_FLIP_ENABLE
    Servo_Port_SetAngle(ARM_FLIP_SERVO_ID, angle_deg);
    osDelay(ARM_FLIP_SETTLE_MS);
#else
    (void)angle_deg;
#endif
}
