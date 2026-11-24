/**
 * @file motion_control.c
 * @author Pcb-yun (pcbyinyun@163.com)
 * @brief 运动控制模块源文件
 * 可调参数见 chassis_config.h。
 */

#include "motion_control.h"
#include "chassis_config.h"
#include "zdt_v5_cfg.h"
#include "zdt_v5_cmd.h"
#include "zdt_v5_port.h"
#include "zdt_v5_engine.h"
#include "nav_math.h"
#include "kinematics.h"
#include "log.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include <math.h>
#include <string.h>

/* 轮位置模式按 Emm 固件实现: 位置命令单位 3200 脉冲/圈, 实时位置回读 65536/圈。
 * X 固件的位置命令单位(0.1°)与回读单位都不同, 未做换算前不允许启用, 避免静默走错。
 * 切换到 X 固件时的规避方式: 关闭 MOTOR_POS_MODE(仅台架不用位置模式), 或先按手册补齐
 * 0.1°/0.1RPM 命令单位与 X 固件回读单位的换算, 再放开本保护。 */
#if MOTOR_POS_MODE && CURRENT_FIRMWARE != FIRMWARE_EMM
#error "chassis wheel position mode is implemented for FIRMWARE_EMM only (cmd 3200 pulses/rev, read 65536 units/rev)"
#endif

/**
 * @brief 运动控制结构体
 */
typedef struct {
    float linear_speed; // 线速度 (cm/s)
    float yaw_speed; // 偏摆速度 (deg/s)
    float car_acc; // 加速度物理量 (cm/s²), 下发时按固件折算
    float car_dec; // 减速度物理量 (cm/s²), 下发时按固件折算

    /* ===== 运动 setpoint 限频/去重状态（见 chassis_config.h 的 CHASSIS_MOTION_SETPOINT_*） =====
     * 速度模式是绝对值指令: 电机收到后保持该速度, 重复下发相同/几乎相同的值没有意义,
     * 只会占用电机总线（多机帧 403us + 同步帧 44us + 两个 1ms 帧间隔）。 */
    float sp_x; // 上一次真正投递的机体系 X 速度 (cm/s)
    float sp_y; // 上一次真正投递的机体系 Y 速度 (cm/s)
    float sp_w; // 上一次真正投递的偏航角速度 (deg/s)
    uint32_t sp_tick; // 上一次投递时刻 (ms)
    bool sp_valid; // 是否有"上一次投递"(停止/位置模式后作废, 保证恢复运动时立即下发)
    uint32_t sp_posted; // 实际投递 setpoint 次数
    uint32_t sp_skipped; // 因"变化不足阈值且未到保持刷新时间"跳过的次数
} MotionControl_t;

static MotionControl_t *s_motion = NULL;

/**
 * @brief 小车线加速度 (cm/s²) 转 电机转速加速度 (RPM/s)
 * @param car_acc 小车线加速度 (cm/s²)
 * @return 电机加速度 (RPM/s)
 */
static float acc_car_to_motor(float car_acc) {
    return car_acc * 60.0f / (2.0f * M_PI * WHEEL_RADIUS);
}

/**
 * @brief 线加速度 (cm/s²) 换算成固件速度/位置模式的加速度参数
 * @param car_acc 小车线加速度 (cm/s²)
 * @return 固件加速度参数
 *
 * X 固件：参数单位 RPM/s，直接填换算值；
 * Emm 固件：参数是 0-255 档位(非线性)，手册公式为"每 (256-档位)×50us 加速 1RPM"，
 *           即 RPM/s = 20000/(256-档位)，反解 档位 = 256 - 20000/RPM_s；
 *           0 档最慢(约 78RPM/s)、255 档最快。
 * 注意：曾经把 RPM/s 直接强转 uint16 当档位下发(Emm 只取低 8 位)，会截断成错误档位
 *       (例如导航下发的 0 变成 0 档最慢、默认 70cm/s² 换算 183RPM/s 恰好凑巧可用)，
 *       电机速度环跟不动导航指令 -> 位置过冲、边走边摆。必须按固件公式换算。
 */
static uint16_t car_acc_to_motor_param(float car_acc) {
    float rpm_s = acc_car_to_motor(car_acc);
#if CURRENT_FIRMWARE == FIRMWARE_EMM
    if (rpm_s <= 0.0f) return 0; // 0 档 = 最慢加速
    if (rpm_s >= 20000.0f) return 255;
    float gear = 256.0f - 20000.0f / rpm_s;
    if (gear < 0.0f) gear = 0.0f;
    if (gear > 255.0f) gear = 255.0f;
    return (uint16_t)(gear + 0.5f);
#else
    if (rpm_s < 0.0f) rpm_s = 0.0f;
    if (rpm_s > 65535.0f) rpm_s = 65535.0f;
    return (uint16_t)(rpm_s + 0.5f);
#endif
}

/**
 * @brief 初始化运动控制模块
 * @return 初始化结果
 */
bool MotionControl_Init(void) {
    if (s_motion != NULL) return true; // 已初始化, 避免重复申请内存

    s_motion = pvPortMalloc(sizeof(MotionControl_t));
    if (s_motion == NULL) return false;
    memset(s_motion, 0, sizeof(MotionControl_t));

    s_motion->linear_speed = 80.0f;
    s_motion->yaw_speed = 140.0f;
    s_motion->car_acc = 70.0f; // cm/s²
    s_motion->car_dec = 100.0f; // cm/s²

    return true;
}

#if MOTOR_VELOCITY_MODE || MOTOR_POS_MODE
/**
 * @brief 轮子索引
 */
typedef enum {
    WHEEL_FRONT_LEFT = 0, // 前左轮
    WHEEL_FRONT_RIGHT, // 前右轮
    WHEEL_BACK_LEFT, // 后左轮
    WHEEL_BACK_RIGHT, // 后右轮
    WHEEL_COUNT // 轮子总数
} WheelIndex_t;

/**
 * @brief 轮子电机描述
 */
typedef struct {
    uint8_t motor_id; // 电机总线 ID
    bool reversed; // 电机安装方向是否与轮子正方向相反
} WheelMotor_t;

/**
 * @brief 轮子电机描述表
 * @note 右侧电机镜像安装: 轮子正方向对应电机 CCW(dir=1), 左侧对应 CW(dir=0)
 */
static const WheelMotor_t s_wheel_motors[WHEEL_COUNT] = {
    [WHEEL_FRONT_LEFT] = {MOTOR_FRONT_LEFT, false},
    [WHEEL_FRONT_RIGHT] = {MOTOR_FRONT_RIGHT, true},
    [WHEEL_BACK_LEFT] = {MOTOR_BACK_LEFT, false},
    [WHEEL_BACK_RIGHT] = {MOTOR_BACK_RIGHT, true},
};

/**
 * @brief 展开轮子解算结果
 * @param wheels 运动学解算结果
 * @param values 输出数组, 单位: 角速度(rad/s) 或 角位移(rad)
 * @note Wheel_t 成员顺序为 fl/fr/rr/rl, 不能直接按索引访问
 */
static void wheel_values(const Wheel_t *wheels, float values[WHEEL_COUNT]) {
    values[WHEEL_FRONT_LEFT] = wheels->fl;
    values[WHEEL_FRONT_RIGHT] = wheels->fr;
    values[WHEEL_BACK_LEFT] = wheels->rl;
    values[WHEEL_BACK_RIGHT] = wheels->rr;
}

/**
 * @brief 轮子目标值方向转电机方向位
 * @param index 轮子索引
 * @param positive 轮子目标值是否非负
 * @return 电机方向: 0=CW(正转), 1=CCW(反转)
 */
static uint8_t wheel_dir(WheelIndex_t index, bool positive) {
    return (positive == s_wheel_motors[index].reversed) ? 1 : 0;
}

#if MOTOR_VELOCITY_MODE
/**
 * @brief 轮子角速度 (rad/s) 转电机转速 (RPM)
 * @param rad_s 轮子角速度 (rad/s)
 * @return 电机转速 (RPM), 取绝对值
 */
static uint16_t wheel_speed_to_rpm(float rad_s) {
    return (uint16_t)fabsf(rad_s * 60.0f / (2.0f * M_PI));
}
#endif /* MOTOR_VELOCITY_MODE */

#if MOTOR_POS_MODE
/**
 * @brief 轮子角位移 (rad) 转电机脉冲数
 * @param rad 轮子角位移 (rad)
 * @return 电机脉冲数, 带符号(四舍五入到最近脉冲, 避免小位移被截断成 0)
 */
static int32_t wheel_angle_to_pulses(float rad) {
    float pulses = rad * MOTOR_PULSES_PER_REV / (2.0f * M_PI);
    return (int32_t)(pulses >= 0.0f ? pulses + 0.5f : pulses - 0.5f);
}

/**
 * @brief 取脉冲数绝对值
 * @param pulses 带符号脉冲数
 * @return 脉冲数绝对值
 */
static uint32_t wheel_pulses_abs(int32_t pulses) {
    return (uint32_t)(pulses >= 0 ? pulses : -pulses);
}

/**
 * @brief 位置命令脉冲数 → 实时位置回读单位数
 * @param pulses 位置命令脉冲数(绝对值)
 * @return 实时位置回读单位数
 * @note  位置命令每圈 MOTOR_PULSES_PER_REV 脉冲, 实时位置回读每圈
 *        MOTOR_READ_POS_UNITS_PER_REV 单位(Emm 固件 65536), 两者不同, 到位判定必须换算后比较。
 */
static uint32_t wheel_pulses_to_read_units(uint32_t pulses) {
    return (uint32_t)(((uint64_t)pulses * MOTOR_READ_POS_UNITS_PER_REV + (MOTOR_PULSES_PER_REV / 2)) /
                      MOTOR_PULSES_PER_REV);
}

/**
 * @brief 位置模式移动状态(供 MotionControl_PositionArrived 判断到位)
 */
typedef struct {
    bool active; // 是否有进行中的位置模式移动
    uint32_t start_tick; // 命令下发时刻(无位置反馈时兜底计时)
    uint32_t last_change_tick; // 最近一次检测到任一电机位置变化的时刻(判断停稳)
    int32_t start_pos[WHEEL_COUNT]; // 下发前各轮实时位置(回读单位)
    int32_t last_pos[WHEEL_COUNT]; // 上一次轮询到的各轮实时位置(回读单位)
    uint32_t target_read[WHEEL_COUNT]; // 各轮目标位移绝对值(回读单位, 已由命令脉冲换算)
    bool pos_valid[WHEEL_COUNT]; // 各轮起点位置是否可信(电机在线)
} MotionPosMove_t;

static MotionPosMove_t s_pos_move;

/**
 * @brief 读取轮子电机实时位置(状态轮询缓存, 回读单位)
 * @param index 轮子索引
 * @param pos   输出: 实时位置(回读单位, Emm 固件 65536/圈)
 * @return true=位置可信(电机最近有应答); false=电机不在线/位置未更新
 */
static bool wheel_real_pos(WheelIndex_t index, int32_t *pos) {
    extern MotorStatus_t *g_motor_status[ZDT_STEP_NUM];
    uint8_t motor_id = s_wheel_motors[index].motor_id;

    if (pos == NULL || motor_id < 1 || motor_id > ZDT_STEP_NUM) return false;
    if (Motor_Rx_Age_Ms(motor_id) > MOTOR_ONLINE_TIMEOUT_MS) return false; // 最近无应答, 位置不可信

#if MOTOR_STATUS_REAL_POS
    MotorStatus_t *status = g_motor_status[motor_id - 1];
    if (status == NULL) return false;
    *pos = status->pos;
    return true;
#else
    (void)pos;
    return false;
#endif
}
#endif /* MOTOR_POS_MODE */

#if MOTOR_MULTI_CMD
/**
 * @brief 把一个打包好的多机指令封装成命令结构体
 * @param multi_cmd 多机指令缓冲区
 * @param cmd       输出: 电机命令
 */
static void build_multi_command(const ZDT_V5_Multi_Cmd_t *multi_cmd, MotorCmd_t *cmd) {
    memset(cmd, 0, sizeof(*cmd));
    cmd->op_type = OP_CONTROL;
    cmd->motor_id = 0;
    cmd->type.ctrl.type = CTRL_MULTI;
    cmd->type.ctrl.p.multi.cmd = *multi_cmd;
}

/**
 * @brief 发送打包好的多机指令（普通队列: 位置模式等一次性命令）
 * @param multi_cmd 多机指令缓冲区
 * @return 是否成功放入电机命令队列
 */
static bool send_multi_command(ZDT_V5_Multi_Cmd_t *multi_cmd) {
    MotorCmd_t cmd;
    build_multi_command(multi_cmd, &cmd);
    return Motor_Send_Cmd(&cmd);
}
#endif /* MOTOR_MULTI_CMD */

#if MOTOR_VELOCITY_MODE
/**
 * @brief 发送轮子速度命令（运动 setpoint 通道, 永不阻塞）
 * @param wheels 四个轮子的角速度 (rad/s)
 *
 * @note 为什么走 Motor_Send_Motion_Setpoint() 而不是 Motor_Send_Cmd():
 *   旧实现把"每个控制周期算出的速度"塞进深度 64 的共享电机命令 FIFO，驱动层每条命令后
 *   osDelay(2) ⇒ 队列服务率约 480 条/s，而导航(400条/s)+状态轮询(280条/s)需求约 680 条/s，
 *   稳态队列必满 ⇒ 速度指令实际执行时刻比算出来的时刻晚 100ms 级，闭环等效为纯延迟。
 *   现在运动 setpoint 走"最新值覆盖"邮箱：这里永不阻塞；驱动层只发最新一件，过期 setpoint
 *   直接丢弃（`step bus` 可看丢弃数/排队时延）。速度是绝对值而非增量，丢旧值不影响正确性。
 */
static void send_wheel_velocity_commands(const Wheel_t *wheels) {
    if (s_motion == NULL) return;

    float values[WHEEL_COUNT];
    wheel_values(wheels, values);

    /* 各轮命令是否"缓存等待同步触发"：true = 由驱动层补一帧 0xFF66 让四轮严格同时启动。
     * CHASSIS_MOTION_SYNC_TRIGGER=0 时改为各驱动器收到即执行（少一帧, 时延再降 ~1ms），
     * 默认 1 保持原行为，改动需台架/实车确认起步直线性。
     * 同时要求固件侧 MOTOR_SYNC_TRIGGER=1：否则命令被缓存后没有同步帧触发，车根本不会动。 */
    const bool wait_sync = (CHASSIS_MOTION_SYNC_TRIGGER != 0) && (MOTOR_SYNC_TRIGGER != 0);

#if MOTOR_MULTI_CMD
    MotorMulti_t multi = {.type = MULTI_VEL};
    ZDT_V5_Multi_Cmd_t multi_cmd = {.buf_size = MOTOR_MULTI_BUF_SIZE};
    ZDT_V5_Multi_Reset(&multi_cmd);

    multi.p.vel.acc = car_acc_to_motor_param(s_motion->car_acc);
    multi.p.vel.snF = wait_sync;

    for (WheelIndex_t i = WHEEL_FRONT_LEFT; i < WHEEL_COUNT; i++) {
        multi.p.vel.dir = wheel_dir(i, values[i] >= 0.0f);
        multi.p.vel.vel = wheel_speed_to_rpm(values[i]);
        ZDT_V5_Process_Multi_Cmd(s_wheel_motors[i].motor_id, &multi, &multi_cmd);
    }

    MotorCmd_t cmd;
    build_multi_command(&multi_cmd, &cmd);
    (void)Motor_Send_Motion_Setpoint(&cmd, 1, wait_sync);
#else
    /* 未启用多机打包：4 条单机速度命令作为一个 setpoint 批次一起投递 */
    MotorCmd_t cmds[WHEEL_COUNT];
    MotorCtrl_t ctrl = {.type = CTRL_VEL};
    ctrl.p.vel.acc = car_acc_to_motor_param(s_motion->car_acc);
    ctrl.p.vel.sync = wait_sync;

    for (WheelIndex_t i = WHEEL_FRONT_LEFT; i < WHEEL_COUNT; i++) {
        memset(&cmds[i], 0, sizeof(cmds[i]));
        cmds[i].op_type = OP_CONTROL;
        cmds[i].motor_id = s_wheel_motors[i].motor_id;
        ctrl.p.vel.dir = wheel_dir(i, values[i] >= 0.0f);
        ctrl.p.vel.vel = wheel_speed_to_rpm(values[i]);
        cmds[i].type.ctrl = ctrl;
    }
    (void)Motor_Send_Motion_Setpoint(cmds, WHEEL_COUNT, wait_sync);
#endif
}
#endif /* MOTOR_VELOCITY_MODE */

#if MOTOR_POS_MODE
/**
 * @brief 位置模式各轮运行速度上限 (RPM)
 * @return 电机转速上限(未初始化时返回 0)
 * @note  位置模式四轮共用同一个速度上限: 取"线速度 + 旋转分量"折算最大轮心线速度。
 *        原来只按线速度算, 纯原地旋转会被限得很慢; 加上偏摆分量后纯旋转也能按设定角速度走。
 */
static uint16_t wheel_position_max_rpm(void) {
    if (s_motion == NULL) return 0;

    const float r_wheel = WHEEL_BASE_LENGTH / 2.0f + WHEEL_BASE_WIDTH / 2.0f; // 旋转时最大轮心力臂 (cm)
    float wheel_cm_s = fabsf(s_motion->linear_speed) + fabsf(s_motion->yaw_speed) * (float)M_PI / 180.0f * r_wheel;
    float rpm = wheel_cm_s * 60.0f / (2.0f * M_PI * WHEEL_RADIUS);
    if (rpm < 1.0f) rpm = 1.0f;
    if (rpm > 3000.0f) rpm = 3000.0f; // 防御: 参数异常时不向电机下发离谱转速
    return (uint16_t)(rpm + 0.5f);
}

/**
 * @brief 发送轮子位置命令
 * @param pulses 四个轮子的目标位移 (电机脉冲, 带符号)
 * @return true=命令已全部放入队列; false=未初始化/发送失败
 */
static bool send_wheel_position_commands(const int32_t pulses[WHEEL_COUNT]) {
    if (s_motion == NULL || pulses == NULL) return false;

    uint16_t max_rpm = wheel_position_max_rpm();
    bool ok = true;

#if MOTOR_MULTI_CMD
    MotorMulti_t multi = {.type = MULTI_POS};
    ZDT_V5_Multi_Cmd_t multi_cmd = {.buf_size = MOTOR_MULTI_BUF_SIZE};
    ZDT_V5_Multi_Reset(&multi_cmd);

    multi.p.pos.acc = car_acc_to_motor_param(s_motion->car_acc);
    multi.p.pos.vel = max_rpm;
    /* 运动模式 2 = 相对当前位置: 保证每次下发的脉冲都是相对"当前实测位置"的位移。
     * 不能用 0(相对上一目标): 速度模式导航后"上一位置目标"已过时, 会一下跳到旧目标附近。 */
    multi.p.pos.mode = 2;
    /* 多机命令(0xAA)是"各电机子命令打包成帧、收到即同时执行"; 位置模式子帧里的这个标志
     * 在 Emm 固件下表示"是否返回到位命令(0x9F)"(见通讯指令手册 3.1), 不是缓存等待同步。
     * 置 false: 不返回 0x9F 到位帧(避免引擎把 0x9F 当未知应答告警), 也不需要再发 0xFF66
     * 同步触发——那是"单机缓存命令 + 同步触发"的用法。 */
    multi.p.pos.rsp = false;
#if CURRENT_FIRMWARE == FIRMWARE_X
    multi.p.pos.dec = car_acc_to_motor_param(s_motion->car_dec);
#endif

    for (WheelIndex_t i = WHEEL_FRONT_LEFT; i < WHEEL_COUNT; i++) {
        multi.p.pos.dir = wheel_dir(i, pulses[i] >= 0);
        multi.p.pos.target = (int32_t)wheel_pulses_abs(pulses[i]);
        ZDT_V5_Process_Multi_Cmd(s_wheel_motors[i].motor_id, &multi, &multi_cmd);
    }

    ok = send_multi_command(&multi_cmd);
#else
    MotorCmd_t cmd = {.op_type = OP_CONTROL};
    MotorCtrl_t ctrl = {.type = CTRL_POS};
    ctrl.p.pos.acc = car_acc_to_motor_param(s_motion->car_acc);
    ctrl.p.pos.vel = max_rpm;
    ctrl.p.pos.mode = 2; // 相对当前位置(见上一段说明)
    ctrl.p.pos.sync = true;
#if CURRENT_FIRMWARE == FIRMWARE_X
    ctrl.p.pos.dec = car_acc_to_motor_param(s_motion->car_dec);
#endif

    for (WheelIndex_t i = WHEEL_FRONT_LEFT; i < WHEEL_COUNT; i++) {
        ctrl.p.pos.dir = wheel_dir(i, pulses[i] >= 0);
        ctrl.p.pos.target = (int32_t)wheel_pulses_abs(pulses[i]);
        cmd.motor_id = s_wheel_motors[i].motor_id;
        cmd.type.ctrl = ctrl;
        if (!Motor_Send_Cmd(&cmd)) ok = false;
    }

    if (ok) ok = send_sync_trigger();
    if (!ok) {
        /* 单机逐轮下发: 前面可能已有部分轮子收到命令, 失败时停车兜底, 不要留下部分运动 */
        MotionControl_Stop();
    }
#endif

    return ok;
}
#endif /* MOTOR_POS_MODE */
#endif /* MOTOR_VELOCITY_MODE || MOTOR_POS_MODE */

#if MOTOR_VELOCITY_MODE
/**
 * @brief 作废 setpoint 去重状态（停止/位置模式后调用，保证下一条速度命令一定下发）
 */
static void motion_setpoint_invalidate(void) {
    if (s_motion == NULL) return;
    s_motion->sp_valid = false;
}

/**
 * @brief 供上层（shell 手动调试）强制下一次速度命令一定下发
 * @note  正常导航/任务流程不需要调用；`car vel` 这类一次性手动命令调用后行为与修复前一致。
 */
void MotionControl_ResetSetpointFilter(void) {
    motion_setpoint_invalidate();
}

/**
 * @brief 判断本次速度指令是否需要真正投递（总线带宽有限，稳态巡航去重）
 * @param x   X 速度 (cm/s)
 * @param y   Y 速度 (cm/s)
 * @param yaw 偏航角速度 (deg/s)
 * @return true=本次需要投递; false=按限频策略本次可跳过
 * @note  三级策略（阈值/周期见 chassis_config.h 的 CHASSIS_MOTION_SETPOINT_*）：
 *        ① 大幅阶跃 → 立即投递（起步/停车/换向/手动调试，导航下一周期生效）；
 *        ② 常规变化 → 最快每 MIN_INTERVAL_MS 一次（默认 10ms，≤100Hz）；
 *        ③ 几乎没变 → 最多每 HOLD_MS 刷新一次（默认 20ms，稳态 50Hz），缓慢漂移不会丢。
 *        跳过的只是"重复值"：Emm 速度模式是绝对值指令，电机保持上一次速度；心跳由状态轮询维持。
 */
static bool motion_setpoint_due(float x, float y, float yaw) {
    if (s_motion == NULL) return false;

    uint32_t now = osKernelGetTickCount();
    bool due = true;

    if (s_motion->sp_valid) {
        float dv = fabsf(x - s_motion->sp_x);
        if (fabsf(y - s_motion->sp_y) > dv) dv = fabsf(y - s_motion->sp_y);
        float dw = fabsf(yaw - s_motion->sp_w);

        if ((dv <= CHASSIS_MOTION_SETPOINT_DV_CM_S) && (dw <= CHASSIS_MOTION_SETPOINT_DW_DEG_S)) {
            /* ③ 指令几乎没变：最多每 HOLD_MS 刷新一次（稳态 50Hz） */
            due = ((now - s_motion->sp_tick) >= (uint32_t)CHASSIS_MOTION_SETPOINT_HOLD_MS);
        } else if ((dv > CHASSIS_MOTION_SETPOINT_URGENT_DV_CM_S) || (dw > CHASSIS_MOTION_SETPOINT_URGENT_DW_DEG_S)) {
            /* ① 大幅阶跃（起步/停车/换向/手动调试）：立即投递，不受最小间隔限制 */
            due = true;
        } else {
            /* ② 常规变化（加减速/接近目标）：最快每 MIN_INTERVAL_MS 一次（默认 10ms → ≤100Hz） */
#if CHASSIS_MOTION_SETPOINT_MIN_INTERVAL_MS > 0
            due = ((now - s_motion->sp_tick) >= (uint32_t)CHASSIS_MOTION_SETPOINT_MIN_INTERVAL_MS);
#else
            due = true;
#endif
        }
    }

    if (due) {
        s_motion->sp_x = x;
        s_motion->sp_y = y;
        s_motion->sp_w = yaw;
        s_motion->sp_tick = now;
        s_motion->sp_valid = true;
        s_motion->sp_posted++;
    } else {
        s_motion->sp_skipped++;
    }
    return due;
}

/**
 * @brief 速度控制
 * @param x X分量
 * @param y Y分量
 * @param yaw Yaw分量
 * @note  速度命令会接管轮子运动: 同时作废可能残留的"位置模式移动"状态,
 *        避免上层之后把速度运动误当成位置移动在等。
 *        本函数**永不阻塞**：最多在驱动层"运动 setpoint 邮箱"里做一次临界区拷贝(~2us)，
 *        不再像旧实现那样在电机命令队列满时阻塞最多 100ms，因此 Nav 的 5ms 周期不会被它拉长。
 */
void MotionControl_SetVelocity(float x, float y, float yaw) {
#if MOTOR_POS_MODE
    s_pos_move.active = false;
#endif
    if (s_motion == NULL) return;
    if (!motion_setpoint_due(x, y, yaw)) return; // 指令几乎未变: 跳过本次下发(省总线带宽)

    Wheel_t wheels;
    Kinematics_Inverse(x, y, yaw, &wheels);
    send_wheel_velocity_commands(&wheels);
}
#endif /* MOTOR_VELOCITY_MODE */

/**
 * @brief 获取运动 setpoint 限频统计（供 `car bus` 查看总线带宽节省效果）
 * @param posted  实际投递次数(输出, 可为 NULL)
 * @param skipped 因重复被跳过次数(输出, 可为 NULL)
 */
void MotionControl_GetSetpointStats(uint32_t *posted, uint32_t *skipped) {
    if (posted != NULL) *posted = (s_motion != NULL) ? s_motion->sp_posted : 0;
    if (skipped != NULL) *skipped = (s_motion != NULL) ? s_motion->sp_skipped : 0;
}

#if MOTOR_POS_MODE
/**
 * @brief 位置控制: 相对当前位姿的机体系偏移(轮位置闭环)
 * @param x X偏移 (cm, 机体系: + 向前)
 * @param y Y偏移 (cm, 机体系: + 向左)
 * @param yaw Yaw偏移 (deg, 俯视逆时针为正)
 * @return true=命令已下发(或偏移不足 1 个脉冲, 无需运动); false=未初始化/参数非法/命令发送失败
 * @note  非阻塞: 下发后立即返回, 需要确认到位时轮询 MotionControl_PositionArrived()。
 */
bool MotionControl_SetPosition(float x, float y, float yaw) {
    if (s_motion == NULL) {
        logError("chassis: position move before MotionControl_Init");
        return false;
    }
    /* 参数防御: NaN/Inf/离谱偏移会解算出错误脉冲(用 "!(a<=b)" 的写法连 NaN 一起挡住) */
    if (!(fabsf(x) <= 1000.0f) || !(fabsf(y) <= 1000.0f) || !(fabsf(yaw) <= 3600.0f)) {
        logError("chassis: position move invalid offset (%.3f, %.3f, %.3f)", (double)x, (double)y, (double)yaw);
        return false;
    }

    Wheel_t wheels;
    Kinematics_Inverse(x, y, yaw, &wheels);

    float values[WHEEL_COUNT];
    wheel_values(&wheels, values);

    int32_t pulses[WHEEL_COUNT];
    uint8_t moving = 0;
    for (WheelIndex_t i = WHEEL_FRONT_LEFT; i < WHEEL_COUNT; i++) {
        pulses[i] = wheel_angle_to_pulses(values[i]);
        if (pulses[i] != 0) moving++;
    }

    s_pos_move.active = false;
    /* 位置命令走普通队列（优先级低于运动 setpoint 邮箱）：先作废邮箱里可能残留的速度 setpoint，
       保证不会出现"先补发一条更早的速度、再执行位置命令"的顺序。位置模式要求底盘静止时下发。 */
    Motor_Cancel_Motion_Setpoint();
#if MOTOR_VELOCITY_MODE
    motion_setpoint_invalidate();
#endif
    if (moving == 0) {
        /* 偏移小于 1 个电机脉冲: 不动作, 视为已到位 */
        logInfo("chassis: pos move skipped (offset < 1 pulse)");
        return true;
    }

    /* 记录下发前各轮实时位置, 供 MotionControl_PositionArrived 判断到位 */
    uint8_t feedback = 0;
    for (WheelIndex_t i = WHEEL_FRONT_LEFT; i < WHEEL_COUNT; i++) {
        /* 目标位移换算成"实时位置回读单位"(每圈 65536), 与回读值同单位才能比较 */
        s_pos_move.target_read[i] = wheel_pulses_to_read_units(wheel_pulses_abs(pulses[i]));
        s_pos_move.pos_valid[i] = wheel_real_pos(i, &s_pos_move.start_pos[i]);
        s_pos_move.last_pos[i] = s_pos_move.start_pos[i];
        if (s_pos_move.pos_valid[i]) feedback++;
    }

    if (!send_wheel_position_commands(pulses)) {
        logError("chassis: pos move command send failed");
        return false;
    }

    s_pos_move.start_tick = osKernelGetTickCount();
    s_pos_move.last_change_tick = s_pos_move.start_tick;
    s_pos_move.active = true;

    logInfo("chassis: pos move (%.2f, %.2f)cm %.1fdeg -> pulses [%ld %ld %ld %ld] feedback=%u",
            (double)x,
            (double)y,
            (double)yaw,
            (long)pulses[WHEEL_FRONT_LEFT],
            (long)pulses[WHEEL_FRONT_RIGHT],
            (long)pulses[WHEEL_BACK_LEFT],
            (long)pulses[WHEEL_BACK_RIGHT],
            (unsigned)feedback);
    return true;
}

/**
 * @brief 查询位置模式移动是否到位(轮子实时位置到达目标并停稳)
 * @return true=已到位/没有进行中的位置移动; false=仍在运动
 */
bool MotionControl_PositionArrived(void) {
    if (s_motion == NULL || !s_pos_move.active) return true;

    uint32_t now = osKernelGetTickCount();
    bool any_checked = false;
    bool all_reached = true;
    bool any_changed = false;

    for (WheelIndex_t i = WHEEL_FRONT_LEFT; i < WHEEL_COUNT; i++) {
        int32_t pos = 0;
        if (!s_pos_move.pos_valid[i]) continue; // 下发前位置不可信的轮不参与判定
        if (!wheel_real_pos(i, &pos)) continue; // 中途掉线也不参与判定(等上层超时)

        any_checked = true;

        int32_t traveled = pos - s_pos_move.start_pos[i];
        if (traveled < 0) traveled = -traveled;
        if ((uint32_t)traveled + MOTOR_POS_ARRIVE_TOL_READ < s_pos_move.target_read[i]) {
            all_reached = false;
        }

        if (pos != s_pos_move.last_pos[i]) {
            s_pos_move.last_pos[i] = pos;
            any_changed = true;
        }
    }

    if (any_changed) s_pos_move.last_change_tick = now;

    if (!any_checked) {
        /* 读不到任何电机位置反馈: 按固定时间兜底(只表示命令已下发, 不保证实际到位) */
        if ((now - s_pos_move.start_tick) >= MOTOR_POS_FALLBACK_DONE_MS) {
            s_pos_move.active = false;
            logWarning("chassis: pos move done fallback (no motor position feedback, arrival unverified)");
            return true;
        }
        return false;
    }

    if (all_reached && (now - s_pos_move.last_change_tick) >= MOTOR_POS_SETTLE_MS) {
        s_pos_move.active = false;
        return true;
    }
    return false;
}
#else /* MOTOR_POS_MODE */
bool MotionControl_SetPosition(float x, float y, float yaw) {
    (void)x;
    (void)y;
    (void)yaw;
    logError("chassis: position mode disabled (MOTOR_POS_MODE=0)");
    return false;
}

bool MotionControl_PositionArrived(void) {
    return true;
}
#endif /* MOTOR_POS_MODE */

/**
 * @brief 设置运动参数
 * @param linear_speed 最大线速度 (cm/s)
 * @param yaw_speed 最大偏摆速度 (deg/s)
 * @param acc 加速度 (cm/s²)
 * @param dec 减速度 (cm/s²)
 */
void MotionControl_SetMotionParams(float linear_speed, float yaw_speed, float acc, float dec) {
    if (s_motion == NULL) return;
    s_motion->linear_speed = linear_speed;
    s_motion->yaw_speed = yaw_speed;
    /* 统一按物理单位(cm/s²)保存, 下发到电机时再按固件(X: RPM/s, Emm: 0-255档位)换算 */
    s_motion->car_acc = acc;
    s_motion->car_dec = dec;
}

/**
 * @brief 获取运动参数
 * @param linear_speed 最大线速度 (cm/s)
 * @param yaw_speed 最大偏摆速度 (deg/s)
 * @param acc 加速度 (cm/s²)
 * @param dec 减速度 (cm/s²)
 */
void MotionControl_GetMotionParams(float *linear_speed, float *yaw_speed, float *acc, float *dec) {
    if (s_motion == NULL) return;
    *linear_speed = s_motion->linear_speed;
    *yaw_speed = s_motion->yaw_speed;
    *acc = s_motion->car_acc;
    *dec = s_motion->car_dec;
}

#if MOTOR_CMD_STOP
/**
 * @brief 停止运动
 * @note 停止命令是单帧广播(addr=0)，必须"可靠且优先"：
 *   1) 走驱动层紧急通道 Motor_Send_Cmd_Urgent()：非阻塞、优先于状态轮询与运动 setpoint 发送，
 *      正常 1~2ms 就能上总线（旧实现排在深度 64 的 FIFO 尾部，队列满时实际延迟 100ms 级，
 *      这正是历史上"导航已到位但某个轮子还在转"以及 Nav 需要等 300ms 再补发一次停止的根因）；
 *   2) 先作废邮箱里还没发出的运动 setpoint，保证不会在停止之后再执行一条更早的速度指令
 *      （通道优先级：紧急 > 运动邮箱，作废后投递停止即可保证顺序）；
 *   3) 保留原有兜底：紧急通道异常（驱动未初始化等）时退回"队列重试 + 逐轮单机停止"。
 */
void MotionControl_Stop(void) {
#if MOTOR_POS_MODE
    /* 立即停止同时作废进行中的位置模式移动, 避免上层继续等待一个已被停掉的目标 */
    s_pos_move.active = false;
#endif

    /* 作废尚未发出的运动 setpoint: 停止之后不允许再出现更早的速度指令 */
    Motor_Cancel_Motion_Setpoint();
#if MOTOR_VELOCITY_MODE
    /* 同时作废"重复指令去重"状态: 停车后即使恢复的速度与停车前几乎相同, 也必须重新下发一次 */
    motion_setpoint_invalidate();
#endif

    MotorCmd_t cmd = {.op_type = OP_CONTROL};
    cmd.type.ctrl.type = CTRL_STOP;
    cmd.type.ctrl.p.stop.sync = false;

    /* 广播停止(addr=0): 总线上所有电机立即停止; 紧急通道非阻塞、先于其它命令发送 */
    cmd.motor_id = 0;
    if (Motor_Send_Cmd_Urgent(&cmd)) {
        return;
    }

    /* 兜底(驱动未初始化/紧急通道不可用): 保持旧语义——队列满时重试, 再退化为逐轮单机停止 */
    for (uint8_t attempt = 0; attempt < MOTOR_STOP_SEND_RETRY; attempt++) {
        cmd.motor_id = 0;
        if (Motor_Send_Cmd(&cmd)) {
            return;
        }
        logWarning("chassis: stop broadcast send failed (%u/%u), retry",
                   (unsigned)(attempt + 1),
                   (unsigned)MOTOR_STOP_SEND_RETRY);
    }

#if MOTOR_VELOCITY_MODE || MOTOR_POS_MODE
    /* 广播连续失败: 逐轮补发单机停止, 兜底避免个别轮子保持上一次速度 */
    for (WheelIndex_t i = WHEEL_FRONT_LEFT; i < WHEEL_COUNT; i++) {
        cmd.motor_id = s_wheel_motors[i].motor_id;
        if (!Motor_Send_Cmd(&cmd)) {
            logError("chassis: stop send failed for wheel motor %u", (unsigned)cmd.motor_id);
        }
    }
#endif
}
#endif /* MOTOR_CMD_STOP */
