/**
 * @file chassis.c
 * @author Pcb-yun (pcbyinyun@163.com)
 * @brief 车体控制模块
 * 可调参数见 chassis_config.h。
 */

#include "chassis.h"
#include "chassis_config.h"
#include "shell.h"
#include "log.h"
#include "shell_cmd_group.h"
#include "motion_control.h"
#include "zdt_v5_cfg.h"
#include "zdt_v5_port.h"
#include "stdlib.h"
#include "string.h"
#include "cmsis_os2.h"
#include "Events.h"

static bool is_init = false;

/**
 * @brief 初始化车体控制模块
 * @return 初始化状态
 */
bool Chassis_Init(void) {
    if (!MotionControl_Init()) return false;

    is_init = true;
    return true;
}

#if MOTOR_POS_MODE
/**
 * @brief 将车移动指定距离
 */
static void Car_Move(int argc, char *argv[]) {
    if (!is_init) {
        logWarning("Chassis not initialized");
        return;
    }

    if (argc != 4) {
        logPrintln("Usage: car pos [x_offset] [y_offset] [yaw_offset]");
        return;
    }

    int32_t x_offset = atoi(argv[1]);
    int32_t y_offset = atoi(argv[2]);
    int32_t yaw_offset = atoi(argv[3]);

    if (!MotionControl_SetPosition((float)x_offset, (float)y_offset, (float)yaw_offset)) {
        logError("car: pos move failed (not init / invalid offset / motor bus busy)");
    }
}
#endif /* MOTOR_POS_MODE */

#if MOTOR_VELOCITY_MODE
/**
 * @brief 设置小车速度
 */
static void Car_Vel(int argc, char *argv[]) {
    if (!is_init) {
        logWarning("Chassis not initialized");
        return;
    }
    if (argc != 4) {
        logPrintln("Usage: car vel [x] [y] [yaw]");
        return;
    }

    int32_t x = atoi(argv[1]);
    int32_t y = atoi(argv[2]);
    int32_t yaw = atoi(argv[3]);

    /* 手动调试命令：强制本次一定下发（否则与上次相同的速度会被 setpoint 去重跳过） */
    MotionControl_ResetSetpointFilter();
    MotionControl_SetVelocity(x, y, yaw);
}

/**
 * @brief 按键遥控
 */
static void Car_Key(void) {
    if (!is_init) {
        logWarning("Chassis not initialized");
        return;
    }

    Shell *shell = shellGetCurrent();
    logPrintln("Key control started. WASD=move, QE=rotate, ^C=exit");

    char prev_key = 0;
    char key;
    uint32_t last_time;
    float linear_speed, yaw_speed, acc, dec;
    MotionControl_GetMotionParams(&linear_speed, &yaw_speed, &acc, &dec);
    osEventFlagsSet(System_StatusHandle, APP_NEED_USART);

    for (;;) {
        uint8_t ret = shell->read(&key, 1);

        if (ret > 0) {
            if (key == 0x03) { // ^C 退出
                MotionControl_Stop();
                break;
            }

            if (key == prev_key) {
                last_time = osKernelGetTickCount();
            } else {
                float x_comp = 0.0f, y_comp = 0.0f, yaw_comp = 0.0f;
                bool valid_key = true;

                switch (key) {
                case 'w':
                case 'W':
                    x_comp = linear_speed;
                    break; // 前进
                case 's':
                case 'S':
                    x_comp = -linear_speed;
                    break; // 后退
                case 'a':
                case 'A':
                    y_comp = linear_speed;
                    break; // 左移
                case 'd':
                case 'D':
                    y_comp = -linear_speed;
                    break; // 右移
                case 'q':
                case 'Q':
                    yaw_comp = yaw_speed;
                    break; // 逆时针（俯视，与 OPS yaw 正方向一致）
                case 'e':
                case 'E':
                    yaw_comp = -yaw_speed;
                    break; // 顺时针（俯视）
                default:
                    valid_key = false;
                    break;
                }

                if (valid_key) {
                    MotionControl_SetVelocity(x_comp, y_comp, yaw_comp);
                    last_time = osKernelGetTickCount();
                    prev_key = key;
                }
            }
        }

        if (osKernelGetTickCount() - last_time > 100) {
            prev_key = 0;
            MotionControl_Stop();
        }
        osDelay(10);
    }
    osEventFlagsClear(System_StatusHandle, APP_NEED_USART);
    logPrintln("\033[%dA\033[J\033[2A", 1);
}
#endif /* MOTOR_VELOCITY_MODE */

/**
 * @brief 打印底盘几何/旋转中心参数（标定与排查用，不产生运动）
 */
static void Car_Info(int argc, char *argv[]) {
    (void)argc;
    (void)argv;

    if (!is_init) {
        logWarning("Chassis not initialized");
        return;
    }

    const float cx = CHASSIS_ROTATION_CENTER_OFFSET_X;
    const float cy = CHASSIS_ROTATION_CENTER_OFFSET_Y;
    const float bx = CHASSIS_BODY_REF_OFFSET_X;
    const float by = CHASSIS_BODY_REF_OFFSET_Y;
    const float r0 = WHEEL_BASE_LENGTH / 2.0f + WHEEL_BASE_WIDTH / 2.0f;

    logPrintln("wheel: R=%.2fcm, wheelbase=%.2fcm, track=%.2fcm",
               WHEEL_RADIUS,
               WHEEL_BASE_LENGTH,
               WHEEL_BASE_WIDTH);
    logPrintln("rotation center offset: x=%.2fcm, y=%.2fcm (from wheel geometric center, +x front/+y left)",
               cx,
               cy);
    logPrintln("body ref offset (body ref - OPS point): x=%.2fcm, y=%.2fcm "
               "(nav pose = OPS + R(yaw)*offset)",
               bx,
               by);
    logPrintln("rotation levers: FL=%.2f FR=%.2f RL=%.2f RR=%.2f cm (r0=%.2f)",
               r0 - cx - cy,
               r0 - cx + cy,
               r0 + cx - cy,
               r0 + cx + cy,
               r0);
    /* 纯 yaw 180°（offset=0）时每轮应转的圈数/脉冲，用于架空空转核对脉冲当量 */
    logPrintln("pure yaw 180deg (offset=0): %.3f rev, %.0f pulses per wheel",
               r0 / (2.0f * WHEEL_RADIUS),
               r0 / (2.0f * WHEEL_RADIUS) * (float)MOTOR_PULSES_PER_REV);
    logPrintln("note: rotation center must equal the nav pose point = OPS + body ref offset");
}

/**
 * @brief 设置车的运动参数
 */
static void Car_Params(int argc, char *argv[]) {
    float linear_speed;
    float yaw_speed;
    float acc;
    float dec;

    if (!is_init) {
        logWarning("Chassis not initialized");
        return;
    }

    if (argc == 1) {
        MotionControl_GetMotionParams(&linear_speed, &yaw_speed, &acc, &dec);
        logPrintln(
            "Current params: linear_speed=%.1f, yaw_speed=%.1f, acc=%.1f, dec=%.1f", linear_speed, yaw_speed, acc, dec);
        return;
    } else if (argc != 5) {
        logPrintln("Usage: car par [linear_speed] [yaw_speed] [acc] [dec]");
        return;
    }

    linear_speed = atof(argv[1]);
    yaw_speed = atof(argv[2]);
    acc = atof(argv[3]);
    dec = atof(argv[4]);

    MotionControl_SetMotionParams(linear_speed, yaw_speed, acc, dec);
    logPrintln("Set params: linear_speed=%.1f, yaw_speed=%.1f, acc=%.1f, dec=%.1f", linear_speed, yaw_speed, acc, dec);
}

/**
 * @brief 打印电机命令链时延/带宽统计（含底盘 setpoint 限频统计）
 *
 * 2026-11 命令链修复的现场验收命令：
 *   `car bus`   —— 打印 setpoint 限频统计 + 电机总线命令链统计（同 `step bus`）
 *   `car bus r` —— 清零统计
 * 判读标准（正常值）：
 *   - setpoint skip 比例：稳态巡航应很高（>50%，限频生效），加减速/接近目标时 post 速率升高；
 *   - motion wait_max < 5ms、motion drop% < 5%、queue peak <= 2、put_timeout = 0；
 *   - urgent latency_max < 2ms（停止命令等待上限）；
 *   - motion rate 稳态约 50Hz、动态段可到 200Hz。
 * 修复前：运动命令走 64 深 FIFO + 每条 osDelay(2)，稳态排队 ≈134ms，Nav 周期被拉长到 9ms+。
 */
static void Car_Bus(int argc, char *argv[]) {
    if (argc > 1 && strcmp(argv[1], "r") == 0) {
        Motor_Bus_Stats_Reset();
        logPrintln("[car] bus statistics cleared");
        return;
    }

    uint32_t posted = 0;
    uint32_t skipped = 0;
    MotionControl_GetSetpointStats(&posted, &skipped);
    uint32_t total = posted + skipped;
    uint32_t skip_permille = (total > 0) ? (uint32_t)(((uint64_t)skipped * 1000U) / total) : 0;
    logPrintln("[car] setpoint: post=%lu skip=%lu(%lu.%lu%%) hold=%dms min_int=%dms dv=%.2f dw=%.1f urgent_dv=%.1f urgent_dw=%.1f",
               (unsigned long)posted,
               (unsigned long)skipped,
               (unsigned long)(skip_permille / 10U),
               (unsigned long)(skip_permille % 10U),
               (int)CHASSIS_MOTION_SETPOINT_HOLD_MS,
               (int)CHASSIS_MOTION_SETPOINT_MIN_INTERVAL_MS,
               (double)CHASSIS_MOTION_SETPOINT_DV_CM_S,
               (double)CHASSIS_MOTION_SETPOINT_DW_DEG_S,
               (double)CHASSIS_MOTION_SETPOINT_URGENT_DV_CM_S,
               (double)CHASSIS_MOTION_SETPOINT_URGENT_DW_DEG_S);

    Motor_Bus_Stats_Print();
    logPrintln("[car] target: wait_max<5ms / drop<5%% / queue peak<=2 / put_timeout=0 / urgent max<2ms = healthy");
}

ShellCommand MoveGroup[] = {
#if MOTOR_POS_MODE
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, pos, Car_Move, Move car),
#endif
#if MOTOR_VELOCITY_MODE
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, vel, Car_Vel, Car Key Control),
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, key, Car_Key, Car Key Control),
#endif
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, par, Car_Params, Set Car Motion Params),
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, info, Car_Info, Show Car Geometry Info),
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, bus, Car_Bus, Motor Bus Chain Stats),
    SHELL_CMD_GROUP_END()};
SHELL_EXPORT_CMD_GROUP(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_DISABLE_RETURN,
                       car,
                       MoveGroup,
                       Car Control CMD Group);
