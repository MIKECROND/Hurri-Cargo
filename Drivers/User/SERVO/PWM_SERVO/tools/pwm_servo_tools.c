/**
 * @file pwm_servo_tools.c
 * @brief PWM 舵机调试工具(shell 命令)
 * @note  仅供台架调试使用：
 *          pwm_servo status                     查看初始化状态与各通道角度/脉宽
 *          pwm_servo init                       初始化 PWM 输出
 *          pwm_servo off                        停止 PWM 输出(舵机卸力)
 *          pwm_servo angle <id> <deg>           按角度设置
 *          pwm_servo pulse <id> <us>            按脉宽设置(标定脉宽范围用)
 *          pwm_servo center [id]                回中位(90 度)，省略 id 时两路一起
 *          pwm_servo sweep <id> [step] [ms]     往复扫动，检查机械行程(默认 5 度/100ms)
 *        sweep 会阻塞 shell 任务，扫动过程中请勿同时操作同一舵机。
 *        PWM_SERVO_ENABLED 为 0 时不注册该命令。
 */

#include "shell.h"
#include "log.h"
#include "cmsis_os2.h"
#include "pwm_servo_driver.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if PWM_SERVO_ENABLED

static void PWMServoTools_Shell(int argc, char *argv[])
{
    if (argc < 4) {
        logPrintln("Usage: pwm_servo angle <id> <deg>\n");
        return;
    }

    PWM_Servo_Init();

    int id = atoi(argv[2]);
    int deg = atoi(argv[3]);
    if (id < PWM_SERVO_ANGLE_MIN || id >= PWM_SERVO_CH_NUM) {
        logPrintln("Invalid id: %d\n", id);
        return;
    }
    if (deg < 0 || deg > PWM_SERVO_ANGLE_MAX) {
        logPrintln("Invalid deg: %d\n", deg);
        return;
    }
    PWM_Servo_SetAngle(id, deg);
}

/**
 * @brief 导出 Shell 命令
 */
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_DISABLE_RETURN,
                 pwm_servo,
                 PWMServoTools_Shell,
                 PWM servo debug tools);

#endif /* PWM_SERVO_ENABLED */
