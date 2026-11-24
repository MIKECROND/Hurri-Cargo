/**
 * @file servo_tools.c
 * @brief 舵机调试工具(shell 命令)
 * @note  仅供台架调试使用。
 *        舵机协议接口不是线程安全的, 用 shell 调试时请确保业务任务没有同时在操作同一舵机;
 *        等应答的命令(ping/read/qangle/monitor 等)最长会阻塞 100ms, 扫描命令按 ID 数累加。
 */

#include "shell.h"
#include "shell_cmd_group.h"
#include "log.h"
#include "servo_protocol.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SERVO_TOOLS_ID_MAX 254U
#define SERVO_TOOLS_READ_SIZE SERVO_READ_DATA_MAX_SIZE

static const char *ServoTools_StatusName(SERVO_STATUS status) {
    switch (status) {
    case SERVO_STATUS_SUCCESS:
        return "SUCCESS";
    case SERVO_STATUS_FAIL:
        return "FAIL";
    case SERVO_STATUS_TIMEOUT:
        return "TIMEOUT";
    case SERVO_STATUS_WRONG_RESPONSE_HEADER:
        return "WRONG_HEADER";
    case SERVO_STATUS_UNKNOWN_CMD_ID:
        return "UNKNOWN_CMD";
    case SERVO_STATUS_SIZE_TOO_BIG:
        return "SIZE_TOO_BIG";
    case SERVO_STATUS_CHECKSUM_ERROR:
        return "CHECKSUM_ERROR";
    case SERVO_STATUS_ID_NOT_MATCH:
        return "ID_NOT_MATCH";
    case SERVO_STATUS_ERROR:
        return "ERROR";
    default:
        return "UNKNOWN";
    }
}

static int ServoTools_ParseLong(const char *text, long min, long max, long *value) {
    char *end = NULL;
    long v;

    if ((text == NULL) || (value == NULL)) {
        return 0;
    }
    v = strtol(text, &end, 0);
    if ((end == text) || (*end != '\0') || (v < min) || (v > max)) {
        return 0;
    }
    *value = v;
    return 1;
}

static int ServoTools_ParseFloat(const char *text, float min, float max, float *value) {
    char *end = NULL;
    double v;

    if ((text == NULL) || (value == NULL)) {
        return 0;
    }
    v = strtod(text, &end);
    if ((end == text) || (*end != '\0') || (v < (double)min) || (v > (double)max)) {
        return 0;
    }
    *value = (float)v;
    return 1;
}

static void ServoTools_PrintBytes(const uint8_t *data, uint8_t size) {
    char text[160];
    int offset = 0;

    if ((data == NULL) || (size == 0U)) {
        logPrintln("  data: (empty)");
        return;
    }

    for (uint8_t i = 0; i < size; i++) {
        int written;
        if (offset > (int)(sizeof(text) - 5U)) {
            break;
        }
        written = sprintf(&text[offset], "%02X ", (unsigned)data[i]);
        if (written <= 0) {
            break;
        }
        offset += written;
    }
    text[offset] = '\0';
    logPrintln("  data: %s", text);
}

static void ServoTools_PrintState(uint8_t state) {
    static const char *const names[8] = {"command-running",
                                         "command-error",
                                         "stall",
                                         "over-voltage",
                                         "under-voltage",
                                         "current-error",
                                         "power-error",
                                         "temperature-error"};

    for (uint8_t bit = 0; bit < 8U; bit++) {
        if ((state & (uint8_t)(1U << bit)) != 0U) {
            logPrintln("  state bit%d: %s", (int)bit, names[bit]);
        }
    }
}

/*============================ 子命令实现 ============================*/

static void ServoTools_Ping(int argc, char *argv[]) {
    long id;

    if ((argc < 2) || !ServoTools_ParseLong(argv[1], 0, SERVO_TOOLS_ID_MAX, &id)) {
        logPrintln("usage: servo ping <id(0-254)>");
        return;
    }

    SERVO_STATUS status = Servo_Ping((uint8_t)id);
    logPrintln("servo[%ld] ping: %s", id, ServoTools_StatusName(status));
}

static void ServoTools_Scan(int argc, char *argv[]) {
    long from = 0;
    long to = 10;

    if ((argc >= 2) && !ServoTools_ParseLong(argv[1], 0, SERVO_TOOLS_ID_MAX, &from)) {
        logPrintln("usage: servo scan [from(0-254)] [to(0-254)]");
        return;
    }
    if ((argc >= 3) && !ServoTools_ParseLong(argv[2], 0, SERVO_TOOLS_ID_MAX, &to)) {
        logPrintln("usage: servo scan [from(0-254)] [to(0-254)]");
        return;
    }
    if (to < from) {
        logPrintln("invalid range: from(%ld) > to(%ld)", from, to);
        return;
    }

    logPrintln("scan servo id %ld..%ld ...", from, to);
    for (long id = from; id <= to; id++) {
        if (Servo_Ping((uint8_t)id) == SERVO_STATUS_SUCCESS) {
            logPrintln("  online: id=%ld", id);
        }
    }
    logPrintln("scan done");
}

static void ServoTools_SetAngle(int argc, char *argv[]) {
    long id;
    long interval = 1000;
    long power = 0;
    float deg;

    if ((argc < 3) || !ServoTools_ParseLong(argv[1], 0, SERVO_TOOLS_ID_MAX, &id) ||
        !ServoTools_ParseFloat(argv[2], -180.0f, 180.0f, &deg)) {
        logPrintln("usage: servo angle <id(0-254)> <deg(-180~180)> [interval_ms] [power_mw]");
        return;
    }
    if ((argc >= 4) && !ServoTools_ParseLong(argv[3], 0, 65535, &interval)) {
        logPrintln("invalid interval: %s", argv[3]);
        return;
    }
    if ((argc >= 5) && !ServoTools_ParseLong(argv[4], 0, 65535, &power)) {
        logPrintln("invalid power: %s", argv[4]);
        return;
    }

    SERVO_STATUS status = Servo_SetServoAngle((uint8_t)id, deg, (uint16_t)interval, (uint16_t)power);
    logPrintln("servo[%ld] angle=%.1f deg interval=%ldms power=%ldmW send: %s",
               id,
               (double)deg,
               interval,
               power,
               ServoTools_StatusName(status));
}

static void ServoTools_QueryAngle(int argc, char *argv[]) {
    long id;
    float angle = 0.0f;

    if ((argc < 2) || !ServoTools_ParseLong(argv[1], 0, SERVO_TOOLS_ID_MAX, &id)) {
        logPrintln("usage: servo qangle <id(0-254)>");
        return;
    }

    SERVO_STATUS status = Servo_QueryServoAngle((uint8_t)id, &angle);
    if (status == SERVO_STATUS_SUCCESS) {
        logPrintln("servo[%ld] angle: %.1f deg", id, (double)angle);
    } else {
        logPrintln("servo[%ld] query angle failed: %s", id, ServoTools_StatusName(status));
    }
}

static void ServoTools_SetAngleMTurn(int argc, char *argv[]) {
    long id;
    long interval = 1000;
    long power = 0;
    float deg;

    if ((argc < 3) || !ServoTools_ParseLong(argv[1], 0, SERVO_TOOLS_ID_MAX, &id) ||
        !ServoTools_ParseFloat(argv[2], -368640.0f, 368640.0f, &deg)) {
        logPrintln("usage: servo mturn <id(0-254)> <deg(-368640~368640)> [interval_ms] [power_mw]");
        return;
    }
    if ((argc >= 4) && !ServoTools_ParseLong(argv[3], 0, 4096000, &interval)) {
        logPrintln("invalid interval: %s", argv[3]);
        return;
    }
    if ((argc >= 5) && !ServoTools_ParseLong(argv[4], 0, 65535, &power)) {
        logPrintln("invalid power: %s", argv[4]);
        return;
    }

    SERVO_STATUS status = Servo_SetServoAngleMTurn((uint8_t)id, deg, (uint32_t)interval, (uint16_t)power);
    logPrintln("servo[%ld] mturn angle=%.1f deg interval=%ldms power=%ldmW send: %s",
               id,
               (double)deg,
               interval,
               power,
               ServoTools_StatusName(status));
}

static void ServoTools_QueryAngleMTurn(int argc, char *argv[]) {
    long id;
    float angle = 0.0f;

    if ((argc < 2) || !ServoTools_ParseLong(argv[1], 0, SERVO_TOOLS_ID_MAX, &id)) {
        logPrintln("usage: servo qmturn <id(0-254)>");
        return;
    }

    SERVO_STATUS status = Servo_QueryServoAngleMTurn((uint8_t)id, &angle);
    if (status == SERVO_STATUS_SUCCESS) {
        logPrintln("servo[%ld] mturn angle: %.1f deg", id, (double)angle);
    } else {
        logPrintln("servo[%ld] query mturn angle failed: %s", id, ServoTools_StatusName(status));
    }
}

static void ServoTools_Monitor(int argc, char *argv[]) {
    long id;
    ServoData data[1];

    if ((argc < 2) || !ServoTools_ParseLong(argv[1], 0, SERVO_TOOLS_ID_MAX, &id)) {
        logPrintln("usage: servo monitor <id(0-254)>");
        return;
    }

    memset(data, 0, sizeof(data));
    SERVO_STATUS status = Servo_Monitor((uint8_t)id, data);
    if (status != SERVO_STATUS_SUCCESS) {
        logPrintln("servo[%ld] monitor failed: %s", id, ServoTools_StatusName(status));
        return;
    }

    logPrintln("servo[%u] monitor:", (unsigned)data[0].id);
    logPrintln("  voltage=%d mV current=%d mA power=%d mW temp=%d C",
               (int)data[0].voltage,
               (int)data[0].current,
               (int)data[0].power,
               (int)data[0].temperature);
    logPrintln("  state=0x%02X angle=%.1f deg circle=%d",
               (unsigned)data[0].status,
               (double)data[0].angle,
               (int)data[0].circle_count);
    ServoTools_PrintState(data[0].status);
}

static void ServoTools_Read(int argc, char *argv[]) {
    long id;
    long addr;
    uint8_t value[SERVO_TOOLS_READ_SIZE];
    uint8_t size = 0;

    if ((argc < 3) || !ServoTools_ParseLong(argv[1], 0, SERVO_TOOLS_ID_MAX, &id) ||
        !ServoTools_ParseLong(argv[2], 0, 255, &addr)) {
        logPrintln("usage: servo read <id(0-254)> <addr(0-255, 支持 0x)>");
        return;
    }

    memset(value, 0, sizeof(value));
    SERVO_STATUS status = Servo_ReadData((uint8_t)id, (uint8_t)addr, value, &size);
    if (status != SERVO_STATUS_SUCCESS) {
        logPrintln("servo[%ld] read addr=%ld failed: %s", id, addr, ServoTools_StatusName(status));
        return;
    }

    logPrintln("servo[%ld] read addr=%ld size=%u", id, addr, (unsigned)size);
    ServoTools_PrintBytes(value, size);
    if (size == 2U) {
        logPrintln("  as int16: %d", (int)(int16_t)((uint16_t)value[0] | ((uint16_t)value[1] << 8)));
    } else if (size == 4U) {
        int32_t v = (int32_t)(((uint32_t)value[3] << 24) | ((uint32_t)value[2] << 16) | ((uint32_t)value[1] << 8) |
                              ((uint32_t)value[0]));
        logPrintln("  as int32: %ld", (long)v);
    }
}

static void ServoTools_Damping(int argc, char *argv[]) {
    long id;
    long power = 500;

    if ((argc < 2) || !ServoTools_ParseLong(argv[1], 0, SERVO_TOOLS_ID_MAX, &id)) {
        logPrintln("usage: servo damp <id(0-254)> [power_mw]");
        return;
    }
    if ((argc >= 3) && !ServoTools_ParseLong(argv[2], 0, 65535, &power)) {
        logPrintln("invalid power: %s", argv[2]);
        return;
    }

    SERVO_STATUS status = Servo_DampingMode((uint8_t)id, (uint16_t)power);
    logPrintln("servo[%ld] damping power=%ldmW send: %s", id, power, ServoTools_StatusName(status));
}

static void ServoTools_Stop(int argc, char *argv[]) {
    long id;
    long mode = 0;
    long power = 0;

    if ((argc < 2) || !ServoTools_ParseLong(argv[1], 0, SERVO_TOOLS_ID_MAX, &id)) {
        logPrintln("usage: servo stop <id(0-254)> [mode(0-2)] [power_mw]");
        return;
    }
    if ((argc >= 3) && !ServoTools_ParseLong(argv[2], 0, 2, &mode)) {
        logPrintln("invalid mode: %s (0=卸力 1=保持 2=阻尼)", argv[2]);
        return;
    }
    if ((argc >= 4) && !ServoTools_ParseLong(argv[3], 0, 65535, &power)) {
        logPrintln("invalid power: %s", argv[3]);
        return;
    }

    SERVO_STATUS status = Servo_StopOnControlMode((uint8_t)id, (uint8_t)mode, (uint16_t)power);
    logPrintln("servo[%ld] stop mode=%ld send: %s", id, mode, ServoTools_StatusName(status));
}

static void ServoTools_ResetAngle(int argc, char *argv[]) {
    long id;

    if ((argc < 2) || !ServoTools_ParseLong(argv[1], 0, SERVO_TOOLS_ID_MAX, &id)) {
        logPrintln("usage: servo reset_angle <id(0-254)>");
        return;
    }

    SERVO_STATUS status = Servo_ResetServoMTurnAngle((uint8_t)id);
    logPrintln("servo[%ld] reset multi-turn angle send: %s", id, ServoTools_StatusName(status));
}

static void ServoTools_SetOriginPoint(int argc, char *argv[]) {
    long id;

    if ((argc < 2) || !ServoTools_ParseLong(argv[1], 0, SERVO_TOOLS_ID_MAX, &id)) {
        logPrintln("usage: servo set_op <id(0-254)>");
        return;
    }

    SERVO_STATUS status = Servo_SetOriginPoint((uint8_t)id);
    if (status != SERVO_STATUS_SUCCESS) {
        logPrintln("servo[%ld] set origin point failed: %s", id, ServoTools_StatusName(status));
        return;
    }

    logPrintln("servo[%ld] set origin point send: %s", id, ServoTools_StatusName(status));
}

/*============================ 命令组注册 ============================*/

ShellCommand ServoToolsGroup[] = {
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, ping, ServoTools_Ping, Ping servo),
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, scan, ServoTools_Scan, Scan online servo id),
    SHELL_CMD_GROUP_ITEM(
        SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, angle, ServoTools_SetAngle, Set single - turn angle),
    SHELL_CMD_GROUP_ITEM(
        SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, qangle, ServoTools_QueryAngle, Query single - turn angle),
    SHELL_CMD_GROUP_ITEM(
        SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, mturn, ServoTools_SetAngleMTurn, Set multi - turn angle),
    SHELL_CMD_GROUP_ITEM(
        SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, qmturn, ServoTools_QueryAngleMTurn, Query multi - turn angle),
    SHELL_CMD_GROUP_ITEM(
        SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, monitor, ServoTools_Monitor, Read servo monitor data),
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, read, ServoTools_Read, Read servo parameter),
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, damp, ServoTools_Damping, Servo damping mode),
    SHELL_CMD_GROUP_ITEM(
        SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, stop, ServoTools_Stop, Stop servo control mode),
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN,
                         reset_angle,
                         ServoTools_ResetAngle,
                         Reset multi - turn angle count),
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, set_op,
                         ServoTools_SetOriginPoint,
                         Set origin point),
    SHELL_CMD_GROUP_END()};
SHELL_EXPORT_CMD_GROUP(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_DISABLE_RETURN,
                       servo,
                       ServoToolsGroup,
                       Servo debug tools);
