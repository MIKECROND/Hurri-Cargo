/**
 * @file rpi_tools.c
 * @author T-Car Team
 * @brief 树莓派调试工具: shell 命令组 `rpi`
 *
 * 命令列表:
 *   rpi color              读取颜色检测结果(单色)
 *   rpi num                读取圆环内数字
 *   rpi cal <type> [move]  执行标定: type = d|Disk / r|Ring1 / r2|Ring2; move 时按误差移动车体
 *                          (标定前自动张开夹爪+翻转相机, 结束后保持翻转角)
 *   rpi stat               查看接收脏数据统计(非等待应答期间丢弃的数据)
 *   rpi send <hex>...      发送原始十六进制字节并打印原始应答(不做帧校验; 收到应答后补发空闲命令 0x11)
 * 可调参数见 rpi_config.h (禁改)。
 */

#include "rpi_port.h"

#include "arm_action.h" /* 标定链路: 夹爪/相机翻转准备与收尾 */
#include "arm_config.h"
#include "arm_ctrl.h"
#include "log.h"
#include "motion_control.h"
#include "shell.h"
#include "shell_cmd_group.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* `rpi send` 单次最多发送的字节数 */
#define RPI_SHELL_TX_MAX 16
/* `rpi send` 等待应答的时间(ms) */
/* `rpi send` 等待应答超时: 树莓派检测要"稳定"后才发帧(颜色约 2s), 太短会误报无应答 */
#define RPI_SHELL_RX_TIMEOUT_MS 6000

/* 颜色枚举 -> 名称(按 RPI_Color_t 索引) */
static const char *const s_color_names[] = {
    [COLOR_UNKNOWN] = "unknown",
    [COLOR_RED] = "red",
    [COLOR_YELLOW] = "yellow",
    [COLOR_BLUE] = "blue",
    [COLOR_GREEN] = "green",
    [COLOR_BLACK] = "black",
    [COLOR_LIGHTBLUE] = "lightblue",
};

/* `rpi cal` 标定类型别名表 */
typedef struct {
    const char *alias;
    RPI_Cal_t type;
} RPI_CalAlias_t;

static const RPI_CalAlias_t s_cal_aliases[] = {
    {"d", CAL_DISK},
    {"Disk", CAL_DISK},
    {"r", CAL_RING_FIRST},
    {"Ring1", CAL_RING_FIRST},
    {"r2", CAL_RING_SECOND},
    {"Ring2", CAL_RING_SECOND},
    {"s", CAL_STORAGE},
    {"storage", CAL_STORAGE},
    {"Storage", CAL_STORAGE},
    {"e1", CAL_STORAGE},
    {"E1", CAL_STORAGE},
};

#define RPI_CAL_ALIAS_COUNT (sizeof(s_cal_aliases) / sizeof(s_cal_aliases[0]))

/**
 * @brief 以十六进制打印一段字节
 * @param tag  打印标签(TX / RX)
 * @param data 数据指针
 * @param len  数据长度
 */
static void RPITools_PrintHex(const char *tag, const uint8_t *data, uint16_t len) {
    char hex[RPI_RX_BUF_SIZE * 3 + 1] = {0};
    uint16_t pos = 0;

    for (uint16_t i = 0; i < len && (pos + 4) < sizeof(hex); i++) {
        pos += (uint16_t)snprintf(hex + pos, sizeof(hex) - pos, "%02X ", data[i]);
    }

    logPrintln("%s(%d): %s", tag, (int)len, hex);
}

/**
 * @brief 解析 `rpi cal` 的标定类型参数
 * @param text 命令行参数
 * @param type 输出: 标定类型
 * @return true=识别成功
 */
static bool RPITools_ParseCalType(const char *text, RPI_Cal_t *type) {
    for (uint8_t i = 0; i < (uint8_t)RPI_CAL_ALIAS_COUNT; i++) {
        if (strcmp(text, s_cal_aliases[i].alias) == 0) {
            *type = s_cal_aliases[i].type;
            return true;
        }
    }

    return false;
}

/**
 * @brief `rpi cal` 用法说明
 */
static void RPITools_CalUsage(void) {
    logPrintln("Usage: rpi cal <type> [move]");
    logPrintln("  type: d|Disk (disk, grab area) | r|Ring1 (ring 1st) | r2|Ring2 (ring 2nd)");
    logPrintln("        s|storage|e1 (storage area, dedicated RPi command 0xE1)");
    logPrintln("  move: also move the car by the measured error (optional)");
    logPrintln("  note: opens gripper to %d deg and flips camera before calibrating,", (int)ARM_CAL_GRIPPER_OPEN_DEG);
    logPrintln("        keeps camera at calibration angle after calibration (same as mission flow)");
}

/**
 * @brief 读取颜色检测结果(单色)
 */
static void RPITools_Color(void) {
    RPI_Color_t color = COLOR_UNKNOWN;

    if (!RPI_Detect_Color(&color)) {
        logPrintln("Color: detect failed");
        return;
    }

    logPrintln("Color: %s", s_color_names[color]);
}

/**
 * @brief 读取圆环内数字
 */
static void RPITools_Num(void) {
    uint8_t num = 0;

    if (!RPI_Detect_NUM(&num)) {
        logPrintln("Num: detect failed");
        return;
    }

    logPrintln("Num: %d", num);
}

/**
 * @brief 标定前准备机械臂: 张开夹爪 + 翻转相机(与任务流程 Arm_Action_CalibrationReady 的后半段一致)
 * @note 相机装在夹爪/末端上时, 闭合夹爪或夹着的物料会挡住标定目标; 夹爪失败只提示不中断标定。
 */
static void RPITools_CalArmPrep(void) {
    if (!Arm_GripperSetAngle(ARM_CAL_GRIPPER_OPEN_DEG)) {
        logPrintln("Cal: gripper open to %d deg FAILED (continue)", (int)ARM_CAL_GRIPPER_OPEN_DEG);
    } else {
        logPrintln("Cal: gripper opened to %d deg", (int)ARM_CAL_GRIPPER_OPEN_DEG);
    }
    Arm_FlipSetAngle(ARM_FLIP_CAL_ANGLE_DEG);
}

/**
 * @brief `rpi stat`: 查看接收脏数据统计(链路污染观察)
 * @note 统计的是"非等待应答期间收到并丢弃"的数据(迟到应答/RPI 调试输出);
 *       正常应基本不增长, 持续增长说明树莓派侧有非请求输出(但不会污染命令解析)。
 *       链路为 USART2 DMA 发送 + 中断接收, 命令事务由模块内部互斥锁串行化。
 */
static void RPITools_Stat(void) {
    uint32_t events = 0;
    uint32_t bytes = 0;

    RPI_GetRxStats(&events, &bytes);
    logPrintln("rx dirty data discarded: %lu bytes (%lu events; uart2 dma tx, it rx)",
               (unsigned long)bytes,
               (unsigned long)events);
}

/**
 * @brief 执行圆盘/圆环标定
 * @note 参数: <type> [move]
 *       标定前自动把夹爪张开到 ARM_CAL_GRIPPER_OPEN_DEG 并翻转相机到标定角度,
 *       标定结束后保持该翻转角(颜色识别同视角), 不再恢复默认角(与任务流程一致)。
 */
static void RPITools_Cal(int argc, char *argv[]) {
    if (argc < 2) {
        RPITools_CalUsage();
        return;
    }

    RPI_Cal_t cal_type;
    if (!RPITools_ParseCalType(argv[1], &cal_type)) {
        logPrintln("Invalid type: %s", argv[1]);
        RPITools_CalUsage();
        return;
    }

    logPrintln("calibrate: type=%s, RPi command 0x%02X", argv[1], (unsigned)RPI_CalCmd(cal_type));

    bool move = false;
    if (argc >= 3) {
        if (strcmp(argv[2], "move") == 0 || strcmp(argv[2], "m") == 0) {
            move = true;
        } else {
            logPrintln("Invalid option: %s", argv[2]);
            RPITools_CalUsage();
            return;
        }
    }

    /* 标定前: 张开夹爪并把相机翻到标定角度 */
    RPITools_CalArmPrep();

    int16_t err_x = 0, err_y = 0;
    float err_angle_deg = 0.0f;
    bool ok = RPI_Calibrate(&err_x, &err_y, &err_angle_deg, cal_type);

    /* 标定结束统一收尾: 保持翻转角不再翻回(颜色识别也用该视角);
     * 默认角只在回零/伸缩校准前或 `arm flip restore` 时恢复 */
    Arm_Action_CalibrationFinish();

    if (!ok) {
        logWarning("calibrate failed");
        return;
    }

    logPrintln("calibrate ok: err_x = %d mm, err_y = %d mm, err_angle = %.1f deg", err_x, err_y, err_angle_deg);

    if (move) {
        /* 按误差补偿车体位置: 误差单位 mm, 位置控制单位为 cm;
           位置模式非阻塞(只下发命令, 不等待到位), 朝向修正交给任务流程/机械臂补偿 */
        if (MotionControl_SetPosition((float)err_x / 10.0f, (float)err_y / 10.0f, 0.0f)) {
            logPrintln("car pos cmd sent: err_x = %d mm, err_y = %d mm (non-blocking)", err_x, err_y);
        } else {
            logWarning("car pos cmd failed (err_x = %d mm, err_y = %d mm)", err_x, err_y);
        }
    }
}

/**
 * @brief 手工向树莓派发送十六进制字节, 并打印原始应答
 * @note 参数按十六进制解析, 例如: rpi send 0B / rpi send 1F 00 / rpi send 0x0A
 */
static void RPITools_Send(int argc, char *argv[]) {
    if (argc < 2) {
        logPrintln("Usage: rpi send <hex byte> [hex byte]...");
        logPrintln("  e.g. rpi send 0B         detect color");
        logPrintln("       rpi send 0A         calibrate ring");
        logPrintln("       rpi send E1         storage-area calibration (dedicated command)");
        logPrintln("       rpi send 11         idle");
        logPrintln("  note: idle 0x11 is sent after receiving the reply to stop repeated frames");
        return;
    }

    uint8_t tx[RPI_SHELL_TX_MAX] = {0};
    uint8_t tx_len = 0;

    for (int i = 1; i < argc; i++) {
        char *end = NULL;
        unsigned long val = strtoul(argv[i], &end, 16);

        if (end == argv[i] || *end != '\0' || val > 0xFF) {
            logPrintln("Invalid byte: %s (hex 00~FF)", argv[i]);
            return;
        }
        if (tx_len >= RPI_SHELL_TX_MAX) {
            logPrintln("Too many bytes (max %d)", RPI_SHELL_TX_MAX);
            return;
        }

        tx[tx_len++] = (uint8_t)val;
    }

    RPITools_PrintHex("TX", tx, tx_len);

    uint8_t rx[RPI_RX_BUF_SIZE] = {0};
    uint16_t rx_len = RPI_Send_Raw(tx, tx_len, rx, sizeof(rx), RPI_SHELL_RX_TIMEOUT_MS);

    if (rx_len == 0) {
        logPrintln("RX: no response (%d ms)", RPI_SHELL_RX_TIMEOUT_MS);
        return;
    }

    RPITools_PrintHex("RX", rx, rx_len);
}

ShellCommand RPIToolsGroup[] = {
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, color, RPITools_Color, Detect color),
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, num, RPITools_Num, Detect num),
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, cal, RPITools_Cal, Calibrate offset),
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, stat, RPITools_Stat, Show rx dirty stats),
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, send, RPITools_Send, Send raw bytes),
    SHELL_CMD_GROUP_END()};
SHELL_EXPORT_CMD_GROUP(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_DISABLE_RETURN,
                       rpi,
                       RPIToolsGroup,
                       RPI Tools);
