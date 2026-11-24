/**
 * @file scan_tools.c
 * @author T-Car Team
 * @brief 实现扫描器模块的 shell 调试命令注册与处理。
 * 可调参数见 Drivers/User/SCANNER/scan_driver.h 与 Drivers/User/SCANNER。
 */

/**
 * @brief 扫描工具
 */

#include "shell.h"
#include "shell_cmd_group.h"
#include "log.h"
#include "scan_driver.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/**
 * @brief 扫描器控制命令
 * @param argc 命令参数数量
 * @param argv 命令参数数组
 */
static void Scan_Shell(int argc, char *argv[]) {
    if (argc < 2) {
        logPrintln("Usage: scan [on|off|get]");
        return;
    }

    if (strcmp(argv[1], "on") == 0) {
        Scan_EnableOutput();
        logPrintln("Scan output: ON");
    } else if (strcmp(argv[1], "off") == 0) {
        Scan_DisableOutput();
        logPrintln("Scan output: OFF");
    } else if (strcmp(argv[1], "get") == 0) {
        uint8_t buf[SCAN_BARCODE_MAX_LEN + 1];
        if (Scan_GetLatestBarcode(buf, sizeof(buf))) {
            logPrintln("Latest barcode: %s", buf);
        } else {
            logPrintln("No new barcode available");
        }
    } else {
        logPrintln("Invalid argument: %s", argv[1]);
        logPrintln("Usage: scan [on|off|get]");
    }
}

/**
 * @brief 导出 Shell 命令
 */
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_DISABLE_RETURN,
                 scan,
                 Scan_Shell,
                 scan control : on | off | get);
