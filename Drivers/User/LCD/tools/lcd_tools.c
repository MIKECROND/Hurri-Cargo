/**
 * @file lcd_tools.c
 * @author T-Car Team
 * @brief LCD调试工具实现
 * 可调参数见 lcd_port.h / LCD 配置宏。
 */

#include "shell.h"
#include "shell_cmd_group.h"
#include "log.h"
#include "cmsis_os2.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "Events.h"
#include "lcd_port.h"
#include "stm32_adafruit_lcd.h"
#include "planner_port.h"
#include "Cstar_map_core.h"
#include "nav_local.h"

static void LCDTools_Display(int argc, char *argv[]) {
    if (argc < 2) {
        logPrintln("Usage: display <on|off>");
        return;
    }
    if (strcmp(argv[1], "on") == 0) {
        LCD_Port_DisplayOn();
    } else if (strcmp(argv[1], "off") == 0) {
        LCD_Port_DisplayOff();
    } else {
        logPrintln("Invalid display mode");
        return;
    }
}

static void LCDTools_SetTextColor(int argc, char *argv[]) {
    if (argc < 2) {
        logPrintln("Usage: set_text_color <color>");
        return;
    }

    uint16_t Color = atoi(argv[1]);

    if (Color == 0) {
        logPrintln("Invalid color value");
        return;
    }

    LCD_Port_SetTextColor(Color);
    logPrintln("Text color set to %d", Color);
}

static void LCDTools_DrawLine(int argc, char *argv[]) {
    if (argc < 5) {
        logPrintln("Usage: draw_line <x1> <y1> <x2> <y2>");
        return;
    }

    uint16_t x1 = atoi(argv[1]);
    uint16_t y1 = atoi(argv[2]);
    uint16_t x2 = atoi(argv[3]);
    uint16_t y2 = atoi(argv[4]);

    LCD_Port_DrawLine(x1, y1, x2, y2);
    logPrintln("Line drawn from (%d, %d) to (%d, %d)", x1, y1, x2, y2);
}

static void LCDTools_DrawRect(int argc, char *argv[]) {
    if (argc < 5) {
        logPrintln("Usage: draw_rect <xpos> <ypos> <width> <height>");
        return;
    }

    uint16_t Xpos = atoi(argv[1]);
    uint16_t Ypos = atoi(argv[2]);
    uint16_t Width = atoi(argv[3]);
    uint16_t Height = atoi(argv[4]);

    LCD_Port_DrawRect(Xpos, Ypos, Width, Height);
    logPrintln("Rect drawn at (%d, %d) with width %d and height %d", Xpos, Ypos, Width, Height);
}

static void LCDTools_DrawCircle(int argc, char *argv[]) {
    if (argc < 4) {
        logPrintln("Usage: draw_circle <xpos> <ypos> <radius>");
        return;
    }

    uint16_t Xpos = atoi(argv[1]);
    uint16_t Ypos = atoi(argv[2]);
    uint16_t Radius = atoi(argv[3]);

    LCD_Port_DrawCircle(Xpos, Ypos, Radius);
    logPrintln("Circle drawn at (%d, %d) with radius %d", Xpos, Ypos, Radius);
}

static void LCDTools_ClearScreen(void) {
    LCD_Port_Clear(LCD_COLOR_WHITE);
    logPrintln("Screen cleared");
}

static void LCDTools_DisplayString(int argc, char *argv[]) {
    if (argc < 5) {
        logPrintln("Usage: dis_str <xpos> <ypos> <string> <mode>");
        logPrintln("mode: CENTER_MODE(c) | RIGHT_MODE(r) | LEFT_MODE(l)");
        return;
    }

    uint16_t Xpos = atoi(argv[1]);
    uint16_t Ypos = atoi(argv[2]);
    uint8_t *Text = (uint8_t *)argv[3];
    Line_ModeTypdef Mode = CENTER_MODE;

    if (strcmp(argv[4], "c") == 0)
        Mode = CENTER_MODE;
    else if (strcmp(argv[4], "r") == 0)
        Mode = RIGHT_MODE;
    else if (strcmp(argv[4], "l") == 0)
        Mode = LEFT_MODE;
    else {
        logPrintln("Invalid mode, using CENTER_MODE by default");
        Mode = CENTER_MODE;
    }

    LCD_Port_DisplayStringAt(Xpos, Ypos, Text, Mode);
    logPrintln("String displayed at (%d, %d)", Xpos, Ypos);
}

static void LCDTools_DisplayChinese(int argc, char *argv[]) {
    if (argc < 5) {
        logPrintln("Usage: dis_zh <x_pos> <y_pos> <mode> <num>");
        logPrintln("mode: Pickup(u) | PutDown(d) ");
        return;
    }

    uint16_t X = atoi(argv[1]);
    uint16_t Y = atoi(argv[2]);
    uint16_t num = atoi(argv[4]);
    if (strcmp(argv[3], "u") == 0)
        LCD_Port_DisplayPickCount(X, Y, num);
    else if (strcmp(argv[3], "d") == 0)
        LCD_Port_DisplayPlaceCount(X, Y, num);
    else {
        logPrintln("Invalid mode");
        return;
    }

    logPrintln("String displayed at (%d, %d)", X, Y);
}

#if PLANNER_ENABLED
#if PLANNER_PLATFORM == PLATFORM_MCU

/* ---- 规划器可视化：地图网格 + 障碍物 + A* 路径 ---- */
#define LCD_PLAN_CELL_PX 6u /* 每格像素边长，48 格 = 288 像素 */
#define LCD_PLAN_X0 8u /* 地图区左上角 X */
#define LCD_PLAN_Y0 16u /* 地图区左上角 Y */

#define LCD_PLAN_C_FREE LCD_COLOR(235, 235, 235) /* 空闲格 */
#define LCD_PLAN_C_OBS LCD_COLOR(40, 40, 40) /* 障碍格 */
#define LCD_PLAN_C_PATH LCD_COLOR(0, 0, 255) /* A* 路径 */
#define LCD_PLAN_C_GOAL LCD_COLOR(255, 0, 0) /* 目标点 */
#define LCD_PLAN_C_CAR LCD_COLOR(0, 170, 0) /* 小车当前位置 */

/**
 * @brief 屏幕坐标画一个实心方块（作为图例色块/标记点）
 */
static void LCDTools_FillSwatch(uint16_t X, uint16_t Y, uint16_t Size, uint16_t Color) {
    LCD_Port_SetTextColor(Color);
    LCD_Port_FillRect(X, Y, Size, Size);
}

/**
 * @brief 地图坐标(米) -> 屏幕像素
 * @note  世界 +y 向上、屏幕 y 向下，故纵向需要翻转
 */
static void LCDTools_MapToScreen(float mx, float my, uint16_t *sx, uint16_t *sy) {
    *sx = (uint16_t)(LCD_PLAN_X0 + (mx / g_map.res) * LCD_PLAN_CELL_PX);
    *sy = (uint16_t)(LCD_PLAN_Y0 + (g_map.grid_size - (my / g_map.res)) * LCD_PLAN_CELL_PX);
}

/**
 * @brief 在地图上按地图坐标(米)画一个标记方块
 */
static void LCDTools_DrawMapMarker(float mx, float my, uint16_t Color, uint8_t HalfSize) {
    uint16_t sx, sy;

    LCDTools_MapToScreen(mx, my, &sx, &sy);
    if (sx < HalfSize || sy < HalfSize) return;

    LCDTools_FillSwatch(sx - HalfSize, sy - HalfSize, 2 * HalfSize + 1, Color);
}

/**
 * @brief 把规划器的地图(障碍物)与最近一次 A* 路径画到 LCD 上
 * @note  地图网格取自 g_map（1=障碍），路径取自 g_plan_path（地图坐标）
 * @param argv 可选参数：thick/1 路径按车的半径画粗，thin/0 画细线；
 *             不带参数沿用上次设置（默认画粗）
 */
static void LCDTools_ShowPlannerView(int argc, char *argv[]) {
    static bool s_thick_path = true; /* 路径是否按车的半径画粗，跨调用保持 */

    if (argc >= 2) {
        if (strcmp(argv[1], "thick") == 0 || strcmp(argv[1], "1") == 0) {
            s_thick_path = true;
        } else if (strcmp(argv[1], "thin") == 0 || strcmp(argv[1], "0") == 0) {
            s_thick_path = false;
        } else {
            logPrintln("Usage: plan_view [thick|thin]  (thick=car-radius width path)");
            return;
        }
    }

    if (g_map.grid == NULL || g_map.res <= 0.0f || g_map.grid_size <= 0 || g_map.grid_size > MAX_GRID_SIZE) {
        logPrintln("ERROR: planner map not ready");
        return;
    }

    uint16_t map_px = (uint16_t)(g_map.grid_size * LCD_PLAN_CELL_PX);
    uint16_t legend_x = (uint16_t)(LCD_PLAN_X0 + map_px + 8);

    /* 1. 底图：整块地图区铺空闲色，逐格覆盖障碍 */
    LCD_Port_Clear(LCD_COLOR_WHITE);
    LCDTools_FillSwatch(LCD_PLAN_X0, LCD_PLAN_Y0, map_px, LCD_PLAN_C_FREE);

    int obs_cnt = 0;
    for (int gx = 0; gx < g_map.grid_size; gx++) {
        for (int gy = 0; gy < g_map.grid_size; gy++) {
            if (!map_cell_is_obstacle(get_grid_val(&g_map, gx, gy))) continue;
            obs_cnt++;
            LCDTools_FillSwatch((uint16_t)(LCD_PLAN_X0 + gx * LCD_PLAN_CELL_PX),
                                (uint16_t)(LCD_PLAN_Y0 + (g_map.grid_size - 1 - gy) * LCD_PLAN_CELL_PX),
                                LCD_PLAN_CELL_PX,
                                LCD_PLAN_C_OBS);
        }
    }

    /* 2. 路径：thick = 按车的半径画成粗带，thin = 细线 */
    LCD_Port_SetTextColor(LCD_PLAN_C_PATH);
    uint16_t path_half_px = (uint16_t)(PHYSICAL_RADIUS / g_map.res * LCD_PLAN_CELL_PX);

    if (s_thick_path) {
        /* 规划器输出的是直线化后的拐点，相邻航点间距可能远大于半径，
         * 因此按半径在屏幕坐标上插值补点，拼成连续实心带 */
        for (int i = 0; i + 1 < g_plan_path_len; i++) {
            uint16_t x1, y1, x2, y2;
            LCDTools_MapToScreen(g_plan_path[i].x, g_plan_path[i].y, &x1, &y1);
            LCDTools_MapToScreen(g_plan_path[i + 1].x, g_plan_path[i + 1].y, &x2, &y2);

            int dx = (int)x2 - (int)x1;
            int dy = (int)y2 - (int)y1;
            int cheb = (abs(dx) > abs(dy)) ? abs(dx) : abs(dy); /* 切比雪夫距离，免开方 */
            int steps = (path_half_px > 0) ? (cheb / (int)path_half_px) : 1;
            if (steps < 1) steps = 1;

            for (int k = 0; k <= steps; k++) {
                LCD_Port_FillCircle((uint16_t)(x1 + dx * k / steps), (uint16_t)(y1 + dy * k / steps), path_half_px);
            }
        }
    } else {
        for (int i = 0; i + 1 < g_plan_path_len; i++) {
            uint16_t x1, y1, x2, y2;
            LCDTools_MapToScreen(g_plan_path[i].x, g_plan_path[i].y, &x1, &y1);
            LCDTools_MapToScreen(g_plan_path[i + 1].x, g_plan_path[i + 1].y, &x2, &y2);
            LCD_Port_DrawLine(x1, y1, x2, y2);
        }
    }

    /* 3. 标记：目标点（规划器缓存）与小车当前位姿（小车按半径画） */
    LCDTools_DrawMapMarker(goal_w_extend.x + MAP_OFFSET_X, goal_w_extend.y + MAP_OFFSET_Y, LCD_PLAN_C_GOAL, 4);

    PoseTimestamp_t pose;
    if (Loc_Get(&pose)) {
        /* Loc_Get 的位姿单位是 cm，换算成米后再平移进地图坐标 */
        LCDTools_DrawMapMarker(pose.pose.x * 0.01f + MAP_OFFSET_X,
                               pose.pose.y * 0.01f + MAP_OFFSET_Y,
                               LCD_PLAN_C_CAR,
                               (uint8_t)path_half_px);
    }

    /* 4. 右侧图例与状态 */
    sFONT *old_font = BSP_LCD_GetFont();
    LCD_Port_SetFont(&Font8);
    LCD_Port_SetBackColor(LCD_COLOR_WHITE);
    LCD_Port_SetTextColor(LCD_COLOR_BLACK);

    char line[32];
    uint16_t ly = LCD_PLAN_Y0;

    LCD_Port_DisplayStringAt(legend_x, ly, (uint8_t *)"PLAN VIEW", LEFT_MODE);
    ly += 20;

    LCDTools_FillSwatch(legend_x, ly, 8, LCD_PLAN_C_OBS);
    LCD_Port_SetTextColor(LCD_COLOR_BLACK);
    LCD_Port_DisplayStringAt(legend_x + 12, ly, (uint8_t *)"obstacle", LEFT_MODE);
    ly += 14;

    LCDTools_FillSwatch(legend_x, ly, 8, LCD_PLAN_C_PATH);
    LCD_Port_SetTextColor(LCD_COLOR_BLACK);
    LCD_Port_DisplayStringAt(legend_x + 12, ly, (uint8_t *)"path", LEFT_MODE);
    ly += 14;

    LCDTools_FillSwatch(legend_x, ly, 8, LCD_PLAN_C_GOAL);
    LCD_Port_SetTextColor(LCD_COLOR_BLACK);
    LCD_Port_DisplayStringAt(legend_x + 12, ly, (uint8_t *)"goal", LEFT_MODE);
    ly += 14;

    LCDTools_FillSwatch(legend_x, ly, 8, LCD_PLAN_C_CAR);
    LCD_Port_SetTextColor(LCD_COLOR_BLACK);
    LCD_Port_DisplayStringAt(legend_x + 12, ly, (uint8_t *)"car", LEFT_MODE);
    ly += 20;

    snprintf(line, sizeof(line), "obs=%d", obs_cnt);
    LCD_Port_DisplayStringAt(legend_x, ly, (uint8_t *)line, LEFT_MODE);
    ly += 14;

    snprintf(line, sizeof(line), "path=%d", g_plan_path_len);
    LCD_Port_DisplayStringAt(legend_x, ly, (uint8_t *)line, LEFT_MODE);

    LCD_Port_SetFont(old_font);
    logPrintln("Planner view: %dx%d, obs=%d, path=%d", g_map.grid_size, g_map.grid_size, obs_cnt, g_plan_path_len);
}

/**
 * @brief 一键导航到指定 id 的点位，并把本次规划结果画到 LCD 上
 * @note  等价于依次执行 `planner to <id>` 和 `lcd plan_view [thick|thin]`
 * @param argv <id> [thick|thin]，thick/thin 含义同 plan_view
 */
static void LCDTools_PlanAndView(int argc, char *argv[]) {
    if (argc < 2) {
        logPrintln("Usage: plan_and_view <id> [thick|thin]");
        return;
    }

    TargetPoint_t *target = Map_GetPoint((uint8_t)atoi(argv[1]));
    if (target == NULL) {
        logPrintln("ERROR: point id %s not found", argv[1]);
        return;
    }

    /* 阻塞到本次导航结束；返回时 g_plan_path 已缓存本次路径 */
    bool ok = Planner_GoToDirect(target);
    logPrintln("plan to id %s %s", argv[1], ok ? "done" : "failed");

    LCDTools_ShowPlannerView(argc - 1, argv + 1);
}

#endif // PLANNER_PLATFORM
#endif // PLANNER_ENABLED

ShellCommand LCDToolsGroup[] = {
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, display, LCDTools_Display, Display On / Off),
    SHELL_CMD_GROUP_ITEM(
        SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, set_text_color, LCDTools_SetTextColor, Set Text Color),
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, draw_line, LCDTools_DrawLine, Draw Line),
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, draw_rect, LCDTools_DrawRect, Draw Rect),
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, draw_circle, LCDTools_DrawCircle, Draw Circle),
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, clear, LCDTools_ClearScreen, Clear Screen),
    SHELL_CMD_GROUP_ITEM(
        SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, dis_str, LCDTools_DisplayString, Display String),
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, dis_zh, LCDTools_DisplayChinese, Display Zh),
#if PLANNER_ENABLED
#if PLANNER_PLATFORM == PLATFORM_MCU
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN,
                         plan_view,
                         LCDTools_ShowPlannerView,
                         Show Planner Map / Obstacle / Path),
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN,
                         plan_and_view,
                         LCDTools_PlanAndView,
                         Plan To ID And Show Planner View),
#endif
#endif
    SHELL_CMD_GROUP_END()};
SHELL_EXPORT_CMD_GROUP(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_DISABLE_RETURN,
                       lcd,
                       LCDToolsGroup,
                       LCD Tools);
