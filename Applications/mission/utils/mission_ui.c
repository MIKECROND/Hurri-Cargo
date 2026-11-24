/**
 * @file mission_ui.c
 * @author T-Car Team
 * @brief 总任务 LCD 显示实现
 * 可调参数见 mission_config.h / Applications/mission/README.md。
 */

#include "mission_ui.h"
#include "lcd_port.h"
#include "stm32_adafruit_lcd.h"
#include <string.h>

void Mission_UiShowBarcode(uint8_t *barcode, bool valid) {
    sFONT *font = BSP_LCD_GetFont();
    LCD_Port_Clear(LCD_COLOR_WHITE);

    if (valid) {
        /* 符合 xxx+xxx+xxx+xxx：在第二个 '+' 之后断行，三个 '+' 都保留
         * 第一行 xxx+xxx+，第二行 xxx+xxx */
        char *plus = strchr((char *)barcode, '+');
        plus = strchr(plus + 1, '+'); /* 第二个 '+' */
        char *p1 = plus + 1; /* 第二个 '+' 之后的第一个字符 */
        char keep = *p1;

        *p1 = '\0'; /* 临时截断为第一行 */
        LCD_Port_DisplayStringAt(1, 30, barcode, CENTER_MODE);

        *p1 = keep; /* 恢复后从 p1 起为第二行 */
        LCD_Port_DisplayStringAt(1, 30 + font->Height, (uint8_t *)p1, CENTER_MODE);
    } else {
        /* 不符合格式：用 Font_note 自动换行完整显示内容 */
        uint16_t per = LCD_Port_GetXSize() / Font_note.Width; /* 每行最多字符数 */
        uint16_t total = (uint16_t)strlen((const char *)barcode);
        const char *p = (const char *)barcode;
        uint16_t y = 30;

        if (per == 0) per = 1;
        LCD_Port_SetFont(&Font_note);

        while (total > 0) {
            uint16_t n = (total > per) ? per : total;
            char *seg = (char *)p;
            char saved = seg[n];

            seg[n] = '\0'; /* 临时截断为一行 */
            LCD_Port_DisplayStringAt(1, y, (uint8_t *)seg, CENTER_MODE);
            seg[n] = saved; /* 恢复，继续下一段 */

            p += n;
            total -= n;
            y += Font_note.Height;

            /* 超出屏幕底部则停止 */
            if (y + Font_note.Height > LCD_Port_GetYSize()) break;
        }
        LCD_Port_SetFont(font); /* 恢复原字体 */
    }
}

void Mission_UiShowComplete(int pick_count, int place_count) {
    LCD_Port_Clear(LCD_COLOR_WHITE);
    LCD_Port_DisplayPickCount(50, 50, pick_count);
    LCD_Port_DisplayPlaceCount(50, 170, place_count);
}
