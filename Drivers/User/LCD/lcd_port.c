/**
 * @file lcd_port.c
 * @author T-Car Team
 * @brief LCD 端口实现：把 LCD 各层接口（初始化 + 显示绘图）集中引出一遍
 *
 * 分层说明：
 *   LCD_Port_IoInit()      -> lcd_io_spi_hal 底层 IO/SPI 接口（速率/方向等）
 *   LCD_Port_DriverInit()  -> BSP_LCD_Init()，内部执行 ILI9488 初始化序列
 *   LCD_Port_Init()        -> 依次完成 IO -> 驱动 -> 背光
 *   LCD_Port_* 绘图接口    -> BSP_LCD_* 常用显示接口的 1:1 封装
 * 可调参数见 lcd_port.h / LCD 配置宏。
 */

#include "lcd_port.h"

#include "lcd_io.h" /* LCD_IO_Init / LCD_IO_Bl_OnOff */

#include "spi.h"

/**
 * @brief LCD 一键初始化
 */
void LCD_Port_Init(void) {
    MX_SPI1_Init();
    LCD_IO_Init(); /* 1. IO/SPI 接口初始化 */
    BSP_LCD_Init(); /* 2. ILI9488 驱动 + 图形库初始化（含面板上电显示） */
    LCD_Port_Backlight(1); /* 3. 打开背光（未配置 BL 引脚时为空操作） */

    LCD_Port_Clear(LCD_COLOR_WHITE);
    LCD_Port_SetTextColor(LCD_COLOR_WHITE);
}

/**
 * @brief 背光控制
 * @param On 1 = 开，0 = 关
 */
void LCD_Port_Backlight(uint8_t On) {
    LCD_IO_Bl_OnOff(On);
}

/**
 * @brief 显示开启（退出休眠）
 */
void LCD_Port_DisplayOn(void) {
    BSP_LCD_DisplayOn();
}

/**
 * @brief 显示关闭（进入休眠）
 */
void LCD_Port_DisplayOff(void) {
    BSP_LCD_DisplayOff();
}

void LCD_Port_SetTextColor(uint16_t Color) {
    BSP_LCD_SetTextColor(Color);
}

void LCD_Port_SetBackColor(uint16_t Color) {
    BSP_LCD_SetBackColor(Color);
}

void LCD_Port_SetFont(sFONT *Fonts) {
    BSP_LCD_SetFont(Fonts);
}

/* 清屏/文本 */
void LCD_Port_Clear(uint16_t Color) {
    BSP_LCD_Clear(Color);
}

void LCD_Port_ClearStringLine(uint16_t Line) {
    BSP_LCD_ClearStringLine(Line);
}

void LCD_Port_DisplayChar(uint16_t Xpos, uint16_t Ypos, uint8_t Ascii) {
    BSP_LCD_DisplayChar(Xpos, Ypos, Ascii);
}

void LCD_Port_DisplayStringAt(uint16_t Xpos, uint16_t Ypos, uint8_t *Text, Line_ModeTypdef Mode) {
    BSP_LCD_DisplayStringAt(Xpos, Ypos, Text, Mode);
}

void LCD_Port_DisplayStringAtLine(uint16_t Line, uint8_t *Ptr) {
    BSP_LCD_DisplayStringAtLine(Line, Ptr);
}

/* ============================================================================
 * 中文字形与统计文本显示
 *   中文标签字形取自 Font_zh（正/确/抓/取/放/置/数，字形序号 0..6）
 *   “:”与数量数字取自 Font120
 * ========================================================================== */

/* Font_zh 字形序号 */
#define LCD_ZH_ZHENG 0u /* 正 */
#define LCD_ZH_QUE 1u /* 确 */
#define LCD_ZH_ZHU 2u /* 抓 */
#define LCD_ZH_QU 3u /* 取 */
#define LCD_ZH_FANG 4u /* 放 */
#define LCD_ZH_ZHI 5u /* 置 */
#define LCD_ZH_SHU 6u /* 数 */

void LCD_Port_DrawGlyph(uint16_t Xpos, uint16_t Ypos, const sFONT *Font, uint8_t GlyphIndex) {
    BSP_LCD_DrawFontChar(Xpos, Ypos, Font, GlyphIndex);
}

/* 按字形表自左向右输出：中文标签 -> “:” -> 十进制数量（同一种数字字体） */
static void LCD_Port_DrawZhNum(uint16_t Xpos, uint16_t Ypos, const uint8_t *ZhIndex, uint8_t ZhCount, uint16_t Count) {
    uint16_t x = Xpos;
    uint16_t yNum; /* 冒号与数字的纵向位置：比中文标签提高 20 像素 */
    uint8_t i, n;
    uint16_t tmp;
    char str[6]; /* uint16 最大 5 位 + 结束符 */

    yNum = (Ypos >= 20) ? (Ypos - 20) : 0;

    for (i = 0; i < ZhCount; i++) {
        LCD_Port_DrawGlyph(x, Ypos, &Font_zh, ZhIndex[i]);
        x += Font_zh.Width;
    }

    /* 冒号 */
    LCD_Port_DrawGlyph(x, yNum, &Font120, (uint8_t)(':' - ' '));
    x += Font120.Width;

    /* 十进制转换（低位先存，最后反转输出） */
    tmp = Count;
    n = 0;
    if (tmp == 0) {
        str[n++] = '0';
    } else {
        while ((tmp > 0) && (n < 5)) {
            str[n++] = (char)('0' + (tmp % 10));
            tmp /= 10;
        }
    }
    for (i = 0; i < n; i++) {
        LCD_Port_DrawGlyph(x, yNum, &Font120, (uint8_t)(str[n - 1 - i] - ' '));
        x += Font120.Width;
    }
}

/* 正确抓取数:Count */
void LCD_Port_DisplayPickCount(uint16_t Xpos, uint16_t Ypos, uint16_t Count) {
    static const uint8_t ZhPick[] = {LCD_ZH_ZHENG, LCD_ZH_QUE, LCD_ZH_ZHU, LCD_ZH_QU, LCD_ZH_SHU};
    LCD_Port_DrawZhNum(Xpos, Ypos, ZhPick, (uint8_t)(sizeof(ZhPick) / sizeof(ZhPick[0])), Count);
}

/* 正确放置数:Count */
void LCD_Port_DisplayPlaceCount(uint16_t Xpos, uint16_t Ypos, uint16_t Count) {
    static const uint8_t ZhPlace[] = {LCD_ZH_ZHENG, LCD_ZH_QUE, LCD_ZH_FANG, LCD_ZH_ZHI, LCD_ZH_SHU};
    LCD_Port_DrawZhNum(Xpos, Ypos, ZhPlace, (uint8_t)(sizeof(ZhPlace) / sizeof(ZhPlace[0])), Count);
}

/* 点/线/框/圆 */
void LCD_Port_DrawPixel(uint16_t Xpos, uint16_t Ypos, uint16_t RGB_Code) {
    BSP_LCD_DrawPixel(Xpos, Ypos, RGB_Code);
}

void LCD_Port_DrawHLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length) {
    BSP_LCD_DrawHLine(Xpos, Ypos, Length);
}

void LCD_Port_DrawVLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length) {
    BSP_LCD_DrawVLine(Xpos, Ypos, Length);
}

void LCD_Port_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    BSP_LCD_DrawLine(x1, y1, x2, y2);
}

void LCD_Port_DrawRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height) {
    BSP_LCD_DrawRect(Xpos, Ypos, Width, Height);
}

void LCD_Port_DrawCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius) {
    BSP_LCD_DrawCircle(Xpos, Ypos, Radius);
}

void LCD_Port_FillRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height) {
    BSP_LCD_FillRect(Xpos, Ypos, Width, Height);
}

void LCD_Port_FillCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius) {
    BSP_LCD_FillCircle(Xpos, Ypos, Radius);
}

void LCD_Port_FillTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3) {
    BSP_LCD_FillTriangle(x1, y1, x2, y2, x3, y3);
}

/* 图像/滚动 */
void LCD_Port_DrawBitmap(uint16_t Xpos, uint16_t Ypos, uint8_t *pBmp) {
    BSP_LCD_DrawBitmap(Xpos, Ypos, pBmp);
}

void LCD_Port_DrawRGB16Image(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint16_t *pData) {
    BSP_LCD_DrawRGB16Image(Xpos, Ypos, Xsize, Ysize, pData);
}

void LCD_Port_Scroll(int16_t Scroll, uint16_t TopFix, uint16_t BottonFix) {
    BSP_LCD_Scroll(Scroll, TopFix, BottonFix);
}

/* 屏幕尺寸查询 */
uint16_t LCD_Port_GetXSize(void) {
    return BSP_LCD_GetXSize();
}

uint16_t LCD_Port_GetYSize(void) {
    return BSP_LCD_GetYSize();
}
