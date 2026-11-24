/**
 * @file lcd_port.h
 * @author T-Car Team
 * @brief LCD 端口：对外统一引出 LCD 各层接口的初始化与控制入口
 *
 * 依赖：
 * - main.c 中已调用 MX_GPIO_Init()/MX_SPI1_Init()，本模块不重复初始化外设；
 * - 本模块把 LCD 库的分层初始化（IO 接口 -> 屏幕驱动/图形库 -> 背光）
 *   集中封装为 LCD_Port_Init()，业务代码只需包含本头文件并调用它。
 * 可调参数见 lcd_port.h / LCD 配置宏。
 */

#ifndef __LCD_PORT_H__
#define __LCD_PORT_H__

#if defined(__cplusplus)
extern "C" {
#endif

#include <stdint.h>

/* 引出自带的图形库(BSP_LCD_*)接口：文本、形状、位图、读屏等 */
#include "stm32_adafruit_lcd.h"

/**
 * @brief 初始化 LCD 底层 IO/SPI 接口
 * @note  封装 lcd_io_spi_hal 的 LCD_IO_Init()：
 *        完成 SPI 读写速率切换、数据方向与传输控制初始化。
 *        必须在 MX_SPI1_Init() 之后调用。
 */
void LCD_Port_IoInit(void);

/**
 * @brief 初始化屏幕驱动与上层图形库
 * @note  封装 BSP_LCD_Init()：内部执行 ILI9488 完整初始化序列，
 *        并设置默认字体/颜色（可在 LCD_Port_IoInit 之后单独调用）。
 */
void LCD_Port_DriverInit(void);

/**
 * @brief LCD 一键初始化：IO 接口 -> 驱动/图形库 -> 开启背光
 * @note  调用前需确保 MX_GPIO_Init()/MX_SPI1_Init() 已执行（main.c 已调用）；
 *        建议在 osKernelInitialize() 之前（main 中）或显示任务创建时调用一次。
 */
void LCD_Port_Init(void);

/**
 * @brief 背光控制
 * @param On 1 = 开，0 = 关（未配置 BL 引脚时为空操作）
 */
void LCD_Port_Backlight(uint8_t On);

/**
 * @brief 显示开启（退出休眠）
 */
void LCD_Port_DisplayOn(void);

/**
 * @brief 显示关闭（进入休眠）
 */
void LCD_Port_DisplayOff(void);

/* ============================================================================
 * 显示(绘图)接口：对上层图形库 stm32_adafruit_lcd 的 BSP_LCD_* 常用接口
 * 做 1:1 封装，统一以 LCD_Port_* 前缀对外，业务代码不再直接依赖 BSP 层。
 * ========================================================================== */

/* 画刷属性 */
void LCD_Port_SetTextColor(uint16_t Color);
void LCD_Port_SetBackColor(uint16_t Color);
void LCD_Port_SetFont(sFONT *Fonts);

/* 清屏/文本 */
void LCD_Port_Clear(uint16_t Color);
void LCD_Port_ClearStringLine(uint16_t Line);
void LCD_Port_DisplayChar(uint16_t Xpos, uint16_t Ypos, uint8_t Ascii);
void LCD_Port_DisplayStringAt(uint16_t Xpos, uint16_t Ypos, uint8_t *Text, Line_ModeTypdef Mode);
void LCD_Port_DisplayStringAtLine(uint16_t Line, uint8_t *Ptr);

/* 中文字形/统计文本（Font_zh 中文字库 + Font120 数字） */
/* 按字形序号在任意字体表里画一个字（0 = 第一个字形） */
void LCD_Port_DrawGlyph(uint16_t Xpos, uint16_t Ypos, const sFONT *Font, uint8_t GlyphIndex);
/* 显示“正确抓取数:Count”，同一行自左向右（Font_zh 80x120 + “:”与数字 Font120 40x120） */
void LCD_Port_DisplayPickCount(uint16_t Xpos, uint16_t Ypos, uint16_t Count);
/* 显示“正确放置数:Count”，同一行自左向右（Font_zh 80x120 + “:”与数字 Font120 40x120） */
void LCD_Port_DisplayPlaceCount(uint16_t Xpos, uint16_t Ypos, uint16_t Count);

/* 点/线/框/圆 */
void LCD_Port_DrawPixel(uint16_t Xpos, uint16_t Ypos, uint16_t RGB_Code);
void LCD_Port_DrawHLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length);
void LCD_Port_DrawVLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length);
void LCD_Port_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void LCD_Port_DrawRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height);
void LCD_Port_DrawCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius);
void LCD_Port_FillRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height);
void LCD_Port_FillCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius);
void LCD_Port_FillTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3);

/* 图像/滚动 */
void LCD_Port_DrawBitmap(uint16_t Xpos, uint16_t Ypos, uint8_t *pBmp);
void LCD_Port_DrawRGB16Image(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint16_t *pData);
void LCD_Port_Scroll(int16_t Scroll, uint16_t TopFix, uint16_t BottonFix);

/* 屏幕尺寸查询 */
uint16_t LCD_Port_GetXSize(void);
uint16_t LCD_Port_GetYSize(void);

#if defined(__cplusplus)
}
#endif

#endif /* __LCD_PORT_H__ */
