#ifndef __ST7789_H__
#define __ST7789_H__

#include "stm32f4xx_hal.h"

#define ST7789_REG        *(volatile uint16_t *)(0x6C000000)
#define ST7789_RAM        *(volatile uint16_t *)(0x6C000080) //A16 右移1位

/*******************供bsp调用的接口API**********************/
//背景光控制

#define LCD_WIDTH   320  // 显示屏宽度像素
#define LCD_HEIGHT  240  // 显示屏高度像素
#define LCDDRV_RAM  ST7789_RAM      //显示GRAM接收地址

void lcddrv_Init(void);      //初始化函数
void lcddrv_FillColorPre(uint16_t X, uint16_t Y, uint16_t Width , uint16_t Height);
void lcddrv_DispOn(void);    //打开显示
void lcddrv_DispOff(void);   //关闭显示

void lcddrv_SetCursor(uint16_t Xpos, uint16_t Ypos);
void lcddrv_DrawPoint(uint16_t x,uint16_t y, uint32_t color);
void lcddrv_ShowChar(uint16_t x, uint16_t y, char chr, uint8_t size, uint8_t mode, uint16_t color);
void lcddrv_ShowString(uint16_t x,uint16_t y,uint16_t width,uint16_t height,uint8_t size,uint8_t *p,uint16_t color);
#endif
