/**
 ****************************************************************************************************
 * @file        lcd.h
 * @author      EYA-DISPLAY
 * @version     V2.0
 * @date        2022-04-28
 * @brief       液晶屏驱动Demo
 * @license     Copyright (c) 2022-2032, 亦亞徽科技集团(广东)
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:EYA-ETmcu开发板
 * 公司网址:www.eya-display.com
 *
 ****************************************************************************************************
 **/
#ifndef __LCD_H
#define __LCD_H		
#include "stdint.h"


/*****************************************需要修改的地方 1 ***************************/
#define LCD_WIDTH  240
#define LCD_HEIGHT 320   //更换屏幕对应修改分辨率 

/*****************************************需要修改的地方 2 ***************************/

#define Interface  I80_16BIT    //I80_16BIT//I80_H8BIT//I80_L8BIT//D4WSPI  //对应显示屏的接口
//#define Interface  I80_H8BIT
//#define Interface  I80_L8BIT
//#define Interface  D4WSPI

/*****************************************需要修改的地方 3 ***************************/
//第三处修改在LCD.C中 LCD_INIT_CODE()  30行开始

/*****************************************需要修改的地方 4 ***************************/
//横竖屏切换
#define Landscape  0 //1:横显示  0：竖显示  PS:横竖按照FPC的出线方向来看  


#define FAST     1  //快速刷图  1:快速刷图  0：慢刷   快刷比慢刷快24%


//LCD重要参数集
typedef struct  
{										    
	uint16_t width;			//LCD 宽度
	uint16_t height;			//LCD 高度
	uint16_t id;				  //LCD ID
	uint8_t  dir;			  //横屏还是竖屏控制：0，竖屏；1，横屏。	
	uint8_t	wramcmd;		//开始写gram指令
	uint8_t  setxcmd;		//设置x坐标指令
	uint8_t  setycmd;		//设置y坐标指令	 
}_lcd_dev; 	  

//LCD参数
extern _lcd_dev lcddev;	//管理LCD重要参数
//LCD的画笔颜色和背景色	   
extern uint16_t  POINT_COLOR;//默认红色    
extern uint16_t  BACK_COLOR; //背景颜色.默认为白色
//////////////////////////////////////////////////////////////////////////////////	 
//-----------------LCD端口定义---------------- 
// #define	LCD_LED PAout(3)   //LCD    		 BL_EN 

// #define	LCD_RST PCout(13)   //LCD    		 RESET
// #if(Interface==D4WSPI)
// 	#define	LCD_CS  PBout(12)  //LCD spi接口的CS用这个脚
// #else
// 	#define	LCD_CS  PAout(8)   //LCD   		 CS
// #endif

// #define	LCD_RS  PAout(11)   //LCD   		 RS
// #define	LCD_WR  PAout(12)   //LCD     	 WR
// #define	LCD_RD  PAout(15)   //LCD    		 RD
	 
//扫描方向定义
#define L2R_U2D  0 //从左到右,从上到下
#define L2R_D2U  1 //从左到右,从下到上
#define R2L_U2D  2 //从右到左,从上到下
#define R2L_D2U  3 //从右到左,从下到上

#define U2D_L2R  4 //从上到下,从左到右
#define U2D_R2L  5 //从上到下,从右到左
#define D2U_L2R  6 //从下到上,从左到右
#define D2U_R2L  7 //从下到上,从右到左	 

extern uint8_t DFT_SCAN_DIR;


//PB0~15,作为数据口
// #if((Interface==I80_16BIT)||(Interface==I80_L8BIT))
// 	#define DATAOUT(x) GPIOB->ODR=x; //数据输出
// #elif(Interface==I80_H8BIT)
//   #define DATAOUT(x) GPIOB->ODR=(GPIOB->ODR&0X00FF)|((uint16_t)(x<<8)&0xFF00)    //数据输出
// #endif

//画笔颜色
#define WHITE         	 0xFFFF
#define BLACK         	 0x0000	  
#define BLUE         	   0x001F  
#define BRED             0XF81F
#define GRED 			 	     0XFFE0
#define GBLUE			       0X07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			     0XBC40 //棕色
#define BRRED 			     0XFC07 //棕红色
#define GRAY  			     0X8430 //灰色
//GUI颜色

#define DARKBLUE      	 0X01CF	//深蓝色
#define LIGHTBLUE      	 0X7D7C	//浅蓝色  
#define GRAYBLUE       	 0X5458 //灰蓝色
//以上三色为PANEL的颜色 
 
#define LIGHTGREEN     	 0X841F //浅绿色
//#define LIGHTGRAY      0XEF5B //浅灰色(PANNEL)
#define LGRAY 			     0XC618 //浅灰色(PANNEL),窗体背景色

#define LGRAYBLUE        0XA651 //浅灰蓝色(中间层颜色)
#define LBBLUE           0X2B12 //浅棕蓝色(选择条目的反色)
	    															  
void LCD_Init(void);													   	//初始化
void LCD_DisplayOn(void);													//开显示
void LCD_DisplayOff(void);													//关显示
void LCD_Clear(uint16_t Color);	 												//清屏
void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos);										//设置光标
void LCD_DrawPoint(uint16_t x,uint16_t y, uint32_t color);											//画点
void LCD_Fast_DrawPoint(uint16_t x,uint16_t y,uint16_t color);								//快速画点
uint16_t  LCD_ReadPoint(uint16_t x,uint16_t y); 											//读点 
void Draw_Circle(uint16_t x0,uint16_t y0,uint8_t r,uint16_t color);										//画圆
void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,uint16_t color);							//画线
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,uint16_t color);		   				//画矩形
void LCD_Fill(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint16_t color);		   				//填充单色
void LCD_Color_Fill(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint16_t *color);				//填充指定颜色
void LCD_ShowChar(uint16_t x, uint16_t y, char chr, uint8_t size, uint8_t mode, uint16_t color);						//显示一个字符
void LCD_ShowNum(uint16_t x,uint16_t y,uint32_t num,uint8_t len,uint8_t size,uint16_t color);  						//显示一个数字
void LCD_ShowxNum(uint16_t x,uint16_t y,uint32_t num,uint8_t len,uint8_t size,uint8_t mode,uint16_t color);				//显示 数字
void LCD_ShowString(uint16_t x,uint16_t y,uint16_t width,uint16_t height,uint8_t size,uint8_t *p,uint16_t color);		//显示一个字符串,12/16字体
void LCD_Set_Window(uint16_t sx,uint16_t sy,uint16_t width,uint16_t height);
void IO_init(void);


void LCD_WR_REG(uint8_t);
void LCD_WR_DATA(uint16_t);
void LCD_WriteReg(uint8_t LCD_Reg, uint16_t LCD_RegValue);
uint16_t LCD_ReadReg(uint8_t LCD_Reg);
void LCD_WriteRAM_Prepare(void);
void LCD_WriteRAM(uint16_t RGB_Code);		  
void LCD_Scan_Dir(uint8_t dir);							//设置屏扫描方向
void LCD_Display_Dir(uint8_t dir);						//设置屏幕显示方向
void TIM2_PWM_Init(uint16_t arr,uint16_t psc);
void SendPWMval(uint16_t value);
 					   																			 			  		 
#endif  
	 
	 



