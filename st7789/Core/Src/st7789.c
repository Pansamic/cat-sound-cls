#include <stdint.h>
#include <stdio.h>
#include <st7789.h>
#include <font.h>
#include <main.h>

#define lcddelay_ms(n) HAL_Delay(n)

//ST7789寄存器定义
#define  ST7789_CMD_RDDID 0x04  //读取ID
#define  ST7789_CMD_SLPIN 0x10
#define  ST7789_CMD_SLPOUT 0x11
#define  ST7789_CMD_DISPOFF 0x28
#define  ST7789_CMD_DISPON 0x29
#define  ST7789_CMD_CASETF 0x2A
#define  ST7789_CMD_RASETF 0x2B
#define  ST7789_CMD_RAMWR 0x2C  //开始写内存数据
#define  ST7789_CMD_PTLON 0x12  //部分显示模式
#define  ST7789_CMD_NORON 0x13  //普通模式
#define  ST7789_CMD_INVOFF 0x20 //关闭反显示
#define  ST7789_CMD_INVON 0x21  //打开反显示
#define  ST7789_CMD_PTLAR 0x30  //部分显示区域 由上下两个水平坐标

//写寄存器(CMD)
//regval:寄存器值
static void ST7789_WR_REG(uint16_t regval)
{
    ST7789_REG = regval;
}
//写数据
//data:要写入的值
static void ST7789_WR_DATA(uint16_t data)
{
    ST7789_RAM = data;
}
//读LCD数据
//返回值:读到的值
static uint16_t ST7789_RD_DATA(void)
{
    uint16_t ram;
    ram = ST7789_RAM;
    return ram;
}

/*
*********************************************************************************************
*    函 数 名: ST7789_ReadID
*    功能说明: 读取LCD驱动芯片ID
*    形    参:  无
*    返 回 值: 无
*********************************************************************************************
*/
uint16_t ST7789_ReadID(void)
{
    uint16_t id = 0;
    ST7789_WR_REG(ST7789_CMD_RDDID);
    ST7789_RD_DATA();
    id = ST7789_RD_DATA();
    ST7789_RD_DATA();
    ST7789_RD_DATA() ;
    return id ;
}

/*
****************************************************************************************
*    函 数 名: ST7789_REG_Init
*    功能说明: 初始化LCD驱动芯片寄存器
*    形    参:  无
*    返 回 值: 无
****************************************************************************************
*/
static void ST7789_REG_Init(void)
{
    /* 初始化LCD，写LCD寄存器进行配置 */
    lcddelay_ms(10);
    ST7789_WR_REG(ST7789_CMD_SLPOUT);    //exit SLEEP mode
    lcddelay_ms(150);

    ST7789_WR_REG(0x00D0);  //PWCTRL1: Power Control 1
    ST7789_WR_DATA(0x00A4);
    ST7789_WR_DATA(0x81);

    ST7789_WR_REG(0x00BB);
    ST7789_WR_DATA(0x0030);  //VCOMS: VCOM setting

    ST7789_WR_REG(0x0036);  //memory data access control
    ST7789_WR_DATA(0x60);   //MADCTL: 方向等等

    ST7789_WR_REG(0x003A);  // Interface Pixel Format
    ST7789_WR_DATA(0x05);   //COLMOD: 16bit

    ST7789_WR_REG(0x00E0);
    ST7789_WR_DATA(0x00D0);
    ST7789_WR_DATA(0x0000);
    ST7789_WR_DATA(0x0002);
    ST7789_WR_DATA(0x0007);
    ST7789_WR_DATA(0x000B);
    ST7789_WR_DATA(0x001A);
    ST7789_WR_DATA(0x0031);
    ST7789_WR_DATA(0x0054);
    ST7789_WR_DATA(0x0040);
    ST7789_WR_DATA(0x0029);
    ST7789_WR_DATA(0x0012);
    ST7789_WR_DATA(0x0012);
    ST7789_WR_DATA(0x0012);
    ST7789_WR_DATA(0x0017);

    ST7789_WR_REG(0x00E1);
    ST7789_WR_DATA(0x00D0);
    ST7789_WR_DATA(0x0000);
    ST7789_WR_DATA(0x0002);
    ST7789_WR_DATA(0x0007);
    ST7789_WR_DATA(0x0005);
    ST7789_WR_DATA(0x0025);
    ST7789_WR_DATA(0x002D);
    ST7789_WR_DATA(0x0044);
    ST7789_WR_DATA(0x0045);
    ST7789_WR_DATA(0x001C);
    ST7789_WR_DATA(0x0018);
    ST7789_WR_DATA(0x0016);
    ST7789_WR_DATA(0x001C);
    ST7789_WR_DATA(0x001D);

    ST7789_WR_REG(0x21);//取反

    ST7789_WR_REG(ST7789_CMD_DISPON);// DISPALY ON
    lcddelay_ms(100);
}


/*
***************************************************************************************************
*    函 数 名: ST7789_Init
*    功能说明: 初始化LCD
*    形    参: 无
*    返 回 值: 0/1：失败/正确
***************************************************************************************************
*/
uint8_t ST7789_Init(void)
{
    //硬重启
    HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
    lcddelay_ms(3);
    HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);
    lcddelay_ms(6);
    HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
    lcddelay_ms(150);

    if ( ST7789_ReadID() == 0x85 )
    {
        ST7789_REG_Init();    /* 初始化屏IC寄存器 */
        return 1;
    }
    return 0;
}


/*
*************************************************************************************************
*    函 数 名: lcddrv_DispOn
*    功能说明: 打开显示
*    形    参: 无
*    返 回 值: 无
*************************************************************************************************
*/
void lcddrv_DispOn(void)
{
    ST7789_WR_REG(ST7789_CMD_DISPON);
}

/*
*************************************************************************************************
*    函 数 名: lcddrv_DispOff
*    功能说明: 关闭显示
*    形    参: 无
*    返 回 值: 无
*************************************************************************************************
*/
void lcddrv_DispOff(void)
{
    ST7789_WR_REG(ST7789_CMD_DISPOFF);
}

/*
*************************************************************************************************
*    函 数 名: lcddrv_FillColorPre
*    功能说明:  设置将要显示的区域。使用此函数后，只需要顺序赋值给LCDDRV_RAM即可绘图此区域.
*    形    参: 无
*    返 回 值: 无
*************************************************************************************************
*/
void lcddrv_FillColorPre(uint16_t _usX, uint16_t _usY, uint16_t _usWidth , uint16_t _usHeight)
{
    //显示区域限制
    ST7789_WR_REG(ST7789_CMD_CASETF);
    ST7789_WR_DATA(_usX >> 8);
    ST7789_WR_DATA(_usX );
    ST7789_WR_DATA( (_usX + _usWidth - 1) >> 8);
    ST7789_WR_DATA( (_usX + _usWidth - 1) );

    ST7789_WR_REG(ST7789_CMD_RASETF );
    ST7789_WR_DATA(_usY >> 8);
    ST7789_WR_DATA(_usY );
    ST7789_WR_DATA( (_usY + _usHeight - 1) >> 8 );
    ST7789_WR_DATA( (_usY + _usHeight - 1) );
    //准备接收数据
    ST7789_REG = ST7789_CMD_RAMWR;
}

/*
*************************************************************************************************
*    函 数 名: lcddrv_Init
*    功能说明: LCD初始化函数
*    形    参:  _
*    返 回 值: 无
**************************************************************************************************
*/
void lcddrv_Init(void)
{
    if ( ST7789_Init() )
    {
        printf("LCD ok r\n");
    }else
    {
        printf("LCD err \r\n");
    }
}

void lcddrv_SetCursor(uint16_t Xpos, uint16_t Ypos)
{
	ST7789_WR_REG(0x2A);
	ST7789_WR_DATA(Xpos>>8);
	ST7789_WR_DATA(Xpos&0XFF);
	ST7789_WR_DATA(Xpos>>8);
	ST7789_WR_DATA(Xpos&0XFF);

	ST7789_WR_REG(0x2B);
	ST7789_WR_DATA(Ypos>>8);
	ST7789_WR_DATA(Ypos&0XFF);
	ST7789_WR_DATA(Ypos>>8);
	ST7789_WR_DATA(Ypos&0XFF);
}

void lcddrv_DrawPoint(uint16_t x,uint16_t y, uint32_t color)
{
	lcddrv_SetCursor(x,y);		//设置光标位置
	ST7789_WR_REG(0x2C);	//开始写入GRAM
	ST7789_WR_DATA(color);
}

void lcddrv_ShowChar(uint16_t x, uint16_t y, char chr, uint8_t size, uint8_t mode, uint16_t color)
{
    uint8_t temp, t1, t;
    uint16_t y0 = y;
    uint8_t csize = 0;
    uint8_t *pfont = 0;

    csize = (size / 8 + ((size % 8) ? 1 : 0)) * (size / 2); /* 得到字体一个字符对应点阵集所占的字节数 */
    chr = chr - ' ';    /* 得到偏移后的值（ASCII字库是从空格开始取模，所以-' '就是对应字符的字库） */

    switch (size)
    {
        case 12:
            pfont = (uint8_t *)asc2_1206[chr];  /* 调用1206字体 */
            break;

        case 16:
            pfont = (uint8_t *)asc2_1608[chr];  /* 调用1608字体 */
            break;

        default:
            return ;
    }

    for (t = 0; t < csize; t++)
    {
        temp = pfont[t];    /* 获取字符的点阵数据 */

        for (t1 = 0; t1 < 8; t1++)   /* 一个字节8个点 */
        {
            if (temp & 0x80)        /* 有效点,需要显示 */
            {
                lcddrv_DrawPoint(x, y, color);        /* 画点出来,要显示这个点 */
            }
            else if (mode == 0)     /* 无效点,不显示 */
            {
                lcddrv_DrawPoint(x, y, 0); // no global variable maintaining background color currently
            }

            temp <<= 1; /* 移位, 以便获取下一个位的状态 */
            y++;

            if (y >= LCD_HEIGHT)return;  /* 超区域了 */

            if ((y - y0) == size)   /* 显示完一列了? */
            {
                y = y0; /* y坐标复位 */
                x++;    /* x坐标递增 */

                if (x >= LCD_WIDTH)return;   /* x坐标超区域了 */

                break;
            }
        }
    }
}


void lcddrv_ShowString(uint16_t x,uint16_t y,uint16_t width,uint16_t height,uint8_t size,uint8_t *p,uint16_t color)
{
	uint8_t x0=x;
	width+=x;
	height+=y;
    while((*p<='~')&&(*p>=' '))//判断是不是非法字符!
    {
        if(x>=width){x=x0;y+=size;}
        if(y>=height)break;//退出
        lcddrv_ShowChar(x,y,*p,size,0,color);
        x+=size/2;
        p++;
    }
}
