#ifndef __OLED_H
#define __OLED_H			  	 
#include "stm32l4xx_hal.h"
#include "stdlib.h"	    	

#define SIZE 16
#define Max_Column	128
#define Max_Row		64
   						  
#define  u8  unsigned char
#define  u32 unsigned int 
#define OLED_SCLK_Clr() HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8, GPIO_PIN_RESET)//CLK
#define OLED_SCLK_Set() HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8, GPIO_PIN_SET)

#define OLED_SDIN_Clr() HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9, GPIO_PIN_RESET)//DIN
#define OLED_SDIN_Set() HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9, GPIO_PIN_SET)

#define OLED_RST_Clr() HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12, GPIO_PIN_RESET)//RES
#define OLED_RST_Set() HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12, GPIO_PIN_SET)

#define OLED_DC_Clr() HAL_GPIO_WritePin(GPIOD,GPIO_PIN_11, GPIO_PIN_RESET)
#define OLED_DC_Set() HAL_GPIO_WritePin(GPIOD,GPIO_PIN_11, GPIO_PIN_SET)
 
//#define OLED_CS_Clr()  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4, GPIO_PIN_RESET)
//#define OLED_CS_Set()  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4, GPIO_PIN_SET)
 
#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据


//OLED控制用函数
void OLED_ColorTurn(u8 i);
void OLED_DisplayTurn(u8 i);
void OLED_WR_Byte(u8 dat,u8 cmd);
void OLED_DisPlay_On(void);
void OLED_DisPlay_Off(void);
void OLED_Refresh(void);
void OLED_Clear(void);
void OLED_DrawPoint(unsigned char x, unsigned char y, unsigned char t);
void OLED_DrawLine(u8 x1,u8 y1,u8 x2,u8 y2);
void OLED_DrawCircle(u8 x,u8 y,u8 r);
void OLED_ShowChar(unsigned char x,unsigned char y,unsigned char chr,unsigned  char size,unsigned  char mode);
void OLED_ShowString(u8 x,u8 y,u8 *chr,u8 size1);
void OLED_ShowNum(unsigned char x,unsigned char y, unsigned int num,unsigned char len,unsigned char size);
//void OLED_ShowChinese(u8 x,u8 y,u8 num,u8 size1);
void OLED_ScrollDisplay(u8 num,u8 space);
void OLED_WR_BP(u8 x,u8 y);
void OLED_ShowPicture(u8 x0,u8 y0,u8 x1,u8 y1,u8 BMP[]);	   							   		    
void OLED_Init(void);

#endif  
	 



