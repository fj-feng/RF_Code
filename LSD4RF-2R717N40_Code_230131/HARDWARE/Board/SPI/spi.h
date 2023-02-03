#ifndef __SPI_H
#define __SPI_H

#include "stm32l4xx_hal.h"	

void SPI3_Init(void);			 //初始化SPI3口
void SPI3_SetSpeed(unsigned char SpeedSet); //设置SPI3速度   
unsigned char SPI3_ReadWriteByte(unsigned char data);//SPI3总线读写一个字节 

#endif
