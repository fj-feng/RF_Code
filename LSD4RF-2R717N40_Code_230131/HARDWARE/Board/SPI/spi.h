#ifndef __SPI_H
#define __SPI_H

#include "stm32l4xx_hal.h"	

void SPI3_Init(void);			 //��ʼ��SPI3��
void SPI3_SetSpeed(unsigned char SpeedSet); //����SPI3�ٶ�   
unsigned char SPI3_ReadWriteByte(unsigned char data);//SPI3���߶�дһ���ֽ� 

#endif
