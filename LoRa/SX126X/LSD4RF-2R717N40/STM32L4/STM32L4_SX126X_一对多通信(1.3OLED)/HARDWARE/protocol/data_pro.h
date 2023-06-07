#ifndef __data_pro_H
#define __data_pro_H
#include <stdint.h>
#include "stm32l4xx_hal.h"
#include "usart.h"


#define Type_LoRa         0X01
#define Type_FSK          0X02


#define W_ID        		 0X00
#define R_ID        		 0X01
#define C_ID         		 0X02
#define W_Config        0X03
#define R_Config        0X04
#define R_Meter        	 0X05

typedef union{
	uint64_t U64Addr;
  uint8_t U8Addr[8];
}tUnionAddr;






void UART2_REC_Process(void);
void Module_setup(uint8_t Type,uint8_t Cmd,uint8_t Length);
uint8_t checksum(uint8_t *pdata, uint16_t Length);
uint16_t Get_Temperature_Sensor(void);
#endif
