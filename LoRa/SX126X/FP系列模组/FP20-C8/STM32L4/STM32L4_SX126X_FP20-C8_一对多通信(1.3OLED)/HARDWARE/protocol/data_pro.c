#include "data_pro.h"
#include "STM32L4_Flash.h"
#include "usart.h"
#include "oled.h"
#include "stm32l4xx_hal.h"
#include "timer.h"
#include "string.h"

extern uint8_t WakeAddr[4];
extern uint8_t R_Databuffer[7];
uint8_t R_Data[20];

void UART2_REC_Process(void)
{
		uint8_t CRC_Data=0;
		uint8_t count=0;
	  uint8_t R_Length=0;
		if(SET == Usart2_RX.receive_flag)
		{
				Usart2_RX.receive_flag = RESET;
				CRC_Data=checksum(Usart2_RX.RX_Buf+1,Usart2_RX.rx_len-3);
				if((Usart2_RX.RX_Buf[Usart2_RX.rx_len-2]==CRC_Data)&&(Usart2_RX.RX_Buf[1]!=0X00))
				{	
					R_Length=Usart2_RX.RX_Buf[3];
					if(R_Length!=0)
					{
						memcpy(R_Data,Usart2_RX.RX_Buf+4,R_Length);
					}
					else
					{
						for(int i=0;i<R_Length;i++)
						{
							R_Data[i]=0;
						}	
					}
					Module_setup(Usart2_RX.RX_Buf[1],Usart2_RX.RX_Buf[2],Usart2_RX.RX_Buf[3]);
					for( count=0;count<RECEIVELEN;count++)
					{
						Usart2_RX.RX_Buf[count]=0;
					}
				}
	  }
}



void Module_setup(uint8_t Type,uint8_t Cmd,uint8_t Length)
{
	uint8_t Read_AddrID[4];
	tUnionAddr addr_data;
	uint8_t W_ID_Buffer[12]={0xAA,0X01,0x80,0x06,0x00,0x00,0x00,0x00,0x4F,0x4B,0x21,0x55};
	uint8_t R_ID_Buffer[10]={0xAA,0X01,0x81,0x04,0x00,0x00,0x00,0x01,0x87,0x55};
	uint8_t C_ID_Buffer[6]={0xAA,0X01,0x82,0x00,0x83,0x55};

	
	uint64_t vValue =0;
	
	memset(&addr_data,0,sizeof(tUnionAddr));
	switch(Type)
	{
		case Type_LoRa:
	 {
       switch(Cmd)
			 {
				 case W_ID:
						memcpy(Read_AddrID,R_Data,4);//取ID
						vValue =(uint64_t)Read_AddrID[3]<<24|(uint64_t)Read_AddrID[2]<<16|(uint64_t)Read_AddrID[1]<<8|(uint64_t)Read_AddrID[0]<<0;
						STMFLASH_Write(ID_ADDR_FLASH,&vValue,4);
						HAL_Delay(5);
						STMFLASH_Read2(ID_ADDR_FLASH,(uint64_t*)&addr_data.U8Addr,4); //读出ID进行对比
						if(memcmp(Read_AddrID, addr_data.U8Addr,4) == 0)
						{
						  memcpy(W_ID_Buffer+4,Read_AddrID,4);
							W_ID_Buffer[8]=0x4F;
							W_ID_Buffer[9]=0x4B;
							W_ID_Buffer[10]=checksum(W_ID_Buffer+1,9);
							Usart2SendData(W_ID_Buffer,sizeof(W_ID_Buffer));
						}		
						else
						{
							memcpy(W_ID_Buffer+4,Read_AddrID,4);
							W_ID_Buffer[8]=0x45;
							W_ID_Buffer[9]=0x52;
							W_ID_Buffer[10]=checksum(W_ID_Buffer+1,9);
							Usart2SendData(W_ID_Buffer,sizeof(W_ID_Buffer));
						}
				   break;
				 case R_ID:
						STMFLASH_Read2(ID_ADDR_FLASH,(uint64_t*)addr_data.U8Addr,4); //读出ID进行对比
						memcpy(Read_AddrID,addr_data.U8Addr,4);
						memcpy(R_ID_Buffer+4,Read_AddrID,4);
						R_ID_Buffer[8]=checksum(R_ID_Buffer+1,7);
						Usart2SendData(R_ID_Buffer,sizeof(R_ID_Buffer));
				   break;
				 case C_ID:
						HAL_FLASH_Unlock();						//解锁
						SET_BIT(FLASH->SR, (FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGSERR| FLASH_FLAG_WRPERR));//清除错误标志
						FLASH_backup_Erase(ID_ADDR_FLASH,2048);
						HAL_FLASH_Lock();
						Usart2SendData(C_ID_Buffer,sizeof(C_ID_Buffer));
				   break;
				 
				 case W_Config:
					 
				   break;
				 case R_Config:
				   break;
				 
				 case R_Meter:
					 memcpy(WakeAddr,R_Data,4);//取ID
				   HAL_TIM_Base_Start_IT(&TIM3_InitStruct);//定时开启
				   break;	
				 
				default:
					break;				 
			 }
	}
		    break;
			 
			case Type_FSK:
				break;
			default:
			  break;		
	}
	
}




uint8_t checksum(uint8_t *pdata, uint16_t Length)
{
	uint8_t CRC_Data=0;
	for(int i=0;i<Length;i++)
	{
		CRC_Data+=pdata[i];
	}
	return CRC_Data;
}


uint16_t Get_Temperature_Sensor(void)
{
	return 0x1A1A;
}

