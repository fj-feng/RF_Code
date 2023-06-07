#ifndef __STM32L4_FLASH_H__
#define __STM32L4_FLASH_H__
#include <stdint.h>
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_flash.h"

#define STM32_FLASH_BASE 0x08000000 	//STM32 FLASH的起始地址
#define ID_ADDR_FLASH 0X08080000 	//STM32 FLASH的起始地址

uint32_t STMFLASH_ReadWord(uint32_t faddr);
void STMFLASH_Read1(uint32_t ReadAddr,uint32_t *pBuffer,uint16_t NumToRead);
	
uint64_t STMFLASH_ReadDoubleWord(uint32_t faddr);
void STMFLASH_Read2(uint32_t ReadAddr,uint64_t *pBuffer,uint16_t NumToRead);

void FLASH_backup_Erase(uint32_t Start_ADDR,uint32_t size);
static uint32_t FLASH_PagesMask(uint32_t Size);
static uint32_t FLASH_STARTPages(uint32_t Start_ADDR);
void Program_Flash_Standard(uint32_t Address, uint64_t Data);
void My_FLASH_ErasePage(uint32_t Page_Address);
void STMFLASH_Write_NoCheck(uint32_t WriteAddr,uint64_t *pBuffer,uint16_t NumToWrite); 
void STMFLASH_Write(uint32_t WriteAddr,uint64_t *pBuffer,uint16_t NumToWrite);

#endif
