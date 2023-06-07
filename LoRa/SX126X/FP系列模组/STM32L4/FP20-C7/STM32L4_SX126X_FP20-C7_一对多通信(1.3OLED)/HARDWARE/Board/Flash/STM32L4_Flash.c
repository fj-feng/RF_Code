#include "STM32L4_Flash.h"


#define STM_SECTOR_SIZE	2048
#define STM32_FLASH_SIZE 1024 	 				//所选STM32的FLASH容量大小(单位为K)
uint64_t STMFLASH_BUF[STM_SECTOR_SIZE/8];//最多是2K字节
uint64_t STMFLASH_BUF1[STM_SECTOR_SIZE/8];//最多是2K字节



//extern HAL_StatusTypeDef FLASH_WaitForLastOperation(uint32_t Timeout);
 extern void FLASH_PageErase(uint32_t Page, uint32_t Banks);



/**
  * @简介：读取指定地址的双字(64位数据)
  * @参数：faddr读地址(此地址必须为4的倍数!!)
  * @返回值:对应数(从faddr开始8字节数据).
  */
uint64_t STMFLASH_ReadDoubleWord(uint32_t faddr)
{
	return *(uint64_t*)faddr; 
}

/**
  * @简介：从指定地址开始读出指定长度的数据
  * @参数：ReadAddr起始地址,pBuffer数据指针
  * @返回值:NumToWrite(64位)数
  */

void STMFLASH_Read2(uint32_t ReadAddr,uint64_t *pBuffer,uint16_t NumToRead)   	
{
	uint16_t i;
	if(0 ==pBuffer)return;
  if(NumToRead%8)
	{
		NumToRead=NumToRead/8+1;
	}
	else
	{
		NumToRead=NumToRead/8;
	}
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadDoubleWord(ReadAddr);//读取8个字节.
		ReadAddr+=8;//偏移8个字节.	
	}
}



/**
  * @??:????(32???)
  * @??:faddr???(?????4????:? faddr%4==0)
  * @???:????(?faddr??4????).
  */
uint32_t STMFLASH_ReadWord(uint32_t faddr)
{
  return *(uint32_t*)faddr; 
}

/**
  * @??:????????????????
  * @??:ReadAddr????(?????4????:? ReadAddr%4==0)
  * @??:pBuffer ????
  * @??:NumToRead ???????:???4????
  */

void STMFLASH_Read1(uint32_t ReadAddr,uint32_t *pBuffer,uint16_t NumToRead)    
{
  if(0 ==pBuffer)return;
  if(NumToRead%4)
	{
		NumToRead=(NumToRead%4)+1;
	}
	else
	{
		NumToRead=NumToRead/4;
	}
  
  for(unsigned char i=0;i<NumToRead;i++)
  {
    pBuffer[i]=STMFLASH_ReadWord(ReadAddr);//读取4个字节；
    ReadAddr+=4;//读取地址+4；
  }
}
/*******************************************************************************
* Function Name :void FLASH_backup_Erase(vu32 Start_ADDR,vu32 size)
* Description   :清空缓存区
* Input         :Start_ADDR-首地址  size - 空间大小
* Output        :
* Other         :
* Date          :2018.02.09
*******************************************************************************/
void FLASH_backup_Erase(uint32_t Start_ADDR,uint32_t size)
{
    uint32_t SOfPage = 0;
    uint32_t EraseCounter = 0;
		uint16_t i = 0;

    HAL_FLASH_Unlock();

		SOfPage = FLASH_STARTPages(Start_ADDR); //计算开始的页数    
		EraseCounter = FLASH_PagesMask(size);   //计算需要擦除的页

    for (i = 0;i<EraseCounter; i++)
    {
    	if((SOfPage+i)>255)
				{
					FLASH_PageErase(SOfPage+i,2);   //擦除Bank2中的页
				}
			else
				{
					FLASH_PageErase(SOfPage+i,1);//擦除Bank1中的页
				}
				
			 while(READ_BIT(FLASH->SR, FLASH_SR_BSY));
			 CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
			 CLEAR_BIT(FLASH->CR, FLASH_CR_PNB);
    }
		
		HAL_FLASH_Lock();
}

/*******************************************************************************
* Function Name :unsigned int FLASH_PagesMask(volatile unsigned int Size)
* Description   :计算所要擦除 的页数
* Input         :
* Output        :
* Other         :
* Date          :2013.02.24
*******************************************************************************/
static uint32_t FLASH_PagesMask(uint32_t Size)
{
    uint32_t pagenumber = 0x0;
    uint32_t size = Size;

    if ((size % STM_SECTOR_SIZE) != 0)
    {
        pagenumber = (size / STM_SECTOR_SIZE) + 1;
    }
    else
    {
        pagenumber = size / STM_SECTOR_SIZE;
    }
    return pagenumber;
}

/*******************************************************************************
* Function Name :static u32 FLASH_PagesMask(vu32 Start_ADDR)
* Description   :计算所要擦除 的页数
* Input         :
* Output        :
* Other         :
* Date          :2013.02.24
*******************************************************************************/
static uint32_t FLASH_STARTPages(uint32_t Start_ADDR)
{
    uint32_t START_pagenumber = 0x0;

	if(Start_ADDR > 0x08000000)
	{
		START_pagenumber = (Start_ADDR - 0x08000000)/STM_SECTOR_SIZE;
	}

    return START_pagenumber;
}
/*******************************************************************************
* Function Name :u32 FLASH_WriteBank(u32 *pData, u32 addr ,uint32_t size)
* Description   :写入一块数据
* Input         :pData:数据；addr:数据的地址
* Output        :TRUE:成功，FALSE:失败。
* Other         :
* Date          :2013.02.24
*******************************************************************************/
void FLASH_WriteBank_1k(uint8_t *pData, uint32_t addr, uint32_t size)
{
	uint64_t FF_Data = 0;    //64位数据缓存

	uint32_t F_Data1 = 0,F_Data2 = 0;

	uint8_t dd[8] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};

	uint32_t size_64 = size/8;
	uint32_t size_bad = size%8;
	
	
	for(uint8_t u=0;u<size_64;u++)
	{
		F_Data1 = (uint32_t)((*(pData+3))*65536*256) + (uint32_t)((*(pData+2))*65536) + (uint32_t)((*(pData+1))*256) + (uint32_t)(*(pData));
		F_Data2 = (uint32_t)((*(pData+7))*65536*256) + (uint32_t)((*(pData+6))*65536) + (uint32_t)((*(pData+5))*256) + (uint32_t)(*(pData+4));

		FF_Data = (uint64_t)F_Data2;
		FF_Data = FF_Data<<32;
		FF_Data = FF_Data + (uint64_t)F_Data1;
		//Program_Flash_Fast(addr,F_Data);
		Program_Flash_Standard(addr,FF_Data);

		pData = pData + 8;
		addr = addr + 8;
	}

	if(size_bad != 0)
	{
		for(uint8_t i=0;i<size_bad;i++)
		{
			dd[i] = *(pData+i);
		}

		F_Data1 = (uint32_t)((dd[3])*65536*256) + (uint32_t)((dd[2])*65536) + (uint32_t)((dd[1])*256) + (uint32_t)(dd[0]);
		F_Data2 = (uint32_t)((dd[7])*65536*256) + (uint32_t)((dd[6])*65536) + (uint32_t)((dd[5])*256) + (uint32_t)(dd[4]);

		FF_Data = (uint64_t)F_Data2;
		FF_Data = FF_Data<<32;
		FF_Data = FF_Data + (uint64_t)F_Data1;
		//Program_Flash_Fast(addr,F_Data);
		Program_Flash_Standard(addr,FF_Data);

		pData = pData + 8;
		addr = addr + 8;
	}
}


/**
  * @brief  Program double-word (64-bit) at a specified address.
  * @param  Address: specifies the address to be programmed.
  * @param  Data: specifies the data to be programmed.
  * @retval None
  */
void Program_Flash_Standard(uint32_t Address, uint64_t Data)    //标准操作flash
{
	
	if(READ_REG(FLASH->SR) == 0)         //检查Flash错误标志
	{
		SET_BIT(FLASH->CR,FLASH_CR_PG);  //使能falsh编程

		/* Program the double word */
  		*(__IO uint32_t*)Address = (uint32_t)Data;
  		*(__IO uint32_t*)(Address+4) = (uint32_t)(Data >> 32);

		while(READ_BIT(FLASH->SR,FLASH_SR_BSY));

		if(READ_BIT(FLASH->SR,FLASH_SR_EOP) == 1)
		{
			__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP);
		}

		CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
	}
}


//注意 需要进行BANK擦除，不能页擦除//
void Program_Flash_Fast(uint32_t Address, uint64_t *Data_addr)  //快速操作Flash
{
	uint64_t Data=0;
	
	if(READ_REG(FLASH->SR) == 0)         //检查Flash错误标志
	{
		SET_BIT(FLASH->CR,FLASH_CR_FSTPG);  //使能falsh编程

		for(uint8_t i=0;i<32;i++)
		{
			/* Program the double word */
			Data = *(Data_addr + i);
	  		*(__IO uint32_t*)Address = (uint32_t)Data;
	  		*(__IO uint32_t*)(Address+4) = (uint32_t)(Data >> 32);
		}

		while(READ_BIT(FLASH->SR,FLASH_SR_BSY));

		if(READ_BIT(FLASH->SR,FLASH_SR_EOP) == 1)
		{
			__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP);
		}

		CLEAR_BIT(FLASH->CR,FLASH_CR_FSTPG);
	}
}



/**
  * @简介：删除扇区
  * @参数：Page_Address扇区地址(包含基地址在内的)
  * @返回值:对应数据.
  */
void My_FLASH_ErasePage(uint32_t Page_Address)
{
	uint32_t off_addr;   //去掉0X08000000后的地址
	uint32_t secpos_addr;	   //扇区地址
	off_addr=Page_Address-STM32_FLASH_BASE;		//实际偏移地址.
	secpos_addr=off_addr/STM_SECTOR_SIZE;
	if(secpos_addr>255)
		{
			secpos_addr=secpos_addr-256;
			FLASH_PageErase(secpos_addr,2);   //擦除Bank2中的页
		}
	else
		{
			FLASH_PageErase(secpos_addr,1);//擦除Bank1中的页
		}
			while(READ_BIT(FLASH->SR, FLASH_SR_BSY));
			 CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
			 CLEAR_BIT(FLASH->CR, FLASH_CR_PNB);
}

//不检查的写入
//WriteAddr:起始地址
//pBuffer:数据指针
//NumToWrite:半字(16位)数   

/**
  * @简介：不检查的写入
  * @参数：WriteAddr:起始地址;pBuffer:数据指针NumToWrite:double(64位)数   
  * @返回值:
  */

void STMFLASH_Write_NoCheck(uint32_t WriteAddr,uint64_t *pBuffer,uint16_t NumToWrite)   
{ 			 		 
	uint16_t i;

	for(i=0;i<NumToWrite;i++)
	{
//		SET_BIT(FLASH->SR, (FLASH_FLAG_EOP | FLASH_FLAG_PGSERR| FLASH_FLAG_WRPERR));
		 Program_Flash_Standard(WriteAddr,pBuffer[i]);
//			MX_USART2_UART_Init();
	    WriteAddr+=8;//地址增加8.
	}  
} 




void STMFLASH_Write(uint32_t WriteAddr,uint64_t *pBuffer,uint16_t NumToWrite)	
{
	uint32_t secpos;	   //扇区地址
	uint16_t secoff;	   //扇区内偏移地址(64位字计算)
	uint16_t secremain; //扇区内剩余地址(64位字计算)	   
 	uint16_t i;    
	uint32_t offaddr;   //去掉0X08000000后的地址
	if(WriteAddr<STM32_FLASH_BASE||(WriteAddr>=(STM32_FLASH_BASE+1024*STM32_FLASH_SIZE)))return;//非法地址
	HAL_FLASH_Unlock();						//解锁
	SET_BIT(FLASH->SR, (FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGSERR| FLASH_FLAG_WRPERR));//清除错误标志
	
	offaddr=WriteAddr-STM32_FLASH_BASE;		//实际偏移地址.
	secpos=offaddr/STM_SECTOR_SIZE;			//扇区地址  0~127 for STM32F103RBT6
	secoff=(offaddr%STM_SECTOR_SIZE)/8;		//在扇区内的偏移(8个字节为基本单位.)
	secremain=STM_SECTOR_SIZE/8-secoff;		//扇区剩余空间大小   
	if(NumToWrite<=secremain)secremain=NumToWrite;//不大于该扇区范围
	while(1) 
	{	
		STMFLASH_Read2(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/8);//读出整个扇区的内容
		for(i=0;i<secremain;i++)//校验数据
		{
			if(STMFLASH_BUF[secoff+i]!=0XFFFFFFFFFFFFFFFF)break;//需要擦除  	  
		}
		if(i<secremain)//需要擦除
		{
			My_FLASH_ErasePage(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE);//擦除这个扇区
			
			for(i=0;i<secremain;i++)//复制
			{
				STMFLASH_BUF[i+secoff]=pBuffer[i];	  
			}
			STMFLASH_Write_NoCheck(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/8);//写入整个扇区  
		}
		else 
		{
			STMFLASH_Write_NoCheck(WriteAddr,pBuffer,secremain);//写已经擦除了的,直接写入扇区剩余区间. 	
		}
		
		if(NumToWrite==secremain)break;//写入结束了
		
		else//写入未结束
		{
			secpos++;				//扇区地址增1
			secoff=0;				//偏移位置为0 	 
		  pBuffer+=secremain;  	//指针偏移
			WriteAddr+=secremain;	//写地址偏移	   
		  NumToWrite-=secremain;	//字节(16位)数递减
			if(NumToWrite>(STM_SECTOR_SIZE/8))secremain=STM_SECTOR_SIZE/8;//下一个扇区还是写不完
			else secremain=NumToWrite;//下一个扇区可以写完了
		}	
		
	};	
	HAL_FLASH_Lock();//上锁
}
