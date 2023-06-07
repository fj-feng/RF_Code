#include "STM32L4_Flash.h"


#define STM_SECTOR_SIZE	2048
#define STM32_FLASH_SIZE 1024 	 				//��ѡSTM32��FLASH������С(��λΪK)
uint64_t STMFLASH_BUF[STM_SECTOR_SIZE/8];//�����2K�ֽ�
uint64_t STMFLASH_BUF1[STM_SECTOR_SIZE/8];//�����2K�ֽ�



//extern HAL_StatusTypeDef FLASH_WaitForLastOperation(uint32_t Timeout);
 extern void FLASH_PageErase(uint32_t Page, uint32_t Banks);



/**
  * @��飺��ȡָ����ַ��˫��(64λ����)
  * @������faddr����ַ(�˵�ַ����Ϊ4�ı���!!)
  * @����ֵ:��Ӧ��(��faddr��ʼ8�ֽ�����).
  */
uint64_t STMFLASH_ReadDoubleWord(uint32_t faddr)
{
	return *(uint64_t*)faddr; 
}

/**
  * @��飺��ָ����ַ��ʼ����ָ�����ȵ�����
  * @������ReadAddr��ʼ��ַ,pBuffer����ָ��
  * @����ֵ:NumToWrite(64λ)��
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
		pBuffer[i]=STMFLASH_ReadDoubleWord(ReadAddr);//��ȡ8���ֽ�.
		ReadAddr+=8;//ƫ��8���ֽ�.	
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
    pBuffer[i]=STMFLASH_ReadWord(ReadAddr);//��ȡ4���ֽڣ�
    ReadAddr+=4;//��ȡ��ַ+4��
  }
}
/*******************************************************************************
* Function Name :void FLASH_backup_Erase(vu32 Start_ADDR,vu32 size)
* Description   :��ջ�����
* Input         :Start_ADDR-�׵�ַ  size - �ռ��С
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

		SOfPage = FLASH_STARTPages(Start_ADDR); //���㿪ʼ��ҳ��    
		EraseCounter = FLASH_PagesMask(size);   //������Ҫ������ҳ

    for (i = 0;i<EraseCounter; i++)
    {
    	if((SOfPage+i)>255)
				{
					FLASH_PageErase(SOfPage+i,2);   //����Bank2�е�ҳ
				}
			else
				{
					FLASH_PageErase(SOfPage+i,1);//����Bank1�е�ҳ
				}
				
			 while(READ_BIT(FLASH->SR, FLASH_SR_BSY));
			 CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
			 CLEAR_BIT(FLASH->CR, FLASH_CR_PNB);
    }
		
		HAL_FLASH_Lock();
}

/*******************************************************************************
* Function Name :unsigned int FLASH_PagesMask(volatile unsigned int Size)
* Description   :������Ҫ���� ��ҳ��
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
* Description   :������Ҫ���� ��ҳ��
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
* Description   :д��һ������
* Input         :pData:���ݣ�addr:���ݵĵ�ַ
* Output        :TRUE:�ɹ���FALSE:ʧ�ܡ�
* Other         :
* Date          :2013.02.24
*******************************************************************************/
void FLASH_WriteBank_1k(uint8_t *pData, uint32_t addr, uint32_t size)
{
	uint64_t FF_Data = 0;    //64λ���ݻ���

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
void Program_Flash_Standard(uint32_t Address, uint64_t Data)    //��׼����flash
{
	
	if(READ_REG(FLASH->SR) == 0)         //���Flash�����־
	{
		SET_BIT(FLASH->CR,FLASH_CR_PG);  //ʹ��falsh���

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


//ע�� ��Ҫ����BANK����������ҳ����//
void Program_Flash_Fast(uint32_t Address, uint64_t *Data_addr)  //���ٲ���Flash
{
	uint64_t Data=0;
	
	if(READ_REG(FLASH->SR) == 0)         //���Flash�����־
	{
		SET_BIT(FLASH->CR,FLASH_CR_FSTPG);  //ʹ��falsh���

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
  * @��飺ɾ������
  * @������Page_Address������ַ(��������ַ���ڵ�)
  * @����ֵ:��Ӧ����.
  */
void My_FLASH_ErasePage(uint32_t Page_Address)
{
	uint32_t off_addr;   //ȥ��0X08000000��ĵ�ַ
	uint32_t secpos_addr;	   //������ַ
	off_addr=Page_Address-STM32_FLASH_BASE;		//ʵ��ƫ�Ƶ�ַ.
	secpos_addr=off_addr/STM_SECTOR_SIZE;
	if(secpos_addr>255)
		{
			secpos_addr=secpos_addr-256;
			FLASH_PageErase(secpos_addr,2);   //����Bank2�е�ҳ
		}
	else
		{
			FLASH_PageErase(secpos_addr,1);//����Bank1�е�ҳ
		}
			while(READ_BIT(FLASH->SR, FLASH_SR_BSY));
			 CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
			 CLEAR_BIT(FLASH->CR, FLASH_CR_PNB);
}

//������д��
//WriteAddr:��ʼ��ַ
//pBuffer:����ָ��
//NumToWrite:����(16λ)��   

/**
  * @��飺������д��
  * @������WriteAddr:��ʼ��ַ;pBuffer:����ָ��NumToWrite:double(64λ)��   
  * @����ֵ:
  */

void STMFLASH_Write_NoCheck(uint32_t WriteAddr,uint64_t *pBuffer,uint16_t NumToWrite)   
{ 			 		 
	uint16_t i;

	for(i=0;i<NumToWrite;i++)
	{
//		SET_BIT(FLASH->SR, (FLASH_FLAG_EOP | FLASH_FLAG_PGSERR| FLASH_FLAG_WRPERR));
		 Program_Flash_Standard(WriteAddr,pBuffer[i]);
//			MX_USART2_UART_Init();
	    WriteAddr+=8;//��ַ����8.
	}  
} 




void STMFLASH_Write(uint32_t WriteAddr,uint64_t *pBuffer,uint16_t NumToWrite)	
{
	uint32_t secpos;	   //������ַ
	uint16_t secoff;	   //������ƫ�Ƶ�ַ(64λ�ּ���)
	uint16_t secremain; //������ʣ���ַ(64λ�ּ���)	   
 	uint16_t i;    
	uint32_t offaddr;   //ȥ��0X08000000��ĵ�ַ
	if(WriteAddr<STM32_FLASH_BASE||(WriteAddr>=(STM32_FLASH_BASE+1024*STM32_FLASH_SIZE)))return;//�Ƿ���ַ
	HAL_FLASH_Unlock();						//����
	SET_BIT(FLASH->SR, (FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGSERR| FLASH_FLAG_WRPERR));//��������־
	
	offaddr=WriteAddr-STM32_FLASH_BASE;		//ʵ��ƫ�Ƶ�ַ.
	secpos=offaddr/STM_SECTOR_SIZE;			//������ַ  0~127 for STM32F103RBT6
	secoff=(offaddr%STM_SECTOR_SIZE)/8;		//�������ڵ�ƫ��(8���ֽ�Ϊ������λ.)
	secremain=STM_SECTOR_SIZE/8-secoff;		//����ʣ��ռ��С   
	if(NumToWrite<=secremain)secremain=NumToWrite;//�����ڸ�������Χ
	while(1) 
	{	
		STMFLASH_Read2(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/8);//������������������
		for(i=0;i<secremain;i++)//У������
		{
			if(STMFLASH_BUF[secoff+i]!=0XFFFFFFFFFFFFFFFF)break;//��Ҫ����  	  
		}
		if(i<secremain)//��Ҫ����
		{
			My_FLASH_ErasePage(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE);//�����������
			
			for(i=0;i<secremain;i++)//����
			{
				STMFLASH_BUF[i+secoff]=pBuffer[i];	  
			}
			STMFLASH_Write_NoCheck(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/8);//д����������  
		}
		else 
		{
			STMFLASH_Write_NoCheck(WriteAddr,pBuffer,secremain);//д�Ѿ������˵�,ֱ��д������ʣ������. 	
		}
		
		if(NumToWrite==secremain)break;//д�������
		
		else//д��δ����
		{
			secpos++;				//������ַ��1
			secoff=0;				//ƫ��λ��Ϊ0 	 
		  pBuffer+=secremain;  	//ָ��ƫ��
			WriteAddr+=secremain;	//д��ַƫ��	   
		  NumToWrite-=secremain;	//�ֽ�(16λ)���ݼ�
			if(NumToWrite>(STM_SECTOR_SIZE/8))secremain=STM_SECTOR_SIZE/8;//��һ����������д����
			else secremain=NumToWrite;//��һ����������д����
		}	
		
	};	
	HAL_FLASH_Lock();//����
}
