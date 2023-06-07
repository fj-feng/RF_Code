#include "spi.h"
#include "stm32l4xx_hal.h"
/** Configure pins
     
     PA15   ------> SPI3_NSS
     PC10   ------> SPI3_SCK
     PC11   ------> SPI3_MISO
     PC12   ------> SPI3_MOSI   
*/

 SPI_HandleTypeDef SPI3_InitStruct;
 
void SPI3_Init(void)
{
	
	 GPIO_InitTypeDef  GPIO_InitStruct;
	
	 __HAL_RCC_GPIOA_CLK_ENABLE();//PORTAÊ±ÖÓÊ¹ÄÜ 
	 __HAL_RCC_GPIOC_CLK_ENABLE();//PORTCÊ±ÖÓÊ¹ÄÜ 
	 __HAL_RCC_SPI3_CLK_ENABLE();//SPI2Ê±ÖÓÊ¹ÄÜ 	
 
	    /* Configure the SX126X_NSS pin */
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
	
	  /* SPI SCK GPIO pin configuration  */
    GPIO_InitStruct.Pin       = GPIO_PIN_10;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;   
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);  
		
    /* SPI MISO GPIO pin configuration  */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;   
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
     /* SPI MoSi GPIO pin configuration  */
    GPIO_InitStruct.Pin       = GPIO_PIN_12;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;       
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
		
		SPI3_InitStruct.Instance = SPI3; //Ê¹ÓÃSPI3
    SPI3_InitStruct.Init.Mode = SPI_MODE_MASTER;//SPIÄ£Ê½£ºÖ÷»úÄ£Ê½ 
    SPI3_InitStruct.Init.Direction = SPI_DIRECTION_2LINES;//Á½ÏßÈ«Ë«¹¤
    SPI3_InitStruct.Init.DataSize = SPI_DATASIZE_8BIT;//Êı¾İ¿í¶È£º8Î»
    SPI3_InitStruct.Init.CLKPolarity = SPI_POLARITY_LOW; //´®ĞĞÍ¬²½Ê±ÖÓµÄ¿ÕÏĞ×´Ì¬ÎªµÍßµçÆ½
    SPI3_InitStruct.Init.CLKPhase = SPI_PHASE_1EDGE;      //CPOL=0;CPHA=0  ´®ĞĞÍ¬²½Ê±ÖÓµÄµÚÒ»¸öÌø±äÑØ£¨ÉÏÉı£©Êı¾İ±»²ÉÑù
    SPI3_InitStruct.Init.NSS = SPI_NSS_SOFT;//NSSĞÅºÅÓÉÈí¼ş¹ÜÀí
    SPI3_InitStruct.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;//¶¨Òå²¨ÌØÂÊÔ¤·ÖÆµµÄÖµ:²¨ÌØÂÊÔ¤·ÖÆµÖµÎª8
    SPI3_InitStruct.Init.FirstBit = SPI_FIRSTBIT_MSB;//Êı¾İ´«Êä´ÓMSBÎ»¿ªÊ¼
    SPI3_InitStruct.Init.TIMode = SPI_TIMODE_DISABLE;//SPI Motorola mode
    SPI3_InitStruct.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;//CRCĞ£Ñé²»Ê¹ÄÜ
    SPI3_InitStruct.Init.CRCPolynomial = 7;//CRCÖµ¼ÆËãµÄ¶àÏîÊ½
    SPI3_InitStruct.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    SPI3_InitStruct.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
		
    if (HAL_SPI_Init(&SPI3_InitStruct) != HAL_OK)//¸ù¾İSPI_InitStructÖĞÖ¸¶¨µÄ²ÎÊı³õÊ¼»¯ÍâÉèSPIx¼Ä´æÆ÷
 
    {
        while(1);
    }
    __HAL_SPI_ENABLE(&SPI3_InitStruct); //Ê¹ÄÜSPIÍâÉè
		
}


//SPI ËÙ¶ÈÉèÖÃº¯Êı
//SpeedSet:
//SPI_BaudRatePrescaler_2   2·ÖÆµ 
//SPI_BAUDRATEPRESCALER_4   4·ÖÆµ
//SPI_BaudRatePrescaler_8   8·ÖÆµ   
//SPI_BaudRatePrescaler_16  16·ÖÆµ
//SPI_BaudRatePrescaler_32  16·ÖÆµ 
//SPI_BaudRatePrescaler_64  64·ÖÆµ 
//SPI_BaudRatePrescaler_128 128·ÖÆµ 
//SPI_BaudRatePrescaler_256 256·ÖÆµ 
  
void SPI3_SetSpeed(unsigned char SPI_BaudRatePrescaler)
{
  	assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));
	SPI3->CR1&=0XFFC7;
	SPI3->CR1|=SPI_BaudRatePrescaler;	//ÉèÖÃSPI2ËÙ¶È 
	 __HAL_SPI_ENABLE(&SPI3_InitStruct); //Ê¹ÄÜSPIÍâÉè
} 


//SPIx ¶ÁĞ´Ò»¸ö×Ö½Ú
//TxData:ÒªĞ´ÈëµÄ×Ö½Ú
//·µ»ØÖµ:¶ÁÈ¡µ½µÄ×Ö½Ú
unsigned char SPI3_ReadWriteByte(unsigned char data)
{
    unsigned char RxDat;
    HAL_SPI_TransmitReceive(&SPI3_InitStruct,&data,&RxDat,1,1000);
    return RxDat;
}
































