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
	
	 __HAL_RCC_GPIOA_CLK_ENABLE();//PORTA时钟使能 
	 __HAL_RCC_GPIOC_CLK_ENABLE();//PORTC时钟使能 
	 __HAL_RCC_SPI3_CLK_ENABLE();//SPI2时钟使能 	
 
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
		
		SPI3_InitStruct.Instance = SPI3; //使用SPI3
    SPI3_InitStruct.Init.Mode = SPI_MODE_MASTER;//SPI模式：主机模式 
    SPI3_InitStruct.Init.Direction = SPI_DIRECTION_2LINES;//两线全双工
    SPI3_InitStruct.Init.DataSize = SPI_DATASIZE_8BIT;//数据宽度：8位
    SPI3_InitStruct.Init.CLKPolarity = SPI_POLARITY_LOW; //串行同步时钟的空闲状态为低叩缙�
    SPI3_InitStruct.Init.CLKPhase = SPI_PHASE_1EDGE;      //CPOL=0;CPHA=0  串行同步时钟的第一个跳变沿（上升）数据被采样
    SPI3_InitStruct.Init.NSS = SPI_NSS_SOFT;//NSS信号由软件管理
    SPI3_InitStruct.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;//定义波特率预分频的值:波特率预分频值为8
    SPI3_InitStruct.Init.FirstBit = SPI_FIRSTBIT_MSB;//数据传输从MSB位开始
    SPI3_InitStruct.Init.TIMode = SPI_TIMODE_DISABLE;//SPI Motorola mode
    SPI3_InitStruct.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;//CRC校验不使能
    SPI3_InitStruct.Init.CRCPolynomial = 7;//CRC值计算的多项式
    SPI3_InitStruct.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    SPI3_InitStruct.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
		
    if (HAL_SPI_Init(&SPI3_InitStruct) != HAL_OK)//根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
 
    {
        while(1);
    }
    __HAL_SPI_ENABLE(&SPI3_InitStruct); //使能SPI外设
		
}


//SPI 速度设置函数
//SpeedSet:
//SPI_BaudRatePrescaler_2   2分频 
//SPI_BAUDRATEPRESCALER_4   4分频
//SPI_BaudRatePrescaler_8   8分频   
//SPI_BaudRatePrescaler_16  16分频
//SPI_BaudRatePrescaler_32  16分频 
//SPI_BaudRatePrescaler_64  64分频 
//SPI_BaudRatePrescaler_128 128分频 
//SPI_BaudRatePrescaler_256 256分频 
  
void SPI3_SetSpeed(unsigned char SPI_BaudRatePrescaler)
{
  	assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));
	SPI3->CR1&=0XFFC7;
	SPI3->CR1|=SPI_BaudRatePrescaler;	//设置SPI2速度 
	 __HAL_SPI_ENABLE(&SPI3_InitStruct); //使能SPI外设
} 


//SPIx 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
unsigned char SPI3_ReadWriteByte(unsigned char data)
{
    unsigned char RxDat;
    HAL_SPI_TransmitReceive(&SPI3_InitStruct,&data,&RxDat,1,1000);
    return RxDat;
}
































