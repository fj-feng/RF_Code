/**
  ******************************************************************************
  * �ļ��� ��   SX127X_HAL.c
  * ����   ��   LSD RF Team
  * �汾   ��   V1.0.0
  * ʱ��   ��   15-Aug-2018
  * �ļ�������
  *     ���ļ�ΪSX127Xģ���Ӳ���㣬����MCU��SX127Xģ���SPI���ã�GPIO�ڳ�ʼ������
  *������SX127X�Ĵ�����FIFO��д������
  *    �ͻ���ʹ��ģ��ʱ����Ҫ��ֲ���ļ�����֤�����������������ββ���������
  *�����Լ���MCUƽ̨�޸ĺ������ݣ�ʹ�������ܿ��������С�Ӳ����ռ����Դ���£�
	*
  *SPI��������ʹ��STM32L4��SPI3������SX127Xģ��ͨ�š�
  *GPIO�ڣ�������ʹ�õ�GPIO���������£�
  *        PB1  ---> DIO1
  *        PC4  ---> DIO2
  *        PB2  ---> Busy
  *        PB0  ---> SW2
  *        PC5  ---> SW1
	*        PA7  ---> RST
	*        PA15 ---> NSS
  *        PC12 ---> M0SI
  *        PC11 ---> MISO
  *        PC10 ---> SCK
*******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "SX126X_Hal.h"
#include "stm32l4xx_hal.h"
SPI_HandleTypeDef SPI3_InitStruct;
//-----------------------------GPIO-----------------------------//
//�ò��ֺ���Ϊϵͳ�õ���GPIO�ĳ�ʼ���������û������Լ���ƽ̨��Ӧ�޸�
//--------------------------------------------------------------//

/**
  * @��飺�ú���ΪDIO1�����ʼ�����жϡ����ȼ����ã�
  * @��������
  * @����ֵ����
  */
void SX126X_DIO1_INPUT(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    __GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 0);
}
/**
  * @��飺�ú���ΪDIO1�����жϿ���ʹ�ܣ�
  * @��������
  * @����ֵ����
  */
void SX126X_DIO1_INTENABLE(void)
{
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}
/**
  * @��飺�ú���ΪDIO1�����жϹر�ʹ�ܣ�
  * @��������
  * @����ֵ����
  */
void SX126X_DIO1_INTDISABLE(void)
{
    HAL_NVIC_DisableIRQ(EXTI1_IRQn);
}
/**
  * @��飺�ú���ΪDIO1����״̬��ȡ��
  * @��������
  * @����ֵ��DIO1״̬"1"or"0"
  */
GPIO_PinState SX126X_DIO1_GetState(void)
{
    GPIO_PinState State;
    State = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
    return State;
}
/**
  * @��飺�ú���ΪDIO2�����ʼ�����жϡ����ȼ����ã�
  * @��������
  * @����ֵ����
  */
void SX126X_DIO2_INPUT(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    __GPIOC_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 0);
}
/**
  * @��飺�ú���ΪDIO2�����жϿ���ʹ�ܣ�
  * @��������
  * @����ֵ����
  */
void SX126X_DIO2_INTENABLE(void)
{
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}
/**
  * @��飺�ú���ΪDIO2�����жϹر�ʹ�ܣ�
  * @��������
  * @����ֵ����
  */
void SX126X_DIO2_INTDISABLE(void)
{
    HAL_NVIC_DisableIRQ(EXTI4_IRQn);
}
/**
  * @��飺�ú���ΪDIO2����״̬��ȡ��
  * @��������
  * @����ֵ��DIO2״̬"1"or"0"
  */
GPIO_PinState SX126X_DIO2_GetState(void)
{
    GPIO_PinState State;
    State = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4);
    return State;
}
/**
  * @��飺�ú���ΪBusy�����ʼ�����жϡ����ȼ����ã�
  * @��������
  * @����ֵ����
  */
void SX126X_Busy_INPUT(void)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    __GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	  //HAL_NVIC_SetPriority(EXTI2_IRQn, 1, 0);
}
/**
  * @��飺�ú���ΪBusy�����жϿ���ʹ�ܣ�
  * @��������
  * @����ֵ����
  */
void SX126X_Busy_INTENABLE(void)
{
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);
}
/**
  * @��飺�ú���ΪBusy�����жϹر�ʹ�ܣ�
  * @��������
  * @����ֵ����
  */
void SX126X_Busy_INTDISABLE(void)
{
    HAL_NVIC_DisableIRQ(EXTI2_IRQn);
}
/**
  * @��飺�ú���ΪBusy����״̬��ȡ��
  * @��������
  * @����ֵ��Busy״̬"1"or"0"
  */
GPIO_PinState SX126X_Busy_GetState(void)
{
    GPIO_PinState State;
    State = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);
    return State;
}
/**
  * @��飺�ú���ΪBusy�ȴ���
  * @��������
  * @����ֵ����
  */
void SX126xWaitOnBusy( void )
{
	  SX126X_Busy_INPUT();
    while(SX126X_Busy_GetState() == 1 );
}
/**
  * @��飺�ú���Ϊ��Ƶ����SWCTL2������ƣ�
  * @������PinStateΪ"1"��ʾ����ߵ�ƽ��"0"����͵�ƽ��
  * @����ֵ����
  */
void SX126X_SWCTL2_OUTPUT(GPIO_PinState PinState)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, PinState);
}
/**
  * @��飺�ú���Ϊ��Ƶ����SWCTL1������ƣ�
  * @������PinStateΪ"1"��ʾ����ߵ�ƽ��"0"����͵�ƽ��
  * @����ֵ����
  */
void SX126X_SWCTL1_OUTPUT(GPIO_PinState PinState)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, PinState);
}

/**
  * @��飺��ͨIO�ڣ����ʹ��
  * @������PinStateΪ"1"��ʾ����ߵ�ƽ��"0"����͵�ƽ��
  * @����ֵ����
  */
void IO_SET(GPIO_PinState PinState)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    __HAL_RCC_GPIOE_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, PinState);
}
/**
  * @��飺�ú���ΪSPI��Ƭѡ����NSS������ƣ�
  * @������PinStateΪ"1"��ʾ����ߵ�ƽ��"0"����͵�ƽ��
  * @����ֵ����
  */
void SX126X_NSS_OUTPUT(GPIO_PinState PinState)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, PinState);
}
/**
  * @��飺�ú���ΪSX126X��λ����NRST������ƣ�
  * @������PinStateΪ"1"��ʾ����ߵ�ƽ��"0"����͵�ƽ��
  * @����ֵ����
  */
void SX126X_RESET_OUTPUT(GPIO_PinState PinState)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, PinState);
}
//-----------------------------SPI-----------------------------//
//�ò��ֺ���ΪMCU��SX127Xģ��SPIͨ�Ų��֣�����SPI�ڼ����ó�ʼ��
//--------------------------------------------------------------//

/**
  * @��飺�ú�������MCU��SPI��ӦIO�ڳ�ʼ����
  * @��������
  * @����ֵ����
  */
void SX126X_SPIGPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    /* Configure the SX126X_NSS pin */
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
}
/**
  * @��飺�ú�������MCU��SPI���ó�ʼ����
  * @��������
  * @����ֵ����
  */
void SX126X_SPI_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();//PORTAʱ��ʹ��
    __HAL_RCC_GPIOC_CLK_ENABLE();//PORTCʱ��ʹ��
    __HAL_RCC_SPI3_CLK_ENABLE();//SPI2ʱ��ʹ��
    SX126X_SPIGPIO_Init();

    SPI3_InitStruct.Instance = SPI3; //ʹ��SPI3
    SPI3_InitStruct.Init.Mode = SPI_MODE_MASTER;//SPIģʽ������ģʽ
    SPI3_InitStruct.Init.Direction = SPI_DIRECTION_2LINES;//����ȫ˫��
    SPI3_InitStruct.Init.DataSize = SPI_DATASIZE_8BIT;//���ݿ�ȣ�8λ
    SPI3_InitStruct.Init.CLKPolarity = SPI_POLARITY_LOW; //����ͬ����ʱ�ӿ���Ϊ����ʱ��
    SPI3_InitStruct.Init.CLKPhase = SPI_PHASE_1EDGE;      //CPOL=0;CPHA=0ģʽ
    SPI3_InitStruct.Init.NSS = SPI_NSS_SOFT;//NSSD���������
    SPI3_InitStruct.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;//�����ʷ�Ƶ��4��Ƶ
    SPI3_InitStruct.Init.FirstBit = SPI_FIRSTBIT_MSB;//���ݴ�MSB��ʼ
    SPI3_InitStruct.Init.TIMode = SPI_TIMODE_DISABLE;//SPI Motorola mode
    SPI3_InitStruct.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;//CRCУ�鲻ʹ��
    SPI3_InitStruct.Init.CRCPolynomial = 7;//CRCֵ����Ķ���ʽ
    SPI3_InitStruct.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    SPI3_InitStruct.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;

    if (HAL_SPI_Init(&SPI3_InitStruct) != HAL_OK)

    {
        while(1);
    }
    __HAL_SPI_ENABLE(&SPI3_InitStruct);
}
/**
  * @��飺SX126X  ��Ĵ�����ַ������������
  * @������uint8_t addr,�Ĵ�����ַ uint8_t *buffer,��������ָ�� uint8_t sizeָ�볤��
  * @����ֵ����
  */
unsigned char SX126X_ReadWriteByte(unsigned char data)
{
    unsigned char RxDat;
    HAL_SPI_TransmitReceive(&SPI3_InitStruct, &data, &RxDat, 1, 1000);
    return RxDat;
}
