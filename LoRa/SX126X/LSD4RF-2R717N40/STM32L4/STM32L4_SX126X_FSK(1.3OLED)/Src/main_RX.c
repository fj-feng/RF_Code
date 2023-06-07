/**
  ******************************************************************************
  * �ļ��� ��   main_TX.c
  * ����   ��   LSD RF Team
  * �汾   ��   V1.0.0
  * ʱ��   ��   15-Oct-2018
  * �ļ�������
  *    Aģ�鷢�����ݰ���Bģ�飬TX������1��Bģ���յ����ݺ�Bģ�齫���ݻش���A
  *ģ�飬Aģ���յ����ݺ�RX������1��ѭ������
	*����Ĭ���������£�
			���ʣ�4.8Kbps
			Dev��5KHz
			RxBW��20KHz
			���ݰ׻�������(0x1FF)
			��ַ�˲��������ڵ��ַ0x11
			ͬ���֣�{ 0xC1, 0x94, 0xC1, 0x00, 0x00, 0x00, 0x00, 0x00 }������3�ֽ�
			CRC������RADIO_CRC_2_BYTES_CCIT
		
*******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "stm32l4xx_hal.h"
#include "led.h"
#include "beep.h"
#include "key.h"
#include "oled.h"
#include "timer.h"
//#include "SX126X_Driver.h"
#include "SX126X_Driver_FSK.h"
//===================================??????===================================================
enum DemoInternalStates
{
    APP_RNG = 0, // nothing to do (or wait a radio interrupt)
    RX_DONE,
    TX_DONE,
    TX_ING,
    APP_IDLE,
};
uint8_t TXbuffer[100] = 
{0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA,0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA,
0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA,0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA,
0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA,0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA,
0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA,0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA,
0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA,0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA};

uint8_t RXbuffer[100] = {0};

//uint8_t TXbuffer[10] = {0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA};
//uint8_t RXbuffer[10] = {0};
uint8_t WakeAddr[8] = {0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8};
uint32_t Fre[5] = {470800000, 492570000, 509770000, 868000000, 915000000};
//long SysTick = 0;
uint16_t T_Cnt = 0;
uint16_t R_Cnt = 0;
uint16_t E_Cnt = 0;
uint8_t SF;


int8_t PktSnr_Value = 0;    //SNR
int8_t SNR = 0;
int8_t PktRssi_Value = 0;   //rssi
int8_t RSSI = 0;


uint8_t communication_states;
void SystemClock_Config(void);
void Error_Handler(void);

//===============================================================================================
////////////////////////////////////////////////////////////////////////////////
// ����������������
// �������룺
// ���ز�����
// ˵��    :
////////////////////////////////////////////////////////////////////////////////
int main(void)
{
    HAL_Init();
    /* Configure the system clock */
    SystemClock_Config();
    LED_Init();//LED��ʼ��
    KEY_Init();
    OLED_Init();
    BEEP_Init();
    //���ø�������
	
    G_FSKConfig.FSK_Freq = Fre[0];      //����Ƶ��470MHz
	  G_FSKConfig.PowerCfig = 22;
	
	  //Modulation Paramenters
    G_FSKConfig.BitRate = 4800;				//600~300000
    G_FSKConfig.Fdev = 5000;						//Fdev<=250K - BR/2;
	  G_FSKConfig.Bandwidth = 20000;     //DSB ,4800~500000��BW>2*Fdev+BR+Fer
	  G_FSKConfig.ModulationShaping = MOD_SHAPING_G_BT_1;
	
	 //Packet Paramenters  
	  G_FSKConfig.PreambleLength = 5;         //1~8191����λ���ֽ�
	  G_FSKConfig.PreambleMinDetect = RADIO_PREAMBLE_DETECTOR_08_BITS;
	  G_FSKConfig.SyncWordLength = 3;         //1~8����λ���ֽ�
		G_FSKConfig.AddrComp = RADIO_ADDRESSCOMP_FILT_NODE;	//��ַ�˲����ã��ڵ��ַ��㲥��ַ
	  G_FSKConfig.HeaderType = RADIO_PACKET_VARIABLE_LENGTH;	//���ݰ���ʽ
		G_FSKConfig.PayloadLength = 100;
		G_FSKConfig.CrcLength = RADIO_CRC_2_BYTES_CCIT; //CRC ����
		G_FSKConfig.DcFree = RADIO_DC_FREEWHITENING;   //���ݰ׻�
	

    if(SX126x_FSK_init() != NORMAL)	 //����ģ���ʼ��
    {
        while(1)
        {
            OLED_Clear();
            OLED_ShowString(0, 0, "ERRoR", 24);
            OLED_Refresh();		//������ʾ��OLED
        }
    }

    MX_TIM3_Init_Ms(500);
    HAL_TIM_Base_Stop_IT(&TIM3_InitStruct);
    OLED_ShowString(0, 0, "FSK-TX:          Hz", 12); //��0�У�0�У���ʾ FSK-TX:        Hz�ַ�,ʹ��12���壻
    OLED_ShowNum(42, 0, G_FSKConfig.FSK_Freq , 9, 12); //��48�У�0�У���ʾ���֣�����λ��ʹ��12���壻
    OLED_ShowString(0, 12, "BitRate:       bps", 12);
    OLED_ShowNum(48, 12, G_FSKConfig.BitRate, 6, 12);
    OLED_ShowString(0, 24, "Fdev:       Hz", 12);
    OLED_ShowNum(30, 24,  G_FSKConfig.Fdev, 6, 12);
    OLED_ShowString(0, 36, "Bandwidth:       Hz", 12);
		OLED_ShowNum(60, 36,  G_FSKConfig.Bandwidth, 6, 12);
    OLED_ShowString(0, 48, "R:", 12);
    OLED_ShowNum(12, 48, R_Cnt, 3, 12);
    OLED_ShowString(72, 48, "E:", 12); //���հ�����
    OLED_ShowNum(84, 48, E_Cnt, 3, 12);
    OLED_Refresh();		//������ʾ��OLED
    DIO1_EnableInterrupt();
    SX126X_FSKStartRx();

    while (1)
    {
        switch(communication_states)
        {
        case APP_IDLE:

            __ASM("NOP");
            break;
				
        case TX_ING:
            communication_states = APP_IDLE;
            DIO1_EnableInterrupt();
            SX126X_FSKTxPacket(TXbuffer);//��������
            break;
				
        case TX_DONE:
            communication_states = APP_IDLE;
            DIO1_EnableInterrupt();
            SX126X_FSKStartRx();//������ɺ�ʼ�������ģʽ
            LEDOFF(LED2_GRREN);//Ϩ���̵�
            break;
				
        case RX_DONE:
            communication_states = APP_IDLE;
			
						if(G_FSKConfig.HeaderType ==RADIO_PACKET_FIXED_LENGTH)
							{
								SX126xClearTimeoutEvent();
							}
					
            DIO1_DisableInterrupt();
            SX126X_FSKRxPacket(RXbuffer);
            R_Cnt++;
            if(R_Cnt > 999)
            {
                R_Cnt = 0;
            }
           OLED_ShowNum(12, 48, R_Cnt, 3, 12);
            if(memcmp(RXbuffer, TXbuffer, 10) == 0)
            {
							LEDON(LED2_GRREN);//��������
              RSSI = G_FSKPara.RssiAvg;	
						  communication_states = TX_ING;
            }
            else
            {
                E_Cnt++;
                if(E_Cnt > 999)
                {
                    E_Cnt = 0;
                }
              OLED_ShowNum(84, 48, E_Cnt, 3, 12);
							communication_states = TX_ING;
            }
            break;
        default :
            break;

        }
        OLED_Refresh();
    }
}


void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler */
    /* User can add his own implementation to report the HAL error return state */
    while(1)
    {
    }
    /* USER CODE END Error_Handler */
}


//DIO0�жϷ��غ���

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(KEY1 == 0)			//��߰���K3���ò���
    {
        HAL_Delay(50);
        if(KEY1 == 0)
        {
						HAL_TIM_Base_Stop_IT(&TIM3_InitStruct);
            static uint8_t t = 0;
            t++;
            switch(t)
            {
            case 1:
            {
							G_FSKConfig.BitRate = 4800;  	//600~300000
							G_FSKConfig.Fdev = 5000;     	//Fdev<=250K - BR/2;
							G_FSKConfig.Bandwidth = 20000;	//DSB ,4800~500000��BW>2*Fdev+BR+Fer
                break;
            }
            case 2:
            {
							G_FSKConfig.BitRate = 38400;  	//600~300000
							G_FSKConfig.Fdev = 40000;     	//Fdev<=250K - BR/2;
							G_FSKConfig.Bandwidth = 160000;	//DSB ,4800~500000��BW>2*Fdev+BR+Fer
                
                break;
            }
						
						 case 3:
            {
							G_FSKConfig.BitRate = 250000;  	//600~300000
							G_FSKConfig.Fdev = 125000;     	//Fdev<=250K - BR/2;
							G_FSKConfig.Bandwidth = 500000;	//DSB ,4800~500000��BW>2*Fdev+BR+Fer
                
                break;
            }
            default :
                break;
            }
						
            OLED_ShowNum(48, 12, G_FSKConfig.BitRate, 6, 12);
						OLED_ShowNum(30, 24,  G_FSKConfig.Fdev, 6, 12);
						OLED_ShowNum(60, 36,  G_FSKConfig.Bandwidth, 6, 12);
						
            OLED_Refresh();
            if(t > 3)
            {
                t = 0;
            }
        }
    }

    if(KEY2 == 0)			//�ұ߰���K2��ʼͨѶ
    {
        HAL_Delay(50);
        if(KEY2 == 0)
        {
            HAL_TIM_Base_Stop_IT(&TIM3_InitStruct);
            DIO1_DisableInterrupt();
            static int8_t j = 0;
            j++;
						G_FSKConfig.FSK_Freq = Fre[j];
            OLED_ShowNum(42, 0, G_FSKConfig.FSK_Freq , 9, 12);
            OLED_Refresh();
            if(j >= 4)
            {
                j = -1;
            }
        }
    }
    if(KEY3 == 0)
    {
        HAL_Delay(50);
        if(KEY3 == 0)
        {
						if(SX126x_FSK_init() != NORMAL)	 //����ģ���ʼ��
            {
                while(1)
                {
                    OLED_Clear();
                    OLED_ShowString(0, 0, "ERoR", 24);
                    OLED_Refresh();
                }
            }
            T_Cnt = 0;
            R_Cnt = 0;
            E_Cnt = 0;
            OLED_ShowNum(12, 48, T_Cnt, 3, 12);
            OLED_ShowNum(48, 48, R_Cnt, 3, 12);
           	OLED_ShowNum(84, 48, E_Cnt, 3, 12);
            DIO1_EnableInterrupt();
            SX126X_FSKStartRx();
        }
    }

    if(DIO1_GetState() == GPIO_PIN_SET)
    {
        uint16_t flag;
        flag = SX126xGetIrqStatus();
        SX126xClearIrqStatus( IRQ_RADIO_ALL ); //clear flags
        if((flag & IRQ_TX_DONE) == IRQ_TX_DONE)
        {
            communication_states = TX_DONE;
        }
	
        else if((flag & (IRQ_RX_DONE|IRQ_CRC_ERROR))==IRQ_RX_DONE)
//        else if((flag & IRQ_RX_DONE) == IRQ_RX_DONE)
        {
            communication_states = RX_DONE;
        }
				 else
        {
            DIO1_EnableInterrupt();
            SX126X_FSKStartRx();
        }
    }
}



/*******************Callback********************/
//��ʱ�����غ���  /* TIM Update event */
/*******************Callback********************/
//��ʱ�����غ���  /* TIM Update event */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
   
}
