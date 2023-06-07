/**
  ******************************************************************************
  * �ļ��� ��   main_TX.c
  * ����   ��   LSD RF Team
  * �汾   ��   V1.0.0
  * ʱ��   ��   15-Aug-2018
  * �ļ�������
  *    Aģ�鷢�����ݰ���Bģ�飬TX������1��Bģ���յ����ݺ�Bģ�齫���ݻش���A
  *ģ�飬Aģ���յ����ݺ�RX������1��ѭ������
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
#include "usart.h"
#include "SX126X_Driver.h"
//===================================????===================================================

uint8_t TXbuffer[10] = {0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA};
uint8_t RXbuffer[10] = {0};
uint32_t Fre[5] = {470800000, 493800000, 509770000, 868000000, 915000000};
//long SysTick = 0;
uint16_t T_Cnt = 0;
uint16_t R_Cnt = 0;
uint16_t E_Cnt = 0;
uint8_t SF;
unsigned int BW;
unsigned int frq;
int8_t PktSnr_Value = 0;    //SNR
int8_t SNR = 0;
int8_t PktRssi_Value = 0;   //rssi
int8_t RSSI = 0;
int8_t Avg_RSSI = 0;
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
    BEEP_Init();
    OLED_Init();
    MX_USART2_UART_Init();

    //���ø�������
    G_LoRaConfig.LoRa_Freq = Fre[0];      //����Ƶ��
    G_LoRaConfig.BandWidth = LORA_BW_125;    //BW = 125KHz  BW125KHZ
    G_LoRaConfig.SpreadingFactor = LORA_SF9;  //SF = 9
    G_LoRaConfig.CodingRate = LORA_CR_4_6;     //CR = 4/6
    G_LoRaConfig.PowerCfig = 22;               //���뷶Χ��-3~22������ʵ��ʹ��Ӳ��LSD4RFC-2L722N10ѡ��SX126xSetTxParams����
    G_LoRaConfig.HeaderType = LORA_PACKET_EXPLICIT;       //��ͷ��ʽ���ã����԰�ͷ��LORA_PACKET_EXPLICIT�����԰�ͷ��LORA_PACKET_IMPLICIT
    //������Ϊ���԰�ͷ�����Ͷ˽��ὫPalyLoad���ȡ������ʡ�CRC�ȼ��뵽��ͷ�з��͸����ն�

    G_LoRaConfig.CrcMode = LORA_CRC_ON;             //CRCУ�鿪����LORA_CRC_ON���رգ�LORA_CRC_OFF
    G_LoRaConfig.InvertIQ = LORA_IQ_NORMAL;         //IQ�źŸ�ʽ��LORA_IQ_NORMAL����׼ģʽ��LORA_IQ_INVERTED����תģʽ��
    G_LoRaConfig.PreambleLength = 8;    //ǰ���볤��
    G_LoRaConfig.PayloadLength = 10;      //���ݰ�����

    if(SX126x_Lora_init() != NORMAL)	 //����ģ���ʼ��
    {
        while(1)
        {
            OLED_Clear();
            OLED_ShowString(0, 0, "ERRoR", 24);
            OLED_Refresh();		//������ʾ��OLED
        }
    }

    //Һ����ʾ����
    switch(G_LoRaConfig.SpreadingFactor)
    {
    case LORA_SF5:
        SF = 5;
        break;
    case LORA_SF6:
        SF = 6;
        break;
    case LORA_SF7:
        SF = 7;
        break;
    case LORA_SF8:
        SF = 8;
        break;
    case LORA_SF9:
        SF = 9;
        break;
    case LORA_SF10:
        SF = 10;
        break;
    case LORA_SF11:
        SF = 11;
        break;
    case LORA_SF12:
        SF = 12;
        break;
    }
    OLED_ShowString(0, 0, "LORA-TX:         Hz", 12); //��0�У�0�У���ʾ LORA-TX:        Hz�ַ�,ʹ��12���壻
    OLED_ShowNum(48, 0, G_LoRaConfig.LoRa_Freq, 9, 12); //��48�У�0�У���ʾ���֣�����λ��ʹ��12���壻
    OLED_ShowString(0, 12, "SF:", 12);
    OLED_ShowNum(18, 12, SF, 2, 12);
    OLED_ShowString(42, 12, "BW:", 12);
    OLED_ShowNum(60, 12, G_BandWidthKHz, 3, 12);
    OLED_ShowString(0, 26, "SNR:", 12);
    OLED_ShowString(60, 26, "RSSI:", 12);
    OLED_ShowString(0, 44, "T:", 12);
    OLED_ShowNum(12, 44, T_Cnt, 3, 12);
    OLED_ShowString(36, 44, "R:", 12); //���հ�����
    OLED_ShowNum(48, 44, R_Cnt, 3, 12);
    OLED_ShowString(72, 44, "E:", 12); //���հ�����
    OLED_ShowNum(84, 44, E_Cnt, 3, 12);
    OLED_Refresh();		//������ʾ��OLED
    beep_off();
    //LEDOFF(LED_ALL);
		HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    MX_TIM3_Init_Ms(1500);//��ʱ����ʼ��
    HAL_TIM_Base_Stop_IT(&TIM3_InitStruct);//��ʱֹͣ
    HAL_TIM_Base_Start_IT(&TIM3_InitStruct);//��ʱ����
		
    while (1)
    {
			  HAL_PWR_EnableSleepOnExit();//MCU����͹���	
        OLED_Refresh();		//������ʾ��OLED	  
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


//�жϷ��غ���
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(KEY1 == 0)			//��߰���K1���ò���
    {
        HAL_Delay(50);
        if(KEY1 == 0)
        {
            HAL_TIM_Base_Stop_IT(&TIM3_InitStruct);
            DIO1_DisableInterrupt();
            static uint8_t t = 4;
            t++;
            switch(t)
            {
            case 5:
            {
                G_LoRaConfig.SpreadingFactor = LORA_SF5;
                MX_TIM3_Init_Ms(1000);
                break;
            }
            case 6:
            {
                G_LoRaConfig.SpreadingFactor = LORA_SF6;
                MX_TIM3_Init_Ms(1000);
                break;
            }
            case 7:
            {
                G_LoRaConfig.SpreadingFactor = LORA_SF7;
                MX_TIM3_Init_Ms(1500);
                break;
            }
            case 8:
            {
                G_LoRaConfig.SpreadingFactor = LORA_SF8;
                MX_TIM3_Init_Ms(1500);
                break;
            }
            case 9:
            {
                G_LoRaConfig.SpreadingFactor = LORA_SF9;
                MX_TIM3_Init_Ms(2000);
                break;
            }
            case 10:
            {
                G_LoRaConfig.SpreadingFactor = LORA_SF10;
                MX_TIM3_Init_Ms(2500);
                break;
            }
            case 11:
            {
                G_LoRaConfig.SpreadingFactor = LORA_SF11;
                MX_TIM3_Init_Ms(3500);
                break;
            }
            case 12:
            {
                G_LoRaConfig.SpreadingFactor = LORA_SF12;
                MX_TIM3_Init_Ms(5000);
                break;
            }
            default :
                break;
            }
            OLED_ShowNum(18, 12, t, 2, 12);
            if(t > 11)
            {
                t = 4;
            }
        }
    }

    if(KEY2 == 0)			//��߰���K2���ò���
    {
        HAL_Delay(50);
        if(KEY2 == 0)
        {
            HAL_TIM_Base_Stop_IT(&TIM3_InitStruct);
            DIO1_DisableInterrupt();
            static int8_t j = 0;
            j++;
            G_LoRaConfig.LoRa_Freq = Fre[j];
            OLED_ShowNum(48, 0, G_LoRaConfig.LoRa_Freq, 9, 12);
            if(j >= 4)
            {
                j = -1;
            }
        }
    }
		
    if(KEY3 == 0)  //��߰���K3��ʼͨ��
    {
        HAL_Delay(50);
        if(KEY3 == 0)
        {
            if(SX126x_Lora_init() != NORMAL)	 //����ģ���ʼ��
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
            OLED_ShowNum(12, 44, T_Cnt, 3, 12);
            OLED_ShowNum(48, 44, R_Cnt, 3, 12);
            OLED_ShowNum(84, 44, E_Cnt, 3, 12);
            DIO1_EnableInterrupt();
            HAL_TIM_Base_Start_IT(&TIM3_InitStruct);
        }
    }

		
    /*********************DIO1 �жϴ���*********************/
    if(DIO1_GetState() == GPIO_PIN_SET)
    {
        uint16_t flag;
        flag = SX126xGetIrqStatus();
        SX126xClearIrqStatus( IRQ_RADIO_ALL ); //clear flags
        if((flag & IRQ_TX_DONE) == IRQ_TX_DONE)//����TxDone�����ݷ������
        {	         					
            LEDOFF(LED3_BLUE);//���ݷ������Ϩ������
            T_Cnt++;
            if(T_Cnt > 999)
            {
                T_Cnt = 0;
            }
            OLED_ShowNum(12, 44, T_Cnt, 3, 12); //��ʾ�ѷ������ݰ�����
            DIO1_EnableInterrupt();
            SX126X_StartRx();//�������ģʽ��׼����������
        }
//				else if((flag & IRQ_RX_DONE) == IRQ_RX_DONE)//����RxDone���н��յ�����
        else if((flag & (IRQ_RX_DONE | IRQ_CRC_ERROR)) == IRQ_RX_DONE)//����RxDone���н��յ�����
        {
					  if(G_LoRaConfig.HeaderType ==LORA_PACKET_IMPLICIT)
						{
							SX126xClearTimeoutEvent();
						}
            DIO1_DisableInterrupt();
            SX126X_RxPacket(RXbuffer);
            R_Cnt++;
            if(R_Cnt > 999)
            {
                R_Cnt = 0;
            }
            OLED_ShowNum(48, 44, R_Cnt, 3, 12);//��ʾ���յ����ݰ�����

            if(memcmp(RXbuffer, TXbuffer, 10) == 0)//���յ����ݶ����ж���ȷ
            {
//							  Usart2SendData(RXbuffer,10);//���ڴ�ӡ�����յ�������
                RSSI = G_LoRaPara.LastPacket_RSSI;//���һ�����ݵ�RSSIֵ��С
                SNR = G_LoRaPara.Packet_SNR;//SNRֵ��С
                Avg_RSSI = G_LoRaPara.AvgPacket_RSSI; //���ݰ�ƽ��RSSIֵ��С
                LEDON(LED2_GRREN);  //�����̵�
                HAL_Delay(50);

                //SNR��RSSI��С��ʾ����
                if(SNR < 0)
                {
                    OLED_ShowString(24, 26, "-", 12);
                    SNR = -SNR;
                    OLED_ShowNum(30, 26, SNR, 4, 12);
//										DEBUG_Printf("SNR:-%d\t",SNR);//���ڴ�ӡ
                }
                else
                {
                    OLED_ShowString(24, 26, " ", 12);
                    OLED_ShowNum(30, 26, SNR, 4, 12);
//									 DEBUG_Printf("SNR:%d\t",SNR);//���ڴ�ӡ
                }

                if(RSSI < 0)
                {
                    OLED_ShowString(90, 26, "-", 12);
                    RSSI = -RSSI;
                    OLED_ShowNum(96, 26, RSSI, 4, 12);
//									  DEBUG_Printf("RSSI:-%d\t Avg_RSSI:%d\t",RSSI,Avg_RSSI);//���ڴ�ӡ
                }
                else
                {
                    OLED_ShowString(90, 26, " ", 12);
                    OLED_ShowNum(96, 26, RSSI, 4, 12);
//										DEBUG_Printf("RSSI:%d\t Avg_RSSI:%d\t",RSSI,Avg_RSSI);//���ڴ�ӡ
                }

                LEDOFF(LED2_GRREN); //Ϩ���̵�
            }

            else
            {
                E_Cnt++;
                if(E_Cnt > 999)
                {
                    E_Cnt = 0;
                }
            }
            OLED_ShowNum(84, 44, E_Cnt, 3, 12);
        }
//				DEBUG_Printf("TX:%d\t",T_Cnt);//���ڴ�ӡ
//				DEBUG_Printf("RX:%d\t",R_Cnt);//���ڴ�ӡ
//				DEBUG_Printf("E:%d\n",E_Cnt);//���ڴ�ӡ
    }


    HAL_PWR_DisableSleepOnExit();//MCU�˳��͹���ģʽ
}

/*******************Callback********************/
//��ʱ�����غ���  /* TIM Update event */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    LEDON(LED3_BLUE);//��������
    DIO1_EnableInterrupt();
    SX126X_TxPacket(TXbuffer);//��������
    HAL_PWR_DisableSleepOnExit();//MCU�˳��͹���ģʽ
}
