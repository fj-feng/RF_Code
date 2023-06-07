/**
  ******************************************************************************
  * 文件名 ：   main_TX.c
  * 作者   ：   LSD RF Team
  * 版本   ：   V1.0.0
  * 时间   ：   15-Aug-2018
  * 文件描述：
  *    A模块发送数据包给B模块，TX计数加1，B模块收到数据后，B模块将数据回传给A
  *模块，A模块收到数据后，RX计数加1，循环操作
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
// 功能描述：主函数
// 参数输入：
// 返回参数：
// 说明    :
////////////////////////////////////////////////////////////////////////////////
int main(void)
{
    HAL_Init();
    /* Configure the system clock */
    SystemClock_Config();
    LED_Init();//LED初始化
    KEY_Init();
    BEEP_Init();
    OLED_Init();
    MX_USART2_UART_Init();

    //配置各个参数
    G_LoRaConfig.LoRa_Freq = Fre[0];      //中心频率
    G_LoRaConfig.BandWidth = LORA_BW_125;    //BW = 125KHz  BW125KHZ
    G_LoRaConfig.SpreadingFactor = LORA_SF9;  //SF = 9
    G_LoRaConfig.CodingRate = LORA_CR_4_6;     //CR = 4/6
    G_LoRaConfig.PowerCfig = 22;               //输入范围：-3~22，根据实际使用硬件LSD4RFC-2L722N10选择SX126xSetTxParams函数
    G_LoRaConfig.HeaderType = LORA_PACKET_EXPLICIT;       //包头格式设置，显性包头：LORA_PACKET_EXPLICIT；隐性包头：LORA_PACKET_IMPLICIT
    //若设置为显性包头，发送端将会将PalyLoad长度、编码率、CRC等加入到包头中发送给接收端

    G_LoRaConfig.CrcMode = LORA_CRC_ON;             //CRC校验开启：LORA_CRC_ON，关闭：LORA_CRC_OFF
    G_LoRaConfig.InvertIQ = LORA_IQ_NORMAL;         //IQ信号格式，LORA_IQ_NORMAL：标准模式，LORA_IQ_INVERTED：反转模式；
    G_LoRaConfig.PreambleLength = 8;    //前导码长度
    G_LoRaConfig.PayloadLength = 10;      //数据包长度

    if(SX126x_Lora_init() != NORMAL)	 //无线模块初始化
    {
        while(1)
        {
            OLED_Clear();
            OLED_ShowString(0, 0, "ERRoR", 24);
            OLED_Refresh();		//更新显示到OLED
        }
    }

    //液晶显示处理
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
    OLED_ShowString(0, 0, "LORA-TX:         Hz", 12); //在0列，0行，显示 LORA-TX:        Hz字符,使用12字体；
    OLED_ShowNum(48, 0, G_LoRaConfig.LoRa_Freq, 9, 12); //在48列，0行，显示数字，多少位，使用12字体；
    OLED_ShowString(0, 12, "SF:", 12);
    OLED_ShowNum(18, 12, SF, 2, 12);
    OLED_ShowString(42, 12, "BW:", 12);
    OLED_ShowNum(60, 12, G_BandWidthKHz, 3, 12);
    OLED_ShowString(0, 26, "SNR:", 12);
    OLED_ShowString(60, 26, "RSSI:", 12);
    OLED_ShowString(0, 44, "T:", 12);
    OLED_ShowNum(12, 44, T_Cnt, 3, 12);
    OLED_ShowString(36, 44, "R:", 12); //接收包计数
    OLED_ShowNum(48, 44, R_Cnt, 3, 12);
    OLED_ShowString(72, 44, "E:", 12); //接收包计数
    OLED_ShowNum(84, 44, E_Cnt, 3, 12);
    OLED_Refresh();		//更新显示到OLED
    beep_off();
    //LEDOFF(LED_ALL);
		HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    MX_TIM3_Init_Ms(1500);//定时器初始化
    HAL_TIM_Base_Stop_IT(&TIM3_InitStruct);//定时停止
    HAL_TIM_Base_Start_IT(&TIM3_InitStruct);//定时开启
		
    while (1)
    {
			  HAL_PWR_EnableSleepOnExit();//MCU进入低功耗	
        OLED_Refresh();		//更新显示到OLED	  
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


//中断返回函数
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(KEY1 == 0)			//左边按键K1设置参数
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

    if(KEY2 == 0)			//左边按键K2设置参数
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
		
    if(KEY3 == 0)  //左边按键K3开始通信
    {
        HAL_Delay(50);
        if(KEY3 == 0)
        {
            if(SX126x_Lora_init() != NORMAL)	 //无线模块初始化
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

		
    /*********************DIO1 中断处理*********************/
    if(DIO1_GetState() == GPIO_PIN_SET)
    {
        uint16_t flag;
        flag = SX126xGetIrqStatus();
        SX126xClearIrqStatus( IRQ_RADIO_ALL ); //clear flags
        if((flag & IRQ_TX_DONE) == IRQ_TX_DONE)//产生TxDone，数据发送完成
        {	         					
            LEDOFF(LED3_BLUE);//数据发送完成熄灭蓝灯
            T_Cnt++;
            if(T_Cnt > 999)
            {
                T_Cnt = 0;
            }
            OLED_ShowNum(12, 44, T_Cnt, 3, 12); //显示已发送数据包数量
            DIO1_EnableInterrupt();
            SX126X_StartRx();//进入接收模式，准备接收数据
        }
//				else if((flag & IRQ_RX_DONE) == IRQ_RX_DONE)//产生RxDone，有接收到数据
        else if((flag & (IRQ_RX_DONE | IRQ_CRC_ERROR)) == IRQ_RX_DONE)//产生RxDone，有接收到数据
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
            OLED_ShowNum(48, 44, R_Cnt, 3, 12);//显示接收到数据包数量

            if(memcmp(RXbuffer, TXbuffer, 10) == 0)//接收到数据读出判断正确
            {
//							  Usart2SendData(RXbuffer,10);//串口打印出接收到的数据
                RSSI = G_LoRaPara.LastPacket_RSSI;//最后一包数据的RSSI值大小
                SNR = G_LoRaPara.Packet_SNR;//SNR值大小
                Avg_RSSI = G_LoRaPara.AvgPacket_RSSI; //数据包平均RSSI值大小
                LEDON(LED2_GRREN);  //点亮绿灯
                HAL_Delay(50);

                //SNR、RSSI大小显示处理
                if(SNR < 0)
                {
                    OLED_ShowString(24, 26, "-", 12);
                    SNR = -SNR;
                    OLED_ShowNum(30, 26, SNR, 4, 12);
//										DEBUG_Printf("SNR:-%d\t",SNR);//串口打印
                }
                else
                {
                    OLED_ShowString(24, 26, " ", 12);
                    OLED_ShowNum(30, 26, SNR, 4, 12);
//									 DEBUG_Printf("SNR:%d\t",SNR);//串口打印
                }

                if(RSSI < 0)
                {
                    OLED_ShowString(90, 26, "-", 12);
                    RSSI = -RSSI;
                    OLED_ShowNum(96, 26, RSSI, 4, 12);
//									  DEBUG_Printf("RSSI:-%d\t Avg_RSSI:%d\t",RSSI,Avg_RSSI);//串口打印
                }
                else
                {
                    OLED_ShowString(90, 26, " ", 12);
                    OLED_ShowNum(96, 26, RSSI, 4, 12);
//										DEBUG_Printf("RSSI:%d\t Avg_RSSI:%d\t",RSSI,Avg_RSSI);//串口打印
                }

                LEDOFF(LED2_GRREN); //熄灭绿灯
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
//				DEBUG_Printf("TX:%d\t",T_Cnt);//串口打印
//				DEBUG_Printf("RX:%d\t",R_Cnt);//串口打印
//				DEBUG_Printf("E:%d\n",E_Cnt);//串口打印
    }


    HAL_PWR_DisableSleepOnExit();//MCU退出低功耗模式
}

/*******************Callback********************/
//定时器返回函数  /* TIM Update event */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    LEDON(LED3_BLUE);//点亮蓝灯
    DIO1_EnableInterrupt();
    SX126X_TxPacket(TXbuffer);//发送数据
    HAL_PWR_DisableSleepOnExit();//MCU退出低功耗模式
}
