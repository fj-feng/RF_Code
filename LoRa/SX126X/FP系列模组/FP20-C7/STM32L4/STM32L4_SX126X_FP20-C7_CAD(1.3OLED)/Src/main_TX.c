/**
  ******************************************************************************
  * 文件名 ：   main_TX.c
  * 作者   ：   LSD RF Team
  * 版本   ：   V1.0.0
  * 时间   ：   15-Oct-2018
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
#include "SX126X_Driver.h"
//===================================??????===================================================
enum DemoInternalStates
{
    APP_RNG = 0, // nothing to do (or wait a radio interrupt)
    RX_DONE,
    TX_DONE,
    TX_ING,
    APP_IDLE,
};
uint8_t TXbuffer[10] = {0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA};
uint8_t WakeAddr[8] = {0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8};
uint8_t RXbuffer[10] = {0};
uint32_t Fre[5] = {470800000, 480570000, 492570000, 500000000, 510000000};
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
    OLED_Init();
    BEEP_Init();
    //配置各个参数
    G_LoRaConfig.LoRa_Freq = Fre[0];      //中心频率470MHz
    G_LoRaConfig.BandWidth = LORA_BW_125;    //BW = 125KHz  BW125KHZ
    G_LoRaConfig.SpreadingFactor = LORA_SF7;  //SF = 7
    G_LoRaConfig.CodingRate = LORA_CR_4_6;     //CR = 4/5
    G_LoRaConfig.PowerCfig = 22;             //输入范围：-3~22，根据实际使用硬件LSD4RF-2R717N30或LSD4RF-2R722N20选择SX126xSetTxParams函数
    G_LoRaConfig.HeaderType = LORA_PACKET_EXPLICIT;       //包头格式设置，显性包头：LORA_PACKET_EXPLICIT；隐性包头：LORA_PACKET_IMPLICIT
    //若设置为显性包头，发送端将会将PalyLoad长度、编码率、CRC等加入到包头中发送给接收端

    G_LoRaConfig.CrcMode = LORA_CRC_ON;             //CRC校验开启：LORA_CRC_ON，关闭：LORA_CRC_OFF
    G_LoRaConfig.InvertIQ = LORA_IQ_NORMAL;         //
    G_LoRaConfig.PreambleLength = 8;      //前导码长度
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

//    for(int i = 0; i < 200; i++)
//    {
//        HAL_Delay(1);
//        beep_off();
//        HAL_Delay(1);
//        beep_on();
//    }
    beep_off();

    //处理SF显示
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

    MX_TIM3_Init_Ms(5500);
    HAL_TIM_Base_Stop_IT(&TIM3_InitStruct);

    OLED_ShowString(0, 0, "CAD-TX:         Hz", 12); //在0列，0行，显示 LORA-TX:        Hz字符,使用12字体；
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

    HAL_TIM_Base_Start_IT(&TIM3_InitStruct);//定时开启

    while (1)
    {
        switch(communication_states)
        {
        case APP_IDLE:

            __ASM("NOP");
            break;
        case TX_ING:
            communication_states = APP_IDLE;
            uint16_t Preamble_Length = (uint16_t)(1500 / G_TsXms + 0.5);
            G_LoRaConfig.PayloadLength = 8;
            LEDON(LED3_BLUE);//点亮蓝灯
            DIO1_EnableInterrupt();
            SX126X_Awake(WakeAddr, Preamble_Length);
            break;
        case TX_DONE:
            communication_states = APP_IDLE;
            LEDOFF(LED3_BLUE);//熄灭蓝灯
//            SX126xSetStandby(STDBY_RC);
            T_Cnt++;
            if(T_Cnt > 999)
            {
                T_Cnt = 0;
            }
            OLED_ShowNum(12, 44, T_Cnt, 3, 12);
            G_LoRaConfig.PayloadLength = 10;      //更新PL长度
            G_LoRaConfig.PreambleLength = 8;      //更新前导码长度
            DIO1_EnableInterrupt();
            SX126X_StartRx();
            break;

        case RX_DONE:
            communication_states = APP_IDLE;
            DIO1_DisableInterrupt();
            SX126X_RxPacket(RXbuffer);
            R_Cnt++;
            if(R_Cnt > 999)
            {
                R_Cnt = 0;
            }
            OLED_ShowNum(48, 44, R_Cnt, 3, 12);
            if(memcmp(RXbuffer, WakeAddr, 8) == 0)
            {
                RSSI = G_LoRaPara.LastPacket_RSSI;
                SNR = G_LoRaPara.Packet_SNR;
                LEDON(LED2_GRREN);  //点亮绿灯
                HAL_Delay(50);
                LEDOFF(LED2_GRREN); //熄灭绿灯
                if(SNR < 0)
                {
                    OLED_ShowString(24, 26, "-", 12);
                    SNR = -SNR;
                    OLED_ShowNum(30, 26, SNR, 4, 12);
                }
                else
                {
                    OLED_ShowString(24, 26, " ", 12);
                    OLED_ShowNum(30, 26, SNR, 4, 12);
                }
                if(RSSI < 0) {
                    OLED_ShowString(90, 26, "-", 12);
                    RSSI = -RSSI;
                    OLED_ShowNum(96, 26, RSSI, 4, 12);
                }
                else
                {
                    OLED_ShowString(90, 26, " ", 12);
                    OLED_ShowNum(96, 26, RSSI, 4, 12);
                }
            }
            else
            {
                E_Cnt++;
                if(E_Cnt > 999)
                {
                    E_Cnt = 0;
                }
                OLED_ShowNum(84, 44, E_Cnt, 3, 12);
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


//DIO0中断返回函数

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(KEY1 == 0)			//左边按键K3设置参数
    {
        HAL_Delay(50);
        if(KEY1 == 0)
        {
            HAL_TIM_Base_Stop_IT(&TIM3_InitStruct);
            static uint8_t t = 4;
            t++;
            switch(t)
            {
            case 5:
            {
                G_LoRaConfig.SpreadingFactor = LORA_SF5;
                break;
            }
            case 6:
            {
                G_LoRaConfig.SpreadingFactor = LORA_SF6;
                break;
            }
            case 7:
            {
                G_LoRaConfig.SpreadingFactor = LORA_SF7;
								SX126xSetCadParams( LORA_CAD_02_SYMBOL, 22, 10, LORA_CAD_ONLY, 0xff );//
                break;
            }
            case 8:
            {
                G_LoRaConfig.SpreadingFactor = LORA_SF8;
								SX126xSetCadParams( LORA_CAD_02_SYMBOL, 22, 10, LORA_CAD_ONLY, 0xff );//
                break;
            }
            case 9:
            {
                G_LoRaConfig.SpreadingFactor = LORA_SF9;
								SX126xSetCadParams( LORA_CAD_04_SYMBOL, 23, 10, LORA_CAD_ONLY, 0xff );//
                break;
            }
            case 10:
            {
                G_LoRaConfig.SpreadingFactor = LORA_SF10;
								SX126xSetCadParams( LORA_CAD_04_SYMBOL, 24, 10, LORA_CAD_ONLY, 0xff );//
                break;
            }
            case 11:
            {
                G_LoRaConfig.SpreadingFactor = LORA_SF11;
								SX126xSetCadParams( LORA_CAD_04_SYMBOL, 25, 10, LORA_CAD_ONLY, 0xff );//
                break;
            }
            case 12:
            {
                G_LoRaConfig.SpreadingFactor = LORA_SF12;
								SX126xSetCadParams( LORA_CAD_04_SYMBOL, 26, 10, LORA_CAD_ONLY, 0xff );//
							  MX_TIM3_Init_Ms(7000);
                break;
            }
            default :
                break;
            }
            OLED_ShowNum(18, 12, t, 2, 12);
            OLED_Refresh();
            if(t > 11)
            {
                t = 4;
            }
        }
    }

    if(KEY2 == 0)			//右边按键K2开始通讯
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
            OLED_Refresh();
            HAL_TIM_Base_Start_IT(&TIM3_InitStruct);
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
					//隐式报头模式(Implicit Header)在开启接收超时时，可能出现接收成功后，超时定时器不关闭现象，增加SX126xClearTimeoutEvent()函数进行停止RTC和清除超时事件。
					if(G_LoRaConfig.HeaderType ==LORA_PACKET_IMPLICIT)
						{
							SX126xClearTimeoutEvent();
						}
						
            communication_states = RX_DONE;
        }
    }
}



/*******************Callback********************/
//定时器返回函数  /* TIM Update event */
/*******************Callback********************/
//定时器返回函数  /* TIM Update event */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim == &TIM3_InitStruct)
    {
        communication_states = TX_ING;
    }
}
