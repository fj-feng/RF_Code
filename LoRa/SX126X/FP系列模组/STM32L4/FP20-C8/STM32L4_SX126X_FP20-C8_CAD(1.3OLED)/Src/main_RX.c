/**
  ******************************************************************************
  * 文件名 ：   main_RX.c
  * 作者   ：   LSD RF Team
  * 版本   ：   V1.0.0
  * 时间   ：   15-Oct-2018
  * 文件描述：
  *    B模块收到A模块发送的数据后，RX计数加1，然后将数据回传给A模块，循环操作
	
	*修改日志：
	2019.02.21 定时器打开初始化函数中增加删除标志，避免第一次开启就进中断；超时定时器初始化移动
	           到定时器开启函数中，每次开启时时间重新初始化，优化之前超时定时器导致未进接收；
	
*******************************************************************************
**/
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
//===================================????===================================================
enum DemoInternalStates
{
    APP_IDLE = 0,               // nothing to do (or wait a radio interrupt)
    TX_ING,
    RX_DONE,
    TX_DONE,
    CAD_ING,
    CAD_DONE,
    CAD_DETE,
    APP_RNG,
};
uint8_t TXbuffer[10] = {0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA};
uint8_t WakeAddr[8] = {0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8};
uint8_t RXbuffer[10] = {0};
uint32_t Fre[5] = {868800000, 902500000,910700000 , 915800000, 928700000};

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
    BEEP_Init();
    LED_Init();//LED初始化
    KEY_Init();
    OLED_Init();
    //配置各个参数
    G_LoRaConfig.LoRa_Freq = Fre[0];      //中心频率470MHz
    G_LoRaConfig.BandWidth = LORA_BW_125;    //BW = 125KHz  BW125KHZ
    G_LoRaConfig.SpreadingFactor = LORA_SF7;  //SF = 9
    G_LoRaConfig.CodingRate = LORA_CR_4_6;     //CR = 4/6
    G_LoRaConfig.PowerCfig = 22;                //输入范围：-3~20，最大输出功率设置为20，即输出最大功率20dBm；
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
    MX_TIM3_Init_Ms(1000);//睡眠定时周期
    HAL_TIM_Base_Stop_IT(&TIM3_InitStruct);
    OLED_ShowString(0, 0, "CAD-RX:         Hz", 12); //在0列，0行，显示 LORA-TX:        Hz字符,使用12字体；
    OLED_ShowNum(48, 0, G_LoRaConfig.LoRa_Freq, 9, 12); //在48列，0行，显示数字，多少位，使用12字体；
    OLED_ShowString(0, 12, "SF:", 12);
    OLED_ShowNum(18, 12, SF, 2, 12);
    OLED_ShowString(42, 12, "BW:", 12);
    OLED_ShowNum(60, 12, G_BandWidthKHz, 3, 12);
    OLED_ShowString(0, 26, "SNR:", 12);
    OLED_ShowString(60, 26, "RSSI:", 12);
    OLED_ShowString(0, 44, "RX:", 16); //接收包计数
    OLED_ShowString(72, 44, "(   )", 16);
    OLED_ShowNum(80, 44, E_Cnt, 3, 16);
    OLED_ShowNum(24, 44, R_Cnt, 6, 16);
    OLED_Refresh();		//更新显示到OLED
    DIO1_EnableInterrupt();
    //SX126X_StartRx();
    SX126X_WORInit();
    SX126X_WOR_Execute(0);
    ON_Sleep_Timerout();     //启动睡眠超时定时器
    communication_states = APP_IDLE;
    while (1)
    {
        switch(communication_states)
        {
        case APP_IDLE:
        {
            __ASM("NOP");
        }
        break;

        case TX_ING:
        {
            communication_states = APP_IDLE;
            LEDON(LED3_BLUE);
            DIO1_EnableInterrupt();
            G_LoRaConfig.PayloadLength = 10;      //更新PL长度
            G_LoRaConfig.PreambleLength = 8;      //更新前导码长度
            SX126X_TxPacket(RXbuffer);
            break;
        }
        case CAD_ING:
        {
            LEDON(LED2_GRREN);
            OFF_Sleep_Timerout();    //关闭睡眠超时定时器
            DIO1_EnableInterrupt();
            SX126X_WOR_Execute(1);   //执行CAD
            communication_states = APP_IDLE;
            break;
        }
        case TX_DONE:
        {
            LEDOFF(LED3_BLUE);//熄灭蓝灯
            SX126X_WORInit();
            SX126X_WOR_Execute(0);
            ON_Sleep_Timerout();     //启动睡眠超时定时器
            communication_states = APP_IDLE;
            break;
        }
        case RX_DONE:
        {
            OFF_Timerout();
            DIO1_DisableInterrupt();
            SX126X_RxPacket(RXbuffer);
            R_Cnt++;
            if(R_Cnt > 999)
            {
                R_Cnt = 0;
            }
            OLED_ShowNum(24, 44, R_Cnt, 6, 16);
            LEDOFF(LED2_GRREN);
            if(memcmp(RXbuffer, WakeAddr, 8) == 0)
            {
                RSSI = G_LoRaPara.LastPacket_RSSI;
                SNR = G_LoRaPara.Packet_SNR;

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

                communication_states = TX_ING;
            }
            else
            {
                E_Cnt++;
                if(E_Cnt > 999)
                {
                    E_Cnt = 0;
                }
                OLED_ShowNum(80, 44, E_Cnt, 3, 16);
                communication_states = TX_ING;
            }
            break;
        }

        case CAD_DONE:
        {
            uint16_t CAD_flag;
            CAD_flag = SX126xGetIrqStatus();
            SX126xClearIrqStatus( IRQ_RADIO_ALL ); //clear flagss
            if((CAD_flag & IRQ_CAD_ACTIVITY_DETECTED) == IRQ_CAD_ACTIVITY_DETECTED)
            {
								
                LEDON(LED2_GRREN);
                communication_states = CAD_DETE;
            }
            else
            {
                LEDOFF(LED2_GRREN);
                SX126X_WOR_Execute(0);
                ON_Sleep_Timerout();     //启动睡眠超时定时器
                communication_states = APP_IDLE;
            }
            break;
        }


        case CAD_DETE:
        {
					  OFF_Sleep_Timerout();
            SX126X_WOR_Exit();
						ON_Timerout();
            communication_states = APP_IDLE;
            break;
            default :
                break;

            }
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
//							  MX_TIM3_Init_Ms(7000);
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
            R_Cnt = 0;
            E_Cnt = 0;
            OLED_ShowNum(24, 44, R_Cnt, 6, 16);
            OLED_ShowNum(80, 44, E_Cnt, 3, 16);
            OLED_Refresh();
            SX126X_WORInit();
            SX126X_WOR_Execute(0);
            communication_states = CAD_ING;
        }
    }

    if(DIO1_GetState() == GPIO_PIN_SET)
    {
        uint16_t flag;
        flag = SX126xGetIrqStatus();
//        SX126xClearIrqStatus( IRQ_RADIO_ALL ); //clear flags
        if((flag & IRQ_TX_DONE) == IRQ_TX_DONE)
        {
            communication_states = TX_DONE;
        }
//        if((flag & IRQ_RX_DONE) == IRQ_RX_DONE)
				if((flag & (IRQ_RX_DONE|IRQ_CRC_ERROR))==IRQ_RX_DONE)
        {
					//隐式报头模式(Implicit Header)在开启接收超时时，可能出现接收成功后，超时定时器不关闭现象，增加SX126xClearTimeoutEvent()函数进行停止RTC和清除超时事件。
					if(G_LoRaConfig.HeaderType ==LORA_PACKET_IMPLICIT)
						{
							SX126xClearTimeoutEvent();
						}
            communication_states = RX_DONE;
        }
        if((flag & IRQ_CAD_DONE) == IRQ_CAD_DONE)
        {
            communication_states = CAD_DONE;
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
        communication_states = CAD_ING;
    }
    else if(htim == &TIM4_InitStruct)
    {
        OFF_Timerout();
        SX126X_WORInit();
        SX126X_WOR_Execute(0);
        ON_Sleep_Timerout();     //启动睡眠超时定时器
    }
}
