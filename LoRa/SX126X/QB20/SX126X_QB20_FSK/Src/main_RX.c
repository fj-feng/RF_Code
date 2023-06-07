/**
  ******************************************************************************
  * 文件名 ：   main_TX.c
  * 作者   ：   LSD RF Team
  * 版本   ：   V1.0.0
  * 时间   ：   15-Oct-2018
  * 文件描述：
  *    A模块发送数据包给B模块，TX计数加1，B模块收到数据后，B模块将数据回传给A
  *模块，A模块收到数据后，RX计数加1，循环操作
	*程序默认配置如下：
			速率：4.8Kbps
			Dev：5KHz
			RxBW：20KHz
			数据白化：开启(0x1FF)
			地址滤波：开启节点地址0x11
			同步字：{ 0xC1, 0x94, 0xC1, 0x00, 0x00, 0x00, 0x00, 0x00 }，设置3字节
			CRC：开启RADIO_CRC_2_BYTES_CCIT
		
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
	
    G_FSKConfig.FSK_Freq = Fre[0];      //中心频率470MHz
	  G_FSKConfig.PowerCfig = 22;
	
	  //Modulation Paramenters
    G_FSKConfig.BitRate = 4800;				//600~300000
    G_FSKConfig.Fdev = 5000;						//Fdev<=250K - BR/2;
	  G_FSKConfig.Bandwidth = 20000;     //DSB ,4800~500000，BW>2*Fdev+BR+Fer
	  G_FSKConfig.ModulationShaping = MOD_SHAPING_G_BT_1;
	
	 //Packet Paramenters  
	  G_FSKConfig.PreambleLength = 5;         //1~8191，单位是字节
	  G_FSKConfig.PreambleMinDetect = RADIO_PREAMBLE_DETECTOR_08_BITS;
	  G_FSKConfig.SyncWordLength = 3;         //1~8，单位是字节
		G_FSKConfig.AddrComp = RADIO_ADDRESSCOMP_FILT_NODE;	//地址滤波设置；节点地址或广播地址
	  G_FSKConfig.HeaderType = RADIO_PACKET_VARIABLE_LENGTH;	//数据包格式
		G_FSKConfig.PayloadLength = 100;
		G_FSKConfig.CrcLength = RADIO_CRC_2_BYTES_CCIT; //CRC 类型
		G_FSKConfig.DcFree = RADIO_DC_FREEWHITENING;   //数据白化
	

    if(SX126x_FSK_init() != NORMAL)	 //无线模块初始化
    {
        while(1)
        {
            OLED_Clear();
            OLED_ShowString(0, 0, "ERRoR", 24);
            OLED_Refresh();		//更新显示到OLED
        }
    }

    MX_TIM3_Init_Ms(500);
    HAL_TIM_Base_Stop_IT(&TIM3_InitStruct);
    OLED_ShowString(0, 0, "FSK-TX:          Hz", 12); //在0列，0行，显示 FSK-TX:        Hz字符,使用12字体；
    OLED_ShowNum(42, 0, G_FSKConfig.FSK_Freq , 9, 12); //在48列，0行，显示数字，多少位，使用12字体；
    OLED_ShowString(0, 12, "BitRate:       bps", 12);
    OLED_ShowNum(48, 12, G_FSKConfig.BitRate, 6, 12);
    OLED_ShowString(0, 24, "Fdev:       Hz", 12);
    OLED_ShowNum(30, 24,  G_FSKConfig.Fdev, 6, 12);
    OLED_ShowString(0, 36, "Bandwidth:       Hz", 12);
		OLED_ShowNum(60, 36,  G_FSKConfig.Bandwidth, 6, 12);
    OLED_ShowString(0, 48, "R:", 12);
    OLED_ShowNum(12, 48, R_Cnt, 3, 12);
    OLED_ShowString(72, 48, "E:", 12); //接收包计数
    OLED_ShowNum(84, 48, E_Cnt, 3, 12);
    OLED_Refresh();		//更新显示到OLED
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
            SX126X_FSKTxPacket(TXbuffer);//发送数据
            break;
				
        case TX_DONE:
            communication_states = APP_IDLE;
            DIO1_EnableInterrupt();
            SX126X_FSKStartRx();//发送完成后开始进入接收模式
            LEDOFF(LED2_GRREN);//熄灭绿灯
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
							LEDON(LED2_GRREN);//点亮蓝灯
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


//DIO0中断返回函数

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(KEY1 == 0)			//左边按键K3设置参数
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
							G_FSKConfig.Bandwidth = 20000;	//DSB ,4800~500000，BW>2*Fdev+BR+Fer
                break;
            }
            case 2:
            {
							G_FSKConfig.BitRate = 38400;  	//600~300000
							G_FSKConfig.Fdev = 40000;     	//Fdev<=250K - BR/2;
							G_FSKConfig.Bandwidth = 160000;	//DSB ,4800~500000，BW>2*Fdev+BR+Fer
                
                break;
            }
						
						 case 3:
            {
							G_FSKConfig.BitRate = 250000;  	//600~300000
							G_FSKConfig.Fdev = 125000;     	//Fdev<=250K - BR/2;
							G_FSKConfig.Bandwidth = 500000;	//DSB ,4800~500000，BW>2*Fdev+BR+Fer
                
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

    if(KEY2 == 0)			//右边按键K2开始通讯
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
						if(SX126x_FSK_init() != NORMAL)	 //无线模块初始化
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
//定时器返回函数  /* TIM Update event */
/*******************Callback********************/
//定时器返回函数  /* TIM Update event */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
   
}
