/**
  ******************************************************************************
  * 文件名 ：   main_TX.c
  * 作者   ：   LSD RF Team
  * 版本   ：   V1.0.0
  * 时间   ：   15-Oct-2018
  * 文件描述：
  *    
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
#include "STM32L4_Flash.h"
#include "usart.h"
#include "data_pro.h"
//===================================??????===================================================

 uint64_t Defaut_AddrID __attribute__ ((at(ID_ADDR_FLASH)))=0xFFFFFF00;  //默认ID:00 ff ff ff(低位在前)，直接写到Flash中
 uint8_t Addr_ID[4]; //本机产品ID
 tUnionAddr R_Addr_ID;//中间变量
 uint8_t Pack_CRC;
 
enum DemoInternalStates
{
    APP_RNG = 0, // nothing to do (or wait a radio interrupt)
    RX_DONE,
    TX_DONE,
    TX_ING,
    APP_IDLE,
};
uint8_t WakeAddr[4] = {0x00,0xFF,0xFF,0xFF};
uint8_t R_Databuffer[7];
uint8_t RXbuffer[64] = {0};
uint32_t Fre_Wake[5] = {471800000,492570000, 509770000,481900000,503200000};
uint32_t Fre_Read[5] = {490200000,492570000, 509770000,481900000,503200000};

//long SysTick = 0;
uint16_t T_Cnt = 0;
uint16_t R_Cnt = 0;
uint16_t Tri_Cnt = 0;
uint8_t  TX_TimeOut_Flag=RESET;//单包数据超时标志

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
		MCU_DMA_Init();
		MX_USART2_UART_Init();
	  Clear_UART2_IT();
		Usart2_RX.receive_flag=RESET;
		//BEEP_Init();
    //配置各个参数
    G_LoRaConfig.LoRa_Freq = Fre_Wake[0];      //中心频率
    G_LoRaConfig.BandWidth = LORA_BW_125;    //BW = 125KHz
    G_LoRaConfig.SpreadingFactor = LORA_SF7;  //SF =7
    G_LoRaConfig.CodingRate = LORA_CR_4_6;     //CR = 4/6
    G_LoRaConfig.PowerCfig = 22;             //输入范围：-3~22，根据实际使用硬件LSD4RFC-2L722N10或LSD4RFC-2R714N10选择SX126xSetTxParams函数
    G_LoRaConfig.HeaderType = LORA_PACKET_EXPLICIT;       //包头格式设置，显性包头：LORA_PACKET_EXPLICIT；隐性包头：LORA_PACKET_IMPLICIT
    //若设置为显性包头，发送端将会将PalyLoad长度、编码率、CRC等加入到包头中发送给接收端
    G_LoRaConfig.CrcMode = LORA_CRC_ON;             //CRC校验开启：LORA_CRC_ON，关闭：LORA_CRC_OFF
    G_LoRaConfig.InvertIQ = LORA_IQ_NORMAL;         //
    G_LoRaConfig.PreambleLength = 8;      //前导码长度
    G_LoRaConfig.PayloadLength = 4;      //数据包长度

    if(SX126x_Lora_init() != NORMAL)	 //无线模块初始化
    {
        while(1)
        {
            OLED_Clear();
            OLED_ShowString(0, 0, "ERRoR", 24);
            OLED_Refresh();		//更新显示到OLED
        }
    }
		//获取ID号
		STMFLASH_Read2(ID_ADDR_FLASH,(uint64_t*)R_Addr_ID.U8Addr,4);
		memcpy(Addr_ID,R_Addr_ID.U8Addr,4);
		OLED_ShowString_N(0, 0, "M_ID: ",12,1); //在48列，0行，显示数字，多少位，使用12字体；
		OLED_ShowNum(30, 0, Addr_ID[0], 3, 12);
		OLED_ShowString(48, 0, ".", 12);
		OLED_ShowNum(54, 0, Addr_ID[1], 3, 12);
		OLED_ShowString(72, 0, ".", 12);
		OLED_ShowNum(78, 0, Addr_ID[2], 3, 12);
		OLED_ShowString(96, 0, ".", 12);
		OLED_ShowNum(102, 0, Addr_ID[3], 3, 12);
		OLED_Refresh();		//更新显示到OLED	
		
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

    MX_TIM3_Init_Ms(500);//TX Start定时器初始值
		HAL_TIM_Base_Stop_IT(&TIM3_InitStruct);
		
		MX_TIM4_Init_Ms(4500);//超时定时器周期(这里是用来控制发包数量的)
    HAL_TIM_Base_Stop_IT(&TIM4_InitStruct);

    OLED_ShowString(0, 12, "CAD-TX:         Hz", 12); //在0列，0行，显示 LORA-TX:        Hz字符,使用12字体；
    OLED_ShowNum(48, 12, G_LoRaConfig.LoRa_Freq, 9, 12); //在48列，0行，显示数字，多少位，使用12字体；
    OLED_ShowString(0, 24, "SF:", 12);
    OLED_ShowNum(18, 24, SF, 2, 12);
    OLED_ShowString(42, 24, "BW:", 12);
    OLED_ShowNum(60, 24, G_BandWidthKHz, 3, 12);
    OLED_ShowString(0, 36, "SNR:", 12);
    OLED_ShowString(60, 36, "RSSI:", 12);
    OLED_ShowString(0, 48, "T:", 12); //抄表次数
    OLED_ShowNum(12, 48, T_Cnt, 3, 12);
    OLED_ShowString(36, 48, "R:", 12); //接收包计数
    OLED_ShowNum(48, 48, R_Cnt, 3, 12);
    OLED_ShowString(72, 48, "TRI:", 12); //唤醒包数
    OLED_ShowNum(96, 48, Tri_Cnt, 3, 12);
    OLED_Refresh();		//更新显示到OLED
    //HAL_TIM_Base_Start_IT(&TIM3_InitStruct);//定时开启
    uint8_t ReadMeter_Buffer[13]={0xAA,0X01,0x85,0x00,0x00,0x00,0x01,0xAA,0xAA,0x4F,0x4B,0x75,0x55};
    while (1)
    {
				UART2_REC_Process();//ID配置接口及主动抄表接口
			
        switch(communication_states)
        {
        case APP_IDLE:

            __ASM("NOP");
            break;
        case TX_ING:
            communication_states = APP_IDLE;
						uint16_t Preamble_Length = 8;
						LEDON(LED3_BLUE);//点亮蓝灯
						DIO1_EnableInterrupt();
					  TX_TimeOut_Flag=SET;
						MX_TIM3_Init_Ms(1000);
						HAL_TIM_Base_Start_IT(&TIM3_InitStruct); //开启单包发送超时
            SX126X_Awake(WakeAddr, Preamble_Length);
				
            break;
        case TX_DONE:

						TX_TimeOut_Flag=RESET;
						HAL_TIM_Base_Stop_IT(&TIM3_InitStruct); //关闭单包发送超时
				
            Tri_Cnt++;
            if(Tri_Cnt > 999)
            {
                Tri_Cnt = 0;
            }
            OLED_ShowNum(96, 48, Tri_Cnt, 3, 12);
						communication_states = TX_ING;
            break;

        case RX_DONE:
            communication_states = APP_IDLE;
            DIO1_DisableInterrupt();
            SX126X_RxPacket(RXbuffer);
				    memcpy(R_Databuffer,RXbuffer,7);
				   
						Pack_CRC=checksum(RXbuffer,6);
						if(RXbuffer[6]==Pack_CRC)
//            if(memcmp(RXbuffer, Databuffer, 10) == 0)
            {	
							 R_Cnt++;
							if(R_Cnt > 999)
							{
									R_Cnt = 0;
							}
							OLED_ShowNum(48, 48, R_Cnt, 3, 12);
						
							memcpy(ReadMeter_Buffer+4,R_Databuffer,6);
				      ReadMeter_Buffer[10]=0x4F;
							ReadMeter_Buffer[11]=0x4B;
							ReadMeter_Buffer[12]=checksum(ReadMeter_Buffer+1,11);
				     	Usart2SendData(ReadMeter_Buffer,sizeof(ReadMeter_Buffer));
							
                RSSI = G_LoRaPara.LastPacket_RSSI;
                SNR = G_LoRaPara.Packet_SNR;
                LEDON(LED2_GRREN);  //点亮绿灯
                HAL_Delay(50);
                LEDOFF(LED2_GRREN); //熄灭绿灯
                if(SNR < 0)
                {
                    OLED_ShowString(24, 36, "-", 12);
                    SNR = -SNR;
                    OLED_ShowNum(30, 36, SNR, 4, 12);
                }
                else
                {
                    OLED_ShowString(24, 36, " ", 12);
                    OLED_ShowNum(30, 36, SNR, 4, 12);
                }
                if(RSSI < 0) {
                    OLED_ShowString(90, 36, "-", 12);
                    RSSI = -RSSI;
                    OLED_ShowNum(96, 36, RSSI, 4, 12);
                }
                else
                {
                    OLED_ShowString(90, 36, " ", 12);
                    OLED_ShowNum(96, 36, RSSI, 4, 12);
                }
            }
            else
            {
							
							memcpy(ReadMeter_Buffer+4,R_Databuffer,6);
				      ReadMeter_Buffer[10]=0x45;
							ReadMeter_Buffer[11]=0x52;
							ReadMeter_Buffer[12]=checksum(ReadMeter_Buffer+1,11);
				     	Usart2SendData(ReadMeter_Buffer,sizeof(ReadMeter_Buffer));
							
            }
						SX126X_WOR_Execute(0);//进入休眠
						OLED_Refresh();
            break;
        default :
            break;

        }
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
            OLED_ShowNum(18, 24, t, 2, 12);
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
            G_LoRaConfig.LoRa_Freq = Fre_Wake[j];
            OLED_ShowNum(48, 12, G_LoRaConfig.LoRa_Freq, 9, 12); 
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
						Tri_Cnt=0;
            OLED_ShowNum(12, 48, T_Cnt, 3, 12);
						OLED_ShowNum(48, 48, R_Cnt, 3, 12);
						OLED_ShowNum(96, 48, Tri_Cnt, 3, 12);
            OLED_Refresh();
          
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
			if(TX_TimeOut_Flag==SET)   //单数据包超时了
			{
					TX_TimeOut_Flag=RESET;
					HAL_TIM_Base_Stop_IT(&TIM3_InitStruct); //关闭单包发送超时
					communication_states = TX_ING;
			}
			
			else
			{
				HAL_TIM_Base_Stop_IT(&TIM3_InitStruct);
				
				 Tri_Cnt = 0;
				 T_Cnt++;
				 if(T_Cnt > 999)
				 {
					 T_Cnt = 0;
				 }
				 OLED_ShowNum(12, 48, T_Cnt, 3, 12); //抄表次数
				 
				SX126xSetStandby(STDBY_RC);
				SX126xCalibrate_err();	//防止上电之后或者是以Cold-Start的条件从Sleep唤醒，或者低温运行时sleep状态切换过来时出现PLL Lock，XoscStart等错误现象，导致RF无法工作；
				
				 G_LoRaConfig.LoRa_Freq = Fre_Wake[0];      //切换成通信信道
				 SX126xSetRfFrequency(G_LoRaConfig.LoRa_Freq);
				 
				 SX126xSetRegulatorMode( USE_DCDC );
				SetRxTxFallbackMode(Fallback_FS);//发送完成后进入FS状态；
				communication_states = TX_ING;
				
				MX_TIM4_Init_Ms(4500);//超时定时器周期(这里是用来控制发包数量的)
				ON_Timerout();//开启超时定时器，控制发射包数
		 }
			
    }
		
		else if(htim == &TIM4_InitStruct)
    {
			communication_states = APP_IDLE;
			
			TX_TimeOut_Flag=RESET;
			HAL_TIM_Base_Stop_IT(&TIM3_InitStruct); //关闭单包发送超时
			
			OLED_Refresh();
			LEDOFF(LED3_BLUE);//熄灭蓝灯
      OFF_Timerout();//关闭发射总时间超时
			SX126xSetStandby(STDBY_RC); 
//			G_LoRaConfig.PayloadLength = 7;      //更新PL长度
//			G_LoRaConfig.PreambleLength =0x1FF;      //更新前导码长度
			DIO1_EnableInterrupt();
			SetRxTxFallbackMode(Fallback_STDBY_RC);//发送完成后进入Fallback_STDBY_RC状态；
			
			SX126xSetStandby(STDBY_RC);
			G_LoRaConfig.LoRa_Freq = Fre_Read[0];      //切换成通信信道
			SX126xSetRfFrequency(G_LoRaConfig.LoRa_Freq);
			SX126X_StartRx();
    }
}



