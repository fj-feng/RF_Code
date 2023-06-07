/**
  ******************************************************************************
  * 文件名 ：   main_RX.c
  * 作者   ：   LSD RF Team
  * 版本   ：   V1.0.0
  * 时间   ：   15-Oct-2018
  * 文件描述：
  *    
	
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
#include "STM32L4_Flash.h"
#include "usart.h"
#include "data_pro.h"
//===================================????===================================================
 uint64_t Defaut_AddrID __attribute__ ((at(ID_ADDR_FLASH)))=0xFFFFFF00;  //默认ID:00 ff ff ff(低位在前)，直接写到Flash中
 uint8_t Addr_ID[4]; //本机产品ID
 tUnionAddr R_Addr_ID;//中间变量


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

uint8_t WakeAddr[4] = {0x00,0xFF,0xFF,0xFF};
uint8_t T_Databuffer[7];
uint8_t RXbuffer[64] = {0};
uint8_t Temperature_Data[2];
uint32_t Fre_Wake[5] = {471800000,492570000, 509770000,481900000,503200000};
uint32_t Fre_Read[5] = {490200000,492570000, 509770000,481900000,503200000};
uint8_t  TX_ING_Flag=RESET;


//long SysTick = 0;

//uint16_t T_Cnt = 0;
uint16_t R_Cnt = 0;
//uint16_t E_Cnt = 0;
uint16_t OK_Cnt = 0;
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
	
		MCU_DMA_Init();
		MX_USART2_UART_Init();
	  Clear_UART2_IT();
		Usart2_RX.receive_flag=RESET;
	
    //配置各个参数
    G_LoRaConfig.LoRa_Freq = Fre_Wake[0];      //中心频率
    G_LoRaConfig.BandWidth = LORA_BW_125;    //BW = 125KHz
    G_LoRaConfig.SpreadingFactor = LORA_SF7;  //SF = 7
    G_LoRaConfig.CodingRate = LORA_CR_4_6;     //CR = 4/6
    G_LoRaConfig.PowerCfig = 22;                ////输入范围：-3~22，根据实际使用硬件LSD4RFC-2L722N10或LSD4RFC-2R714N10选择SX126xSetTxParams函数
    G_LoRaConfig.HeaderType = LORA_PACKET_EXPLICIT;       //包头格式设置，显性包头：LORA_PACKET_EXPLICIT；隐性包头：LORA_PACKET_IMPLICIT
    //若设置为显性包头，发送端将会将PalyLoad长度、编码率、CRC等加入到包头中发送给接收端

    G_LoRaConfig.CrcMode = LORA_CRC_ON;             //CRC校验开启：LORA_CRC_ON，关闭：LORA_CRC_OFF
    G_LoRaConfig.InvertIQ = LORA_IQ_NORMAL;         //
    G_LoRaConfig.PreambleLength = 8;      //前导码长度
    G_LoRaConfig.PayloadLength = 7;      //数据包长度

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
		OLED_ShowString_N(0, 0, "S_ID: ",12,1); //在48列，0行，显示数字，多少位，使用12字体；
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
		MX_TIM4_Init_Ms(5000);//超时定时器周期
    MX_TIM3_Init_Ms(4000);//睡眠定时周期
    HAL_TIM_Base_Stop_IT(&TIM3_InitStruct);
		HAL_TIM_Base_Stop_IT(&TIM4_InitStruct);
    OLED_ShowString(0, 12, "CAD-RX:         Hz", 12); //在0列，0行，显示 LORA-TX:        Hz字符,使用12字体；
    OLED_ShowNum(48, 12, G_LoRaConfig.LoRa_Freq, 9, 12); //在48列，0行，显示数字，多少位，使用12字体；
    OLED_ShowString(0, 24, "SF:", 12);
    OLED_ShowNum(18, 24, SF, 2, 12);
    OLED_ShowString(42, 24, "BW:", 12);
    OLED_ShowNum(60, 24, G_BandWidthKHz, 3, 12);
    OLED_ShowString(0, 36, "SNR:", 12);
    OLED_ShowString(60, 36, "RSSI:", 12);
		
    OLED_ShowString(0, 48, "RX:", 12); //接收到唤醒包次数
		OLED_ShowNum(24, 48, R_Cnt, 3, 12);
		
		OLED_ShowString(54, 48, "OK:", 12); //成功回复次数
		OLED_ShowNum(78, 48, OK_Cnt, 3, 12);
		
    
   
    OLED_Refresh();		//更新显示到OLED
    DIO1_EnableInterrupt();
    SX126X_WORInit();
    SX126X_WOR_Execute(0); //进入Sleep
    ON_Sleep_Timerout();     //启动睡眠超时定时器
    communication_states = APP_IDLE;
		
		
    while (1)
    {
			  UART2_REC_Process();//ID配置接口及主动抄表接口
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
					
            G_LoRaConfig.PayloadLength = 7;      //更新PL长度
					  SX126xSetStandby(STDBY_RC);
						//切换成抄表信道
					  G_LoRaConfig.LoRa_Freq = Fre_Read[0];      
					  SX126xSetRfFrequency(G_LoRaConfig.LoRa_Freq);
            SX126X_TxPacket(T_Databuffer);
					
						MX_TIM4_Init_Ms(2000); 
					  ON_Timerout();   //开启TX 执行超时
            break;
        }
        case CAD_ING:
        {
					  OFF_Sleep_Timerout();    //关闭睡眠超时定时器
            LEDON(LED2_GRREN);
            DIO1_EnableInterrupt();
            SX126X_WOR_Execute(1);   //执行CAD
						MX_TIM4_Init_Ms(1000);   
					  ON_Timerout();  				 //开启CAD 执行超时,正常是做完CAD，立即就会产生CADDone
            communication_states = APP_IDLE;
            break;
        }
        case TX_DONE:
        {
					  OFF_Timerout();  //关闭TX超时
            LEDOFF(LED3_BLUE);//熄灭蓝灯
						OK_Cnt++;       //成功回复包数
  					if(OK_Cnt > 999)
              {
                OK_Cnt = 0;
              }
							OLED_ShowNum(78, 48, OK_Cnt, 3, 12);
						SX126xSetStandby(STDBY_RC);
						//切换成唤醒包信道	
					  G_LoRaConfig.LoRa_Freq = Fre_Wake[0];      
					  SX126xSetRfFrequency(G_LoRaConfig.LoRa_Freq);
            SX126X_WORInit();
            SX126X_WOR_Execute(0);
					  MX_TIM3_Init_Ms(4000);//睡眠定时周期
            ON_Sleep_Timerout();     //启动睡眠超时定时器
            communication_states = APP_IDLE;
            break;
        }
        case RX_DONE:
        {
            OFF_Timerout();      //关闭接收超时
						LEDOFF(LED2_GRREN);  //熄灭绿灯
					  DIO1_DisableInterrupt();
						SX126X_RxPacket(RXbuffer);
            if(memcmp(RXbuffer, Addr_ID, 4) == 0)
            {						
								R_Cnt++;
								if(R_Cnt > 999)
								{
										R_Cnt = 0;
								}
								OLED_ShowNum(24, 48, R_Cnt, 3, 12);
                RSSI = G_LoRaPara.LastPacket_RSSI;
                SNR = G_LoRaPara.Packet_SNR;
								
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
								
								//获取温度，整合数据包
								
							 uint16_t Temp;
							 Temp=Get_Temperature_Sensor();
							 Temperature_Data[0]=Temp>>8;		    
							 Temperature_Data[1]=Temp>>0;	
								
							 memcpy(T_Databuffer,Addr_ID,4);
							 memcpy(T_Databuffer+4,Temperature_Data,2);
							 T_Databuffer[6]=checksum(T_Databuffer,6);
				
                //需要延时后进入TX，开启超时
								TX_ING_Flag=SET;
								SX126X_WOR_Execute(0);
								MX_TIM4_Init_Ms(4500); 
								ON_Timerout();//启动超时定时器，等待集中器端切换至接收模式
								communication_states = APP_IDLE;
            }
            else
            {	
								SX126X_WORInit();
								SX126X_WOR_Execute(0);
								MX_TIM3_Init_Ms(4000);//睡眠定时周期
								ON_Sleep_Timerout();     //启动睡眠超时定时器
							  communication_states = APP_IDLE;
            }
            break;
        }

        case CAD_DONE:
        {
					  OFF_Timerout();  //关闭CAD执行超时
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
                LEDOFF(LED2_GRREN);    //熄灭绿灯
                SX126X_WOR_Execute(0); //sleep
							  MX_TIM3_Init_Ms(4000);//睡眠定时周期
                ON_Sleep_Timerout();     //启动睡眠超时定时器
                communication_states = APP_IDLE;
            }
            break;
        }


        case CAD_DETE:
        {
            SX126X_WOR_Exit();
					  MX_TIM4_Init_Ms(1000); //误唤醒时，这个周期会影响整体功耗，时间越短，可以快速恢复，时间长会停留在接收模式时间过长
						ON_Timerout();  //开启接收超时
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
                break;
            }
            default :
                break;
            }
            OLED_ShowNum(18, 24,t, 2, 12);
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
                    OLED_ShowString(0, 12, "ERoR", 24);
                    OLED_Refresh();
                }
            }
            R_Cnt = 0;
            OK_Cnt = 0;
            OLED_ShowNum(24, 48, R_Cnt, 3, 12);
            OLED_ShowNum(78, 48, OK_Cnt, 3, 12);
            OLED_Refresh();
            SX126X_WORInit();
            SX126X_WOR_Execute(0);
						MX_TIM3_Init_Ms(4000);//睡眠定时周期
						ON_Sleep_Timerout();     //启动睡眠超时定时器
            communication_states = APP_IDLE;
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
			if(TX_ING_Flag==SET)
			{
				OFF_Timerout();
				TX_ING_Flag=RESET;
				communication_states = TX_ING;
			}
			else
			{
        OFF_Timerout();
			  LEDOFF(LED_ALL);
			  SX126xSetStandby(STDBY_RC);
				G_LoRaConfig.LoRa_Freq = Fre_Wake[0];      //切换成唤醒包信道
				SX126xSetRfFrequency(G_LoRaConfig.LoRa_Freq);
			
        SX126X_WORInit();
        SX126X_WOR_Execute(0);
				MX_TIM3_Init_Ms(4000);//睡眠定时周期
        ON_Sleep_Timerout();     //启动睡眠超时定时器
			}
    }
}
