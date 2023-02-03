/**
  ******************************************************************************
  * File Name          : USART.h
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __usart_H
#define __usart_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN Private defines */
#define RECEIVELEN 1024 
#define USART_DMA_SENDING 1		//发送未完成  
#define USART_DMA_SENDOVER 0	//发送完成  

typedef struct  
{  
uint8_t receive_flag;			//空闲接收标记  
uint8_t dmaSend_flag;			//发送完成标记  
uint16_t rx_len;				//接收长度  
uint8_t RX_Buf[RECEIVELEN];//DMA接收缓存  
}USART_RECEIVETYPE; 


/* USER CODE END Private defines */

extern void Error_Handler(void);

void MX_LPUART1_UART_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);

/* USER CODE BEGIN Prototypes */

void Usart2SendData(uint8_t *pdata, uint16_t Length); 
void UART2_SendByte(int8_t data);
void USART2_SendString(char *str);
void Clear_UART2_IT(void);
void Usart2Receive_IDLE(void);
void IntToStr(int8_t *i, char *c, int len);
//void Usart1Receive_IDLE(void);
//void Usart1SendData(uint8_t *pdata, uint16_t Length);
//void UART1_SendByte(uint8_t data);
//void USART1_SendString(char *str);
//void Clear_UART1_IT(void);

//void LPUsart1SendData(uint8_t *pdata, uint16_t Length);
//void LPUART1_SendByte(uint8_t data);
//void LPUART1_SendString(char *str);
//void LPUsart1Receive_IDLE(void);
//void Clear_LPUART1_IT(void);
//void PrepareLPUARTToStopMode(void);


void DEBUG_Printf(char *fmt, ...);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
