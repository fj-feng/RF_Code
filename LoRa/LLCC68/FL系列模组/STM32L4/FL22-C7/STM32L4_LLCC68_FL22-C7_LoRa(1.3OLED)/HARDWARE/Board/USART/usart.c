/**
******************************************************************************
* File Name          : USART.c
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

/* Includes ------------------------------------------------------------------*/
#include "usart.h"
//#include "gpio.h"
#include <stdarg.h>
#include "utility.h"

USART_RECEIVETYPE Usart2_RX; 
USART_RECEIVETYPE Usart1_RX;
USART_RECEIVETYPE LPUsart1_RX;




UART_HandleTypeDef huart2;




/* USART2 init function */

void MX_USART2_UART_Init(void)
{
    
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        return;
    }
    
}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{
    
    GPIO_InitTypeDef GPIO_InitStruct;
    if(uartHandle->Instance==USART2)
    {
        /* USER CODE BEGIN USART2_MspInit 0 */
        
        /* USER CODE END USART2_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_USART2_CLK_ENABLE();
        
        /**USART2 GPIO Configuration    
        PA2     ------> USART2_TX
        PA3     ------> USART2_RX 
        */
        GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        
        /* Peripheral interrupt init */
        HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART2_IRQn);

    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{
    
    if(uartHandle->Instance==LPUART1)
    {
        /* USER CODE BEGIN LPUART1_MspDeInit 0 */
        
        /* USER CODE END LPUART1_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_LPUART1_CLK_DISABLE();
        
        /**LPUART1 GPIO Configuration    
        PC0     ------> LPUART1_RX
        PC1     ------> LPUART1_TX 
        */
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_0|GPIO_PIN_1);
        
        /* Peripheral DMA DeInit*/
        HAL_DMA_DeInit(uartHandle->hdmarx);
        
        /* Peripheral interrupt Deinit*/
        HAL_NVIC_DisableIRQ(LPUART1_IRQn);
        
        /* USER CODE BEGIN LPUART1_MspDeInit 1 */
        
        /* USER CODE END LPUART1_MspDeInit 1 */
    }
    else if(uartHandle->Instance==USART1)
    {
        /* USER CODE BEGIN USART1_MspDeInit 0 */
        
        /* USER CODE END USART1_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_USART1_CLK_DISABLE();
        
        /**USART1 GPIO Configuration    
        PB6     ------> USART1_TX
        PB7     ------> USART1_RX 
        */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);
        
        /* Peripheral DMA DeInit*/
        HAL_DMA_DeInit(uartHandle->hdmarx);
        
        /* Peripheral interrupt Deinit*/
        HAL_NVIC_DisableIRQ(USART1_IRQn);
        
        /* USER CODE BEGIN USART1_MspDeInit 1 */
        
        /* USER CODE END USART1_MspDeInit 1 */
    }
    else if(uartHandle->Instance==USART2)
    {
        /* USER CODE BEGIN USART2_MspDeInit 0 */
        
        /* USER CODE END USART2_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_USART2_CLK_DISABLE();
        
        /**USART2 GPIO Configuration    
        PA2     ------> USART2_TX
        PA3     ------> USART2_RX 
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);
        
        /* Peripheral DMA DeInit*/
        HAL_DMA_DeInit(uartHandle->hdmarx);
        
        /* Peripheral interrupt Deinit*/
        HAL_NVIC_DisableIRQ(USART2_IRQn);
        
        /* USER CODE BEGIN USART2_MspDeInit 1 */
        
        /* USER CODE END USART2_MspDeInit 1 */
    }
} 

/* USER CODE BEGIN 1 */

//--------------------UART2---------------------------------

void Usart2SendData(uint8_t *pdata, uint16_t Length)  
{  
    uint32_t i = 0;
    
    for (i = 0; i < Length; i++)
    {
        UART2_SendByte(pdata[i]);
    }
}  

void UART2_SendByte(int8_t data)
{
    while((USART2->ISR&UART_FLAG_TXE)!=UART_FLAG_TXE);//等待发送寄存器为空
    USART2->TDR = data;
}

void USART2_SendString(char *str)
{
    while((*str)!=0)
    {
        while((USART2->ISR&UART_FLAG_TXE)!=UART_FLAG_TXE);//等待发送寄存器为空
        USART2->TDR = *str++;       
    }
}

void Clear_UART2_IT(void)
{
    __HAL_UART_CLEAR_FLAG(&huart2,UART_FLAG_IDLE);               //清除空闲中断标志
    HAL_Delay(2);
    __HAL_UART_CLEAR_FLAG(&huart2,UART_FLAG_TC);                 //清除发送标志
    HAL_UART_Receive_DMA(&huart2, Usart2_RX.RX_Buf, RECEIVELEN); //开启DMA接收 
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);                 //使能空闲中断
    __HAL_UART_CLEAR_FLAG(&huart2,UART_FLAG_IDLE);               //清除空闲中断标志	
}

//串口接收空闲中断  
void Usart2Receive_IDLE(void)  
{  
    uint32_t temp;  
    
    if((__HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE) != RESET))  
    {   
        __HAL_UART_CLEAR_IDLEFLAG(&huart2);  
        HAL_UART_DMAStop(&huart2);  
        temp = huart2.hdmarx->Instance->CNDTR;  
        Usart2_RX.rx_len =  RECEIVELEN - temp;   
        Usart2_RX.receive_flag=1;  
        HAL_UART_Receive_DMA(&huart2,Usart2_RX.RX_Buf,RECEIVELEN);  
    }  
}


//-----------------------------------------------------------------------------------------

void DEBUG_Printf(char *fmt, ...)
{		
    char buf[128];
    va_list va_args;
    
    // Start the varargs processing.
    va_start(va_args, fmt);
    
    vsnprintf((char *)buf, sizeof(buf), fmt, va_args);
    
    // End the varargs processing.
    va_end(va_args);
    
    /*
    * 真正的打印输出函数，不同平台修改
    */
    //    Serial_PutString(buf);
    USART2_SendString(buf);
}

void IntToStr(int8_t *i, char *c, int len)
{//i为整形数，C为字符数组，len为长度
    int k;
    char tmp[10];
    for(k=0;k<len;k++)
    {
        itoa(i[k],tmp,10);
        strcat(c,tmp);
    }
}


/* USER CODE END 1 */

/**
* @}
*/

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
