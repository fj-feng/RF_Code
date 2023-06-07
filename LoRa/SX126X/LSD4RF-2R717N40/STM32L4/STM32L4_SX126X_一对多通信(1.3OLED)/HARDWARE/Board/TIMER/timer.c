#include "timer.h"
#include <stdint.h>
#include "stm32l4xx_hal.h"

TIM_HandleTypeDef TIM3_InitStruct;
TIM_HandleTypeDef TIM4_InitStruct;
/* TIM3 init function */

#include "Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l4xx.h" // Device header
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;
    /**Initializes the CPU, AHB and APB busses clocks
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = 16;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 8;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        //Error_Handler();
    }

    /**Initializes the CPU, AHB and APB busses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
    {
        //Error_Handler();
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_USART2;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        //Error_Handler();
    }

    /**Configure the main internal regulator output voltage
    */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    {
        //Error_Handler();
    }

    /**Configure the Systick interrupt time
    */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

    /**Configure the Systick
    */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

void MX_TIM3_Init_Ms(uint16_t time)
{
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;

    TIM3_InitStruct.Instance = TIM3;
    TIM3_InitStruct.Init.Prescaler = 64000 - 1;
    TIM3_InitStruct.Init.CounterMode = TIM_COUNTERMODE_UP;
    TIM3_InitStruct.Init.Period = time - 1;
    TIM3_InitStruct.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_Base_Init(&TIM3_InitStruct) != HAL_OK)
    {
        return;
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&TIM3_InitStruct, &sClockSourceConfig) != HAL_OK)
    {
        return;
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&TIM3_InitStruct, &sMasterConfig) != HAL_OK)
    {
        return;
    }

}

/* TIM4 init function */
void MX_TIM4_Init_Ms(uint16_t time)
{
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;

    TIM4_InitStruct.Instance = TIM4;
    TIM4_InitStruct.Init.Prescaler = 64000 - 1; ////设置用来作为TIMx时钟频率除数的预分频值(这里64分频，则：64MHz/64)
    TIM4_InitStruct.Init.CounterMode = TIM_COUNTERMODE_UP;//TIM向上计数模式
    TIM4_InitStruct.Init.Period = time - 1;        ////设置在下一个更新事件装入活动的自动重装载寄存器周期的值,定时64MHz/64 *1000=1mS
    TIM4_InitStruct.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;//设置时钟分割:TDTS = Tck_tim
    if (HAL_TIM_Base_Init(&TIM4_InitStruct) != HAL_OK)
    {
        return;
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&TIM4_InitStruct, &sClockSourceConfig) != HAL_OK)
    {
        return;
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&TIM4_InitStruct, &sMasterConfig) != HAL_OK)
    {
        return;
    }
}


void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{
    if(tim_baseHandle->Instance == TIM3)
    {
        __HAL_RCC_TIM3_CLK_ENABLE();
        HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);//TM3中断，先优先级0，从优先级0
        HAL_NVIC_EnableIRQ(TIM3_IRQn);//IRQ通道被使能
    }
    else if(tim_baseHandle->Instance == TIM4)
    {
        __HAL_RCC_TIM4_CLK_ENABLE();
        HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);//TM3中断，先优先级0，从优先级1
        HAL_NVIC_EnableIRQ(TIM4_IRQn);//IRQ通道被使能
    }
}


void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{
    if(tim_baseHandle->Instance == TIM3)
    {
        __HAL_RCC_TIM3_CLK_DISABLE();
        HAL_NVIC_DisableIRQ(TIM3_IRQn);

    }
    else if(tim_baseHandle->Instance == TIM4)
    {
        /* Peripheral clock disable */
        __HAL_RCC_TIM4_CLK_DISABLE();

        /* Peripheral interrupt Deinit*/
        HAL_NVIC_DisableIRQ(TIM4_IRQn);

    }
}


void ON_Sleep_Timerout(void)
{
    HAL_TIM_Base_Start_IT(&TIM3_InitStruct);
}


void OFF_Sleep_Timerout(void)
{
    HAL_TIM_Base_Stop_IT(&TIM3_InitStruct);
}


void ON_Timerout(void)
{
    HAL_TIM_Base_Start_IT(&TIM4_InitStruct);
}


void OFF_Timerout(void)
{
    HAL_TIM_Base_Stop_IT(&TIM4_InitStruct);
}

