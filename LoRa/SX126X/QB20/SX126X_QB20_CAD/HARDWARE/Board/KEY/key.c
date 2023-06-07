#include "stm32l4xx_hal.h"
#include "key.h"

void KEY_Init(void)
{

    GPIO_InitTypeDef GPIO_InitStruct;
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /*Configure GPIO pin : PB15 */
    /*Configure GPIO pin : PB15 */
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : PD8 PD9 */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

unsigned char KEY_Scan( unsigned char mode)
{
    static unsigned char key_up = 1; //???????
    if(mode)
        key_up = 1; //????
    if(key_up && (KEY1 == 0 || KEY2 == 0 || KEY3 == 0))
    {
        HAL_Delay(10);
        key_up = 0;
        if(KEY1 == 0) return KEY1_PRES;
        else if(KEY2 == 0)return KEY2_PRES;
        else if(KEY3 == 0)return KEY3_PRES;
    } else if(KEY1 == 1 && KEY2 == 1 && KEY3 == 1)key_up = 1;

    return 0;	// ?????
}
