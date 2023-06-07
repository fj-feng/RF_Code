#ifndef __tim_H
#define __tim_H
#include "stm32l4xx_hal.h"
#include <stdint.h>

void SystemClock_Config(void);
extern  TIM_HandleTypeDef TIM3_InitStruct;
extern  TIM_HandleTypeDef TIM4_InitStruct;
void MX_TIM3_Init_Ms(uint16_t time);
void MX_TIM4_Init_Ms(uint16_t time);
void ON_Sleep_Timerout(void);
void OFF_Sleep_Timerout(void);
void ON_Timerout(void);
void OFF_Timerout(void);

#endif

