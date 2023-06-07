#ifndef __LED_H
#define __LED_H	 

#define LED1_RED          GPIO_PIN_4
#define LED2_GRREN        GPIO_PIN_5
#define LED3_BLUE         GPIO_PIN_6
#define LED_ALL          £®GPIO_PIN_4|GPIO_PIN_5| GPIO_PIN_6£©

#define  LEDON(x)   HAL_GPIO_WritePin(GPIOA, x,GPIO_PIN_RESET);
#define  LEDOFF(x)   HAL_GPIO_WritePin(GPIOA, x,GPIO_PIN_SET);

void LED_Init(void);//≥ı ºªØ

		 				    
#endif
