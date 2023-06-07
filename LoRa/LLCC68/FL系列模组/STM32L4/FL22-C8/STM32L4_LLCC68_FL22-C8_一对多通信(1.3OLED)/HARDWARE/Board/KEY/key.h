#ifndef __KEY_H
#define __KEY_H	 

#define KEY1  HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15)//????0
#define KEY2  HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_8)//????1
#define KEY3  HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_9)//????2 

#define KEY1_PRES 1	//KEY1??
#define KEY2_PRES	2	//KEY2??
#define KEY3_PRES	3	//KEY3??
void KEY_Init(void);
unsigned char KEY_Scan(unsigned char mode);
#endif
