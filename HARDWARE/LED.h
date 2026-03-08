#ifndef __LED_H
#define __LED_H

#include "stm32f10x.h"
#include "LED.h"
#include "ALL_DATA.h"
#include "ALL_DEFINE.h"

/***************LED GPIO定义******************/
#define LED_GPIOB  GPIOB
#define rfLED_io    GPIO_Pin_1		//机身右后灯	
#define lfLED_io    GPIO_Pin_2		//机身左后灯	
#define lbLED_io    GPIO_Pin_8		//机身左前灯	
#define rbLED_io    GPIO_Pin_9		//机身右前灯	

void LED_Init(void);                     //初始化LED
void lfLED_ON(void);
void lfLED_OFF(void);
void lbLED_ON(void);
void lbLED_OFF(void);
void rfLED_ON(void);
void rfLED_OFF(void);
void rbLED_ON(void);
void rbLED_OFF(void);

#endif

