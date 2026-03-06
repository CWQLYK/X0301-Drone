#ifndef __ALL_DEFINE_H
#define __ALL_DEFINE_H
#include "stm32f10x.h"
#include "USART1.h"
#include "LED.h"
#include "ALL_DATA.h"
#include "INIT.h"
#include "delay.h"
#include "IWDG.h"
#include "scheduler.h"
#include "MPU6050.h"
#include "I2C.h"
#include "STM32F10x_IWDG.h"

#undef SUCCESS
#define SUCCESS 0
#undef FAILED
#define FAILED  1


/***************UART1 GPIO定义******************/
#define RCC_UART1		RCC_APB2Periph_GPIOA
#define GPIO_UART1		GPIOA
#define UART1_Pin_TX	GPIO_Pin_9
#define UART1_Pin_RX	GPIO_Pin_10


/***************LED GPIO定义******************/
#define LED_GPIOB  GPIOB
#define rfLED_io    GPIO_Pin_1		//机身右后灯	
#define lfLED_io    GPIO_Pin_2		//机身左后灯	
#define lbLED_io    GPIO_Pin_8		//机身左前灯	
#define rbLED_io    GPIO_Pin_9		//机身右前灯	

#endif

