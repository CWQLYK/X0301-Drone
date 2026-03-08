#ifndef __USART1_H
#define __USART1_H

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include <stdio.h>
#include <stdarg.h>

/***************UART1 GPIO定义******************/
#define RCC_UART1		RCC_APB2Periph_GPIOA
#define GPIO_UART1		GPIOA
#define UART1_Pin_TX	GPIO_Pin_9
#define UART1_Pin_RX	GPIO_Pin_10


void USART1_Init(void);                          //串口1初始化
void USART1_SendByte(uint8_t Byte);              //串口发送一个字节
void USART1_SendString(char *String);            //串口发送字符串
void USART1_Printf(char *format, ...);           //打印数据


#endif

