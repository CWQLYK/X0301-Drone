//	作者：咸
//	电话:18878881386
//	邮箱:3078510877@qq.com
//	日期：2026.3.4
//	版本：1.0

//串口1控制程序
//串口1用于调试程序
#include "ALL_DEFINE.h"
#include <stdio.h>
#include <stdarg.h>
void USART1_Init(void)                          //串口1初始化
{
    //开启外设时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    //配置GPIO口
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    //配置USART1
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 19200;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART1, &USART_InitStructure);

    //使能USART1
    USART_Cmd(USART1, ENABLE);
    
}

void USART1_SendByte(uint8_t Byte)              //串口发送一个字节
{
    USART_SendData(USART1, Byte);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) != SET);    //等待字节发送完成
}

void USART1_SendString(char *String)            //串口发送字符串
{
    for(uint8_t i = 0; String[i] != '\0'; i ++)
    {
        USART1_SendByte(String[i]);
    }
}

void USART1_Printf(char *format, ...)           //打印数据
{
    char String[100];
    va_list arg;
    va_start(arg, format);
    vsnprintf(String, sizeof(String), format, arg);
    va_end(arg);
    USART1_SendString(String);
}

