//	作者：咸
//	电话:18878881386
//	邮箱:3078510877@qq.com
//	日期：2026.3.4
//	版本：1.0

//灯泡模块
#include "ALL_DEFINE.h"
void LED_Init(void)                     //初始化LED
{
	GPIO_InitTypeDef GPIO_InitStructure;
	AFIO->MAPR = 0X02000000; //使能4线烧写 释放某些与烧写相关的引脚
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO , ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
  	GPIO_InitStructure.GPIO_Pin = lfLED_io | rfLED_io|lbLED_io | rbLED_io;		     //LED12
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(LED_GPIOB, &GPIO_InitStructure); 	 

    GPIO_SetBits(LED_GPIOB, lfLED_io | rfLED_io|lbLED_io | rbLED_io);                //默认LED为熄灭状态
}

void lfLED_ON(void)
{
	GPIO_ResetBits(GPIOB,lfLED_io);
}

void lfLED_OFF(void)
{
	GPIO_SetBits(GPIOB,lfLED_io);
}

void lbLED_ON(void)
{
    GPIO_ResetBits(GPIOB,lbLED_io);
}

void lbLED_OFF(void)
{
	GPIO_SetBits(GPIOB,lbLED_io);
}



