#ifndef __DELAY_H
#define __DELAY_H
#include "stm32f10x.h"

void cycleCounterInit(void);             //SysTick 基础参数初始化
void SysTick_IRQ(void);                  //1ms中断
uint32_t GetSysTime_us(void);            //获取系统运行时间（单位：us）
void delay_ms(uint16_t ms);              //延时函数（单位：ms）
void delay_us(uint16_t us);              //延时函数（单位：us）
float micros(void);                      //返回系统当前时间(单位：ms)

#endif
