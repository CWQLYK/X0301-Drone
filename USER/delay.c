//	作者：咸
//	电话:18878881386
//	邮箱:3078510877@qq.com
//	日期：2026.3.3
//	版本：1.0
#include "ALL_DEFINE.h"
static volatile uint32_t usTicks = 0;   //1us需要的计数量
uint8_t sys_init_ok = 1;                //系统初始化完成标志位（1 = 完成， 0 = 未完成）
void cycleCounterInit(void)             //SysTick 基础参数初始化
{
    RCC_ClocksTypeDef ClockStructure;
    RCC_GetClocksFreq(&ClockStructure);
    usTicks = ClockStructure.SYSCLK_Frequency / 1000000;
}

void SysTick_IRQ(void)                  //1ms中断
{
    static uint8_t cnt;
    SysTick_count ++;
    if(!sys_init_ok) return ;
    cnt ++, cnt %= 2;
    if(cnt) Loop_Check();               //每2ms执行一次四轴核心任务

}

uint32_t GetSysTime_us(void)            //获取系统运行时间（单位：us）
{
    register uint32_t ms, cycle_count;
    do{
        ms = SysTick_count;
        cycle_count = SysTick->VAL;
    }while(ms != SysTick_count);

    // 计算总微秒数：已运行的ms*1000 + 剩余计数换算成us
    return ms * 1000 + (usTicks * 1000 - cycle_count) / usTicks;
}

void delay_ms(uint16_t ms)              //延时函数（单位：ms）
{   
    uint32_t Now_Time = GetSysTime_us();
    while(GetSysTime_us() - Now_Time < ms * 1000);
}

void delay_us(uint16_t us)              //延时函数（单位：us）
{
    uint32_t Now_Time = GetSysTime_us();
    while(GetSysTime_us() - Now_Time < us);
}

float micros(void)                      //返回系统当前时间(单位：ms)
{
    return SysTick_count + (SysTick->LOAD - SysTick->VAL)/(72000.0f/8.0f);
}

