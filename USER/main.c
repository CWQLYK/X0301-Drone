//	作者：咸
//	电话:18878881386
//	邮箱:3078510877@qq.com
//	日期：2026.3.3
//	版本：1.0
#include "ALL_DEFINE.h"

int main(void)
{
    cycleCounterInit();                             //初始化usTicks变量
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); //配置中断优先级分组
    SysTick_Config(SystemCoreClock / 1000);         //开启定时器中断，每1ms触发一次中断
    
    ALL_Init();                                     //系统初始化
    IWDG_Init(4, 625);                              //初始化看门狗,分频系数：4，重装值：625，溢出时间：1s

    lbLED_ON();
    while(1)
    {   
        IWDG_Feed();                                //喂狗
    }


}

