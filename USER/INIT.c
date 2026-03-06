//	作者：咸
//	电话:18878881386
//	邮箱:3078510877@qq.com
//	日期：2026.3.3
//	版本：1.0
#include "ALL_DEFINE.h"
volatile uint32_t SysTick_count;            //系统时间计数

void ALL_Init(void)                         //系统初始化
{
    USART1_Init();
    LED_Init();
}


