//	作者：咸
//	电话:18878881386
//	邮箱:3078510877@qq.com
//	日期：2026.3.3
//	版本：1.0
#include "ALL_DEFINE.h"
//独立看门狗超时时间公式：超时时间（秒）= (4 × 2^prer × rlr) / LSI频率
void IWDG_Init(uint8_t prer, uint16_t rlr)            //初始化独立看门狗
{
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

    IWDG_SetPrescaler(prer);                          //设置预分频值

    IWDG_SetReload(rlr);                              //设置重装值

    IWDG_ReloadCounter();                             //重载重装值（喂狗）
    
    IWDG_Enable();                                    //使能独立看门狗
}

void IWDG_Feed(void)                                  //喂狗
{
    IWDG_ReloadCounter();                             //重载重装值（喂狗）
}
