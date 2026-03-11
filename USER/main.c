// 作者：咸
// 电话:18878881386
// 邮箱:3078510877@qq.com
// 日期:2026.3.3
// 版本:1.0
//  无人机系统主函数（程序入口）
#include "ALL_DEFINE.h"

/**
 * @brief  程序入口函数
 * @param  无
 * @retval int: 程序返回值（嵌入式系统中通常不返回）
 * @note   1. 初始化系统核心组件，配置中断和定时器
 *         2. 启用独立看门狗防止程序卡死
 *         3. 主循环执行核心控制逻辑，喂狗保证系统稳定
 */
int main(void)
{
    cycleCounterInit();                             // 初始化usTicks计数器（用于us级时间计算）
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); // 配置中断优先级分组4（抢占优先级0-15，无响应优先级）
    SysTick_Config(SystemCoreClock / 1000);         // 配置系统定时器中断，每1ms触发一次中断
    
    ALL_Init();                                     // 系统总初始化（外设/传感器/PID等）
    IWDG_Init(4, 625);                              // 初始化独立看门狗：分频系数4，重装值625，喂狗周期1s
                                                    // 注：IWDG时钟为40kHz/4=10kHz，625/10kHz=0.0625s？实际需根据硬件确认

    // 主循环（程序核心执行体）
    while(1)
    {
        Main_Loop();                                // 执行主控制循环（姿态解算/PID控制/遥控器处理等）

        IWDG_Feed();                                // 喂独立看门狗（必须1s内执行，否则系统复位）
    }
}


