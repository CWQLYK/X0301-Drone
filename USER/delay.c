// 作者：咸
// 电话:18878881386
// 邮箱:3078510877@qq.com
// 日期:2026.3.3
// 版本:1.0
//  STM32系统时钟与延时函数模块
#include "ALL_DEFINE.h"
#include "stm32f10x.h"
#include "misc.h"
#include "delay.h"
#include "ALL_DATA.h"
#include "scheduler.h"

static volatile uint32_t usTicks = 0;          // 每个微秒的系统时钟周期数

volatile uint32_t SysTick_count22;            // 系统运行时间计数器（ms级）
u8 sys_init_ok = 1;                           // 系统初始化完成标志（1=完成）

/**
 * @brief  初始化系统时钟周期计数器
 * @param  无
 * @retval 无
 * @note   获取系统时钟频率，计算每个微秒对应的时钟周期数
 *         用于后续精确的us级时间计算
 */
void cycleCounterInit(void)
{
    RCC_ClocksTypeDef clocks;                 // 时钟结构体
    RCC_GetClocksFreq(&clocks);               // 获取系统各时钟频率
    usTicks = clocks.SYSCLK_Frequency / 1000000; // 计算1us对应的时钟周期数
}

/**
 * @brief  SysTick定时器中断服务函数
 * @param  无
 * @retval 无
 * @note   1. SysTick中断周期为1ms，用于系统时间计数和任务调度
 *         2. 每2ms调用一次Loop_Check()函数（任务巡检）
 *         3. 系统未初始化完成时不执行任何操作
 */
void SysTick_IRQ(void)
{
    static u8 cnt;                            // 循环计数（用于2ms周期控制）
	
    SysTick_count++;                          // 系统1ms计数器递增
    if(!sys_init_ok) return;                  // 系统未初始化完成则返回
	
    //LED_1ms_DRV();                         // LED 1ms驱动（注释未启用）
	
    // cnt++;
    // cnt %= 2;                                 // 每2ms执行一次
    // if(cnt) Loop_Check();                     // 调用任务巡检函数
}  

/**
 * @brief  获取系统当前运行时间（微秒级）
 * @param  无
 * @retval uint32_t: 当前系统时间（us）
 * @note   1. 结合SysTick计数和当前计数值，实现高精度us级时间获取
 *         2. 双重校验防止计数溢出导致的误差
 */
uint32_t GetSysTime_us(void) 
{
    register uint32_t ms, cycle_cnt;
    // 双重读取确保ms值和cycle_cnt值匹配（防止中断导致的数值不一致）
    do {
        ms = SysTick_count;                   // 获取ms级计数
        cycle_cnt = SysTick->VAL;             // 获取当前SysTick计数值
    } while (ms != SysTick_count);            // 校验一致性
    // 计算总us数：ms*1000 + 剩余未到1ms的us数
    return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

/**
 * @brief  毫秒级延时函数（高精度）
 * @param  nms: 延时毫秒数
 * @retval 无
 * @note   基于系统us级时间戳实现，延时精度高，不占用CPU
 */
void delay_ms(uint16_t nms)
{
    uint32_t t0 = GetSysTime_us();            // 记录起始时间（us）
    // 循环等待直到达到指定延时时间
    while(GetSysTime_us() - t0 < nms * 1000);	
    SysTick_count22++;                        // 延时计数器递增（用于统计）
}

/**
 * @brief  微秒级延时函数（简易版）
 * @param  i: 延时微秒数
 * @retval 无
 * @note   基于CPU空循环实现，精度受编译器优化和CPU频率影响
 *         适用于对精度要求不高的短延时场景
 */
void delay_us(unsigned int i)
 {  
    char x = 0;   
    while( i-- ) {	
        for(x=1; x>0; x--);                  // 空循环延时
    }
 }	

/**************************************************************
 * @brief  获取系统当前运行时间（毫秒级）
 * @param  无
 * @return float: 当前系统时间（单位：ms）
 * @note   1. 结合SysTick_count22和SysTick计数值计算精确ms数
 *         2. 72000.0f/8.0f = 9000：对应72MHz系统时钟下1ms的计数值
 *         3. SysTick->LOAD为重装值，SysTick->VAL为当前计数值
 ***************************************************************/	
float micros(void)
{
    // SysTick_count22 = 系统运行时间ms计数器
    // SysTick->VAL = 系统定时器当前counter值（重装值168000.0f/8.0f为1ms）
    // 72MHZ/1000 = 72000每个ms需要的时钟周期，除以8为分频系数
    // 系统运行时间计算：整数ms + 小数ms
    return SysTick_count22 + (SysTick->LOAD - SysTick->VAL)/(72000.0f/8.0f);
}


