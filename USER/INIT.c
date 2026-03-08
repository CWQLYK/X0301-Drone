//	作者：咸
//	电话:18878881386
//	邮箱:3078510877@qq.com
//	日期：2026.3.3
//	版本：1.0
#include "ALL_DEFINE.h"
volatile uint32_t SysTick_count;            //系统时间计数
_st_Mpu MPU6050; //MPU6050原始数据
_st_Remote Remote; //遥控通道值
_st_ALL_flag ALL_flag; //系统标志位，包含解锁标志位等

void ALL_Init(void)                         //系统初始化
{
    I2C_SoftWare_Init();

    USART1_Init();

    LED_Init();

	MPU6050_Init();              //MPU6050初始化
}


