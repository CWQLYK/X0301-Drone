//	作者：咸
//	电话:18878881386
//	邮箱:3078510877@qq.com
//	日期：2026.3.3
//	版本：1.0
#include "ALL_DEFINE.h"
volatile uint32_t SysTick_count; //系统时间计数
volatile uint8_t spl_flag; //系统时间计数
_st_Mpu MPU6050;   //MPU6050原始数据
_st_Mag AK8975;   
_st_AngE Angle;    //当前角度姿态值
_st_Remote Remote; //遥控通道值


volatile uint32_t ST_CpuID;
 
 
_st_ALL_flag ALL_flag; //系统标志位，包含解锁标志位等



 _st_FlightData FlightData;
 //飞控命令
st_Command Command;

PidObject pidRateX; //内环PID数据
PidObject pidRateY;
PidObject pidRateZ;

PidObject pidPitch; //外环PID数据
PidObject pidRoll;
PidObject pidYaw;

PidObject pidHeightRate;
PidObject pidHeightHigh;

PidObject Flow_PosPid_x;    //外环光流
PidObject Flow_PosPid_y;

PidObject Flow_SpeedPid_x;  //内环光流
PidObject Flow_SpeedPid_y;

_st_IMU IMU;

//获取CPU的ID
void GetLockCode(void)
{
	ST_CpuID = *(vu32*)(0x1ffff7e8);//低字节芯片ID用来做通讯对频通道
}

void ALL_Init(void)                         //系统初始化
{
    I2C_SoftWare_Init();

    USART1_Init();

    LED_Init();

	MPU6050_Init();              //MPU6050初始化
}


