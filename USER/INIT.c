// 作者：咸
// 电话:18878881386
// 邮箱:3078510877@qq.com
// 日期:2026.3.3
// 版本:1.0
//  无人机系统初始化与PID参数配置模块
#include "ALL_DEFINE.h"

// 全局变量定义
volatile uint32_t SysTick_count;    // 系统时间计数器（ms级）
volatile uint8_t spl_flag;          // 系统状态标志位
_st_Mpu MPU6050;                   // MPU6050原始数据结构体（加速度计+陀螺仪）
_st_Mag AK8975;                    // AK8975磁力计数据结构体
_st_AngE Angle;                    // 当前角度测量值结构体（Pitch/Roll/Yaw）
_st_Remote Remote;                 // 遥控器通信数据结构体

volatile uint32_t ST_CpuID;         // STM32芯片ID（用于蓝牙配对）
 
_st_ALL_flag ALL_flag;              // 系统标志位结构体，存储系统状态位

_st_FlightData FlightData;          // 飞行数据结构体
st_Command Command;                 // 飞行指令结构体（包含飞行模式等）

// PID控制器定义 - 内环（角速度）控制
PidObject pidRateX;                // X轴角速度PID（Roll角速度）
PidObject pidRateY;                // Y轴角速度PID（Pitch角速度）
PidObject pidRateZ;                // Z轴角速度PID（Yaw角速度）

// PID控制器定义 - 外环（角度）控制
PidObject pidPitch;                // 俯仰角PID（Pitch）
PidObject pidRoll;                 // 横滚角PID（Roll）
PidObject pidYaw;                  // 偏航角PID（Yaw）

// 高度控制PID
PidObject pidHeightRate;           // 高度速率PID（内环）
PidObject pidHeightHigh;           // 高度位置PID（外环）

// 光流位置控制PID
PidObject Flow_PosPid_x;           // X轴位置PID（外环）
PidObject Flow_PosPid_y;           // Y轴位置PID（外环）

PidObject Flow_SpeedPid_x;         // X轴速度PID（内环）
PidObject Flow_SpeedPid_y;         // Y轴速度PID（内环）

_st_IMU IMU;                       // IMU数据融合结构体

/**
 * @brief  PID参数初始化函数
 * @note   PID参数基于经验值设定，用户可根据实际无人机特性调整
 *         调整时注意保持参数格式（浮点型），避免语法错误
 */
void pid_param_Init(void); 

/**
 * @brief  获取STM32芯片ID
 * @param  无
 * @retval 无
 * @note   读取芯片唯一ID（地址0x1ffff7e8），用于蓝牙配对等唯一标识场景
 */
void GetLockCode(void)
{
    ST_CpuID = *(vu32*)(0x1ffff7e8); // 读取芯片ID，用于蓝牙配对
}

///////////////系统初始化函数//////////////////////////////////
/**
 * @brief  无人机系统总初始化函数
 * @param  无
 * @retval 无
 * @note   按硬件依赖顺序初始化所有外设，确保系统正常启动
 */
void ALL_Init(void)
{
    //float STBy;

    I2C_SoftWare_Init();             // 软件I2C初始化（用于MPU6050/AK8975通信）
		
    pid_param_Init();                // PID参数初始化（控制算法核心参数）
	  
    LEDInit();                       // LED指示灯初始化（状态指示）
	
    ANO_Uart1_Init(19200);           // 匿名串口1初始化（波特率19200，调参/数据上传）
	
    MPU6050_Init();                  // MPU6050传感器初始化（姿态检测核心）
	
    // ADC初始化（电池电压检测/高度气压检测）
    ADC1_Init();
	
    // printf("ANO_Uart1_Init  \r\n")
    if (FLY_TYPE == 2) 
    {
        UART2_Init(115200);          // 串口2初始化（波特率115200，特定飞行模式使用）
    }      //
	
    USART3_Config(500000);           // 调试串口初始化（波特率500000，高速数据传输）
    // printf("USART3_Config  \r\n");

    NRF24L01_init();                 // 2.4G遥控器初始化（无线控制）
	
    spl_flag = 0;                    // 系统标志位清零
    SPL_Err = 1;                     // 错误标志位初始化
	
    TIM2_PWM_Config();               // TIM2 PWM初始化（电机驱动1-2路）		
    TIM3_PWM_Config();               // TIM3 PWM初始化（电机驱动3-4路）		
}

/**
 * @brief  PID控制器参数初始化
 * @param  无
 * @retval 无
 * @note   1. 内环控制角速度，外环控制角度，遵循串级PID控制逻辑
 *         2. 高度/光流PID默认值为0，需根据实际需求配置
 *         3. 参数为经验值，不同无人机需微调（尤其是kp/kd）
 */
void pid_param_Init(void)
{
    ////////////////// 内环角速度PID参数 ///////////////////////	
    // 角速度环参数（响应快，kd用于抑制超调）
    pidRateX.kp = 1.7f;    // X轴角速度比例系数
    pidRateY.kp = 1.7f;    // Y轴角速度比例系数
    pidRateZ.kp = 3.0f;    // Z轴角速度比例系数（偏航响应稍快）
	
    pidRateX.ki = 0.0f;    // X轴角速度积分系数（暂未启用）
    pidRateY.ki = 0.0f;    // Y轴角速度积分系数（暂未启用）
    pidRateZ.ki = 0.0f;    // Z轴角速度积分系数（暂未启用）	
	
    pidRateX.kd = 0.08f;   // X轴角速度微分系数（抑制抖动）
    pidRateY.kd = 0.08f;   // Y轴角速度微分系数（抑制抖动）
    pidRateZ.kd = 0.05f;   // Z轴角速度微分系数（偏航抖动较小）	
	
    ///////////// 外环角度PID参数 /////////////////////////////	
    // 角度环参数（稳态精度，ki用于消除静差）
    pidPitch.kp = 7.0f;    // 俯仰角比例系数
    pidRoll.kp = 7.0f;     // 横滚角比例系数
    pidYaw.kp = 7.0f;      // 偏航角比例系数	
	
    pidPitch.ki = 0.0f;    // 俯仰角积分系数（暂未启用）
    pidRoll.ki = 0.0f;     // 横滚角积分系数（暂未启用）
    pidYaw.ki = 0.0f;      // 偏航角积分系数（暂未启用）	
	
    pidPitch.kd = 0.0f;    // 俯仰角微分系数（暂未启用）
    pidRoll.kd = 0.0f;     // 横滚角微分系数（暂未启用）
    pidYaw.kd = 0.0f;      // 偏航角微分系数（暂未启用）	
	
    // 高度控制PID（默认禁用，需根据气压计/超声波配置）
    // 内环PID控制 速度
    pidHeightRate.kp = 0.0f; // 高度速率比例系数
    pidHeightRate.ki = 0.0f; // 高度速率积分系数
    pidHeightRate.kd = 0.0f; // 高度速率微分系数
    // 外环PID控制 高度
    pidHeightHigh.kp = 0.0f; // 高度位置比例系数
    pidHeightHigh.ki = 0.0f; // 高度位置积分系数
    pidHeightHigh.kd = 0.0f; // 高度位置微分系数
	
    /////////////////////////////////////////////////////////////////////
    // 光流X轴控制PID（默认禁用，需配合光流模块使用）
    // X内环速度PID控制
    Flow_SpeedPid_x.kp = 0.0f; // 比例
    Flow_SpeedPid_x.ki = 0.0f; // 积分
    Flow_SpeedPid_x.kd = 0.0f; // 微分
	
    // X外环位置PID控制
    Flow_PosPid_x.kp = 0.0f;   // 比例
    Flow_PosPid_x.ki = 0.0f;   // 积分
    Flow_PosPid_x.kd = 0.0f;   // 微分
	
    //////////////////////////////////////////////////////////
    // 光流Y轴控制PID（默认禁用，需配合光流模块使用）
    // Y内环速度PID控制
    Flow_SpeedPid_y.kp = 0.0f; // 比例
    Flow_SpeedPid_y.ki = 0.0f; // 积分
    Flow_SpeedPid_y.kd = 0.0f; // 微分
	
    // Y外环位置PID控制
    Flow_PosPid_y.kp = 0.0f;   // 比例
    Flow_PosPid_y.ki = 0.0f;   // 积分
    Flow_PosPid_y.kd = 0.0f;   // 微分

    // 初始化飞行模式为正常模式
    Command.FlightMode = NORMOL;
}


