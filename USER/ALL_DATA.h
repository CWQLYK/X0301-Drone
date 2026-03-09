#ifndef _ALL_USER_DATA_H_
#define _ALL_USER_DATA_H_

/*****************************************************************************************
* 模块说明：飞控系统全局数据定义头文件
* 功能概述：定义飞控系统所需的基础数据类型、传感器数据结构、PID控制器结构、
*           飞行状态/命令结构等全局数据，为整个飞控系统提供统一的数据接口
*****************************************************************************************/

//==============================================================================
// 基础数据类型重定义（兼容不同编译器，保证数据宽度一致）
//==============================================================================
typedef   signed          char int8_t;    // 有符号8位整型（-128 ~ 127）
typedef   signed short     int int16_t;   // 有符号16位整型（-32768 ~ 32767）
typedef   signed           int int32_t;   // 有符号32位整型（-2^31 ~ 2^31-1）
typedef   signed       long long int64_t;// 有符号64位整型

/* exact-width unsigned integer types */
typedef unsigned          char uint8_t;  // 无符号8位整型（0 ~ 255）
typedef unsigned short     int uint16_t; // 无符号16位整型（0 ~ 65535）
typedef unsigned           int uint32_t; // 无符号32位整型（0 ~ 2^32-1）
typedef unsigned       long long uint64_t;// 无符号64位整型

#define NULL 0  // 空指针定义

//==============================================================================
// 全局系统变量声明（外部定义，此处仅声明）
//==============================================================================
extern volatile uint32_t SysTick_count;  // 系统滴答定时器计数（ms级，用于计时/延时）
extern volatile uint8_t spl_flag;        // 系统分割标志位（任务调度/状态切换）
extern volatile uint32_t ST_CpuID;       // CPU ID标识（用于多核/硬件识别）

//==============================================================================
// 滤波器相关数据结构（巴特沃斯滤波器）
//==============================================================================
/**
 * @brief 巴特沃斯滤波器缓存结构体
 * @note  存储滤波器的输入/输出历史值，用于二阶IIR滤波计算
 *        Input_Butter[0] = 前前次输入(x(n-2))
 *        Input_Butter[1] = 前次输入(x(n-1))
 *        Input_Butter[2] = 当前输入(x(n))
 *        Output_Butter[0] = 前前次输出(y(n-2))
 *        Output_Butter[1] = 前次输出(y(n-1))
 *        Output_Butter[2] = 当前输出(y(n))
 */
typedef struct
{
 float Input_Butter[3];   // 滤波器输入缓存（3个历史值）
 float Output_Butter[3];  // 滤波器输出缓存（3个历史值）
}Butter_BufferData;

/**
 * @brief 巴特沃斯滤波器参数结构体
 * @note  存储二阶巴特沃斯滤波器的系数，格式为：
 *        y(n) = b0*x(n) + b1*x(n-1) + b2*x(n-2) - a1*y(n-1) - a2*y(n-2)
 *        a[0]固定为1（归一化系数），a[1]=-a1, a[2]=-a2
 */
typedef struct
{
 const float a[3];  // 分母系数（反馈项）：a[0]=1, a[1], a[2]
 const float b[3];  // 分子系数（前馈项）：b[0], b[1], b[2]
}Butter_Parameter;

//==============================================================================
// 传感器原始数据结构
//==============================================================================
/**
 * @brief MPU6050（六轴IMU）原始数据结构体
 * @note  存储加速度计和陀螺仪的原始数字量（未校准、未滤波）
 *        单位：加速度（LSB/g），角速度（LSB/(°/s)）
 */
typedef struct{
 int16_t accX;   // X轴加速度原始值
 int16_t accY;   // Y轴加速度原始值
 int16_t accZ;   // Z轴加速度原始值
 int16_t gyroX;  // X轴角速度原始值
 int16_t gyroY;  // Y轴角速度原始值
 int16_t gyroZ;  // Z轴角速度原始值
}_st_Mpu;

/**
 * @brief AK8975（磁力计）原始数据结构体
 * @note  存储磁力计原始数字量（未校准、未滤波），单位：LSB/μT
 *        保留字段，需外接磁力计模块才能使用
 */
typedef struct{
 int16_t magX;   // X轴磁力原始值
 int16_t magY;   // Y轴磁力原始值
 int16_t magZ;   // Z轴磁力原始值
}_st_Mag;

//==============================================================================
// 高度相关数据结构
//==============================================================================
/**
 * @brief 高度基础数据结构体
 * @note  存储高度和高度变化率（升降速度）
 */
typedef struct{
 float rate;    // 高度变化率（cm/s 或 m/s，上升为正，下降为负）
 float height;  // 绝对高度（cm 或 m，相对起飞点）
}High;

//==============================================================================
// 姿态角数据结构
//==============================================================================
/**
 * @brief 欧拉角数据结构体
 * @note  存储飞行器的姿态角（Roll/Pitch/Yaw）
 *        单位：弧度(rad) 或 角度(°)
 */
typedef struct{
 float roll;   // 横滚角（绕X轴旋转，左滚为正，右滚为负）
 float pitch;  // 俯仰角（绕Y轴旋转，抬头为正，低头为负）
 float yaw;    // 偏航角（绕Z轴旋转，顺时针为正，逆时针为负）
}_st_AngE;

/**
 * @brief 飞行数据结构体（高度相关）
 * @note  整合不同传感器的高度数据，用于高度控制
 */
typedef struct
{	
 // struct{  // 角度数据（注释保留，可扩展）
 //  float roll;
 //  float pitch;
 //  float yaw;
 // }Angle;
 
 struct{  // 高度数据子结构体
  float rate;             // 高度变化率（cm/s）
  float bara_height;      // 气压计高度（cm，原始值）
  float ultra_height;     // 超声波高度（cm，近距离高精度）
  float ultra_baro_height;// 融合后的高度（cm，超声+气压计）
 }High;		 
}_st_FlightData;

//==============================================================================
// 遥控器数据结构
//==============================================================================
/**
 * @brief 遥控器通道数据结构体
 * @note  存储遥控器各通道的PWM值（范围通常为1000~2000us）
 *        对应飞行器的姿态、油门、辅助功能控制
 */
typedef struct
{
 uint16_t roll;   // 横滚通道（左/右打杆）
 uint16_t pitch;  // 俯仰通道（前/后打杆）
 uint16_t thr;    // 油门通道（升/降，1000=最低，2000=最高）
 uint16_t yaw;    // 偏航通道（左/右旋转）
 uint16_t AUX1;   // 辅助通道1（自定义功能，如飞行模式切换）
 uint16_t AUX2;   // 辅助通道2（自定义功能）
 uint16_t AUX3;   // 辅助通道3（自定义功能）
 uint16_t AUX4;   // 辅助通道4（自定义功能）
 uint16_t AUX5;   // 辅助通道5（自定义功能）
 uint16_t AUX6;   // 辅助通道6（自定义功能）
 uint16_t AUX7;   // 辅助通道7（自定义功能）
}_st_Remote;

//==============================================================================
// PID控制器核心结构体
//==============================================================================
/**
 * @brief 增强型PID控制器结构体
 * @note  包含标准PID参数、积分限幅、输出限幅、微分滤波等功能，
 *        适用于飞控的角速度/角度/高度/位置等闭环控制
 */
typedef volatile struct
{
 float desired;          /// 期望目标值（设定值）
 float offset;           // 偏移量（用于校准/补偿）
 float prevError;        // 上次偏差（e(n-1)，用于计算微分）
 float integ;            // 误差积分累加值（∑e(i)，用于积分项）
 float kp;               // 比例系数（P参数，快速响应偏差）
 float ki;               // 积分系数（I参数，消除静态误差）
 float kd;               // 微分系数（D参数，抑制超调）
 float IntegLimitHigh;   // 积分上限（防止积分饱和）
 float IntegLimitLow;    // 积分下限
 float measured;         // PID反馈量（实际测量值）
 float out;              // PID基础输出（P+I+D）
 float OutLimitHigh;     // 输出上限（保护执行器）
 float OutLimitLow;      // 输出下限
 float Control_OutPut;   // 控制器总输出（含限幅/滤波后）
 float Last_Control_OutPut; // 上次控制器总输出（用于输出滤波）
 float Control_OutPut_Limit; // 最终输出限幅值
 
 // 微分增强相关
 float Last_FeedBack;    // 上次反馈值（用于微分计算：d(measured)/dt）
 float Dis_Err;          // 微分量（d(e)/dt 或 d(measured)/dt）
 float Dis_Error_History[5]; // 微分历史值（用于微分滤波）
 float Err_LPF;          // 偏差低通滤波后的值
 float Last_Err_LPF;     // 上次偏差滤波值
 float Dis_Err_LPF;      // 滤波后的微分量

 // 功能标志位（注释保留，可扩展为位域）
 // int8_t Err_Limit_Flag :1;        // 偏差限幅标志
 // int8_t Integrate_Limit_Flag :1; // 积分限幅标志
 // int8_t Integrate_Separation_Flag :1; // 积分分离标志		
 
 Butter_BufferData Control_Device_LPF_Buffer; // 控制器输出低通滤波缓存
}PidObject;

//==============================================================================
// 全局状态标志位结构体
//==============================================================================
/**
 * @brief 飞控全局状态标志位结构体
 * @note  使用位域节省内存，存储飞行控制的核心状态标志
 */
typedef volatile struct
{
 uint8_t unlock;         // 解锁标志（0=锁定，1=解锁，解锁后才能控制电机）
 uint32_t  slock_flag;   // 软件锁定标志（扩展用）
 uint8_t height_lock:1;  // 高度锁定标志位（1=定高模式生效）
 uint8_t take_off:1;     // 起飞标志位（1=正在起飞）
 uint8_t take_down:1;    // 降落标志位（1=正在降落）
}_st_ALL_flag;

//==============================================================================
// 飞控命令结构体
//==============================================================================
/**
 * @brief 飞控控制命令结构体
 * @note  存储校准命令、飞行模式切换等核心控制指令
 */
typedef volatile struct
{
 uint8_t AccOffset :1;   // 加速度计校准命令（1=执行校准）
 uint8_t GyroOffset :1;  // 陀螺仪校准命令（1=执行校准）
 uint8_t MagOffset :1;   // 磁力计校准命令（1=执行校准）
 uint8_t six_acc_offset; // 加速度计六面校准标志（0=未校准，1=校准中，2=校准完成）
 
 // 飞行模式枚举（自定义）
 enum{ 								 
  LOCK = 0x00,				// 锁定模式（电机不转，无控制输出）
  NORMOL, 					// 手动模式（无自稳，纯遥控器控制）		
  HEIGHT,					// 定高模式（高度闭环，姿态手动）
  Flow_POSITION,  			// GPS定点模式（位置/高度闭环）
 }FlightMode; // 当前飞行模式
}st_Command;

//==============================================================================
// 全局变量声明（外部可访问）
//==============================================================================
extern _st_Remote Remote;                // 遥控器数据
extern _st_Mpu MPU6050;                  // MPU6050原始数据
extern _st_Mag AK8975;                   // AK8975磁力计数据（保留）
extern _st_AngE Angle;                   // 飞行器姿态角（欧拉角）

extern _st_ALL_flag ALL_flag;            // 全局状态标志

// PID控制器实例（角速度环）
extern PidObject pidRateX;               // X轴角速度PID（横滚角速度）
extern PidObject pidRateY;               // Y轴角速度PID（俯仰角速度）
extern PidObject pidRateZ;               // Z轴角速度PID（偏航角速度）

// PID控制器实例（角度环）
extern PidObject pidPitch;               // 俯仰角PID
extern PidObject pidRoll;                // 横滚角PID
extern PidObject pidYaw;                 // 偏航角PID

// PID控制器实例（高度环）
extern PidObject pidHeightRate;          // 高度速率PID（内环）
extern PidObject pidHeightHigh;          // 高度位置PID（外环）

// PID控制器实例（GPS/光流位置环）
extern PidObject Flow_PosPid_x;          // X轴位置PID（东西方向）
extern PidObject Flow_PosPid_y;          // Y轴位置PID（南北方向）

// PID控制器实例（GPS/光流速度环）
extern PidObject Flow_SpeedPid_x;        // X轴速度PID
extern PidObject Flow_SpeedPid_y;        // Y轴速度PID

extern _st_FlightData FlightData;        // 飞行数据（高度相关）
extern st_Command Command;               // 飞控控制命令

//==============================================================================
// 函数声明
//==============================================================================
void GetLockCode(void);    // 获取解锁码（电机解锁验证）
void pid_param_Init(void); // PID参数初始化（加载默认/校准参数）

#endif

