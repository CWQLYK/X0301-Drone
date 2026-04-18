// 作者：咸
// 电话:18878881386
// 邮箱:3078510877@qq.com
// 日期:2026.3.3
// 版本:1.0
//  无人机任务调度核心模块（基于时间片的多任务管理）

#include "ALL_DEFINE.h"
#include "scheduler.h"
#include "ANO_Data_Transfer.h"
#include "ADC.h"

// 全局调度变量定义
loop_t loop;                  // 任务调度计数器结构体
u32 time[10], time_sum;       // 各任务执行时间统计数组/总耗时
// 光流/定位模块信号质量计数与状态
u8 Flow_SSI_CNT, Locat_SSI_CNT, Locat_SSI, Flow_SSI, Locat_Mode;

/**
 * @brief  4ms周期任务（通信相关）
 * @param  无
 * @retval 无
 * @note   处理2.4G遥控器、数据交互、连接状态检测
 */
void Duty_4ms()
{
    time[1] = GetSysTime_us();  // 记录任务开始时间（us）
    
    ANO_NRF_Check_Event();      // 检查2.4G NRF24L01数据接收事件
    ANO_DT_Data_Exchange();     // 匿名数据传输交互（调参/状态上传）
    Rc_Connect();               // 遥控器连接状态检测
    
    // printf("Duty_4ms\r\n");
    time[1] = GetSysTime_us() - time[1]; // 计算任务执行耗时
}

//////////////////////////////////////////////////////////
/**
 * @brief  6ms周期任务（姿态解算）
 * @param  无
 * @retval 无
 * @note   基于MPU6050原始数据计算无人机姿态角（Pitch/Roll/Yaw）
 */
void Duty_6ms()
{
    time[2] = GetSysTime_us();  // 记录任务开始时间（us）
    
    // 计算无人机姿态角（周期0.006s）
    GetAngle(&MPU6050, &Angle, 0.006f);
    
    // printf("Duty_6ms\r\n");
    time[2] = GetSysTime_us() - time[2]; // 计算任务执行耗时
}

/////////////////////////////////////////////////////////
/**
 * @brief  10ms周期任务（遥控器解析）
 * @param  无
 * @retval 无
 * @note   解析遥控器通道数据（油门/横滚/俯仰/偏航/模式）
 */
void Duty_10ms()
{
    time[3] = GetSysTime_us();  // 记录任务开始时间（us）
    
    RC_Analy();                 // 遥控器数据解析
    
    // printf("Duty_10ms\r\n");
    time[3] = GetSysTime_us() - time[3]; // 计算任务执行耗时
}

/////////////////////////////////////////////////////////
/**
 * @brief  20ms周期任务（串口通信）
 * @param  无
 * @retval 无
 * @note   串口3轮询处理，负责数据发送和接收（WiFi/蓝牙通信）
 */
void Duty_20ms()
{
    time[4] = GetSysTime_us();  // 记录任务开始时间（us）

    ANTO_polling();             // 串口3轮询，数据发送和接收
    
    // printf("Duty_20ms\r\n");
    time[4] = GetSysTime_us() - time[4]; // 计算任务执行耗时
}

//////////////////////////////////////////////////////////
/**
 * @brief  50ms周期任务（状态监控）
 * @param  无
 * @retval 无
 * @note   LED状态指示、系统标志位检查、电池电压检测
 */
void Duty_50ms()
{
    time[5] = GetSysTime_us();  // 记录任务开始时间（us）
    
    PilotLED();                 // LED状态指示灯控制（飞行状态/错误指示）
    Flag_Check();               // 系统标志位检查（飞行模式/故障状态）
    Voltage_Check();            // 电池电压检测（低电压保护）
    
    // printf("Duty_50ms\r\n");
    time[5] = GetSysTime_us() - time[5]; // 计算任务执行耗时
}

//////////////////////////////////////////////////////////
/**
 * @brief  500ms周期任务（喂狗）
 * @param  无
 * @retval 无
 * @note   喂狗
 */
void Duty_500ms()
{
    IWDG_Feed(); 
}

/////////////////////////////////////////////////////////////
/**
 * @brief  1秒周期任务（状态统计与故障检测）
 * @param  无
 * @retval 无
 * @note   1. 统计各无线模块信号质量，检测模块故障
 *         2. 计算所有任务总执行耗时，用于性能分析
 *         3. 设置定位/光流模块错误标志位，触发保护机制
 */
void Duty_1000ms()
{
    u8 i;
    
    // 读取并清零NRF24L01信号质量计数
    NRF_SSI = NRF_SSI_CNT;      // NRF信号强度
    NRF_SSI_CNT = 0;
    

    
    // 读取并清零定位模块信号质量计数（预留）
    Locat_SSI = Locat_SSI_CNT;  // 定位模块信号成功率
    Locat_SSI_CNT = 0;
    
    // 读取并清零光流模块信号质量计数（预留）
    Flow_SSI = Flow_SSI_CNT;    // 光流模块成功率
    Flow_SSI_CNT = 0;
    
    // 检测定位模块是否正常（信号成功率>10视为正常）
    if(Locat_SSI > 10)  
        Locat_Err = 0;          // 定位模块正常
    else 
    {
        Locat_Mode = 0;         // 关闭定位模式
        Locat_Err = 1;          // 定位模块错误
    }
    
    // 检测光流模块是否正常（信号成功率>10视为正常）
    if(Flow_SSI > 10)  
        Flow_Err = 0;           // 光流模块正常
    else                         
        Flow_Err = 1;           // 光流模块错误
    
    // 计算所有任务总执行耗时
    time_sum = 0;
    for(i=0; i<6; i++)
        time_sum += time[i];
    
    // printf("Duty_1000ms\r\n");
}