// 作者：咸
// 电话:18878881386
// 邮箱:3078510877@qq.com
// 日期：2026.3.8
// 版本：1.0
//  气压计融合INS高度滤波算法
#include "ALL_DEFINE.h"

// 高度解算相关结构体实例
_st_Height Height;

// 高度/速度/加速度修正量（互补滤波用）
float pos_correction;
float acc_correction;
float vel_correction;

// 高度融合时间常数（控制收敛速度）
const float TIME_CONTANST_ZER=3.0f;
// 高度融合PID参数（基于时间常数的三阶互补滤波系数）
#define K_ACC_ZER 	        (1.0f / (TIME_CONTANST_ZER * TIME_CONTANST_ZER * TIME_CONTANST_ZER))  // 加速度修正系数
#define K_VEL_ZER	        (3.0f / (TIME_CONTANST_ZER * TIME_CONTANST_ZER))                      // 速度修正系数
#define K_POS_ZER           (3.0f / TIME_CONTANST_ZER)                                          // 位置修正系数

// 高度测试用临时变量
float high_test;

/**************************************************************
 * @brief  捷联惯导(INS)高度解算核心函数（融合气压计数据）
 * @param  high: 气压计原始高度值（单位：cm）
 * @retval 无
 * @note   1. 采用三阶互补滤波融合气压计和IMU加速度数据
 *         2. 初始500ms为预热阶段，用于校准气压计零偏
 *         3. 积分解算高度/速度，通过气压计反馈修正积分漂移
 ***************************************************************/
void Strapdown_INS_High(float high)
{
    float dt;                      // 控制周期（ms）
    float Altitude_Estimate=0;     // 气压计估计高度
    const uint8_t High_Delay_Cnt=2;// 高度历史缓存长度（对应150ms）
    static float History_Z[High_Delay_Cnt+1]; // 高度历史缓存数组
    float temp;                    // 临时计算变量
    float speed_Z;                 // Z轴瞬时速度（cm/s）
    uint8_t Cnt;                   // 循环计数变量
    static float t_high;           // 高度积分值（cm）
    static float t_speed;          // 速度积分值（cm/s）
    static float last_accZ;        // 上一帧Z轴加速度（cm/s?）
    static uint16_t Save_Cnt=0;    // 缓存更新计数
    static float Current_accZ;     // 当前Z轴机体坐标系加速度（cm/s?）
    static uint16_t delay_cnt;     // 初始化延时计数
    static float high_offset;      // 气压计零偏校准值

    // ------------------------ 计算控制周期dt ------------------------
    {
        static float last_time;    // 上一帧时间戳（微秒）
        float now_time;            // 当前时间戳（微秒）
        now_time = micros();       // 获取系统微秒数
        Height.sutgbl = micros();  // 记录高度解算时间戳
        dt = now_time - last_time; // 计算时间差（微秒）
        dt /=1000;                 // 转换为毫秒
        // dt = 0.003f;             // 调试用固定周期（3ms）
        last_time = now_time;      // 更新上一帧时间
    }

    // ------------------------ 初始化阶段（零偏校准） ------------------------
    if(delay_cnt<500) // 前500ms为预热阶段，校准气压计零偏
    {
        pos_correction = acc_correction = vel_correction = 0; // 修正量清零
        high_offset = high;                                   // 记录初始气压计值作为零偏
        delay_cnt++;
        return;
    }
    high -= high_offset; // 气压计值减去零偏，得到相对高度

    // ------------------------ 气压计高度估计 ------------------------
    Altitude_Estimate=high; // 气压计修正后的高度作为估计值
    high_test = high;       // 保存测试用高度值

    // ------------------------ 三阶互补滤波修正 ------------------------
    // 计算气压计与INS积分高度的差值（单位：cm）
    temp= Altitude_Estimate - History_Z[High_Delay_Cnt];
    // 积分修正量（通过差值调整加速度/速度/位置）
    acc_correction += temp * K_ACC_ZER * dt; // 加速度修正量积分
    vel_correction += temp * K_VEL_ZER * dt; // 速度修正量积分
    pos_correction += temp * K_POS_ZER * dt; // 位置修正量积分

    // ------------------------ Z轴加速度解算 ------------------------
    last_accZ = Current_accZ;                          // 保存上一帧加速度
    Current_accZ = Attitude.acc_yaw_sensor + acc_correction; // 当前加速度 = 机体垂向加速度 + 修正量

    // ------------------------ 速度积分（梯形积分） ------------------------
    // 梯形积分计算瞬时速度：(上一帧加速度 + 当前加速度) * 时间/2
    speed_Z = +(last_accZ + Current_accZ) * dt / 2.0f;

    // ------------------------ 高度积分 ------------------------
    // 原始高度积分：上一帧速度*时间 + 0.5*加速度*时间?
    t_high += (Height.Speed + 0.5f * speed_Z) * dt;
    // 高度修正：积分值 + 位置修正量
    Height.High = t_high + pos_correction;

    // ------------------------ 速度修正 ------------------------
    // 原始速度积分：累加瞬时速度
    t_speed += speed_Z;
    // 速度修正：积分值 + 速度修正量
    Height.Speed = t_speed + vel_correction;

    // ------------------------ 高度历史缓存更新 ------------------------
    Save_Cnt++; // 缓存更新计数

    if(Save_Cnt>=1) // 每5ms更新一次缓存
    {
        // 缓存数据移位（最新数据在最前）
        for(Cnt=High_Delay_Cnt; Cnt>0; Cnt--)
        {
            History_Z[Cnt] = History_Z[Cnt-1];
        }
        History_Z[0] = Height.High; // 保存当前解算高度到缓存
        Save_Cnt=0;                 // 计数清零
    }
}

