#ifndef __Algorithm_filter_H
#define	__Algorithm_filter_H

#include "ALL_DEFINE.h"
#include "myMath.h"

/**
 * @brief 一阶低通滤波器结构体
 * @details 用于存储一阶低通滤波器的历史数据和滤波系数
 * @param old_data 上一次滤波输出的历史数据
 * @param new_data 当前输入的新数据
 * @param factor 滤波系数(不同模型下含义不同:模型1为权重系数;模型2为截止频率Hz)
 */
typedef struct{
    float old_data;   // 上一次滤波输出值
    float new_data;   // 当前输入的原始数据
    float factor;     // 滤波系数(模型1:0~1;模型2:截止频率Hz)
}Filter_LPF_1;

/**
 * @brief 一阶低通滤波器(模型1)
 * @details 基础一阶低通实现，公式:y = old*(1-factor) + new*factor
 * @param LPF_1 指向一阶低通滤波器结构体的指针
 * @return 滤波后的输出值
 */
extern float LPF_1_Filter_1(Filter_LPF_1 *LPF_1);

/**
 * @brief 一阶低通滤波器(模型2)
 * @details 基于截止频率的一阶低通实现，适配不同采样周期
 * @param LPF_1 指向一阶低通滤波器结构体的指针
 * @param dt 采样周期(单位:s)
 * @return 滤波后的输出值
 * @note 公式:y = old + (dt/(1/(2π*fc)+dt))*(new - old)，fc为factor(截止频率)
 */
extern float LPF_1_Filter_2(Filter_LPF_1 *LPF_1,float dt);

/**
 * @brief 滑动平均滤波器结构体
 * @details 用于存储滑动平均滤波的缓存、计数和配置参数
 * @param cnt 当前数据存储的索引位置
 * @param input 当前输入的待滤波数据
 * @param average 指向滤波数据缓存数组的指针
 * @param max_cnt 缓存数组的最大长度(滤波窗口大小)
 */
typedef struct {
    uint16_t cnt;        // 当前数据索引
    uint16_t input;      // 待滤波的输入值
    uint16_t *average;   // 滑动窗口缓存数组
    uint8_t  max_cnt;    // 滑动窗口长度(数组大小)
}MovAverage;

/**
 * @brief 抗脉冲干扰滑动平均滤波器
 * @details 滑动平均基础上剔除最大值和最小值，抑制脉冲干扰
 * @param _MovAverage 指向滑动平均结构体的指针
 * @return 滤波后的输出值
 * @note 计算方式:(总和-最大值-最小值)/(窗口长度-2)
 */
extern uint16_t AntiPulse_MovingAverage_Filter(MovAverage *_MovAverage);

/**
 * @brief 标准滑动平均滤波器
 * @details 简单滑动平均滤波，对窗口内所有数据求平均
 * @param _MovAverage 指向滑动平均结构体的指针
 * @return 滤波后的输出值
 * @note 计算方式:总和/窗口长度
 */
extern uint16_t MovingAverage_Filter(MovAverage *_MovAverage);

/**
 * @brief IIR直接I型滤波器
 * @details 实现IIR滤波器的直接I型结构，支持多阶配置
 * @param InputData 当前输入的原始数据
 * @param x 输入数据缓存数组(存储x(n),x(n-1),x(n-2)...)
 * @param y 输出数据缓存数组(存储y(n),y(n-1),y(n-2)...)
 * @param b 分子系数数组(b0,b1,b2...)
 * @param nb 分子系数数组长度
 * @param a 分母系数数组(a1,a2,a3...)
 * @param na 分母系数数组长度
 * @return 滤波后的输出值
 * @note 滤波公式:y(n) = b0*x(n)+b1*x(n-1)+... -a1*y(n-1)-a2*y(n-2)-...
 */
extern float IIR_I_Filter(float InputData, float *x, float *y,  const float *b, uint8_t nb, const float *a, uint8_t na);

/**
 * @brief 增量式滑动平均滤波器
 * @details 增量计算的滑动平均，避免重复求和，提升效率
 * @param moavarray 滑动窗口缓存数组
 * @param len 窗口长度(数组大小)
 * @param fil_cnt 当前数据索引(指针)
 * @param in 当前输入的原始数据
 * @param out 滤波输出值(指针)
 * @note 增量公式:out += (in - 被替换的旧值)/len，附加微小限幅修正
 */
extern void Moving_Average(float moavarray[],u16 len ,u16 *fil_cnt,float in,float *out);

/**
 * @brief 滑动中值滤波器
 * @details 对窗口内数据排序后取中值，抑制随机脉冲干扰
 * @param item 滤波器通道编号(0~MED_FIL_ITEM-1)
 * @param width_num 窗口宽度(≤MED_WIDTH_NUM)
 * @param in 当前输入的原始数据
 * @return 滤波后的中值
 * @note 窗口宽度建议设为奇数，避免取平均
 */
extern float Moving_Median(uint8_t item,uint8_t width_num,float in);

/**
 * @brief 设置二阶低通滤波器1的截止频率
 * @param sample_freq 采样频率(Hz)
 * @param cutoff_freq 截止频率(Hz)
 */
void LPF2pSetCutoffFreq_1(float sample_freq, float cutoff_freq);

/**
 * @brief 应用二阶低通滤波器1
 * @param sample 当前输入的原始数据
 * @return 滤波后的输出值
 */
float LPF2pApply_1(float sample);

/**
 * @brief 设置二阶低通滤波器2的截止频率
 * @param sample_freq 采样频率(Hz)
 * @param cutoff_freq 截止频率(Hz)
 */
void LPF2pSetCutoffFreq_2(float sample_freq, float cutoff_freq);

/**
 * @brief 应用二阶低通滤波器2
 * @param sample 当前输入的原始数据
 * @return 滤波后的输出值
 */
float LPF2pApply_2(float sample);

/**
 * @brief 设置二阶低通滤波器3的截止频率
 * @param sample_freq 采样频率(Hz)
 * @param cutoff_freq 截止频率(Hz)
 */
void LPF2pSetCutoffFreq_3(float sample_freq, float cutoff_freq);

/**
 * @brief 应用二阶低通滤波器3
 * @param sample 当前输入的原始数据
 * @return 滤波后的输出值
 */
float LPF2pApply_3(float sample);

#endif /* __Algorithm_filter_H */

