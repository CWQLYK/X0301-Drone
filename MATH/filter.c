// 作者：咸
// 电话:18878881386
// 邮箱:3078510877@qq.com
// 日期：2026.3.8
// 版本：1.0
// MPU6050运动滤波算法库
#include <string.h>
#include "filter.h"
#include <math.h>
#include "myMath.h"
 
/**************************************************************
 * @brief  移动中值滤波（5点中值滤波）
 * @param  input: 输入的原始数据（int16_t类型）
 * @retval int16_t: 滤波后的中值
 * @note   1. 固定5个采样点，排序后取中间值（第3个）
 *         2. 使用异或交换排序，避免临时变量
 *         3. 适用于去除脉冲干扰（如传感器尖峰噪声）
 ***************************************************************/
int16_t MovMiddle(int16_t input)
{	
    uint8_t i,j;
    const uint8_t MOV_MIDDLE_NUM = 5;  // 中值滤波窗口大小（5点）
    static int16_t middle[5]={0};      // 历史数据缓存
    int16_t middle_t[5];               // 排序用临时数组
    // MOV_MIDDLE_NUM = pidHeightRate.ki; // 预留：动态调整窗口大小（注释）

    // 1. 数据移位：丢弃最旧数据，新数据入队
    for(i=1; i<MOV_MIDDLE_NUM; i++)
    {
        middle[i-1] = middle[i];
    }
    middle[MOV_MIDDLE_NUM-1] = input;

    // 2. 复制数据到临时数组（避免修改原缓存）
    memcpy(middle_t, middle, MOV_MIDDLE_NUM*sizeof(uint32_t));

    // 3. 冒泡排序（升序）
    for(i=0; i<MOV_MIDDLE_NUM-1; i++)
    {
        for(j=i+1; j<MOV_MIDDLE_NUM; j++)
        {
            if(middle_t[i] > middle_t[j])
            {
                // 异或交换（无临时变量交换两个数）
                middle_t[i] ^= middle_t[j];
                middle_t[j] ^= middle_t[i];
                middle_t[i] ^= middle_t[j];
            }
        }
    }

    // 4. 返回中值（(5+1)/2=3 → 索引2）
    return middle_t[(MOV_MIDDLE_NUM+1)>>1];
}	

/**************************************************************
 * @brief  抗脉冲移动平均滤波（去掉最大值最小值后平均）
 * @param  _MovAverage: 移动平均滤波器结构体指针
 * @retval uint16_t: 滤波后的值
 * @note   1. 先去掉窗口内最大值和最小值，再求平均
 *         2. 有效抑制脉冲干扰，保留趋势
 *         3. 适用于含尖峰噪声的传感器数据（如气压计）
 ***************************************************************/
uint16_t AntiPulse_MovingAverage_Filter(MovAverage *_MovAverage)
{
    uint8_t i;	
    uint32_t sum=0;          // 数据累加和（防止溢出）
    uint16_t max=0;          // 窗口内最大值
    uint16_t min=0xffff;     // 窗口内最小值

    // 1. 新数据入队，更新计数
    _MovAverage->average[_MovAverage->cnt] = _MovAverage->input;	
    _MovAverage->cnt++;			
    if(_MovAverage->cnt == _MovAverage->max_cnt)
    {
        _MovAverage->cnt = 0; // 计数溢出后复位
    }	

    // 2. 遍历窗口数据，找最大/最小值并累加
    for(i=0; i<_MovAverage->max_cnt; i++)
    {
        if(_MovAverage->average[i] > max)
            max = _MovAverage->average[i];
        else if(_MovAverage->average[i] < min)
            min = _MovAverage->average[i];
        sum += _MovAverage->average[i];
    }

    // 3. 去掉最大/最小值后求平均
    return ((sum - max - min) / (_MovAverage->max_cnt - 2));                                    
}

/**************************************************************
 * @brief  普通移动平均滤波（简单均值滤波）
 * @param  _MovAverage: 移动平均滤波器结构体指针
 * @retval uint16_t: 滤波后的值
 * @note   1. 窗口内所有数据直接求平均
 *         2. 算法简单，平滑效果好，但对脉冲干扰敏感
 *         3. 适用于噪声平稳的传感器数据
 ***************************************************************/
uint16_t MovingAverage_Filter(MovAverage *_MovAverage)
{
    uint8_t i;	
    uint32_t sum=0;          // 数据累加和

    // 1. 新数据入队，更新计数
    _MovAverage->average[_MovAverage->cnt] = _MovAverage->input;	
    _MovAverage->cnt++;			
    if(_MovAverage->cnt == _MovAverage->max_cnt)
    {
        _MovAverage->cnt = 0; // 计数溢出后复位
    }	

    // 2. 累加窗口内所有数据
    for(i=0; i<_MovAverage->max_cnt; i++)
    {
        sum += _MovAverage->average[i];
    }

    // 3. 返回平均值
    return (sum / _MovAverage->max_cnt);                                    
}

/**************************************************************
 * @brief  IIR无限冲击响应滤波器（通用IIR滤波实现）
 * @param  InputData: 输入数据
 * @param  x: 输入历史缓存数组
 * @param  y: 输出历史缓存数组
 * @param  b: 分子系数数组（前向系数）
 * @param  nb: 分子系数个数
 * @param  a: 分母系数数组（反馈系数）
 * @param  na: 分母系数个数
 * @retval float: 滤波后输出值
 * @note   1. 实现标准IIR滤波公式：y[n] = Σb[i]x[n-i] - Σa[i]y[n-i]
 *         2. 适用于低通/高通/带通等自定义滤波
 ***************************************************************/
float IIR_I_Filter(float InputData, float *x, float *y, const float *b, uint8_t nb, const float *a, uint8_t na)
{
    float z1, z2=0;  // 临时计算变量（z1=前向项，z2=反馈项）
    int16_t i;
	
    // 1. 历史数据移位（x[n-1]=x[n-2], y[n-1]=y[n-2]...）
    for(i=nb-1; i>0; i--)
    {
        x[i] = x[i-1];
        y[i] = y[i-1];
    }

    // 2. 新输入数据入队
    x[0] = InputData;

    // 3. 计算前向项（Σb[i]x[n-i]）
    z1 = x[0] * b[0];
    for(i=1; i<nb; i++)
    {
        z1 += x[i] * b[i];
        z2 += y[i] * a[i]; // 计算反馈项（Σa[i]y[n-i]）
    }

    // 4. 计算当前输出
    y[0] = z1 - z2; 
    return y[0];
}

/**************************************************************
 * @brief  一阶低通滤波（系数版）
 * @param  LPF_1: 一阶低通滤波器结构体指针
 * @retval float: 滤波后的值
 * @note   1. 公式：y = 旧值*(1-α) + 新值*α（α=滤波系数）
 *         2. α越大，响应越快，滤波效果越差；α越小则相反
 ***************************************************************/
float LPF_1_Filter_1(Filter_LPF_1 *LPF_1)
{
    return LPF_1->old_data * (1 - LPF_1->factor) + LPF_1->new_data * LPF_1->factor;
}

/**************************************************************
 * @brief  一阶低通滤波（截止频率版）
 * @param  LPF_1: 一阶低通滤波器结构体指针
 * @param  dt: 采样周期（秒）
 * @retval float: 滤波后的值
 * @note   1. 公式：y = 旧值 + (dt/(τ+dt))*(新值-旧值)
 *         2. τ=1/(2πf)，f为截止频率（LPF_1->factor存储截止频率）
 *         3. 按截止频率动态计算系数，适配不同采样率
 ***************************************************************/
float LPF_1_Filter_2(Filter_LPF_1 *LPF_1, float dt)
{
    // τ = 1/(2πf)，f=LPF_1->factor（截止频率）
    return LPF_1->old_data + (dt / (1 / (2 * PI * LPF_1->factor) + dt)) * (LPF_1->new_data - LPF_1->old_data);    
}

// 移动中值滤波参数定义
#define MED_WIDTH_NUM 11    // 中值滤波最大窗口宽度
#define MED_FIL_ITEM  4     // 支持的滤波通道数（4路）

// 中值滤波缓存：[通道][窗口]
float med_filter_tmp[MED_FIL_ITEM][MED_WIDTH_NUM];
// 中值滤波输出缓存
float med_filter_out[MED_FIL_ITEM];
// 各通道计数
uint8_t med_fil_cnt[MED_FIL_ITEM];

/**************************************************************
 * @brief  多通道移动中值滤波（可配置窗口宽度）
 * @param  item: 通道索引（0~3）
 * @param  width_num: 窗口宽度（≤11）
 * @param  in: 输入数据
 * @retval float: 滤波后中值
 * @note   1. 支持4路独立通道，每路可配置不同窗口宽度
 *         2. 窗口宽度需≤MED_WIDTH_NUM（11），否则返回0
 ***************************************************************/
float Moving_Median(uint8_t item, uint8_t width_num, float in)
{
    uint8_t i,j;
    float t;                // 排序临时变量
    float tmp[MED_WIDTH_NUM]; // 排序用临时数组

    // 边界检查：通道/窗口宽度超出范围则返回0
    if(item >= MED_FIL_ITEM || width_num >= MED_WIDTH_NUM )
    {
        return 0;
    }
    else
    {
        // 1. 更新计数，溢出后复位
        if( ++med_fil_cnt[item] >= width_num )	
        {
            med_fil_cnt[item] = 0;
        }

        // 2. 新数据入队
        med_filter_tmp[item][ med_fil_cnt[item] ] = in;

        // 3. 复制缓存到临时数组
        for(i=0; i<width_num; i++)
        {
            tmp[i] = med_filter_tmp[item][i];
        }

        // 4. 冒泡排序（升序）
        for(i=0; i<width_num-1; i++)
        {
            for(j=0; j<(width_num-1-i); j++)
            {
                if(tmp[j] > tmp[j+1])
                {
                    t = tmp[j];
                    tmp[j] = tmp[j+1];
                    tmp[j+1] = t;
                }
            }
        }		

        // 5. 返回中值（窗口宽度/2的位置）
        return ( tmp[(width_num/2)] );
    }
}

/**************************************************************
 * @brief  增量式移动平均滤波（高效计算）
 * @param  moavarray: 历史数据缓存数组
 * @param  len: 窗口长度
 * @param  fil_cnt: 计数指针（溢出复位）
 * @param  in: 输入数据
 * @param  out: 输出值指针（滤波结果）
 * @retval 无
 * @note   1. 增量计算：新平均值 = 旧平均值 + (新值-旧值)/窗口长度
 *         2. 增加微小偏置补偿，防止输出长期漂移
 *         3. 无需每次累加所有数据，计算效率高
 ***************************************************************/
void Moving_Average(float moavarray[], u16 len, u16 *fil_cnt, float in, float *out)
{
    u16 width_num;
    float last; // 被替换的旧数据
 
    width_num = len; // 窗口长度

    // 1. 更新计数，溢出后复位
    if( ++*fil_cnt >= width_num )	
    {
        *fil_cnt = 0; 
    }

    // 2. 保存被替换的旧数据
    last = moavarray[ *fil_cnt ];

    // 3. 新数据入队，替换旧数据
    moavarray[ *fil_cnt ] = in;

    // 4. 增量计算平均值：out = out + (in - last)/len
    *out += ( in - last ) / (float)( width_num );
    
    // 5. 微小偏置补偿（防止漂移）
    *out += 0.00001f * LIMIT((in - *out), -1, 1);  
}


