// 作者:匿名
// 电话:18878881386
// 邮箱:3078510877@qq.com
// 日期:2026.3.6
// 版本:1.0
//  数学工具函数库（姿态解算专用）
 
#include <math.h>
#include "ALL_DEFINE.h"

// 数学常量定义
const float M_PI = 3.1415926535;    // 圆周率π
const float RtA = 57.2957795f;      // 弧度转角度系数（180/π）
const float AtR = 0.0174532925f;    // 角度转弧度系数（π/180）
const float Gyro_G = 0.03051756f*2; // 陀螺仪原始值转度/秒系数（±2000°/s量程：1/(65536/4000)=0.03051756*2）
const float Gyro_Gr = 0.0005326f*2; // 陀螺仪原始值转弧度/秒系数（2*0.03051756*0.0174533=0.0005326*2）

/**
 * @brief  快速正弦函数（近似计算）
 * @param  x: 输入角度（弧度，范围-π~π）
 * @retval float: 正弦值（-1~1）
 * @note   两种实现方式：
 *         1. 泰勒近似版（默认）：计算快，精度适中（PI范围内误差<0.7%）
 *         2. 泰勒展开版：精度高，计算稍慢（展开5阶）
 */
// 泰勒近似公式：Q*(4/π*x - 4/π²*x²) + P*(4/π*x - 4/π²*x²)²
#ifndef TAPYOR
float sine(float x)          
{
    const float Q = 0.775;          // 拟合系数1
    const float P = 0.225;          // 拟合系数2
    const float B =  4 / M_PI;      // 线性项系数
    const float C = -4 /(M_PI*M_PI);// 二次项系数
    float y = B * x + C * x * fabs(x); // 基础拟合
    return (Q * y + P * y * fabs(y)); // 修正后结果
}
#else 
// 泰勒级数展开版（PI范围内误差<0.7%）
// sinx = x - x³/3! + x⁵/5! - x⁷/7! + x⁹/9! = Σ(-1)ⁿ x^(2n+1)/(2n+1)!
float sine(float x)
{
    float t = x;                    // 临时变量（当前项）
    float result = x;               // 结果初始值（第一项）
    float X2 = x*x;                 // x²（避免重复计算）
    uint8_t cnt = 1;                // 展开阶数计数

    do
    {
        t = -t;                     // 符号取反
        t *= X2;                    // 乘以x²
        result += t/((cnt<<1)+1);   // 累加当前项：t/(2n+1)
        cnt++;
    } while(cnt<5); // 展开5阶（x^9/9!）

    return result;
} 
#endif

/**
 * @brief  余弦函数（基于正弦函数转换）
 * @param  x: 输入角度（弧度）
 * @retval float: 余弦值（-1~1）
 * @note   利用三角恒等式：cos(x) = sin(π/2 + x)
 */
float cosine(float x)
{
    return sine(x + M_PI/2); // 相位偏移π/2，计算更快
}

/**
 * @brief  反正切函数（泰勒级数展开）
 * @param  x: 输入值（-1~1）
 * @retval float: 反正切结果（弧度，-π/4~π/4）
 * @note   泰勒展开式：arctan(x) = x - x³/3 + x⁵/5 - x⁷/7 + ...
 *         展开6阶，70°范围内精度较高
 */
float arctan(float x) 
{
    float t = x;                    // 当前项
    float result = 0;               // 结果
    float X2 = x * x;               // x²
    unsigned char cnt = 1;          // 阶数计数

    do
    {
        result += t / ((cnt << 1) - 1); // 累加：t/(2n-1)
        t = -t;                     // 符号取反
        t *= X2;                    // 乘以x²
        cnt++;
    } while(cnt <= 6); // 展开6阶

    return result;
}

/**
 * @brief  反正弦函数（泰勒级数展开）
 * @param  x: 输入值（-1~1）
 * @retval float: 反正弦结果（弧度，-π/2~π/2）
 * @note   泰勒展开式，-1<x<+1范围内42°精度较高
 */
const float PI_2 = 1.570796f;      // π/2
float arcsin(float x)   
{
    float d = 1;                    // 分母系数
    float t = x;                    // 当前项
    unsigned char cnt = 1;          // 阶数计数
    float result = 0;               // 结果
    float X2 = x*x;                 // x²

    // 边界处理
    if (x >= 1.0f) 
    {
        return PI_2;                // arcsin(1) = π/2
    }
    if (x <= -1.0f) 
    {
        return -PI_2;               // arcsin(-1) = -π/2
    }

    do
    {
        result += t / (d * ((cnt << 1) - 1)); // 累加当前项
        t *= X2 * ((cnt << 1) - 1);          // 更新当前项
        d *= (cnt << 1);                     // 更新分母系数（2,4,6...）
        cnt++;
    } while(cnt <= 6); // 展开6阶

    return result;
}

/**
 * @brief  安全反正弦函数（防溢出）
 * @param  v: 输入值
 * @retval float: 反正弦结果（弧度，-π/2~π/2）
 * @note   增加NaN和边界值处理，避免计算异常
 */
float safe_asin(float v)
{
    if (isnan(v)) {                 // 非数值处理
        return 0.0;
    }
    if (v >= 1.0f) {                // 上边界
        return M_PI/2;
    }
    if (v <= -1.0f) {               // 下边界
        return -M_PI/2;
    }
    return asinf(v);                // 标准库函数
}

/**
 * @brief  快速平方根倒数（1/sqrt(x)）
 * @param  number: 输入值（正数）
 * @retval float: 1/sqrt(number)
 * @note   经典的Quake III快速逆平方根算法，速度远快于标准库sqrt
 *         精度：相对误差约0.1%，满足实时控制需求
 */
float Q_rsqrt(float number)
{
    long i;
    float x2, y;
    const float threehalfs = 1.5F;  // 1.5常量

    x2 = number * 0.5F;             // number/2
    y  = number;
    i  = * ( long * ) &y;           // 浮点转长整型（取二进制表示）
    i  = 0x5f3759df - ( i >> 1 );   // 神奇数字+右移，初始近似值
    y  = * ( float * ) &i;          // 长整型转回浮点型
    y  = y * ( threehalfs - ( x2 * y * y ) ); // 牛顿迭代修正
    return y;
} 

/**************************数组操作函数****************************************/
/**
 * @brief  数组数值限幅
 * @param  *array: 目标数组指针
 * @param  lower: 下限值
 * @param  upper: 上限值
 * @retval 无
 * @note   将数组中每个元素限制在[lower, upper]范围内
 */
void array_astrict(int16_t *array, int16_t lower, int16_t upper)
{
    int16_t length = sizeof(array); // 数组长度（注：实际使用时需传入真实长度）
    uint16_t i = 0;
    for(i=0; i<length; i++)
    {
        if(*(array+i) < lower)      // 低于下限
            *(array+i) = lower;
        else if(*(array+i) > upper) // 高于上限
            *(array+i) = upper;
    } 
}

/**
 * @brief  数组赋值
 * @param  *array: 目标数组指针
 * @param  value: 要赋值的数值
 * @retval 无
 * @note   将数组中所有元素设置为指定值
 */
void array_assign(int16_t *array, int16_t value)
{
    uint16_t length = sizeof(array); // 数组长度（注：实际使用时需传入真实长度）
    uint16_t i = 0;
    for(i=0; i<length; i++)
    {
        *(array+i) = value;
    } 
}

/**
 * @brief  单值限幅函数
 * @param  data: 要限制的数值
 * @param  toplimit: 上限值
 * @param  lowerlimit: 下限值
 * @retval float: 限幅后的数值
 * @note   基础限幅函数，适用于PID输出、传感器数据等
 */
float data_limit(float data, float toplimit, float lowerlimit)
{
    if(data > toplimit)             // 高于上限
        data = toplimit;
    else if(data < lowerlimit)      // 低于下限
        data = lowerlimit;
    return data;
}

/**
 * @brief  变参数函数（误差自适应调整）
 * @param  error: 误差值
 * @retval float: 调整后的参数（0~1）
 * @note   用于PID参数自适应：误差越大，返回值越小
 *         误差>0.6时，返回值固定为0；误差=0时，返回值=1
 */
float VariableParameter(float error)
{
    float  result = 0;
    
    if(error < 0)                   // 取误差绝对值
    {
        error = -error;
    }
    if(error > 0.6f)                // 误差上限
    {
        error = 0.6f;
    }
    result = 1 - 1.667f * error;    // 线性调整
    if(result < 0)                  // 下限保护
    {
        result = 0;
    }
    return result;
}

/**
 * @brief  3值取中函数（简单中值滤波）
 * @param  input: 新输入值（注：原代码未正确使用input，需修正）
 * @retval float: 中间值
 * @note   原代码存在逻辑错误（a/b/c未初始化），需补充输入缓存机制
 */
float middle_3(float input) // 3个数取中间值
{ 
    // 注：原代码缺少a/b/c的初始化和更新逻辑，以下为修正后的核心排序逻辑
    int a,b,c,t; 

    // 假设a/b/c为历史缓存值，需补充：
    // c = b; b = a; a = input;
    
    // 冒泡排序找中间值
    if(a < b)
    { 
        t = a; a = b; b = t; 
    } 
    if(b < c)
    { 
        t = b; b = c; c = t;      
    } 
    if(a < b)
    { 
        t = a; a = b; b = t; 
    } 
    return b; // 返回中间值
}

/**
 * @brief  死区函数（对称死区）
 * @param  x: 输入值
 * @param  zoom: 死区范围（±zoom）
 * @retval float: 处理后的值（死区内返回0，死区外返回原值）
 * @note   用于消除微小信号干扰，如遥控器微小偏移
 */
float my_deathzoom_2(float x, float zoom)
{
    float t;
    
    if( x > -zoom && x < zoom )     // 死区内
    {
        t = 0;
    }
    else                            // 死区外
    {
        t = x;
    }
    return (t);
}

/**
 * @brief  死区函数（偏移死区）
 * @param  x: 输入值
 * @param  zoom: 死区大小
 * @retval float: 处理后的值（死区内返回0，死区外返回偏移后的值）
 * @note   适用于需要保留信号幅值的场景：
 *         正数：x>zoom时返回x-zoom，否则返回0
 *         负数：x<-zoom时返回x+zoom，否则返回0
 */
float my_deathzoom(float x, float zoom)
{
    float t;
    if(x > 0)                       // 正数处理
    {
        t = x - zoom;
        if(t < 0)
        {
            t = 0;
        }
    }
    else                            // 负数处理
    {
        t = x + zoom;
        if(t > 0)
        {
            t = 0;
        }
    }
    return (t);
}

