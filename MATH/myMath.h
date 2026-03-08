#ifndef __MY_MATH_H
#define	__MY_MATH_H

/**
 * @brief 数学常量定义 - 圆周率
 */
extern const float M_PI;

/**
 * @brief 角度转弧度系数 (Angle to Radian)
 * @note 换算公式: 弧度 = 角度 * AtR
 */
extern const float AtR;

/**
 * @brief 弧度转角度系数 (Radian to Angle)
 * @note 换算公式: 角度 = 弧度 * RtA
 */
extern const float RtA;

/**
 * @brief 陀螺仪量程系数(原始值转角速度 °/s)
 * @note 对应±2000°/s量程，计算方式: 1 / (65536 / 4000) = 0.03051756*2
 */
extern const float Gyro_G;

/**
 * @brief 陀螺仪量程系数(原始值转角速度 rad/s)
 * @note 计算方式: Gyro_G * AtR = 0.03051756*2 * 0.0174533f = 0.0005326*2
 */
extern const float Gyro_Gr;

/**
 * @brief 圆周率宏定义(单精度浮点型)
 */
#define PI 3.1415926f

/**
 * @brief 计算数值的平方
 * @param Sq 输入数值(任意数值类型，会强制转换为float)
 * @return 输入值的平方(float类型)
 */
#define squa( Sq )        (((float)Sq)*((float)Sq))

/**
 * @brief 计算16位整数的绝对值
 * @param Math_X 输入16位整数
 * @return 输入值的绝对值(同类型)
 */
#define absu16( Math_X )  ((Math_X)<0? -(Math_X):(Math_X))

/**
 * @brief 计算浮点型数值的绝对值
 * @param Math_X 输入浮点型数值
 * @return 输入值的绝对值(float类型)
 */
#define absFloat( Math_X )((Math_X)<0? -(Math_X):(Math_X))

/**
 * @brief 取两个数值的最小值
 * @param a 第一个比较值
 * @param b 第二个比较值
 * @return 较小的数值(与输入类型一致)
 * @note 输入a/b需为相同数值类型
 */
#define min(a, b) ((a) < (b) ? (a) : (b))

/**
 * @brief 取两个数值的最大值
 * @param a 第一个比较值
 * @param b 第二个比较值
 * @return 较大的数值(与输入类型一致)
 * @note 输入a/b需为相同数值类型
 */
#define max(a, b) ((a) > (b) ? (a) : (b))

/**
 * @brief 通用绝对值宏(支持任意数值类型)
 * @param x 输入数值
 * @return 输入值的绝对值(与输入类型一致)
 */
#define ABS(x) ((x) > 0 ? (x) : -(x))

/**
 * @brief 数值范围限制宏
 * @param x 待限制的数值
 * @param min 最小值下限
 * @param max 最大值上限
 * @return 限制后的数值: 
 *         - 若x < min，返回min
 *         - 若x > max，返回max
 *         - 否则返回x本身
 * @note 输入x/min/max需为相同数值类型
 */
#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )

/**
 * @brief 安全的反正弦函数(防止输入值越界/非数值)
 * @param v 输入值(理论范围[-1,1])
 * @return 反正弦计算结果(弧度值):
 *         - 若v为NaN，返回0.0
 *         - 若v ≥ 1.0，返回M_PI/2
 *         - 若v ≤ -1.0，返回-M_PI/2
 *         - 否则返回asinf(v)的计算结果
 * @note 相比原生asin更健壮，避免参数越界导致的程序异常
 */
extern float safe_asin(float v);

/**
 * @brief 反正弦函数(泰勒级数展开实现)
 * @param x 输入值(范围[-1,1])
 * @return 反正弦计算结果(弧度值)
 * @note 泰勒级数展开迭代6次，42°范围内精度较高
 */
extern float arcsin(float x);

/**
 * @brief 反正切函数(泰勒级数展开实现)
 * @param x 输入值(任意浮点型数值)
 * @return 反正切计算结果(弧度值)
 * @note 泰勒级数展开迭代6次，70°范围内精度较高
 */
extern float arctan(float x);

/**
 * @brief 正弦函数(分段多项式/泰勒级数实现，由宏TAPYOR控制)
 * @param x 输入角度(弧度值)
 * @return 正弦计算结果(float类型)
 * @note 1. 未定义TAPYOR时：使用分段多项式近似，计算效率高
 *       2. 定义TAPYOR时：使用泰勒级数展开(迭代5次)，精度更高但效率稍低
 */
extern float sine(float x);

/**
 * @brief 余弦函数(基于正弦函数实现)
 * @param x 输入角度(弧度值)
 * @return 余弦计算结果(float类型)
 * @note 计算公式：cos(x) = sin(x + M_PI/2)
 */
extern float cosine(float x);

/**
 * @brief 快速平方根倒数(Quake III算法)
 * @param number 输入值(正浮点型数值)
 * @return 1/sqrt(number)的近似值(float类型)
 * @note 计算速度远快于原生sqrt+除法，精度满足大部分嵌入式场景需求
 */
extern float Q_rsqrt(float number);

/**
 * @brief 可变参数计算(根据误差值调整参数)
 * @param error 输入误差值(float类型)
 * @return 调整后的参数值：
 *         - 先取error绝对值，超过0.6则限制为0.6
 *         - 计算公式：result = 1 - 1.667f * error
 *         - 结果下限为0
 * @note 常用于控制算法中动态调整参数(如PID参数)
 */
extern float VariableParameter(float error);

#endif 