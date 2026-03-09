//	作者：咸
//	电话:18878881386
//	邮箱:3078510877@qq.com
//	日期：2026.3.8
//	版本：1.0
//  姿态数据解算
#include "ALL_DEFINE.h"
#include <math.h>

// 全局姿态数据结构体实例，存储所有姿态相关的加速度、角速度等数据
_st_Attitude Attitude;

/*****************************************************************************************
* 模块说明：加速度滤波处理
* 功能概述：定义不同截止频率的巴特沃斯低通滤波器参数，实现加速度信号的低通滤波，
*           滤除高频噪声，为后续姿态解算和控制提供稳定的加速度数据
*****************************************************************************************/

//-----巴特沃斯滤波器参数结构体定义（此处为外部定义，这里仅使用）-----//
// Butter_Parameter结构体说明：
// a[0],a[1],a[2]：滤波器分母系数（反馈项）
// b[0],b[1],b[2]：滤波器分子系数（前馈项）
// 滤波器公式：y(n) = b0*x(n) + b1*x(n-1) + b2*x(n-2) - a1*y(n-1) - a2*y(n-2)
// 注：a[0]固定为1，是滤波器归一化系数

// 200Hz采样率下，80Hz截止频率的巴特沃斯滤波器参数（加速度专用）
Butter_Parameter Butter_80HZ_Parameter_Acce={
    1,     1.14298050254,   0.4128015980962,  // a系数：a0=1, a1=1.14298, a2=0.41280
    0.638945525159,    1.277891050318,    0.638945525159   // b系数：b0=0.63895, b1=1.27789, b2=0.63895
};

// 200Hz采样率下，60Hz截止频率的巴特沃斯滤波器参数（加速度专用）
Butter_Parameter Butter_60HZ_Parameter_Acce={
    1,   0.3695273773512,   0.1958157126558,
    0.3913357725018,   0.7826715450035,   0.3913357725018
};

// 200Hz采样率下，50Hz截止频率的巴特沃斯滤波器参数（加速度专用）
Butter_Parameter Butter_50HZ_Parameter_Acce={
    1,-1.300707181133e-16,   0.1715728752538,
    0.2065720838261,   0.4131441676523,   0.2065720838261,
};

// 200Hz采样率下，30Hz截止频率的巴特沃斯滤波器参数（加速度专用）
Butter_Parameter Butter_30HZ_Parameter_Acce={
    1,  -0.7477891782585,    0.272214937925,
    0.1311064399166,   0.2622128798333,   0.1311064399166
};

// 200Hz采样率下，20Hz截止频率的巴特沃斯滤波器参数（加速度专用）
Butter_Parameter Butter_20HZ_Parameter_Acce={
    1,    -1.14298050254,   0.4128015980962,
    0.06745527388907,   0.1349105477781,  0.06745527388907
};

// 200Hz采样率下，15Hz截止频率的巴特沃斯滤波器参数（加速度专用）
Butter_Parameter Butter_15HZ_Parameter_Acce={
    1,   -1.348967745253,   0.5139818942197,
    0.04125353724172,  0.08250707448344,  0.04125353724172
};

// 200Hz采样率下，10Hz截止频率的巴特沃斯滤波器参数（加速度专用）
Butter_Parameter Butter_10HZ_Parameter_Acce={
    1,   -1.561018075801,   0.6413515380576,
    0.02008336556421,  0.04016673112842,  0.02008336556421
};

// 200Hz采样率下，5Hz截止频率的巴特沃斯滤波器参数（加速度专用）
Butter_Parameter Butter_5HZ_Parameter_Acce={
    1,   -1.778631777825,   0.8008026466657,
    0.005542717210281,  0.01108543442056, 0.005542717210281
};

// 200Hz采样率下，2Hz截止频率的巴特沃斯滤波器参数（加速度专用）
Butter_Parameter Butter_2HZ_Parameter_Acce={
    1,   -1.911197067426,   0.9149758348014,
    0.0009446918438402,  0.00188938368768,0.0009446918438402
};

// 200Hz采样率下，1Hz截止频率的巴特沃斯滤波器参数（加速度专用）
Butter_Parameter Butter_1HZ_Parameter_Acce={
    1,   -1.955578240315,   0.9565436765112,
    0.000241359049042, 0.000482718098084, 0.000241359049042
};

/**
 * @brief 低通巴特沃斯滤波器实现函数
 * @param curr_input  当前输入的原始数据（单次采样值）
 * @param Buffer      滤波器缓存结构体指针（存储历史输入/输出值）
 * @param Parameter   滤波器参数结构体指针（a/b系数）
 * @return float      滤波后的输出值
 * @note  1. 前100次采样使用直通模式（避免滤波器启动初期的瞬态响应）
 *        2. 采用二阶IIR结构，需要缓存前两次的输入和输出值
 */
float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter)
{
    // 滤波器启动计数器，用于前100次采样的直通处理
    static int LPB_Cnt=0;
    
    // 将当前输入值存入缓存的最新位置（x(n)）
    Buffer->Input_Butter[2] = curr_input;
    
    // 前100次采样：输出等于输入（滤波器预热，避免初始值为0导致的突变）
    if(LPB_Cnt >= 100)
    {
        // 二阶巴特沃斯滤波器核心计算
        Buffer->Output_Butter[2] = Parameter->b[0] * Buffer->Input_Butter[2]  // b0*x(n)
                                 + Parameter->b[1] * Buffer->Input_Butter[1]  // b1*x(n-1)
                                 + Parameter->b[2] * Buffer->Input_Butter[0]  // b2*x(n-2)
                                 - Parameter->a[1] * Buffer->Output_Butter[1] // -a1*y(n-1)
                                 - Parameter->a[2] * Buffer->Output_Butter[0];// -a2*y(n-2)
    }
    else
    {
        // 预热阶段：输出等于输入
        Buffer->Output_Butter[2] = Buffer->Input_Butter[2];
        LPB_Cnt++;
    }
    
    // 更新输入缓存：x(n-2) = x(n-1), x(n-1) = x(n)
    Buffer->Input_Butter[0] = Buffer->Input_Butter[1];
    Buffer->Input_Butter[1] = Buffer->Input_Butter[2];
    
    // 更新输出缓存：y(n-2) = y(n-1), y(n-1) = y(n)
    Buffer->Output_Butter[0] = Buffer->Output_Butter[1];
    Buffer->Output_Butter[1] = Buffer->Output_Butter[2];

    // 返回本次滤波结果
    return Buffer->Output_Butter[2];
}

// 加速度校正专用滤波器缓存（3轴：X/Y/Z）
Butter_BufferData Butter_Buffer_Correct[3];

// 加速度控制环专用滤波器缓存（3轴：X/Y/Z）
Butter_BufferData Butter_Buffer[3];

// 加速度反馈环专用滤波器缓存（3轴：X/Y/Z）
Butter_BufferData Butter_Buffer_Feedback[3];

// 备用巴特沃斯滤波器参数（200Hz采样率-21Hz截止频率）
float Butter_parameter[2][3]={
    0.07319880848434,   0.1463976169687,  0.07319880848434,  // b系数
    1,   -1.102497726129,   0.3952929600662   // a系数
};

// 物理常数与加速度计标定参数定义
#define GRAVITY_MSS     9.80665f       // 标准重力加速度 (m/s²)
#define AcceMax_1G      4096.0f        // 1G重力下加速度计的输出值（数字量）
#define One_G_TO_Accel  AcceMax_1G/GRAVITY_MSS  // 数字量转实际加速度的系数 (count/(m/s²))
#define AcceMax         4096.0f        // 加速度计最大量程对应的数字量
#define AcceGravity     9.80f          // 简化的重力加速度（工程使用）

// 加速度控制临时数组
float Acce_Control[3]={0};

/*****************************************************************************************
* 函数名：Acce_Control_Filter
* 功能：  用于内环PID加速度环的加速度数据处理
* 流程：  1. 对原始加速度进行30Hz低通滤波，得到控制用加速度
*         2. 对原始加速度进行5Hz低通滤波，得到反馈用加速度
*         3. 将反馈加速度投影到导航坐标系，计算垂直/东西/南北方向的加速度
*         4. 单位转换：数字量→实际加速度（cm/s²），并去除重力分量
*****************************************************************************************/
void Acce_Control_Filter(void)
{
    // 加速度反馈量临时数组（5Hz滤波后）
    float Acce_Control_Feedback[3]={0};

    /********************** 控制用加速度：30Hz低通滤波 **************************/ 
    // X轴控制用加速度（30Hz滤波）
    Attitude.accX_control = LPButterworth(Attitude.accX_origion, &Butter_Buffer[0], &Butter_30HZ_Parameter_Acce);
    // Y轴控制用加速度（30Hz滤波）
    Attitude.accY_control = LPButterworth(Attitude.accY_origion, &Butter_Buffer[1], &Butter_30HZ_Parameter_Acce);
    // Z轴控制用加速度（30Hz滤波）
    Attitude.accZ_control = LPButterworth(Attitude.accZ_origion, &Butter_Buffer[2], &Butter_30HZ_Parameter_Acce);

    /********************** 反馈用加速度：5Hz低通滤波 ***************************/
    // X轴反馈用加速度（5Hz滤波）
    Acce_Control_Feedback[0] = LPButterworth(Attitude.accX_origion, &Butter_Buffer_Feedback[0], &Butter_5HZ_Parameter_Acce);
    // Y轴反馈用加速度（5Hz滤波）
    Acce_Control_Feedback[1] = LPButterworth(Attitude.accY_origion, &Butter_Buffer_Feedback[1], &Butter_5HZ_Parameter_Acce);
    // Z轴反馈用加速度（5Hz滤波）
    Acce_Control_Feedback[2] = LPButterworth(Attitude.accZ_origion, &Butter_Buffer_Feedback[2], &Butter_5HZ_Parameter_Acce);

    // 将机体坐标系的加速度投影到导航坐标系（垂直海拔方向）
    // 公式说明：基于欧拉角（Roll/Pitch/Yaw）的坐标变换，提取垂直方向加速度
    Attitude.acc_high_feedback = -IMU.Sin_Roll * Acce_Control_Feedback[0]
                               + IMU.Sin_Pitch * IMU.Cos_Roll * Acce_Control_Feedback[1]
                               + IMU.Cos_Pitch * IMU.Cos_Roll * Acce_Control_Feedback[2];

    // 以下为东西/南北方向加速度投影（预留GPS融合使用，当前注释）
    // Attitude.acc_pitch_feedback = ...; // 东西方向加速度
    // Attitude.acc_roll_feedback = ...;  // 南北方向加速度

    // 单位转换1：数字量 → 实际加速度（m/s²）
    Attitude.acc_high_feedback *= AcceGravity / AcceMax;
    // 去除重力加速度（只保留运动产生的垂直加速度）
    Attitude.acc_high_feedback -= AcceGravity;
    // 单位转换2：m/s² → cm/s²（适配控制环单位）
    Attitude.acc_high_feedback *= 100;

    // 以下为东西/南北方向加速度的单位转换（预留）
    Attitude.acc_pitch_feedback *= AcceGravity / AcceMax;
    Attitude.acc_pitch_feedback *= 100;
    Attitude.acc_roll_feedback *= AcceGravity / AcceMax;
    Attitude.acc_roll_feedback *= 100;
}

// 加速度巴特沃斯滤波缓存结构体（自定义实现，3轴）
struct _ButterWorth2d_Acc_Tag
{
    int16_t input[3];   // 输入缓存：input[0]=x(n-2), input[1]=x(n-1), input[2]=x(n)
    int16_t output[3];  // 输出缓存：output[0]=y(n-2), output[1]=y(n-1), output[2]=y(n)
};

// 加速度滤波缓存（3轴：X/Y/Z）
struct _ButterWorth2d_Acc_Tag accButter[3] =
{
    {0, 0, 0, 0, 0, 0},  // X轴缓存初始化
    {0, 0, 0, 0, 0, 0},  // Y轴缓存初始化
    {0, 0, 0, 0, 0, 0}   // Z轴缓存初始化
};

// 角速度巴特沃斯滤波缓存（3轴：X/Y/Z，未使用）
struct _ButterWorth2d_Acc_Tag gyroButter[3] =
{
    {0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0}
};

// 加速度六面校准参数（椭球校正）
float K[3]={1.0,1.0,1.0};  // 标度误差系数（默认1.0，校准后更新）
float B[3]={0,0,0};        // 零位误差系数（默认0，校准后更新）

/*****************************************************************************************
* 函数名：ACC_IMU_Filter
* 功能：  用于GPS和气压计融合的加速度数据处理
* 输入：  ax/ay/az - 加速度计原始数字量（X/Y/Z轴）
* 流程：  1. 原始数据1Hz低通滤波，用于六面校准
*         2. 原始数据进行椭球校正（标度+零位补偿）
*         3. 校正后数据30Hz低通滤波（或卡尔曼滤波），用于姿态解算
*****************************************************************************************/
void ACC_IMU_Filter(int16_t ax,int16_t ay,int16_t az)
{
    // 30Hz巴特沃斯滤波器固定参数（200Hz采样率）
    const static float b_acc[3] ={0.1311064399166, 0.2622128798333, 0.1311064399166};
    const static float a_acc[3] ={1, -0.7477891782585, 0.272214937925};
    
    // 滤波后加速度临时数组
    float accelFilter[3];
    uint8_t axis;  // 轴索引（0=X,1=Y,2=Z）

    //------------ 1Hz低通滤波：用于六面加速度校准的原始数据 -------------
    Attitude.accX_correct = (int16_t)(LPButterworth(ax, &Butter_Buffer_Correct[0], &Butter_1HZ_Parameter_Acce));
    Attitude.accY_correct = (int16_t)(LPButterworth(ay, &Butter_Buffer_Correct[1], &Butter_1HZ_Parameter_Acce));
    Attitude.accZ_correct = (int16_t)(LPButterworth(az, &Butter_Buffer_Correct[2], &Butter_1HZ_Parameter_Acce));

    //------------ 椭球校正：标度误差+零位误差补偿 -------------
    // K[轴]：标度系数（补偿传感器灵敏度误差）
    // B[轴]：零位系数（补偿传感器零漂）
    // One_G_TO_Accel：零位误差的单位转换系数
    Attitude.accX_origion = K[0]*ax - B[0]*One_G_TO_Accel;
    Attitude.accY_origion = K[1]*ay - B[1]*One_G_TO_Accel;
    Attitude.accZ_origion = K[2]*az - B[2]*One_G_TO_Accel;

    //------------ 姿态解算用加速度滤波 -------------
#ifndef Butterworth  // 宏定义选择：巴特沃斯滤波（默认）
    // 步骤1：将校正后的加速度存入输入缓存最新位置
    accButter[0].input[2] = (int16_t)(Attitude.accX_origion);
    accButter[1].input[2] = (int16_t)(Attitude.accY_origion);
    accButter[2].input[2] = (int16_t)(Attitude.accZ_origion);

    // 步骤2：遍历3轴执行巴特沃斯滤波
    for (axis = 0; axis < 3; axis++)
    {
        // 二阶巴特沃斯滤波核心计算
        accButter[axis].output[2] = (int16_t)(b_acc[0] * accButter[axis].input[2]
                                             + b_acc[1] * accButter[axis].input[1]
                                             + b_acc[2] * accButter[axis].input[0]
                                             - a_acc[1] * accButter[axis].output[1]
                                             - a_acc[2] * accButter[axis].output[0]);
        // 保存滤波结果
        accelFilter[axis] = accButter[axis].output[2];
    }

    // 步骤3：更新缓存（滑动窗口）
    for (axis = 0; axis < 3; axis++)
    {
        // 输入缓存更新：x(n-2) = x(n-1), x(n-1) = x(n)
        accButter[axis].input[0] = accButter[axis].input[1];
        accButter[axis].input[1] = accButter[axis].input[2];
        // 输出缓存更新：y(n-2) = y(n-1), y(n-1) = y(n)
        accButter[axis].output[0] = accButter[axis].output[1];
        accButter[axis].output[1] = accButter[axis].output[2];
    }

    // 步骤4：将滤波结果存入姿态结构体（用于姿态解算）
    Attitude.accX_IMU = accelFilter[0];
    Attitude.accY_IMU = accelFilter[1];
    Attitude.accZ_IMU = accelFilter[2];

#else  // 宏定义选择：卡尔曼滤波（备选方案）
    // 一维卡尔曼滤波器参数初始化（3轴）
    static struct _1_ekf_filter ekf[3] = {
        {0.02,0,0,0,0.001,0.543},  // X轴：dt=0.02s, Q=0.001, R=0.543
        {0.02,0,0,0,0.001,0.543},  // Y轴
        {0.02,0,0,0,0.001,0.543}   // Z轴
    };

    // 执行卡尔曼滤波
    kalman_1(&ekf[0], Attitude.accX_origion);
    kalman_1(&ekf[1], Attitude.accY_origion);
    kalman_1(&ekf[2], Attitude.accZ_origion);

    // 保存卡尔曼滤波结果
    Attitude.accX_IMU = ekf[0].out;
    Attitude.accY_IMU = ekf[1].out;
    Attitude.accZ_IMU = ekf[2].out;

#endif  // Butterworth宏定义结束
}

/*****************************************************************************************
* 函数名：SINS_Prepare
* 功能：  捷联惯导（SINS）数据预处理
* 流程：  1. 调用加速度控制环滤波函数
*         2. 将机体坐标系加速度投影到导航坐标系（垂直/东西/南北）
*         3. 单位转换并去除重力分量
*         4. 计算加速度模长（动态步长）
*         5. 导航坐标系→机体坐标系加速度投影（控制用）
*****************************************************************************************/
void  SINS_Prepare(void)
{
    // 步骤1：执行加速度控制环滤波
    Acce_Control_Filter();

    /* 坐标变换说明：
       Z-Y-X欧拉角顺规，载体坐标系→导航坐标系旋转矩阵R(b2n)
       R(b2n) = R(Ψ)^T * R(θ)^T * R(Φ)^T （Ψ=Yaw, θ=Pitch, Φ=Roll）
       
       旋转矩阵展开后，加速度投影公式：
       - 垂直海拔方向：acc_yaw_sensor
       - 东西方向（经度）：acc_pitch_sensor
       - 南北方向（纬度）：acc_roll_sensor
    */

    // 步骤2：机体坐标系→导航坐标系加速度投影（GPS/气压计融合用）
    // 垂直海拔方向加速度（气压计三阶融合）
    Attitude.acc_yaw_sensor = -IMU.Sin_Roll * Attitude.accX_control
                            + IMU.Sin_Pitch * IMU.Cos_Roll * Attitude.accY_control
                            + IMU.Cos_Pitch * IMU.Cos_Roll * Attitude.accZ_control;

    // 东西方向加速度（GPS经度融合）
    Attitude.acc_pitch_sensor = IMU.Cos_Yaw * IMU.Cos_Roll * Attitude.accX_control
                              + (IMU.Sin_Pitch*IMU.Sin_Roll*IMU.Cos_Yaw - IMU.Cos_Pitch*IMU.Sin_Yaw) * Attitude.accY_control
                              + (IMU.Sin_Pitch*IMU.Sin_Yaw + IMU.Cos_Pitch*IMU.Sin_Roll*IMU.Cos_Yaw) * Attitude.accZ_control;

    // 南北方向加速度（GPS纬度融合）
    Attitude.acc_roll_sensor = IMU.Sin_Yaw * IMU.Cos_Roll * Attitude.accX_control
                             + (IMU.Sin_Pitch*IMU.Sin_Roll*IMU.Sin_Yaw + IMU.Cos_Pitch*IMU.Cos_Yaw) * Attitude.accY_control
                             + (IMU.Cos_Pitch*IMU.Sin_Roll*IMU.Sin_Yaw - IMU.Sin_Pitch*IMU.Cos_Yaw) * Attitude.accZ_control;

    // 步骤3：单位转换与重力去除
    // 垂直方向：数字量→m/s² → 去除重力 → cm/s²
    Attitude.acc_yaw_sensor *= AcceGravity / AcceMax;
    Attitude.acc_yaw_sensor -= AcceGravity;  // 去除重力加速度
    Attitude.acc_yaw_sensor *= 100;          // 转cm/s²

    // 东西/南北方向：数字量→m/s² → cm/s²（无需去重力）
    Attitude.acc_pitch_sensor *= AcceGravity / AcceMax;
    Attitude.acc_pitch_sensor *= 100;
    Attitude.acc_roll_sensor *= AcceGravity / AcceMax;
    Attitude.acc_roll_sensor *= 100;

    // 步骤4：计算加速度模长（动态步长，表征飞行器运动幅度/速度）
    Attitude.Acceleration_Length = sqrt(Attitude.acc_yaw_sensor*Attitude.acc_yaw_sensor
                                      + Attitude.acc_pitch_sensor*Attitude.acc_pitch_sensor
                                      + Attitude.acc_roll_sensor*Attitude.acc_roll_sensor);

    // 步骤5：导航坐标系→机体坐标系加速度投影（控制用）
    // 地理坐标系加速度赋值（正东/正北）
    Attitude.acc_x_earth = Attitude.acc_pitch_sensor;  // 正东方向 (cm/s²)
    Attitude.acc_y_earth = Attitude.acc_roll_sensor;   // 正北方向 (cm/s²)

    // 地理坐标系→机体坐标系转换（基于偏航角Yaw）
    Attitude.acc_x_body = Attitude.acc_x_earth * IMU.Cos_Yaw + Attitude.acc_y_earth * IMU.Sin_Yaw;  // 横滚方向
    Attitude.acc_y_body = -Attitude.acc_x_earth * IMU.Sin_Yaw + Attitude.acc_y_earth * IMU.Cos_Yaw; // 俯仰方向
}

/*****************************************************************************************
* 模块说明：角速度滤波
* 功能概述：实现角速度信号的巴特沃斯低通滤波，滤除高频噪声，为姿态解算提供稳定的角速度数据
*****************************************************************************************/

// 角速度巴特沃斯滤波器参数（200Hz采样率，30Hz截止频率）
Butter_Parameter Gyro_Parameter={
    1,  -0.7477891782585,    0.272214937925,   // a系数
    0.1311064399166,   0.2622128798333,   0.1311064399166   // b系数
};

// 角速度滤波器缓存（3轴：X/Y/Z）
Butter_BufferData Gyro_BufferData[3];

/**
 * @brief 角速度专用三阶巴特沃斯低通滤波函数（实际为二阶）
 * @param curr_inputer  当前输入的角速度原始值
 * @param Buffer        滤波器缓存结构体指针
 * @param Parameter     滤波器参数结构体指针
 * @return float        滤波后的角速度值
 * @note  无预热阶段，直接滤波（角速度噪声特性更稳定）
 */
float GYRO_LPF(float curr_inputer, Butter_BufferData *Buffer, Butter_Parameter *Parameter)
{
    // 存入当前输入值（x(n)）
    Buffer->Input_Butter[2] = curr_inputer;
    
    // 二阶巴特沃斯滤波核心计算
    Buffer->Output_Butter[2] = Parameter->b[0] * Buffer->Input_Butter[2]
                             + Parameter->b[1] * Buffer->Input_Butter[1]
                             + Parameter->b[2] * Buffer->Input_Butter[0]
                             - Parameter->a[1] * Buffer->Output_Butter[1]
                             - Parameter->a[2] * Buffer->Output_Butter[0];
    
    // 更新输入缓存
    Buffer->Input_Butter[0] = Buffer->Input_Butter[1];
    Buffer->Input_Butter[1] = Buffer->Input_Butter[2];
    
    // 更新输出缓存
    Buffer->Output_Butter[0] = Buffer->Output_Butter[1];
    Buffer->Output_Butter[1] = Buffer->Output_Butter[2];
    
    // 返回滤波结果
    return (Buffer->Output_Butter[2]);
}

/**
 * @brief 角速度滤波入口函数
 * @param gx/gy/gz  角速度计原始数字量（X/Y/Z轴）
 * @功能           对3轴角速度分别进行30Hz低通滤波，结果存入姿态结构体
 */
void GYRO_IMU_Filter(short gx,short gy,short gz)
{
    // X轴角速度滤波
    Attitude.gyroX_IMU = GYRO_LPF(gx, &Gyro_BufferData[0], &Gyro_Parameter);
    // Y轴角速度滤波
    Attitude.gyroY_IMU = GYRO_LPF(gy, &Gyro_BufferData[1], &Gyro_Parameter);
    // Z轴角速度滤波
    Attitude.gyroZ_IMU = GYRO_LPF(gz, &Gyro_BufferData[2], &Gyro_Parameter);
}

