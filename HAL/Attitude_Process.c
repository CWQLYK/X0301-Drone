//	作者：咸
//	电话:18878881386
//	邮箱:3078510877@qq.com
//	日期：2026.3.7
//	版本：1.0
//  陀螺仪滤波模块
#include "ALL_DEFINE.h"
#include <math.h>

// 全局静态数据结构，用于存储姿态解算相关的加速度、角速度等数据
_st_Attitude Attitude;

/*****************************************************************************************
* 模块说明：加速度滤波核心函数
* 功能描述：通过不同截止频率的巴特沃斯低通滤波器，实现加速度信号的低通滤波
*           输出的滤波结果可为姿态解算提供稳定的加速度数据
*****************************************************************************************/

//----- 巴特沃斯滤波器参数结构体声明（外部调用，按需使用）-----//
// Butter_Parameter结构体说明：
// a[0],a[1],a[2]为滤波器分母系数（反馈项）
// b[0],b[1],b[2]为滤波器分子系数（前馈项）
// 滤波公式：y(n) = b0*x(n) + b1*x(n-1) + b2*x(n-2) - a1*y(n-1) - a2*y(n-2)
// 注意a[0]固定为1，为滤波器归一化系数

// 200Hz采样率下：80Hz截止频率的巴特沃斯滤波器参数（加速度专用）
Butter_Parameter Butter_80HZ_Parameter_Acce={
    1,     1.14298050254,   0.4128015980962,  // a系数：a0=1, a1=1.14298, a2=0.41280
    0.638945525159,    1.277891050318,    0.638945525159   // b系数：b0=0.63895, b1=1.27789, b2=0.63895
};

// 200Hz采样率下：60Hz截止频率的巴特沃斯滤波器参数（加速度专用）
Butter_Parameter Butter_60HZ_Parameter_Acce={
    1,   0.3695273773512,   0.1958157126558,
    0.3913357725018,   0.7826715450035,   0.3913357725018
};

// 200Hz采样率下：50Hz截止频率的巴特沃斯滤波器参数（加速度专用）
Butter_Parameter Butter_50HZ_Parameter_Acce={
    1,-1.300707181133e-16,   0.1715728752538,
    0.2065720838261,   0.4131441676523,   0.2065720838261,
};

// 200Hz采样率下：30Hz截止频率的巴特沃斯滤波器参数（加速度专用）
Butter_Parameter Butter_30HZ_Parameter_Acce={
    1,  -0.7477891782585,    0.272214937925,
    0.1311064399166,   0.2622128798333,   0.1311064399166
};

// 200Hz采样率下：20Hz截止频率的巴特沃斯滤波器参数（加速度专用）
Butter_Parameter Butter_20HZ_Parameter_Acce={
    1,    -1.14298050254,   0.4128015980962,
    0.06745527388907,   0.1349105477781,  0.06745527388907
};

// 200Hz采样率下：15Hz截止频率的巴特沃斯滤波器参数（加速度专用）
Butter_Parameter Butter_15HZ_Parameter_Acce={
    1,   -1.348967745253,   0.5139818942197,
    0.04125353724172,  0.08250707448344,  0.04125353724172
};

// 200Hz采样率下：10Hz截止频率的巴特沃斯滤波器参数（加速度专用）
Butter_Parameter Butter_10HZ_Parameter_Acce={
    1,   -1.561018075801,   0.6413515380576,
    0.02008336556421,  0.04016673112842,  0.02008336556421
};

// 200Hz采样率下：5Hz截止频率的巴特沃斯滤波器参数（加速度专用）
Butter_Parameter Butter_5HZ_Parameter_Acce={
    1,   -1.778631777825,   0.8008026466657,
    0.005542717210281,  0.01108543442056, 0.005542717210281
};

// 200Hz采样率下：2Hz截止频率的巴特沃斯滤波器参数（加速度专用）
Butter_Parameter Butter_2HZ_Parameter_Acce={
    1,   -1.911197067426,   0.9149758348014,
    0.0009446918438402,  0.00188938368768,0.0009446918438402
};

// 200Hz采样率下：1Hz截止频率的巴特沃斯滤波器参数（加速度专用）
Butter_Parameter Butter_1HZ_Parameter_Acce={
    1,   -1.955578240315,   0.9565436765112,
    0.000241359049042, 0.000482718098084, 0.000241359049042
};

/**
 * @brief 低通巴特沃斯滤波器实现函数
 * @param curr_input  当前输入的原始数据（浮点型数值）
 * @param Buffer      滤波器缓存结构体指针（存储历史输入/输出值）
 * @param Parameter   滤波器参数结构体指针（a/b系数）
 * @return float      滤波器输出值
 * @note  1. 前100次调用使用直通模式，避免滤波器初始阶段的瞬态响应
 *        2. 采用IIR结构，需要保存前两次的输入和输出值
 */
float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter)
{
    // 滤波器初始化计数器：前100次调用直通输入
    static int LPB_Cnt=0;
    
    // 保存当前输入值到缓存最新位置，x(n)
    Buffer->Input_Butter[2] = curr_input;
    
    // 前100次调用直通模式（滤波器预热，避免初始值为0导致的突变）
    if(LPB_Cnt >= 100)
    {
        // 执行巴特沃斯滤波器核心计算
        Buffer->Output_Butter[2] = Parameter->b[0] * Buffer->Input_Butter[2]  // b0*x(n)
                                 + Parameter->b[1] * Buffer->Input_Butter[1]  // b1*x(n-1)
                                 + Parameter->b[2] * Buffer->Input_Butter[0]  // b2*x(n-2)
                                 - Parameter->a[1] * Buffer->Output_Butter[1] // -a1*y(n-1)
                                 - Parameter->a[2] * Buffer->Output_Butter[0];// -a2*y(n-2)
    }
    else
    {
        // 预热阶段，输出等于输入
        Buffer->Output_Butter[2] = Buffer->Input_Butter[2];
        LPB_Cnt++;
    }
    
    // 输入缓存移位：x(n-2) = x(n-1), x(n-1) = x(n)
    Buffer->Input_Butter[0] = Buffer->Input_Butter[1];
    Buffer->Input_Butter[1] = Buffer->Input_Butter[2];
    
    // 输出缓存移位：y(n-2) = y(n-1), y(n-1) = y(n)
    Buffer->Output_Butter[0] = Buffer->Output_Butter[1];
    Buffer->Output_Butter[1] = Buffer->Output_Butter[2];

    // 返回滤波器输出
    return Buffer->Output_Butter[2];
}

// 加速度校准专用滤波器缓存（3轴：X/Y/Z）
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

// 加速度计相关常量定义
#define GRAVITY_MSS     9.80665f       // 标准重力加速度 (m/s²)
#define AcceMax_1G      4096.0f        // 1G对应的加速度计量化值（根据硬件配置）
#define One_G_TO_Accel  AcceMax_1G/GRAVITY_MSS  // 计数值转实际加速度的系数 (count/(m/s²))
#define AcceMax         4096.0f        // 加速度计满量程对应计数值
#define AcceGravity     9.80f          // 实用重力加速度（简化计算用）

// 加速度控制环临时变量
float Acce_Control[3]={0};

/*****************************************************************************************
* 函数名：Acce_Control_Filter
* 功能：  用于PID控制环的加速度数据处理
* 流程：  1. 对原始加速度进行30Hz低通滤波，得到控制用加速度
*         2. 对原始加速度进行5Hz低通滤波，得到反馈用加速度
*         3. 计算机体坐标系加速度在地理坐标系的投影，得到垂向/俯仰/横滚方向的加速度
*         4. 单位转换：将计数值转为实际加速度（cm/s²），并去除重力分量
*****************************************************************************************/
void Acce_Control_Filter(void)
{
    // 加速度反馈环临时变量（5Hz滤波后）
    float Acce_Control_Feedback[3]={0};

    /********************** 控制用加速度（30Hz低通滤波） **************************/ 
    // X轴控制用加速度（30Hz滤波）
    Attitude.accX_control = LPButterworth(Attitude.accX_origion, &Butter_Buffer[0], &Butter_30HZ_Parameter_Acce);
    // Y轴控制用加速度（30Hz滤波）
    Attitude.accY_control = LPButterworth(Attitude.accY_origion, &Butter_Buffer[1], &Butter_30HZ_Parameter_Acce);
    // Z轴控制用加速度（30Hz滤波）
    Attitude.accZ_control = LPButterworth(Attitude.accZ_origion, &Butter_Buffer[2], &Butter_30HZ_Parameter_Acce);

    /********************** 反馈用加速度（5Hz低通滤波） ***************************/
    // X轴反馈用加速度（5Hz滤波）
    Acce_Control_Feedback[0] = LPButterworth(Attitude.accX_origion, &Butter_Buffer_Feedback[0], &Butter_5HZ_Parameter_Acce);
    // Y轴反馈用加速度（5Hz滤波）
    Acce_Control_Feedback[1] = LPButterworth(Attitude.accY_origion, &Butter_Buffer_Feedback[1], &Butter_5HZ_Parameter_Acce);
    // Z轴反馈用加速度（5Hz滤波）
    Acce_Control_Feedback[2] = LPButterworth(Attitude.accZ_origion, &Butter_Buffer_Feedback[2], &Butter_5HZ_Parameter_Acce);

    // 计算地理坐标系的垂向加速度（Roll/Pitch/Yaw为姿态角，仅提取垂向分量）
    Attitude.acc_high_feedback = -IMU.Sin_Roll * Acce_Control_Feedback[0]
                               + IMU.Sin_Pitch * IMU.Cos_Roll * Acce_Control_Feedback[1]
                               + IMU.Cos_Pitch * IMU.Cos_Roll * Acce_Control_Feedback[2];

    // 预留：俯仰/横滚方向加速度投影（为GPS融合预留，当前注释）
    // Attitude.acc_pitch_feedback = ...; // 俯仰方向加速度
    // Attitude.acc_roll_feedback = ...;  // 横滚方向加速度

    // 单位转换1：计数值 -> 实际加速度（m/s²）
    Attitude.acc_high_feedback *= AcceGravity / AcceMax;
    // 去除重力加速度（仅保留运动产生的垂向加速度）
    Attitude.acc_high_feedback -= AcceGravity;
    // 单位转换2：m/s² -> cm/s²（控制环常用单位）
    Attitude.acc_high_feedback *= 100;

    // 预留：俯仰/横滚加速度的单位转换（未启用）
    Attitude.acc_pitch_feedback *= AcceGravity / AcceMax;
    Attitude.acc_pitch_feedback *= 100;
    Attitude.acc_roll_feedback *= AcceGravity / AcceMax;
    Attitude.acc_roll_feedback *= 100;
}

// 加速度备用巴特沃斯滤波结构体（自定义实现，3轴）
struct _ButterWorth2d_Acc_Tag
{
    int16_t input[3];   // 输入缓存：input[0]=x(n-2), input[1]=x(n-1), input[2]=x(n)
    int16_t output[3];  // 输出缓存：output[0]=y(n-2), output[1]=y(n-1), output[2]=y(n)
};

// 加速度滤波缓存（3轴：X/Y/Z）
struct _ButterWorth2d_Acc_Tag accButter[3] =
{
    {0, 0, 0, 0, 0, 0},  // X轴初始值
    {0, 0, 0, 0, 0, 0},  // Y轴初始值
    {0, 0, 0, 0, 0, 0}   // Z轴初始值
};

// 角速度备用巴特沃斯滤波缓存（3轴：X/Y/Z，未使用）
struct _ButterWorth2d_Acc_Tag gyroButter[3] =
{
    {0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0}
};

// 加速度计校准参数（比例+偏置）
float K[3]={1.0,1.0,1.0};  // 比例系数（默认1.0，校准后修正）
float B[3]={0,0,0};        // 偏置系数（默认0，校准后修正）

/*****************************************************************************************
* 函数名：ACC_IMU_Filter
* 功能：  为GPS/气压计融合提供的加速度数据处理
* 输入：  ax/ay/az - 加速度计原始计数值（X/Y/Z轴）
* 流程：  1. 原始数据1Hz低通滤波，用于加速度计校准
*         2. 原始数据进行校准（比例+偏置修正）
*         3. 校准后数据30Hz低通滤波，备用滤波实现（姿态解算用）
*****************************************************************************************/
void ACC_IMU_Filter(int16_t ax,int16_t ay,int16_t az)
{
    // 30Hz巴特沃斯滤波器固定参数（200Hz采样率）
    const static float b_acc[3] ={0.1311064399166, 0.2622128798333, 0.1311064399166};
    const static float a_acc[3] ={1, -0.7477891782585, 0.272214937925};
    
    // 滤波后加速度临时变量
    float accelFilter[3];
    uint8_t axis;  // 轴索引：0=X,1=Y,2=Z

    //------------ 1Hz低通滤波（加速度计校准用原始数据） -------------
    Attitude.accX_correct = (int16_t)(LPButterworth(ax, &Butter_Buffer_Correct[0], &Butter_1HZ_Parameter_Acce));
    Attitude.accY_correct = (int16_t)(LPButterworth(ay, &Butter_Buffer_Correct[1], &Butter_1HZ_Parameter_Acce));
    Attitude.accZ_correct = (int16_t)(LPButterworth(az, &Butter_Buffer_Correct[2], &Butter_1HZ_Parameter_Acce));

    //------------ 加速度计校准（比例+偏置修正） -------------
    // K[轴]：比例系数（修正传感器增益误差）
    // B[轴]：偏置系数（修正传感器零漂）
    // One_G_TO_Accel：单位转换系数
    Attitude.accX_origion = K[0]*ax - B[0]*One_G_TO_Accel;
    Attitude.accY_origion = K[1]*ay - B[1]*One_G_TO_Accel;
    Attitude.accZ_origion = K[2]*az - B[2]*One_G_TO_Accel;

    //------------ 姿态解算用加速度滤波 -------------
#ifndef Butterworth  // 宏定义选择：默认使用巴特沃斯滤波
    // 步骤1：将校准后的加速度存入输入缓存（最新位置）
    accButter[0].input[2] = (int16_t)(Attitude.accX_origion);
    accButter[1].input[2] = (int16_t)(Attitude.accY_origion);
    accButter[2].input[2] = (int16_t)(Attitude.accZ_origion);

    // 步骤2：对3轴依次执行巴特沃斯滤波
    for (axis = 0; axis < 3; axis++)
    {
        // 巴特沃斯滤波器核心计算
        accButter[axis].output[2] = (int16_t)(b_acc[0] * accButter[axis].input[2]
                                             + b_acc[1] * accButter[axis].input[1]
                                             + b_acc[2] * accButter[axis].input[0]
                                             - a_acc[1] * accButter[axis].output[1]
                                             - a_acc[2] * accButter[axis].output[0]);
        // 保存滤波结果
        accelFilter[axis] = accButter[axis].output[2];
    }

    // 步骤3：缓存移位（更新历史值）
    for (axis = 0; axis < 3; axis++)
    {
        // 输入缓存移位：x(n-2) = x(n-1), x(n-1) = x(n)
        accButter[axis].input[0] = accButter[axis].input[1];
        accButter[axis].input[1] = accButter[axis].input[2];
        // 输出缓存移位：y(n-2) = y(n-1), y(n-1) = y(n)
        accButter[axis].output[0] = accButter[axis].output[1];
        accButter[axis].output[1] = accButter[axis].output[2];
    }

    // 步骤4：将滤波结果存入姿态结构体（姿态解算用）
    Attitude.accX_IMU = accelFilter[0];
    Attitude.accY_IMU = accelFilter[1];
    Attitude.accZ_IMU = accelFilter[2];

#else  // 宏定义选择：滤波方式切换为卡尔曼滤波
    // 一维卡尔曼滤波器初始化（3轴）
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
* 功能：  为捷联惯导（SINS）解算做准备
* 流程：  1. 执行加速度控制环滤波
*         2. 计算机体坐标系加速度在地理坐标系的投影（垂向/俯仰/横滚）
*         3. 单位转换并去除重力分量
*         4. 计算加速度模长（运动状态判断：静止/运动）
*         5. 地理坐标系加速度转换到机体坐标系（补偿Yaw角）
*****************************************************************************************/
void  SINS_Prepare(void)
{
    // 步骤1：执行加速度控制环滤波
    Acce_Control_Filter();

    /* 坐标变换说明：
       Z-Y-X旋转顺序（航偏-俯仰-横滚），机体坐标系到地理坐标系的转换矩阵R(b2n)
       R(b2n) = R(ψ)^T * R(θ)^T * R(φ)^T （ψ=Yaw, θ=Pitch, φ=Roll）
       
       矩阵展开后，加速度投影公式：
       - 垂向加速度（航偏方向）：acc_yaw_sensor
       - 俯仰加速度（θ角）：acc_pitch_sensor
       - 横滚加速度（φ角）：acc_roll_sensor
    */

    // 步骤2：机体坐标系加速度投影到地理坐标系（GPS/气压计融合用）
    // 垂向加速度（航偏方向，气压计融合用）
    Attitude.acc_yaw_sensor = -IMU.Sin_Roll * Attitude.accX_control
                            + IMU.Sin_Pitch * IMU.Cos_Roll * Attitude.accY_control
                            + IMU.Cos_Pitch * IMU.Cos_Roll * Attitude.accZ_control;

    // 俯仰方向加速度（GPS融合用）
    Attitude.acc_pitch_sensor = IMU.Cos_Yaw * IMU.Cos_Roll * Attitude.accX_control
                              + (IMU.Sin_Pitch*IMU.Sin_Roll*IMU.Cos_Yaw - IMU.Cos_Pitch*IMU.Sin_Yaw) * Attitude.accY_control
                              + (IMU.Sin_Pitch*IMU.Sin_Yaw + IMU.Cos_Pitch*IMU.Sin_Roll*IMU.Cos_Yaw) * Attitude.accZ_control;

    // 横滚方向加速度（GPSφ角融合用）
    Attitude.acc_roll_sensor = IMU.Sin_Yaw * IMU.Cos_Roll * Attitude.accX_control
                             + (IMU.Sin_Pitch*IMU.Sin_Roll*IMU.Sin_Yaw + IMU.Cos_Pitch*IMU.Cos_Yaw) * Attitude.accY_control
                             + (IMU.Cos_Pitch*IMU.Sin_Roll*IMU.Sin_Yaw - IMU.Sin_Pitch*IMU.Cos_Yaw) * Attitude.accZ_control;

    // 步骤3：单位转换并去除重力分量
    // 垂向加速度：计数值→m/s² → 去重力 → cm/s²
    Attitude.acc_yaw_sensor *= AcceGravity / AcceMax;
    Attitude.acc_yaw_sensor -= AcceGravity;  // 去除重力加速度
    Attitude.acc_yaw_sensor *= 100;          // 转cm/s²

    // 俯仰/横滚加速度：计数值→m/s² → cm/s²（不去重力）
    Attitude.acc_pitch_sensor *= AcceGravity / AcceMax;
    Attitude.acc_pitch_sensor *= 100;
    Attitude.acc_roll_sensor *= AcceGravity / AcceMax;
    Attitude.acc_roll_sensor *= 100;

    // 步骤4：计算加速度模长（运动状态判断：静止/运动）
    Attitude.Acceleration_Length = sqrt(Attitude.acc_yaw_sensor*Attitude.acc_yaw_sensor
                                      + Attitude.acc_pitch_sensor*Attitude.acc_pitch_sensor
                                      + Attitude.acc_roll_sensor*Attitude.acc_roll_sensor);

    // 步骤5：地理坐标系加速度转换到机体坐标系（补偿Yaw角）
    // 地理坐标系东向加速度（cm/s²）
    Attitude.acc_x_earth = Attitude.acc_pitch_sensor;  
    // 地理坐标系北向加速度（cm/s²）
    Attitude.acc_y_earth = Attitude.acc_roll_sensor;   

    // 机体坐标系加速度（补偿Yaw角旋转）
    Attitude.acc_x_body = Attitude.acc_x_earth * IMU.Cos_Yaw + Attitude.acc_y_earth * IMU.Sin_Yaw;  // 机体东向
    Attitude.acc_y_body = -Attitude.acc_x_earth * IMU.Sin_Yaw + Attitude.acc_y_earth * IMU.Cos_Yaw; // 机体北向
}

/*****************************************************************************************
* 模块说明：角速度滤波
* 功能描述：实现角速度信号的巴特沃斯低通滤波，输出的滤波结果可为姿态解算提供稳定的角速度数据
*****************************************************************************************/

// 角速度巴特沃斯滤波器参数（200Hz采样率，30Hz截止频率）
Butter_Parameter Gyro_Parameter={
    1,  -0.7477891782585,    0.272214937925,   // a系数
    0.1311064399166,   0.2622128798333,   0.1311064399166   // b系数
};

// 角速度滤波缓存（3轴：X/Y/Z）
Butter_BufferData Gyro_BufferData[3];

/**
 * @brief 角速度专用的优化版巴特沃斯低通滤波函数（无预热）
 * @param curr_inputer  当前输入的原始角速度值
 * @param Buffer        滤波器缓存结构体指针
 * @param Parameter     滤波器参数结构体指针
 * @return float        滤波后的角速度值
 * @note  无预热阶段，直接执行滤波（角速度传感器初始值稳定）
 */
float GYRO_LPF(float curr_inputer, Butter_BufferData *Buffer, Butter_Parameter *Parameter)
{
    // 保存当前输入值到x(n)
    Buffer->Input_Butter[2] = curr_inputer;
    
    // 巴特沃斯滤波器核心计算
    Buffer->Output_Butter[2] = Parameter->b[0] * Buffer->Input_Butter[2]
                             + Parameter->b[1] * Buffer->Input_Butter[1]
                             + Parameter->b[2] * Buffer->Input_Butter[0]
                             - Parameter->a[1] * Buffer->Output_Butter[1]
                             - Parameter->a[2] * Buffer->Output_Butter[0];
    
    // 输入缓存移位
    Buffer->Input_Butter[0] = Buffer->Input_Butter[1];
    Buffer->Input_Butter[1] = Buffer->Input_Butter[2];
    
    // 输出缓存移位
    Buffer->Output_Butter[0] = Buffer->Output_Butter[1];
    Buffer->Output_Butter[1] = Buffer->Output_Butter[2];
    
    // 返回滤波结果
    return (Buffer->Output_Butter[2]);
}

/**
 * @brief 角速度滤波入口函数
 * @param gx/gy/gz  角速度计原始计数值（X/Y/Z轴）
 * @功能           对3轴角速度分别进行30Hz低通滤波，存入姿态结构体
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


