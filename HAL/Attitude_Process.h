#ifndef __ATTITUDE__H_
#define __ATTITUDE__H_

// 姿态数据结构体定义
typedef struct 
{
    // 滤波后用于姿态解算的角速度(rad/s   dps)
    float gyroX_IMU;
    float gyroY_IMU;
    float gyroZ_IMU;

    // 滤波后用于六面校准的加速度值（数字量）
    float accX_correct;
    float accY_correct;
    float accZ_correct;

    // 经过椭球校正后的三轴加速度量（数字量，已补偿标度/零位误差）
    float accX_origion;
    float accY_origion;
    float accZ_origion;

    // 滤波后用于控制/传感器融合的加速度（数字量）
    float accX_control;
    float accY_control;
    float accZ_control;

    // 导航坐标系下的加速度（cm/s²，用于GPS/气压计融合）
    float acc_yaw_sensor;    // 垂直海拔方向
    float acc_pitch_sensor;  // 东西方向（经度）
    float acc_roll_sensor;   // 南北方向（纬度）

    // 反馈用加速度（cm/s²，用于高度控制PID）
    float acc_high_feedback; // 垂直方向加速度
    float acc_pitch_feedback;// 东西方向加速度
    float acc_roll_feedback; // 南北方向加速度

    // 用于姿态解算的加速度（数字量，滤波后）
    float accX_IMU;
    float accY_IMU;
    float accZ_IMU;

    // 加速度模长（cm/s²，表征飞行器运动幅度）
    float Acceleration_Length;

    // 地理坐标系下的加速度（cm/s²）
    float acc_x_earth; // 正东方向
    float acc_y_earth; // 正北方向

    // 机体坐标系下的加速度（cm/s²）
    float acc_x_body;  // 横滚方向（X轴）
    float acc_y_body;  // 俯仰方向（Y轴）

    // 预留观测用加速度
    float acc_yaw;
    float acc_pitch;
    float acc_roll;
}_st_Attitude;

// 加速度六面校准椭球矫正参数（外部可访问）
extern float K[3];  // 标度误差系数
extern float B[3];  // 零位误差系数

// 全局姿态结构体实例（外部可访问）
extern _st_Attitude  Attitude;

// 函数声明（外部可调用）
extern void GYRO_IMU_Filter(short ax,short ay,short az); // 角速度滤波
extern void ACC_IMU_Filter(short ax,short ay,short az);   // 加速度滤波
extern void  SINS_Prepare(void);     

#endif
