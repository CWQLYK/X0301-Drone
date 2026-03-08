#ifndef __IMU_H
#define	__IMU_H

#include "ALL_DATA.h"

/**
 * @brief 角度单位转换宏：角度转弧度 (π/180)
 */
#define DEG2RAD  0.017453293f
/**
 * @brief 角度单位转换宏：弧度转角度 (180/π)
 */
#define RAD2DEG  57.29578f   

/**
 * @brief IMU姿态数据结构体
 * @note 存储解算后的姿态角、姿态角的三角函数值及磁力计修正偏航角
 */
typedef struct {
    float yaw;          // 偏航角(绕Z轴)，单位：度
    float pitch;        // 俯仰角(绕Y轴)，单位：度
    float roll;         // 横滚角(绕X轴)，单位：度
    float yaw_mag;      // 磁力计修正后的偏航角，单位：度
    float Cos_Roll;     // 横滚角的余弦值
    float Sin_Roll;     // 横滚角的正弦值
    float Cos_Pitch;    // 俯仰角的余弦值
    float Sin_Pitch;    // 俯仰角的正弦值
    float Cos_Yaw;      // 偏航角的余弦值
    float Sin_Yaw;      // 偏航角的正弦值
}_st_IMU;

/**
 * @brief 三维向量数据结构体
 * @note 用于存储加速度、角速度、重力向量等三维数据
 */
typedef struct V{
    float x;            // X轴分量
    float y;            // Y轴分量
    float z;            // Z轴分量
} MPUDA;

/**
 * @brief 偏航角控制量（外部全局变量）
 */
extern float yaw_control;
/**
 * @brief 偏航角修正量（外部全局变量）
 */
extern float Yaw_Correct;
/**
 * @brief IMU姿态数据全局实例（外部全局变量）
 */
extern _st_IMU IMU;
/**
 * @brief 三维向量全局实例（外部全局变量）
 * @note 分别存储重力向量、加速度向量、角速度向量、加速度重力误差向量
 */
extern MPUDA  Gravity, Acc, Gyro, AccGravity;

/**
 * @brief 获取Z轴坐标系方向的合加速度值
 * @retval float Z轴坐标系方向的合加速度（归一化后）
 * @note 该值映射为载体在坐标系垂直方向的运动加速度，可用于运动状态判断
 */
extern float GetAccz(void);

/**
 * @brief 基于四元数的姿态解算核心函数（互补滤波融合加速度计+陀螺仪）
 * @param pMpu 指向MPU6050原始数据结构体的指针（包含加速度、陀螺仪原始值）
 * @param pAngE 指向姿态角输出结构体的指针（存储解算后的yaw/pitch/roll）
 * @param dt 采样时间间隔，单位：秒（用于积分计算）
 * @retval 无
 * @note 1. 算法核心：四元数初始化后，通过加速度计修正陀螺仪漂移，更新四元数
 *       2. 输出角度范围：俯仰角/横滚角为±90°，偏航角为0~360°（未磁力计修正时为相对角度）
 *       3. 依赖MPU6050原始数据的有效性，需保证传感器已完成初始化
 */
extern void GetAngle(const _st_Mpu *pMpu,_st_AngE *pAngE, float dt);

#endif