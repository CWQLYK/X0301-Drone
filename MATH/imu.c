// 作者：咸
// 电话:18878881386
// 邮箱:3078510877@qq.com
// 日期:2026.3.8
// 版本:1.0
//  MPU6050动态姿态解算算法（四元数+PI修正）

#include "imu.h"
#include "myMath.h"
#include <math.h>

// 全局变量：机体Z轴方向的合加速度（用于垂直方向运动检测）
static float NormAccz;

/**
 * @brief 四元数结构体定义
 * @note  四元数形式：q0(实部) + q1*i + q2*j + q3*k，用于表示三维空间旋转
 */
typedef volatile struct {
  float q0;  // 四元数实部
  float q1;  // 四元数i分量（X轴）
  float q2;  // 四元数j分量（Y轴）
  float q3;  // 四元数k分量（Z轴）
} Quaternion;

/**
 * @brief  基于四元数的动态姿态解算主函数
 * @param  pMpu: 指向MPU6050原始数据结构体的指针（包含加速度、陀螺仪原始值）
 * @param  pAngE: 指向动态角度结构体的指针（存储解算后的欧拉角/角度偏差）
 * @param  dt: 两次解算的时间间隔（单位：秒）
 * @retval 无
 * @note   1. 算法流程：四元数初始化→加速度归一化→计算误差→PI修正陀螺仪→更新四元数→转欧拉角
 *         2. 通过PI控制器修正陀螺仪积分误差，利用加速度计重力分量进行静态校准
 *         3. 动态解算的角度（度）= 陀螺仪原始值转换为角度速率后积分 + PI修正量
 */
void GetAngle(const _st_Mpu *pMpu, _st_AngE *pAngE, float dt) 
{		
    /**
     * @brief 三维向量结构体（用于表示加速度、角速度、重力分量等）
     */
    volatile struct V {
        float x;  // X轴分量
        float y;  // Y轴分量
        float z;  // Z轴分量
    } Gravity, Acc, Gyro, AccGravity;

    static struct V GyroIntegError = {0};  // 陀螺仪积分误差（PI控制器的积分项）
    static float KpDef = 0.8f;             // 比例修正系数（快速修正加速度计误差）
    static float KiDef = 0.0003f;          // 积分修正系数（消除静态角度偏差）
    static Quaternion NumQ = {1, 0, 0, 0}; // 四元数初始值（无旋转状态）
    float q0_t, q1_t, q2_t, q3_t;          // 四元数一阶微分中间变量
    float NormQuat;                        // 归一化系数（用于四元数/向量归一化）
    float HalfTime = dt * 0.5f;            // 时间间隔的一半（四元数微分用）

    // ====================== 步骤1：从四元数提取机体坐标系的重力向量 ======================
    // 四元数转换为旋转矩阵，提取Z轴重力分量（机体坐标系下的重力方向）
    Gravity.x = 2*(NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);								
    Gravity.y = 2*(NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);						  
    Gravity.z = 1 - 2*(NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);	
	
    // ====================== 步骤2：加速度计数据归一化 ======================
    // 计算加速度计原始数据的模长倒数（快速平方根代替1/sqrt()）
    NormQuat = Q_rsqrt(squa(MPU6050.accX) + squa(MPU6050.accY) + squa(MPU6050.accZ));
    // 加速度归一化（消除重力幅值影响，得到单位向量）
    Acc.x = pMpu->accX * NormQuat;
    Acc.y = pMpu->accY * NormQuat;
    Acc.z = pMpu->accZ * NormQuat;	
	
    // ====================== 步骤3：计算加速度计与重力向量的误差（叉乘） ======================
    // 加速度计实测重力向量 与 四元数推导重力向量 的叉乘（表征姿态偏差）
    // 叉乘结果越大，说明当前姿态与实际重力方向偏差越大
    AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
    AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
    AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);
	
    // ====================== 步骤4：PI修正陀螺仪消除积分漂移 ======================
    // 积分项：累积误差，消除静态角度偏差
    GyroIntegError.x += AccGravity.x * KiDef;
    GyroIntegError.y += AccGravity.y * KiDef;
    GyroIntegError.z += AccGravity.z * KiDef;
    
    // 比例项+积分项修正陀螺仪原始值，得到无漂移的角速度（弧度/秒）
    // Gyro_Gr：陀螺仪原始值转弧度/秒的系数（MPU6050量程相关）
    Gyro.x = pMpu->gyroX * Gyro_Gr + KpDef * AccGravity.x + GyroIntegError.x;
    Gyro.y = pMpu->gyroY * Gyro_Gr + KpDef * AccGravity.y + GyroIntegError.y;
    Gyro.z = pMpu->gyroZ * Gyro_Gr + KpDef * AccGravity.z + GyroIntegError.z;		
	
    // ====================== 步骤5：一阶积分更新四元数 ======================
    // 四元数微分方程：dQ/dt = 0.5 * Q ⊗ Ω（Ω为角速度向量）
    q0_t = (-NumQ.q1*Gyro.x - NumQ.q2*Gyro.y - NumQ.q3*Gyro.z) * HalfTime;
    q1_t = ( NumQ.q0*Gyro.x - NumQ.q3*Gyro.y + NumQ.q2*Gyro.z) * HalfTime;
    q2_t = ( NumQ.q3*Gyro.x + NumQ.q0*Gyro.y - NumQ.q1*Gyro.z) * HalfTime;
    q3_t = (-NumQ.q2*Gyro.x + NumQ.q1*Gyro.y + NumQ.q0*Gyro.z) * HalfTime;
    
    // 积分更新四元数
    NumQ.q0 += q0_t;
    NumQ.q1 += q1_t;
    NumQ.q2 += q2_t;
    NumQ.q3 += q3_t;
	
    // ====================== 步骤6：四元数归一化，防止数值漂移 ======================
    NormQuat = Q_rsqrt(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3));
    NumQ.q0 *= NormQuat;
    NumQ.q1 *= NormQuat;
    NumQ.q2 *= NormQuat;
    NumQ.q3 *= NormQuat;	
	
    // ====================== 步骤7：四元数转换为动态角度（横滚/俯仰/偏航） ======================
    {
        // 提取旋转矩阵第三行（机体Z轴在世界坐标系的投影）
        float vecxZ = 2 * NumQ.q0 * NumQ.q2 - 2 * NumQ.q1 * NumQ.q3;  // 旋转矩阵(3,1)项
        float vecyZ = 2 * NumQ.q2 * NumQ.q3 + 2 * NumQ.q0 * NumQ.q1;  // 旋转矩阵(3,2)项
        float veczZ = 1 - 2 * NumQ.q1 * NumQ.q1 - 2 * NumQ.q2 * NumQ.q2; // 旋转矩阵(3,3)项		
		
        #ifdef	YAW_GYRO
        // 模式1：通过四元数直接计算偏航角（yaw）
        *(float *)pAngE = atan2f(2 * NumQ.q1 * NumQ.q2 + 2 * NumQ.q0 * NumQ.q3, 
                                 1 - 2 * NumQ.q2 * NumQ.q2 - 2 * NumQ.q3 * NumQ.q3) * RtA;
        #else
        // 模式2：通过Z轴陀螺仪积分计算偏航角（避免加速度计无法修正偏航漂移）
        float yaw_G = pMpu->gyroZ * Gyro_G;  // Z轴角速度值转换为度/秒
        // 低通阈值：仅当角速度大于1度/秒时积分（过滤静态噪声）
        if((yaw_G > 1.0f) || (yaw_G < -1.0f))
        {
            pAngE->yaw += yaw_G * dt;  // 角速度积分得到偏航角
        }
        #endif

        // 计算俯仰角（pitch）：asin(旋转矩阵(3,1)项) 转换为度
        pAngE->pitch = asin(vecxZ) * RtA;					
        // 计算横滚角（roll）：atan2(旋转矩阵(3,2), 旋转矩阵(3,3)) 转换为度
        pAngE->roll = atan2f(vecyZ, veczZ) * RtA;

        // 计算机体Z轴方向的合加速度（用于垂直方向运动检测）
        NormAccz = pMpu->accX * vecxZ + pMpu->accY * vecyZ + pMpu->accZ * veczZ;				
    }
}

/**
 * @brief  获取机体Z轴方向的合加速度值
 * @param  无
 * @retval NormAccz: 机体Z轴方向的合加速度（用于垂直运动检测）
 * @note   该值直接反映无人机在垂直方向的运动加速度，可用于高度控制补偿
 */
float GetAccz(void)
{
    return NormAccz;
}

