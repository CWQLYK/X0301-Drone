//	作者：咸
//	电话:18878881386
//	邮箱:3078510877@qq.com
//	日期：2026.3.6
//	版本：1.0
//  pid计算相关函数
#include "ALL_DEFINE.h"

/**
 * @brief  批量复位PID控制器状态
 * @param  pid  指向PID控制器指针数组的首地址（多个PID的管理入口）
 * @param  len  PID控制器的数量（数组长度）
 * @note   复位PID的核心状态变量，用于系统启动/模式切换/异常恢复时重置PID，
 *         避免历史状态（积分、误差、输出）影响新的控制过程
 */
void pidRest(PidObject **pid, const uint8_t len)
{
    uint8_t i;  // 循环索引
    
    // 遍历所有PID控制器，逐一复位状态
    for(i = 0; i < len; i++)
    {
        pid[i]->integ = 0;        // 清零积分累加值（消除积分饱和）
        pid[i]->prevError = 0;    // 清零上次误差（微分项初始化为0）
        pid[i]->out = 0;          // 清零PID基础输出（避免初始输出突变）
        pid[i]->offset = 0;       // 清零偏移补偿值（复位校准参数）
    }
}

/**
 * @brief  单PID控制器核心计算函数（标准位置式PID）
 * @param  pid  指向单个PID控制器结构体的指针
 * @param  dt   本次计算的时间步长（s，通常为控制周期，如0.005s=5ms）
 * @note   1. 计算偏差→积分累加→微分计算→PID输出→更新历史误差
 *         2. 积分限幅、输出限幅功能已注释，可根据需求启用
 *         3. 适用于飞控的角速度环、角度环、高度环等所有单环PID控制
 */
void pidUpdate(PidObject* pid, const float dt)
{
    float error;   // 当前偏差（期望 - 反馈）
    float deriv;   // 微分量（偏差的变化率）
    
    // 步骤1：计算当前控制偏差（期望目标值 - 实际测量值）
    // 例：角度环中，desired=目标俯仰角，measured=实际俯仰角
    error = pid->desired - pid->measured;

    // 步骤2：积分项累加（误差×时间步长，积分是误差对时间的累积）
    // 积分项作用：消除静态误差，让系统稳定在目标值
    pid->integ += error * dt;	
	
    // 【可选】积分限幅（注释待启用）：防止积分项过大导致的积分饱和，
    // 避免控制器输出超出执行器能力（如电机最大转速）
    // pid->integ = LIMIT(pid->integ, pid->IntegLimitLow, pid->IntegLimitHigh);

    // 步骤3：微分项计算（当前误差 - 上次误差）/时间步长
    // 微分项作用：预测误差变化趋势，抑制超调，提升系统响应速度
    deriv = (error - pid->prevError) / dt;  

    // 步骤4：PID输出计算（比例+积分+微分）
    // kp*error：比例项，快速响应偏差；ki*integ：积分项，消除静差；kd*deriv：微分项，抑制超调
    pid->out = pid->kp * error + pid->ki * pid->integ + pid->kd * deriv;
	
    // 【可选】输出限幅（注释待启用）：限制PID输出范围，保护执行器（如电机、舵机）
    // pid->out = LIMIT(pid->out, pid->OutLimitLow, pid->OutLimitHigh);
		
    // 步骤5：更新历史误差（保存当前误差，用于下次微分项计算）
    pid->prevError = error;
}

/**
 * @brief  串级PID控制器计算函数（外环+内环）
 * @param  pidRate  内环PID控制器指针（通常为角速度环，如俯仰角速度）
 * @param  pidAngE  外环PID控制器指针（通常为角度环，如俯仰角）
 * @param  dt       控制周期时间步长（s）
 * @note   1. 串级PID是飞控姿态控制的核心架构：外环（角度）输出作为内环（角速度）的输入
 *         2. 执行顺序：先算外环角度PID→将角度PID输出作为角速度期望→再算内环角速度PID
 *         3. 优势：内环快速抑制扰动，外环保证稳态精度，提升系统抗干扰能力
 */
void CascadePID(PidObject* pidRate, PidObject* pidAngE, const float dt)
{	 
    // 第一步：计算外环（角度环）PID
    // 例：pidAngE为俯仰角PID，输出是目标俯仰角速度
    pidUpdate(pidAngE, dt);
    
    // 第二步：将外环输出作为内环的期望目标值
    // 例：角度环输出→角速度环的desired，实现角度→角速度的串级控制
    pidRate->desired = pidAngE->out;
    
    // 第三步：计算内环（角速度环）PID
    // 例：pidRate为俯仰角速度PID，输出直接控制电机
    pidUpdate(pidRate, dt);
}

