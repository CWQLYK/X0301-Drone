// 作者：咸
// 电话:18878881386
// 邮箱:3078510877@qq.com
// 日期:2026.3.6
// 版本:1.0
//  PID控制算法模块（位置式+串级PID）
#include "ALL_DEFINE.h"

/**
 * @brief  批量复位PID控制器状态
 * @param  pid  指向PID控制器指针数组的首地址（存储PID的配置参数）
 * @param  len  PID控制器数组的有效长度
 * @note   复位PID的核心状态变量，适用于系统重启/模式切换/异常恢复时重置PID：
 *         清除积分历史、误差历史，避免历史状态对新控制过程的干扰
 */
void pidRest(PidObject **pid, const uint8_t len)
{
    uint8_t i;  // 循环计数器
    
    // 遍历所有PID控制器，逐一复位状态
    for(i = 0; i < len; i++)
    {
        pid[i]->integ = 0;        // 积分累加值清零（防止积分饱和）
        pid[i]->prevError = 0;    // 上一次误差（微分项初始化为0）
        pid[i]->out = 0;          // PID输出值清零（初始无输出）
        pid[i]->offset = 0;       // 补偿偏移值清零（复位校准）
    }
}

/**
 * @brief  标准位置式PID控制器计算函数
 * @param  pid  指向PID控制器结构体的指针
 * @param  dt   两次计算的时间间隔（s），通常为采样周期（如0.005s=5ms）
 * @note   1. 包含比例、积分、微分三要素计算，记录误差历史
 *         2. 预留积分限幅/输出限幅接口（可根据需求启用）
 *         3. 适用于稳定的角速度/角度/高度等闭环PID控制
 */
void pidUpdate(PidObject* pid, const float dt)
{
    float error;   // 当前误差（期望值 - 测量值）
    float deriv;   // 微分项（误差的变化率）
    
    // 步骤1：计算当前控制误差（设定目标值 - 实际反馈值）
    // 角度控制中：desired=目标角度，measured=实际角度
    error = pid->desired - pid->measured;

    // 步骤2：积分项累加（误差×时间间隔，实现积分时间累积）
    // 积分作用：消除静态误差，使系统稳定到目标值
    pid->integ += error * dt;	
	
    // 可选：积分限幅（建议启用），防止积分饱和导致系统失控
    // 例如无人机电机堵转/大幅值指令时，限制积分范围
    // pid->integ = LIMIT(pid->integ, pid->IntegLimitLow, pid->IntegLimitHigh);

    // 步骤3：微分项计算（当前误差 - 上一次误差）/时间间隔
    // 微分作用：预测误差变化趋势，提高系统响应速度，抑制超调
    deriv = (error - pid->prevError) / dt;  

    // 步骤4：PID总输出计算（比例+积分+微分）
    // kp*error=比例项，快速响应误差；ki*integ=积分项，消除静差；kd*deriv=微分项，抑制超调
    pid->out = pid->kp * error + pid->ki * pid->integ + pid->kd * deriv;
	
    // 可选：输出限幅（建议启用），限制PID输出范围（如电机PWM范围）
    // pid->out = LIMIT(pid->out, pid->OutLimitLow, pid->OutLimitHigh);
		
    // 步骤5：保存当前误差为历史误差（供下次微分计算使用）
    pid->prevError = error;
}

/**
 * @brief  串级PID控制器计算函数（角度+角速度）
 * @param  pidRate  内环PID指针（通常为角速度PID，如电机转速）
 * @param  pidAngE  外环PID指针（通常为角度PID，如无人机姿态角）
 * @param  dt       计算时间间隔（s）
 * @note   1. 串级PID是无人机稳定控制的核心：外环控制角度，内环控制角速度
 *         2. 执行流程：先计算外环角度PID → 其输出作为内环角速度的目标值 → 计算内环角速度PID
 *         3. 优势：内环快速响应动态变化，外环保证稳态精度，提升系统抗干扰能力
 */
void CascadePID(PidObject* pidRate, PidObject* pidAngE, const float dt)
{	 
    // 第一步：计算外环角度PID
    // pidAngE为角度PID，其输出为目标角速度
    pidUpdate(pidAngE, dt);
    
    // 第二步：将外环输出作为内环的目标值
    // 角度PID的输出（误差补偿）转换为角速度的期望值
    pidRate->desired = pidAngE->out;
    
    // 第三步：计算内环角速度PID
    // pidRate为角速度PID，直接控制执行机构（如电机）
    pidUpdate(pidRate, dt);
}


