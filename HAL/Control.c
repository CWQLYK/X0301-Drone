// 作者：咸
// 电话:18878881386
// 邮箱:3078510877@qq.com
// 日期：2026.3.8
// 版本：1.0
//  飞控核心控制

#include "ALL_DEFINE.h"

// 通用宏定义：系统默认常量，统一整个系统的状态标识
#undef NULL
#define NULL 0           // 空指针定义
#undef DISABLE 
#define DISABLE 0        // 禁用状态
#undef ENABLE 
#define ENABLE 1         // 启用状态
#undef REST
#define REST 0           // 复位状态
#undef SET 
#define SET 1            // 设置状态
#undef EMERGENT
#define EMERGENT 0       // 紧急状态（紧急停止/故障）


//------------------------------------------------------------------------------
// PID对象指针数组：集中管理所有PID控制器，统一单位/计算逻辑
// 元素说明：
// &pidRateX/&pidRateY/&pidRateZ  角速度环PID（X/Y/Z轴，单位：度/s）
// &pidRoll/&pidPitch/&pidYaw     角度环PID（横滚/俯仰/偏航，单位：度）
// &pidHeightRate/&pidHeightHigh  高度环PID
// &Flow_SpeedPid_x/Flow_PosPid_x/Flow_SpeedPid_y/Flow_PosPid_y  光流控制PID
PidObject *(pPidObject[])={
    &pidRateX,&pidRateY,&pidRateZ,
    &pidRoll,&pidPitch,&pidYaw,
    &pidHeightRate,&pidHeightHigh,
    &Flow_SpeedPid_x,&Flow_PosPid_x,
    &Flow_SpeedPid_y,&Flow_PosPid_y
};

/**************************************************************
 * @brief  无人机核心PID控制主函数
 * @param  dt: 控制周期（本次调用的时间间隔，单位：秒）
 * @retval 无
 * @note   1. 采用分层PID控制：角度环（慢）→ 角速度环（快）
 *         2. 状态机驱动：等待解锁→准备→运行→退出
 *         3. 容错机制：任何异常时先复位PID参数，再通过状态机退出控制逻辑
 ***************************************************************/
void FlightPidControl(float dt)
{
    // 状态机变量：记录当前飞控所处的状态
    // 状态说明：未解锁时为等待，解锁后依次进入准备、运行
    // WAITING_1  等待解锁状态
    // READY_11   准备就绪状态（复位PID）
    // PROCESS_31 运行状态（执行PID计算）
    // EXIT_255   退出/紧急停止状态
    volatile static uint8_t status=WAITING_1;

    // 状态机逻辑分支
    switch(status)
    {        
        case WAITING_1: // 状态1：等待解锁
            // 检测全局解锁标志位ALL_flag.unlock，触发解锁流程
            if(ALL_flag.unlock)
            {
                status = READY_11; // 解锁成功，进入准备状态
            }			
            break;

        case READY_11:  // 状态2：准备阶段（初始化参数）
            // 复位前8个PID控制器的历史数据（积分、微分等），避免初始值影响
            pidRest(pPidObject,8); 

            // 初始化偏航角相关参数，初始偏航角设为0
            Angle.yaw = pidYaw.desired =  pidYaw.measured = 0;  
        
            status = PROCESS_31; // 进入运行状态
            break;			

        case PROCESS_31: // 状态3：运行阶段（核心PID计算）
            // ====================== 步骤1：更新PID测量值 ======================
            // 角速度环PID测量值：MPU6050原始角速度值转换为度/s（Gyro_G为转换系数）
            pidRateX.measured = MPU6050.gyroX * Gyro_G; // X轴角速度（横滚角速度）
            pidRateY.measured = MPU6050.gyroY * Gyro_G; // Y轴角速度（俯仰角速度）
            pidRateZ.measured = MPU6050.gyroZ * Gyro_G; // Z轴角速度（偏航角速度）
        
            // 角度环PID测量值：姿态解算后的角度（单位：度）
            pidPitch.measured = Angle.pitch;  // 俯仰角测量值
            pidRoll.measured = Angle.roll;    // 横滚角测量值
            pidYaw.measured = Angle.yaw;      // 偏航角测量值
        
            // ====================== 步骤2：执行PID计算（横滚轴） ======================
            pidUpdate(&pidRoll,dt);          // 执行横滚角度环PID计算
            pidRateX.desired = pidRoll.out;  // 角度环输出作为角速度环期望值（串级控制）
            pidUpdate(&pidRateX,dt);         // 执行横滚角速度环PID计算

            // ====================== 步骤3：执行PID计算（俯仰轴） ======================
            pidUpdate(&pidPitch,dt);         // 执行俯仰角度环PID计算
            pidRateY.desired = pidPitch.out; // 角度环输出作为角速度环期望值
            pidUpdate(&pidRateY,dt);         // 执行俯仰角速度环PID计算

            // ====================== 步骤4：执行PID计算（偏航轴） ======================
            CascadePID(&pidRateZ,&pidYaw,dt);	
            break;

        case EXIT_255:   // 状态4：退出控制（解锁关闭/故障）
            pidRest(pPidObject,8);  // 复位所有PID参数
            status = WAITING_1;     // 回到等待解锁状态
            break;

        default: // 异常状态：强制退出
            status = EXIT_255;
            break;
    }

    // 紧急停止逻辑：任何时候检测到紧急状态，立即退出控制
    if(ALL_flag.unlock == EMERGENT) 
    {
        status = EXIT_255;
    }
}

// 电机数组：存储4个电机的PWM目标值（0~1000 或 1000~2000，取决于FLY_TYPE）
int16_t motor[4];
// 电机宏定义（方便调用）
#define MOTOR1 motor[0] // 电机1（前左/上左，根据机架定义）
#define MOTOR2 motor[1] // 电机2（前右/上右）
#define MOTOR3 motor[2] // 电机3（后左/下左）
#define MOTOR4 motor[3] // 电机4（后右/下右）

/**************************************************************
 * @brief  电机混控函数：将PID控制输出转换为电机PWM值
 * @param  无
 * @retval 无
 * @note   1. 混控逻辑：基础油门值 + 姿态控制PID输出
 *         2. 安全机制：遥控器油门低于阈值时关闭所有电机（电机停转）
 *         3. PWM值适配：不同机架类型（FLY_TYPE）对应不同PWM范围
 ***************************************************************/
void MotorControl(void)
{	
    // 电机控制状态机：记录当前电机所处的状态
    volatile static uint8_t status=WAITING_1;
	
    // 紧急停止逻辑：等待状态下任何时候触发紧急停止
    if(ALL_flag.unlock == EMERGENT) 
    {
        status = EXIT_255;	
    }

    // 电机控制状态机分支
    switch(status)
    {		
        case WAITING_1: // 状态1：等待解锁
            // 未解锁时所有电机设为0（停转）
            MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = 0;  
            // 检测到解锁指令，进入下一状态
            if(ALL_flag.unlock)
            {
                status = WAITING_2;
            }
            break; 

        case WAITING_2: // 状态2：等待起飞指令（油门解锁）
            // 检测遥控器油门是否超过1100（用户推油门，准备起飞）
            if(Remote.thr>1100)
            {
                status = PROCESS_31; // 进入电机运行状态
            }
            break;

        case PROCESS_31: // 状态3：电机混控计算
            {
            // 提取基础油门值：减去基础值1000，范围转为0~1000
            int16_t thr_temp;
            thr_temp = Remote.thr - 1000; 

            // 安全保护：油门低于1020时（接近最小值），关闭所有电机防止误启动
            if(Remote.thr<1020)												
            {
                MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = 0;
                break;
            }

            // 基础油门值限幅（0~900），留100余量做姿态控制
            MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = LIMIT(thr_temp,0,900);

            // ====================== 混控算法 ======================
            // 核心混控逻辑：基础油门 + 姿态PID输出分配到4个电机
            // 注意：PID输出需根据机架类型（四轴/六轴）做适配
            MOTOR1 += + pidRateX.out - pidRateY.out - pidRateZ.out;
            MOTOR2 += + pidRateX.out + pidRateY.out + pidRateZ.out;
            MOTOR3 += - pidRateX.out + pidRateY.out - pidRateZ.out;
            MOTOR4 += - pidRateX.out - pidRateY.out + pidRateZ.out;
            }
            break;

        case EXIT_255: // 状态4：退出/紧急停止
            // 所有电机设为0（停转）
            MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = 0;  
            // 回到等待解锁状态
            status = WAITING_1;	
            break;

        default: // 异常状态：强制停电机
            break;
    }

    // ====================== PWM值适配（根据机架类型） ======================
    #if (FLY_TYPE == 1 || FLY_TYPE == 2)
    // 机架1/2：PWM范围0~1000，直接赋值
    PWM0 = LIMIT(MOTOR1,0,1000);  // 电机1 PWM值（限幅防止超范围）
    PWM1 = LIMIT(MOTOR2,0,1000);  // 电机2
    PWM2 = LIMIT(MOTOR3,0,1000);  // 电机3
    PWM3 = LIMIT(MOTOR4,0,1000);  // 电机4
	
    #elif (FLY_TYPE >= 3)
    // 机架3+：PWM范围1000~2000（标准舵机PWM）
    PWM0 = 1000 + LIMIT(MOTOR1,0,1000);  
    PWM1 = 1000 + LIMIT(MOTOR2,0,1000);  
    PWM2 = 1000 + LIMIT(MOTOR3,0,1000);  
    PWM3 = 1000 + LIMIT(MOTOR4,0,1000);  
	
    #else
    // 未定义机架类型，编译报错
    #error Please define FLY_TYPE!
    #endif
}


