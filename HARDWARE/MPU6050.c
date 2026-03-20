// 作者：咸
// 电话:18878881386
// 邮箱:3078510877@qq.com
// 日期：2026.3.4
// 版本：1.0
// MPU6050传感器驱动模块
#include "ALL_DEFINE.h"
#include <string.h>

// 宏定义：加速度/陀螺仪数据读取（I2C批量读取）
#define  Acc_Read() I2C_Read_Bytes(MPU6050_ADDRESS, 0X3B,&buffer[0],6)  // 读取加速度X/Y/Z（0x3B-0x40）
#define  Gyro_Read() I2C_Read_Bytes(MPU6050_ADDRESS, 0x43,&buffer[6],6) // 读取陀螺仪X/Y/Z（0x43-0x48）

// MPU6050校准偏移量（加速度X/Y/Z + 陀螺仪X/Y/Z）
int16_t MpuOffset[6] = {0};
// 指向MPU6050数据结构体的指针（简化数组访问）
static volatile int16_t *pMpu = (int16_t *)&MPU6050;

/**************************************************************
 * @brief  初始化MPU6050传感器
 * @param  无
 * @retval uint8_t: 初始化状态（SUCCESS=成功，FAILED=失败）
 * @note   1. 配置传感器采样率、量程、低通滤波等参数
 *         2. 最多重试10次，读取WHO_AM_I寄存器验证通信
 ***************************************************************/
uint8_t MPU6050_Init(void)
{
    uint8_t retry = 0;          // 初始化重试计数
    uint8_t data = SUCCESS;     // 初始化状态标志

    // 初始化指示灯状态
    bLED_L();	 	// 前左灯灭
    aLED_L();		// 前右灯灭
    fLED_H();		// 后左灯亮
    hLED_H();		// 后右灯亮
	
    // 循环初始化传感器参数（最多重试10次）
    do
    {
        // 复位MPU6050（0x80=BIT7复位位）
        data += I2C_Write_One_Byte(MPU6050_ADDRESS, PWR_MGMT_1, 0x80);
        delay_ms(30);
        
        // 设置采样率分频器（0x02 → 采样率=1000/(1+2)=333Hz）
        data += I2C_Write_One_Byte(MPU6050_ADDRESS, SMPLRT_DIV, 0x02);
        
        // 电源管理：唤醒传感器，使用Z轴陀螺仪作为时钟源（0x03）
        data += I2C_Write_One_Byte(MPU6050_ADDRESS, PWR_MGMT_1, 0x03);
        
        // 配置低通滤波（0x03 → 截止频率42Hz）
        data += I2C_Write_One_Byte(MPU6050_ADDRESS, CONFIGL, 0x03);
        
        // 陀螺仪量程配置（0x18 → ±2000°/s）
        data += I2C_Write_One_Byte(MPU6050_ADDRESS, GYRO_CONFIG, 0x18);
        
        // 加速度计量程配置（0x09 → ±4G）
        data += I2C_Write_One_Byte(MPU6050_ADDRESS, ACCEL_CONFIG, 0x09);
		
        retry ++;
        if(retry >= 10) break;  // 超过10次重试则退出
    } while (data != SUCCESS);

    // 读取WHO_AM_I寄存器（0x75）验证通信
    data = I2C_Read_One_Byte(MPU6050_ADDRESS, 0x75);
    printf("retry:%d\r\n",retry); // 打印重试次数（调试用）
    Remote.AUX7 = data;          // 保存传感器ID到辅助通道

    // 验证传感器ID（MPU6050_PRODUCT_ID应为0x68）
    if(data == MPU6050_PRODUCT_ID)
    {
        Mpu6050GetOffset();     // 执行传感器校准
        MPU_Err = 0;            // 标记传感器正常
        return SUCCESS;
    }
    else 
    {
        return FAILED;          // 初始化失败
    }
}

/**************************************************************
 * @brief  读取MPU6050原始数据并进行滤波处理
 * @param  无
 * @retval 无
 * @note   1. 加速度数据使用一阶卡尔曼滤波去噪
 *         2. 陀螺仪数据使用一阶低通滤波平滑
 *         3. 数据读取后减去校准偏移量
 ***************************************************************/
void MPU6050GetData(void)
{
    uint8_t i;
    uint8_t buffer[12]; // 原始数据缓冲区（加速度6字节+陀螺仪6字节）
    
    // 一阶卡尔曼滤波器参数（加速度X/Y/Z各一个）
    static struct _1_ekf_filter ekf[3] = {
        {0.02,0,0,0,0.001,0.543},  // X轴加速度滤波参数
        {0.02,0,0,0,0.001,0.543},  // Y轴加速度滤波参数
        {0.02,0,0,0,0.001,0.543}   // Z轴加速度滤波参数
    };	
    
    // 陀螺仪低通滤波缓存
    static float tBuff[3] = {0};	

    // 读取加速度和陀螺仪原始数据
    Acc_Read();
    Gyro_Read();

    // 处理6轴数据（0-2=加速度X/Y/Z，3-5=陀螺仪X/Y/Z）
    for(i = 0; i < 6; i ++)
    {
        // 步骤1：拼接16位数据并减去校准偏移量
        pMpu[i] = (((int16_t)buffer[i<<1] << 8) | buffer[(i<<1)+1]) - MpuOffset[i];
        
        // 步骤2：加速度数据（i<3）执行卡尔曼滤波
        if(i < 3)
        {
            kalman_1(&ekf[i], (float)pMpu[i]); // 卡尔曼滤波
            pMpu[i] = (int16_t)ekf[i].out;     // 保存滤波后的值
        }
        
        // 步骤3：陀螺仪数据（i>2）执行一阶低通滤波
        if(i > 2)
        {
            uint8_t k = i - 3;                // 陀螺仪轴索引（0=X，1=Y，2=Z）
            const float factor = 0.15f;       // 低通滤波系数（0.15=15%新数据+85%历史数据）
            static float tBuff[3];            // 历史数据缓存

            // 一阶低通滤波公式：新值 = 历史值*(1-系数) + 原始值*系数
            pMpu[i] = tBuff[k] = tBuff[k] * (1 - factor) + pMpu[i] * factor;         
        }
    }
}

/**************************************************************
 * @brief  MPU6050传感器零偏校准
 * @param  无
 * @retval 无
 * @note   1. 先等待传感器静止（陀螺仪波动<5），再采集256次数据求平均
 *         2. 校准期间关闭TIM3中断，避免干扰
 *         3. Z轴加速度补偿1G重力（对应8192 LSB）
 ***************************************************************/
void Mpu6050GetOffset(void)
{
    // 32位缓冲区：存储累加值（防止16位溢出）
    int32_t buffer[6] = {0};
    // 循环计数变量
    int16_t i;  
    // 等待传感器静止的次数（30次）
    uint8_t k = 30;
    // 陀螺仪静止判定阈值（±5 LSB）
    const int8_t MAX_GYRO_QUIET = 5;
    const int8_t MIN_GYRO_QUIET = -5;	
	
    /* ===================== 第一步：等待传感器完全静止 ===================== */
    // 上一次陀螺仪数据（用于判断波动）
    int16_t LastGyro[3] = {0};
    // 当前与上一次陀螺仪的差值（波动量）
    int16_t ErrorGyro[3];	
	
    /* 初始化偏移量数组 */
    memset(MpuOffset, 0, 12);          // 6个int16_t共12字节，全部清零
    MpuOffset[2] = 8192;               // Z轴加速度补偿1G重力（±4G量程下1G=8192 LSB）
	
    /* 关闭TIM3中断：避免中断干扰校准过程 */
    TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);	
	
    /* 等待传感器静止（最多30次） */
    while(k--)
    {
        // 循环读取数据直到陀螺仪波动小于阈值
        do
        {
            delay_ms(10);                   // 10ms读取一次（降低采样频率）
            MPU6050GetData();               // 读取传感器数据
            
            // 计算陀螺仪波动量
            for(i=0; i<3; i++)
            {
                ErrorGyro[i] = pMpu[i+3] - LastGyro[i];  // 当前值 - 上一次值
                LastGyro[i] = pMpu[i+3];	                // 更新上一次值
            }			
        } while (
            // 只要任一轴陀螺仪波动超过阈值，继续等待
            (ErrorGyro[0] > MAX_GYRO_QUIET || ErrorGyro[0] < MIN_GYRO_QUIET) ||
            (ErrorGyro[1] > MAX_GYRO_QUIET || ErrorGyro[1] < MIN_GYRO_QUIET) ||
            (ErrorGyro[2] > MAX_GYRO_QUIET || ErrorGyro[2] < MIN_GYRO_QUIET)
        );
    }	

    /* ===================== 第二步：采集数据计算平均偏移量 ===================== */
    /* 总共采集356次，前100次丢弃（稳定），后256次累加求平均 */	
    for(i=0; i<356; i++)
    {		
        MPU6050GetData();               // 读取传感器数据
        
        // 仅累加后256次有效数据
        if(100 <= i)
        {
            uint8_t k;
            // 累加6轴数据到32位缓冲区（防止溢出）
            for(k=0; k<6; k++)
            {
                buffer[k] += pMpu[k];
            }
        }
    }

    /* 计算256次数据的平均值（右移8位 = 除以256） */
    for(i=0; i<6; i++)
    {
        MpuOffset[i] = buffer[i] >> 8;  // 平均值作为偏移量
    }
	
    /* 校准完成，重新使能TIM3中断 */
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
}


