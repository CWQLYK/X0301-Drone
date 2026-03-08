//	作者：咸
//	电话:18878881386
//	邮箱:3078510877@qq.com
//	日期：2026.3.4
//	版本：1.0
//MPU6050陀螺仪模块
#include "ALL_DEFINE.h"
#include <string.h>
#define  Acc_Read() I2C_Read_Bytes(MPU6050_ADDRESS, 0X3B,&buffer[0],6)
#define  Gyro_Read() I2C_Read_Bytes(MPU6050_ADDRESS, 0x43,&buffer[6],6)

int16_t MpuOffset[6] = {0};
static volatile int16_t *pMpu = (int16_t *)&MPU6050;

uint8_t MPU6050_Init(void)                             //初始化MPU6050
{
    uint8_t data = SUCCESS;
    lfLED_ON();
    rfLED_ON();
    lbLED_OFF();
    rbLED_OFF();
	
    do
    {
        data += I2C_Write_One_Byte(MPU6050_ADDRESS, PWR_MGMT_1, 0x80);      //复位MPU6050
        delay_ms(30);
        data += I2C_Write_One_Byte(MPU6050_ADDRESS, SMPLRT_DIV, 0x02);      //设置陀螺仪采样率
        data += I2C_Write_One_Byte(MPU6050_ADDRESS, PWR_MGMT_1, 0x03);	    //设置设备时钟源，陀螺仪Z轴
        data += I2C_Write_One_Byte(MPU6050_ADDRESS, CONFIGL, 0x03);         //设置低通滤波频率，0x03(42Hz)
        data += I2C_Write_One_Byte(MPU6050_ADDRESS, GYRO_CONFIG, 0x18);     //+-2000deg/s
        data += I2C_Write_One_Byte(MPU6050_ADDRESS, ACCEL_CONFIG, 0x09);    //+-4G
    } while (data != SUCCESS);
    data = I2C_Read_One_Byte(MPU6050_ADDRESS, 0x75);

    Remote.AUX7 = data;

    if(data == MPU6050_PRODUCT_ID)
    {
		Mpu6050GetOffset();
		MPU_Err = 0;
		return SUCCESS;
    }
    else 
    {
        return FAILED;
    }
}

/**
 * 读取陀螺仪数据加滤波
 */
void MPU6050GetData(void)
{
    uint8_t i;
    uint8_t buffer[12];
    // 定义3个一维卡尔曼滤波器（X/Y/Z加速度计各1个）
    static struct _1_ekf_filter ekf[3] = {{0.02,0,0,0,0.001,0.543},{0.02,0,0,0,0.001,0.543},{0.02,0,0,0,0.001,0.543}};	
    static float tBuff[3] = {0};	
    Acc_Read();
    Gyro_Read();

    // i=0~5：对应X/Y/Z加速度计 + X/Y/Z陀螺仪
    for(i = 0; i < 6; i ++)
    {
        // 步骤1：字节拼接+零点校准
        pMpu[i] = (((int16_t)buffer[i<<1] << 8) | buffer[(i<<1)+1])-MpuOffset[i];
        // 步骤2：加速度计（i<3）做卡尔曼滤波
        if(i < 3)
        {
            kalman_1(&ekf[i],(float)pMpu[i]);
            pMpu[i] = (int16_t)ekf[i].out;
        }
        // 步骤3：陀螺仪（i>2）做一阶低通滤波
        if(i > 2)
        {
            uint8_t k=i-3;
			const float factor = 0.15f;  //滤波因素			
			static float tBuff[3];		

			pMpu[i] = tBuff[k] = tBuff[k] * (1 - factor) + pMpu[i] * factor;         
        }

    }
}

/**
 * MPU6050传感器零点校准函数
 */
void Mpu6050GetOffset(void) //校准
{
	// 32位缓冲区：存储多组传感器数据累加值（防止16位溢出）
	int32_t buffer[6]={0};
	// 循环计数器（16位，适配更大的循环范围）
	int16_t i;  
	// 静止检测重试次数：最多等待30次静止状态
	uint8_t k=30;
	// 陀螺仪静止判断阈值：抖动绝对值不超过5视为静止
	const int8_t MAX_GYRO_QUIET = 5;
	const int8_t MIN_GYRO_QUIET = -5;	
	
	/* ===================== 第一步：等待陀螺仪完全静止 ===================== */
	// 存储上一次陀螺仪数据，用于计算抖动误差
	int16_t LastGyro[3] = {0};
	// 存储当前与上一次陀螺仪数据的差值（抖动量）
	int16_t ErrorGyro[3];	
	
	/* 初始化偏移值数组：将MpuOffset全部置0 */
	memset(MpuOffset,0,12);  // 12字节：6个int16_t元素（X/Y/Z加速度+X/Y/Z陀螺仪）
	MpuOffset[2] = 8192;     // Z轴加速度计初始偏移补偿（对应1G重力，8192是±2G量程下1G的近似值）
	
	/* 关闭TIM3更新中断：校准期间禁止中断干扰传感器数据读取 */
	TIM_ITConfig(  
		TIM3,                // 目标定时器：TIM3
		TIM_IT_Update ,      // 中断类型：更新中断（溢出/更新事件）
		DISABLE              // 失能中断
		);	
	
	/* 最多等待30次静止状态，确保传感器稳定 */
	while(k--)
	{
		/* 循环检测陀螺仪抖动，直到满足静止条件 */
		do
		{
			delay_ms(10);               // 10ms延时：降低采样频率，减少数据波动
			MPU6050GetData();               // 读取一次传感器数据（加速度+陀螺仪）
			
			// 计算X/Y/Z三轴陀螺仪的抖动误差（当前值 - 上一次值）
			for(i=0;i<3;i++)
			{
				ErrorGyro[i] = pMpu[i+3] - LastGyro[i];  // pMpu[3/4/5]对应X/Y/Z陀螺仪
				LastGyro[i] = pMpu[i+3];	                // 更新上一次陀螺仪数据
			}			
		}while (
			// 只要任意一轴陀螺仪抖动超过阈值，就继续等待（未静止）
			(ErrorGyro[0] >  MAX_GYRO_QUIET )|| (ErrorGyro[0] < MIN_GYRO_QUIET) ||
			(ErrorGyro[1] > MAX_GYRO_QUIET )|| (ErrorGyro[1] < MIN_GYRO_QUIET) ||
			(ErrorGyro[2] > MAX_GYRO_QUIET )|| (ErrorGyro[2] < MIN_GYRO_QUIET)
		);
	}	

	/* ===================== 第二步：采集数据并计算平均偏移值 ===================== */
	/* 总共采集356组数据：前100组丢弃（不稳定），后256组用于求平均 */	
	for(i=0;i<356;i++)
	{		
		MPU6050GetData();               // 读取传感器数据
		
		// 跳过前100组数据，只累加后256组有效数据
		if(100 <= i)
		{
			uint8_t k;
			// 累加6轴数据（X/Y/Z加速度 + X/Y/Z陀螺仪）到32位缓冲区
			for(k=0;k<6;k++)
			{
				buffer[k] += pMpu[k];  // buffer[k]存储第k轴的累加和
			}
		}
	}

	/* 计算256组数据的平均值：累加和 >> 8 等价于 累加和 / 256 */
	for(i=0;i<6;i++)
	{
		MpuOffset[i] = buffer[i]>>8;  // 将平均值存入偏移数组，用于后续数据校准
	}
	
	/* 校准完成，重新使能TIM3更新中断 */
	TIM_ITConfig(  
		TIM3,                // 目标定时器：TIM3
		TIM_IT_Update ,      // 中断类型：更新中断
		ENABLE               // 使能中断
		);
}




