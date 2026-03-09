//	作者：咸
//	电话:18878881386
//	邮箱:3078510877@qq.com
//	日期：2026.3.3
//	版本：1.0

#include "ALL_DEFINE.h"

loop_t loop;
uint32_t time[10];

void Loop_Check()                       //时间计数器累加 + 执行校验
{   
	loop.cnt_2ms++;
	loop.cnt_4ms++;
	loop.cnt_6ms++;
	loop.cnt_10ms++;
	loop.cnt_20ms++;
	loop.cnt_50ms++;
	loop.cnt_1000ms++;

    if(loop.check_flag >= 1)
    {
        loop.err_flag ++;
    }
    else 
    {
        loop.check_flag ++;
    }
}

void main_loop()
{
	if( loop.check_flag >= 1 )
	{
		if( loop.cnt_2ms >= 1 )
		{
			loop.cnt_2ms = 0;
			Duty_2ms();	 					//周期2ms的任务
		}
		if( loop.cnt_4ms >= 2 )
		{
			loop.cnt_4ms = 0;
			Duty_4ms();						//周期4ms的任务
		}
		if( loop.cnt_6ms >= 3 )
		{
			loop.cnt_6ms = 0;
			Duty_6ms();						//周期6ms的任务
		}
		if( loop.cnt_10ms >= 5 )
		{
			loop.cnt_10ms = 0;
			Duty_10ms();					//周期10ms的任务
		} 
		if( loop.cnt_20ms >= 10 )
		{
			loop.cnt_20ms = 0;
			Duty_20ms();					//周期20ms的任务
		}
		if( loop.cnt_50ms >= 25 )
		{
			loop.cnt_50ms = 0;
			Duty_50ms();					//周期50ms的任务
		}
		if( loop.cnt_1000ms >= 500)
		{
			loop.cnt_1000ms = 0;
			Duty_1000ms();				//周期1s的任务
		}
		loop.check_flag = 0;		//循环运行完毕标志
	}
}

void Duty_2ms(void)
{
	time[0] = GetSysTime_us();
	Mpu6050GetOffset();				          //读取陀螺仪数据
	FlightPidControl(0.002f);     /// 姿态控制
	MotorControl();               //电机控制

	time[0] = GetSysTime_us() - time[0];
}

void Duty_4ms(void)
{
	time[1] = GetSysTime_us();

	time[1] = GetSysTime_us() - time[1];
}

void Duty_6ms(void)
{

}

void Duty_10ms(void)
{

}

void Duty_20ms(void)
{

}

void Duty_50ms(void)
{

}

void Duty_1000ms(void)
{

}





