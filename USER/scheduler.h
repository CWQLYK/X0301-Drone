#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "stm32f10x.h"

typedef struct
{
	uint8_t check_flag;
	uint16_t err_flag;
	int16_t cnt_2ms;
	int16_t cnt_4ms;
	int16_t cnt_6ms;
	int16_t cnt_10ms;
	int16_t cnt_20ms;
	int16_t cnt_50ms;
	int16_t cnt_1000ms;
}loop_t;

void Main_Loop(void);			//任务调度
void Loop_Check(void);			//时间计数器累加 + 执行校准
void Duty_2ms(void);
void Duty_4ms(void);
void Duty_6ms(void);
void Duty_10ms(void);
void Duty_20ms(void);
void Duty_50ms(void);
void Duty_1000ms(void);
#endif

