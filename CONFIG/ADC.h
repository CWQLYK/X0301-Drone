#ifndef __ADC_H
#define __ADC_H

/*************** ADC硬件GPIO配置宏 ******************/
#define RCC_GPIO_ADC	RCC_APB2Periph_GPIOB    // ADC引脚时钟：GPIOB时钟
#define GPIO_ADC			GPIOB               // ADC引脚所属GPIO端口：GPIOB
#define GPIO_Pin_ADC	GPIO_Pin_0              // ADC输入引脚：PB0
#define ADC_Channel   ADC_Channel_8             // ADC通道：通道8（对应PB0）
// ADC数据寄存器地址定义
#define ADC3_DR_Address    ((u32)0x40013C4C)   // ADC3数据寄存器地址
#define ADC1_DR_Address    ((uint32_t)0x4001244C) // ADC1数据寄存器地址

void ADC1_Init(void);
void Voltage_Check(void);

extern __IO uint16_t ADC_ConvertedValue[];
extern int16_t voltage;

#endif
