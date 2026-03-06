//	作者：咸
//	电话:18878881386
//	邮箱:3078510877@qq.com
//	日期：2026.3.4
//	版本：1.0
//  软件I2C模块

#include "ALL_DEFINE.h"
#include "stm32f10x_gpio.h"
//软件I2C初始化
void I2C_SoftWare_Init(void)                            
{
    GPIO_InitTypeDef I2C_InitStructure;
    RCC_APB2PeriphClockCmd(I2C_RCC, ENABLE);
    
    I2C_InitStructure.GPIO_Pin = SCL_PIN | SDA_PIN;
    I2C_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    I2C_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    
    GPIO_Init(I2C_GPIO, &I2C_InitStructure);
}

//I2C延时
void I2C_Delay()
{
    volatile unsigned char i = 1;
    while(i)
    {
        i --;
    }
}


//I2C输出起始信号
static uint8_t I2C_Start(void)  
{   
    SDA_H;
    SCL_H;
    I2C_Delay();
    if(!SDA_Read)
    {
        return FAILED;
    }
    SDA_L;
    I2C_Delay();
    if(!SCL_Read)
    {
        return FAILED
    }
    SCL_H;
    I2C_Delay();
    return SUCCESS;
}

//I2C输出停止信号
static uint8_t I2C_Stop(void)
{
    SCL_L;
    I2C_Delay();
    SDA_L;
	I2C_Delay();
    I2C_Delay();
    SCL_H;
	I2C_Delay();
    SDA_H;
    I2C_Delay();
}

//I2C输出应答信号
static void I2c_Ack(void)
{
    SCL_L;
    I2C_Delay();
    SDA_L;
    I2C_Delay();
    SCL_H;
	I2C_Delay();
	I2C_Delay();
	I2C_Delay();
    I2C_Delay();
    SCL_L;
    I2C_Delay();
}

//I2C输出非应答信号
static void I2C_NoAck(void)
{
    SCL_L;
    I2C_Delay();
    SDA_H;
    I2C_Delay();
    SCL_H;
	I2C_Delay();
	I2C_Delay();
	I2C_Delay();
    I2C_Delay();
    SCL_L;
    I2C_Delay();
}

//I2C接受应答信号
static 

