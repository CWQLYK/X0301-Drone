//	作者：咸
//	电话:18878881386
//	邮箱:3078510877@qq.com
//	日期：2026.3.4
//	版本：1.0
//  软件I2C模块 —— 400KHz 

#include "ALL_DEFINE.h"
#include "stm32f10x_gpio.h"

// 400kHz I2C 延时
void I2C_Delay(void)
{
    volatile uint16_t i = 60;
    while(i--);
}

//软件I2C初始化
void I2C_SoftWare_Init(void)                            
{
    GPIO_InitTypeDef I2C_InitStructure;
    RCC_APB2PeriphClockCmd(I2C_RCC, ENABLE);
    
    I2C_InitStructure.GPIO_Pin = SCL_PIN | SDA_PIN;
    I2C_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    I2C_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    
    GPIO_Init(I2C_GPIO, &I2C_InitStructure);

    SDA_H;
    SCL_H;
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
    
    SCL_L;   // 起始信号必须拉低SCL
    I2C_Delay();
    
    return SUCCESS;
}

//I2C输出停止信号
static void I2C_Stop(void)
{
    SCL_L;
    I2C_Delay();
    SDA_L;
    I2C_Delay();
    
    SCL_H;
    I2C_Delay();
    SDA_H;
    I2C_Delay();
}

//I2C输出应答信号
static void I2C_SendAck(uint8_t ack)
{
    SCL_L;
    I2C_Delay();
    
    if(ack)
        SDA_H;
    else
        SDA_L;
    
    I2C_Delay();
    SCL_H;
    I2C_Delay();
    SCL_L;
    I2C_Delay();
}

//I2C接受应答信号
static uint8_t I2C_WaitAck(void)
{
    uint8_t retry = 10;
    
    SCL_L;
    I2C_Delay();
    SDA_H;
    I2C_Delay();
    
    SCL_H;
    I2C_Delay();
    
    while(SDA_Read)
    {
        retry--;
        if(retry == 0)
        {
            SCL_L;
            return FAILED;
        }
    }
    
    SCL_L;
    return SUCCESS;
}

//I2C发送一个字节数据
static void I2C_SendByte(uint8_t byte)
{
    for(uint8_t i = 0; i < 8 ; i ++)
    {
        SCL_L;
        I2C_Delay();
        
        if(byte & (0x80 >> i))
            SDA_H;
        else
            SDA_L;
        
        I2C_Delay();
        SCL_H;
        I2C_Delay();
    }
    SCL_L;
}

//I2C接受一个字节数据
static uint8_t I2C_ReadByte(void)
{
    uint8_t res = 0; 
    
    SDA_H;
    for(uint8_t i = 0;i < 8; i ++)
    {
        res <<= 1;
        
        SCL_L;
        I2C_Delay();
        SCL_H;
        I2C_Delay();
        
        if(SDA_Read)
            res |= 0x01;
    }
    SCL_L;
    return res;
}

//读取一个字节数据并发送ACK数据
int8_t I2C_ReadByte_SendAck(uint8_t ack)
{
    uint8_t res;
    res = I2C_ReadByte();
    I2C_SendAck(ack);
    return res;
}

//I2C 写多个字节
uint8_t I2C_Write_Bytes(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len)
{
    if(I2C_Start() == FAILED) return FAILED;

    I2C_SendByte(addr);
    if(I2C_WaitAck() == FAILED){ I2C_Stop(); return FAILED; }

    I2C_SendByte(reg);
    if(I2C_WaitAck() == FAILED){ I2C_Stop(); return FAILED; }

    for(uint8_t i=0; i<len; i++)
    {
        I2C_SendByte(data[i]);
        if(I2C_WaitAck() == FAILED){ I2C_Stop(); return FAILED; }
    }

    I2C_Stop();
    return SUCCESS;
}

//I2C 读单个字节
int8_t I2C_Read_One_Byte(uint8_t addr, uint8_t reg)
{
    uint8_t res = 0;

    if(I2C_Start() == FAILED) return FAILED;

    I2C_SendByte(addr);
    if(I2C_WaitAck() == FAILED){ I2C_Stop(); return FAILED; }

    I2C_SendByte(reg);
    if(I2C_WaitAck() == FAILED){ I2C_Stop(); return FAILED; }


    I2C_Start();

    I2C_SendByte(addr+1);
    if(I2C_WaitAck() == FAILED){ I2C_Stop(); return FAILED; }

    res = I2C_ReadByte();
    I2C_SendAck(nACK);
    I2C_Stop();

    return res;
}

//I2C 写单个字节
int8_t I2C_Write_One_Byte(uint8_t addr,uint8_t reg,uint8_t data)
{
    if(I2C_Start() == FAILED) return FAILED;

    I2C_SendByte(addr);
    if(I2C_WaitAck() == FAILED){ I2C_Stop(); return FAILED; }

    I2C_SendByte(reg);
    if(I2C_WaitAck() == FAILED){ I2C_Stop(); return FAILED; }

    I2C_SendByte(data); 
    if(I2C_WaitAck() == FAILED){ I2C_Stop(); return FAILED; }

    I2C_Stop();
    return SUCCESS;
}

//I2C 读多个字节
int8_t I2C_Read_Bytes(uint8_t addr,uint8_t reg,uint8_t *data,uint8_t len)
{
    if(data == NULL || len == 0) return FAILED;

    if(I2C_Start() == FAILED) return FAILED;

    I2C_SendByte(addr);
    if(I2C_WaitAck() == FAILED){ I2C_Stop(); return FAILED; }

    I2C_SendByte(reg);
    if(I2C_WaitAck() == FAILED){ I2C_Stop(); return FAILED; }

    I2C_Start();

    I2C_SendByte(addr+1);
    if(I2C_WaitAck() == FAILED){ I2C_Stop(); return FAILED; }

    for(uint8_t i=0; i<len; i++)
    {
        data[i] = I2C_ReadByte();
        if(i == len-1)
            I2C_SendAck(nACK);
        else
            I2C_SendAck(yACK);
    }

    I2C_Stop();
    return SUCCESS;
}

