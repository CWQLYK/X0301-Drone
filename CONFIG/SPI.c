//	作者：咸
//	电话:18878881386
//	邮箱:3078510877@qq.com
//	日期：2026.3.9
//	版本：1.0
//  SPI通讯模块

#include "ALL_DEFINE.h"


/**
 * @brief  SPI2外设初始化配置函数（适配NRF24L01无线模块）
 * @note   1. 配置SPI2的GPIO引脚（PB13=SCK, PB14=MISO, PB15=MOSI）
 *         2. 配置SPI2为主机模式、8位数据、低电平空闲、第一个时钟沿采样
 *         3. NRF24L01的片选引脚(NSS/CSN)由软件控制，预置为高电平（未选中）
 * @param  无
 * @retval 无
 */
void SPI_Config(void)
{
    // 定义SPI初始化结构体（STM32标准库专用，存储SPI配置参数）
    SPI_InitTypeDef  SPI_InitStructure;
    // 定义GPIO初始化结构体（存储引脚配置参数）
    GPIO_InitTypeDef GPIO_InitStructure;

    // 步骤1：使能SPI2和GPIOB时钟（外设使用前必须先开启时钟）
    // RCC_APB1Periph_SPI2：SPI2挂载在APB1总线，时钟频率36MHz/72MHz
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);  
    // RCC_APB2Periph_GPIOB：GPIOB挂载在APB2总线；AFIO：复用功能IO时钟（SPI是GPIO复用功能）
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    
    // 步骤2：预置NRF24L01的片选引脚为高电平（未选中模块）
    GPIO_SetBits(NRF_CSN_GP, NRF24L01_CSN); 

    // 步骤3：配置SPI2的GPIO引脚（PB13=SCK, PB14=MISO, PB15=MOSI）
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15; // 选择PB13/14/15
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 引脚输出速度50MHz（高速）
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    // 复用推挽输出（SPI专用模式）
    GPIO_Init(GPIOB, &GPIO_InitStructure); // 应用配置到GPIOB

    // 步骤4：配置SPI2核心参数（必须先禁能SPI才能修改配置）
    SPI_Cmd(SPI2, DISABLE);  // 禁能SPI2，进入配置模式

    // SPI通信方向：全双工（同时收发，MOSI发、MISO收）
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    // SPI模式：主机模式（STM32作为主设备，NRF24L01作为从设备）
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    // 数据宽度：8位（NRF24L01通信协议为8位）
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    // 时钟极性(CPOL)：低电平空闲（SCK线默认状态为低电平）
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    // 时钟相位(CPHA)：第一个时钟沿采样（NRF24L01要求的采样方式）
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    // NSS引脚：软件控制（片选CSN由GPIO手动控制，不使用硬件NSS引脚）
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    // 波特率预分频：8分频（APB1时钟72MHz → 72/8=9MHz，NRF24L01最大支持10MHz）
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    // 数据传输顺序：高位先行（MSB First，NRF24L01通信协议要求）
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    // CRC校验多项式：7（默认值，NRF24L01不使用CRC，此参数无实际作用）
    SPI_InitStructure.SPI_CRCPolynomial = 7;

    // 应用SPI配置参数到SPI2外设
    SPI_Init(SPI2, &SPI_InitStructure);

    // 步骤5：使能SPI2，完成初始化（配置生效，可开始通信）
    SPI_Cmd(SPI2, ENABLE);
}

/**
 * @brief  SPI2单字节收发函数（SPI全双工通信核心）
 * @note   1. SPI是同步全双工通信，发一个字节的同时会收一个字节
 *         2. 先等待发送缓冲区空→发送数据→等待接收缓冲区满→返回接收数据
 *         3. 适用于NRF24L01的寄存器读写、数据收发
 * @param  dat：需要发送的8位数据
 * @retval 接收到的8位数据
 */
u8 SPI_RW(u8 dat)
{
    // 步骤1：等待发送缓冲区为空（TXE标志=1，表示可以发送新数据）
    // 防止在上一次数据未发送完成时写入新数据，导致通信错误
    while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);

    // 步骤2：通过SPI2发送一个字节数据
    SPI_I2S_SendData(SPI2, dat);

    // 步骤3：等待接收缓冲区非空（RXNE标志=1，表示已收到数据）
    while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);

    // 步骤4：返回接收到的字节数据
    return SPI_I2S_ReceiveData(SPI2);  
}









