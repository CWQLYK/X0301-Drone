#ifndef __I2C_H
#define __I2C_H

//引脚宏定义
#define I2C_RCC       RCC_APB2Periph_GPIOB
#define I2C_GPIO      GPIOB
#define SCL_PIN       GPIO_Pin_6
#define SDA_PIN       GPIO_Pin_7

// 应答信号定义
#define yACK          0                       
#define nACK          1

// 引脚操作宏
#define SCL_H         GPIOB->BSRR = GPIO_Pin_6 //置高
#define SCL_L         GPIOB->BRR  = GPIO_Pin_6 //置低
#define SDA_H         GPIOB->BSRR = GPIO_Pin_7 //置高
#define SDA_L         GPIOB->BRR  = GPIO_Pin_7 //置低
#define SCL_Read      GPIOB->IDR  & GPIO_Pin_6 //读取数据
#define SDA_Read      GPIOB->IDR  & GPIO_Pin_7 //读取数据

/*************************** 函数声明 ***************************/
/**
 * @brief  软件I2C初始化
 * @note   配置SCL/SDA引脚为开漏输出，初始拉高总线
 * @param  无
 * @retval 无
 */
void I2C_SoftWare_Init(void);

/**
 * @brief  I2C延时函数（适配400KHz时序）
 * @note   72MHz主频下约0.83μs，保证400KHz通信稳定
 * @param  无
 * @retval 无
 */
void I2C_Delay(void);

/**
 * @brief  发送I2C起始信号
 * @note   时序：SCL高→SDA低→SCL低
 * @param  无
 * @retval SUCCESS-成功，FAILED-失败（总线被占用）
 */
uint8_t I2C_Start(void);

/**
 * @brief  发送I2C停止信号
 * @note   时序：SCL低→SDA低→SCL高→SDA高
 * @param  无
 * @retval 无
 */
void I2C_Stop(void);

/**
 * @brief  发送I2C应答信号
 * @note   ack=0发送ACK，ack=1发送NACK
 * @param  ack: 应答类型（yACK/nACK）
 * @retval 无
 */
void I2C_SendAck(uint8_t ack);

/**
 * @brief  等待I2C应答信号
 * @note   超时重试机制，避免死等
 * @param  无
 * @retval SUCCESS-收到ACK，FAILED-未收到ACK（超时）
 */
uint8_t I2C_WaitAck(void);

/**
 * @brief  I2C发送1个字节数据
 * @note   从最高位（bit7）到最低位（bit0）发送
 * @param  byte: 要发送的字节
 * @retval 无
 */
void I2C_SendByte(uint8_t byte);

/**
 * @brief  I2C接收1个字节数据
 * @note   从最高位（bit7）到最低位（bit0）接收
 * @param  无
 * @retval 接收到的字节
 */
uint8_t I2C_ReadByte(void);

/**
 * @brief  读取1字节并发送应答
 * @note   封装读取+应答操作，简化多字节读取逻辑
 * @param  ack: 应答类型（yACK/nACK）
 * @retval 接收到的字节
 */
int8_t I2C_ReadByte_SendAck(uint8_t ack);

/**
 * @brief  I2C写入多字节数据到指定寄存器
 * @note   适用于大部分I2C设备的批量写操作
 * @param  addr: 设备I2C地址（7位地址，最低位为0）
 * @param  reg:  寄存器地址
 * @param  data: 要写入的数据缓冲区
 * @param  len:  要写入的字节数
 * @retval SUCCESS-成功，FAILED-失败
 */
uint8_t I2C_Write_Bytes(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len);

/**
 * @brief  I2C读取指定寄存器的1个字节数据
 * @note   单次读取常用接口，如读取传感器单个寄存器值
 * @param  addr: 设备I2C地址（7位地址，最低位为0）
 * @param  reg:  寄存器地址
 * @retval 成功-读取到的字节，失败-FAILED
 */
int8_t I2C_Read_One_Byte(uint8_t addr, uint8_t reg);

/**
 * @brief  I2C写入1个字节数据到指定寄存器
 * @note   单次写入常用接口，如配置传感器单个寄存器
 * @param  addr: 设备I2C地址（7位地址，最低位为0）
 * @param  reg:  寄存器地址
 * @param  data: 要写入的字节
 * @retval SUCCESS-成功，FAILED-失败
 */
int8_t I2C_Write_One_Byte(uint8_t addr, uint8_t reg, uint8_t data);

/**
 * @brief  I2C读取指定寄存器的多字节数据
 * @note   适用于大部分I2C设备的批量读操作
 * @param  addr: 设备I2C地址（7位地址，最低位为0）
 * @param  reg:  寄存器地址
 * @param  data: 存储读取数据的缓冲区
 * @param  len:  要读取的字节数
 * @retval SUCCESS-成功，FAILED-失败
 */
int8_t I2C_Read_Bytes(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len);

#endif
