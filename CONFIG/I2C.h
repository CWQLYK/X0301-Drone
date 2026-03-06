#ifndef __I2C_H
#define __I2C_H


#define I2C_RCC       RCC_APB2Periph_GPIOB
#define I2C_GPIO      GPIOB
#define SCL_PIN       GPIO_Pin_6
#define SDA_PIN       GPIO_Pin_7
#define SCL_H         GPIOB->BSRR = GPIO_Pin_6 //置高
#define SCL_L         GPIOB->BRR  = GPIO_Pin_6 //置低

#define SDA_H         GPIOB->BSRR = GPIO_Pin_7 //置高
#define SDA_L         GPIOB->BRR  = GPIO_Pin_7 //置低
#define SCL_Read      GPIOB->IDR  & GPIO_Pin_6 //读取数据
#define SDA_Read      GPIOB->IDR  & GPIO_Pin_7 //读取数据
#endif
