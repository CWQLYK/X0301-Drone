//========================================================================
//	开发者：小杨
//	联系方式:13728698082 / 1042763631@qq.com
//	日期：2018.05.17
//	版本：V1.0
//	说明：STM32F103驱动NRF24L01无线模块底层代码
//	      仅用于学习交流，禁止商用
//========================================================================
#include "ALL_DATA.h"
#include "nrf24l01.h"
#include "SPI.h"
#include <string.h>
#include "LED.h"
#include "ANO_DT.h"
#include "sys.h"

// 状态定义重定义（避免与其他库冲突）
#undef SUCCESS
#define SUCCESS 0   // 操作成功
#undef FAILED
#define FAILED  1    // 操作失败

// 中断标志位宏定义
#define MAX_TX  		0x10  // 达到最大重传次数触发中断
#define TX_OK   		0x20  // TX发送完成触发中断
#define RX_OK   		0x40  // RX接收完成触发中断

// 状态寄存器中断位定义
#define RX_DR			6		// 接收完成中断位（STATUS寄存器bit6）
#define TX_DS			5		// 发送完成中断位（STATUS寄存器bit5）
#define MAX_RT			4		// 最大重传中断位（STATUS寄存器bit4）

// 全局变量定义
u8 MPU_Err=1,NRF_Err=1,SPL_Err=1;    // 模块错误标志（1=错误，0=正常）
uint8_t NRF_SSI,NRF_SSI_CNT;        // NRF通信状态计数
uint16_t Nrf_Erro;                  // NRF通信超时/错误计数
uint8_t _CH;                        // 当前RF通信通道号
uint8_t NRF24L01_2_RXDATA[RX_PLOAD_WIDTH]; // NRF接收数据缓冲区（32字节）
uint8_t NRF24L01_2_TXDATA[RX_PLOAD_WIDTH]; // NRF发送数据缓冲区（32字节）

// 通信地址定义（收发地址需一致才能通信）
const uint8_t TX_ADDRESS[]= {0xAA,0xBB,0xCC,0x00,0x01};	// 发送地址（5字节）
const uint8_t RX_ADDRESS[]= {0xAA,0xBB,0xCC,0x00,0x01};	// 接收地址（需与TX_ADDRESS一致）

// NRF24L01硬件IO操作宏（需在ALL_DATA.h中定义NRF_CSN_GP/NRF24L01_CSN等引脚）
#define Set_NRF24L01_CSN    (NRF_CSN_GP->BSRR = NRF24L01_CSN)  // CSN引脚置高（SPI失能）
#define Clr_NRF24L01_CSN    (NRF_CSN_GP->BRR = NRF24L01_CSN)   // CSN引脚置低（SPI使能）
#define Set_NRF24L01_CE 	(NRF_CE_GP->BSRR = NRF24L01_CE)    // CE引脚置高（启动收发）
#define Clr_NRF24L01_CE  	(NRF_CE_GP->BRR = NRF24L01_CE)     // CE引脚置低（待机/关闭收发）
#define READ_NRF24L01_IRQ   (NRF_IRQ_GP->IDR&NRF24L01_IRQ)     // 读取IRQ引脚状态（低电平表示有中断）

/**
 * @brief  初始化NRF24L01的硬件IO口
 * @param  无
 * @retval 无
 */
void NRF24L01_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	// 使能GPIOA/GPIOB时钟（CE/CSN/IRQ引脚所在端口）
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);   
	
	// 配置CE引脚（输出模式）
	GPIO_InitStructure.GPIO_Pin = NRF24L01_CE;          
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    // 推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   // 50MHz速率
	GPIO_Init(NRF_CE_GP, &GPIO_InitStructure);

	// 配置CSN引脚（SPI片选，输出模式）
	GPIO_InitStructure.GPIO_Pin = NRF24L01_CSN;      
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    // 推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(NRF_CSN_GP, &GPIO_InitStructure); 
	
	// 初始化状态：CE高、CSN高（待机状态）
	Set_NRF24L01_CE;                                    
	Set_NRF24L01_CSN;                                   

    // 配置IRQ引脚（中断输入，上拉模式）
	GPIO_InitStructure.GPIO_Pin = NRF24L01_IRQ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;       // 上拉输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(NRF_IRQ_GP, &GPIO_InitStructure);
	
	// 初始化SPI总线（NRF24L01使用SPI通信）
	SPI_Config();                                
	// 进入待机模式：CE低、CSN高
	Clr_NRF24L01_CE; 	                               
	Set_NRF24L01_CSN;                                   
}

/**
 * @brief  检测NRF24L01是否存在（通过读写TX_ADDR寄存器校验）
 * @param  无
 * @retval 0: 存在（匹配）；1: 不存在（不匹配）
 */
u8 NRF24L01_Check(void)
{
	u8 buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};  // 测试数据
	u8 buf1[5];                             // 读取缓冲区
	u8 i;   	 
	
	// 写入5字节测试地址到TX_ADDR寄存器
    //  TX_ADDR 是 NRF24L01 芯片的发射地址寄存器
	NRF24L01_Write_Buf(SPI_WRITE_REG + TX_ADDR, buf, 5);	
	// 读取TX_ADDR寄存器内容
	NRF24L01_Read_Buf(TX_ADDR, buf1, 5);             
	// 校验读取的数据是否与写入的一致
	for(i=0;i<5;i++)
	{
		if(buf1[i]!=0XA5) break;					   
	}
	if(i!=5) return FAILED;  // 不一致，NRF24L01未正常连接
	return SUCCESS;		    // 一致，NRF24L01存在
}	 	 

/**
 * @brief  通过SPI写单个寄存器
 * @param  regaddr: 寄存器地址
 * @param  data: 要写入的数值
 * @retval 状态寄存器的值
 */
u8 NRF24L01_Write_Reg(u8 regaddr,u8 data)
{
	u8 status;	
	
    Clr_NRF24L01_CSN;                    // 拉低CSN，使能SPI通信
  	status = SPI_RW(regaddr);            // 发送寄存器地址，返回状态值
  	SPI_RW(data);                        // 发送要写入寄存器的数据
  	Set_NRF24L01_CSN;                    // 拉高CSN，关闭SPI通信	   
  	return status;                       // 返回状态寄存器值
}

/**
 * @brief  通过SPI读单个寄存器
 * @param  regaddr: 要读取的寄存器地址
 * @retval 寄存器的数值
 */
u8 NRF24L01_Read_Reg(u8 regaddr)
{
	u8 reg_val;	    
    
	Clr_NRF24L01_CSN;                // 拉低CSN，使能SPI通信		
  	SPI_RW(regaddr);                 // 发送寄存器地址
  	reg_val = SPI_RW(0XFF);          // 发送空字节，读取寄存器返回值
  	Set_NRF24L01_CSN;                // 拉高CSN，关闭SPI通信		    
  	return reg_val;                  // 返回寄存器值
}	

/**
 * @brief  从指定寄存器读取多字节数据
 * @param  regaddr: 寄存器地址
 * @param  *pBuf: 数据缓冲区指针（存储读取的数据）
 * @param  datalen: 要读取的字节数
 * @retval 状态寄存器的值
 */
u8 NRF24L01_Read_Buf(u8 regaddr,u8 *pBuf,u8 datalen)
{
	u8 status,u8_ctr;	       
  	
  	Clr_NRF24L01_CSN;                     // 拉低CSN，使能SPI通信
  	status = SPI_RW(regaddr);             // 发送寄存器地址，获取状态值   	   
  	// 循环读取指定长度的数据
	for(u8_ctr=0;u8_ctr<datalen;u8_ctr++)
	{
		pBuf[u8_ctr] = SPI_RW(0XFF);      // 读取1字节数据
	}
  	Set_NRF24L01_CSN;                     // 拉高CSN，关闭SPI通信
  	return status;                        // 返回状态寄存器值
}

/**
 * @brief  向指定寄存器写入多字节数据
 * @param  regaddr: 寄存器地址
 * @param  *pBuf: 数据缓冲区指针（要写入的数据）
 * @param  datalen: 要写入的字节数
 * @retval 状态寄存器的值
 */
u8 NRF24L01_Write_Buf(u8 regaddr, u8 *pBuf, u8 datalen)
{
	u8 status,u8_ctr;	    
	
	Clr_NRF24L01_CSN;                                    // 拉低CSN，使能SPI通信
  	status = SPI_RW(regaddr);                            // 发送寄存器地址，获取状态值
  	// 循环写入指定长度的数据
	for(u8_ctr=0; u8_ctr<datalen; u8_ctr++)
	{
		SPI_RW(*pBuf++); 
	}
  	Set_NRF24L01_CSN;                                    // 拉高CSN，关闭SPI通信
  	return status;                                       // 返回状态寄存器值
}				   

/**
 * @brief  通过NRF24L01发送一个数据包
 * @param  *txbuf: 要发送的数据缓冲区（32字节）
 * @retval SUCCESS(0): 发送成功；MAX_TX(0x10): 达到最大重传；FAILED(1): 其他失败
 */
u8 NRF24L01_TxPacket(u8 *txbuf)
{
	u8 state;   
	
	Clr_NRF24L01_CE;  // 拉低CE，进入待机模式
	// 将数据写入TX FIFO（32字节）
	NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);
	Set_NRF24L01_CE;  // 拉高CE，启动发送	                                   
	while(READ_NRF24L01_IRQ!=0);  // 等待IRQ中断（低电平表示有中断发生）	                        
	state = NRF24L01_Read_Reg(STATUS);  // 读取状态寄存器值	   
	// 清除中断标志（写1清除）
	NRF24L01_Write_Reg(SPI_WRITE_REG+STATUS,state);      
	
	if(state & MAX_TX)  // 达到最大重传次数（发送失败）
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xff);  // 清空TX FIFO
		return MAX_TX; 
	}
	if(state & TX_OK)   // 发送完成（成功）
	{
		return SUCCESS;
	}
	return FAILED;      // 其他原因发送失败
}

/**
 * @brief  从NRF24L01接收一个数据包
 * @param  *rxbuf: 接收数据缓冲区（32字节）
 * @retval SUCCESS(0): 接收成功；FAILED(1): 未接收到数据
 */
u8 NRF24L01_RxPacket(u8 *rxbuf)
{
	u8 state;		    							      
	
	state = NRF24L01_Read_Reg(STATUS);  // 读取状态寄存器值    	 
	// 清除中断标志（写1清除）
	NRF24L01_Write_Reg(SPI_WRITE_REG+STATUS,state); 
	
	if(state & RX_OK)  // 接收到数据
	{
		// 从RX FIFO读取数据（32字节）
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);
		NRF24L01_Write_Reg(FLUSH_RX,0xff);  // 清空RX FIFO
		return SUCCESS; 
	}	   
	return FAILED;  // 未接收到数据
}

/**
 * @brief  将NRF24L01设置为接收模式
 * @param  无
 * @retval 无
 */
void RX_Mode(void)
{
	Clr_NRF24L01_CE;	  
    // 写入接收通道0的地址（5字节）
  	NRF24L01_Write_Buf(SPI_WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH);
    // 使能通道0的自动应答
  	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_AA,0x01);    
    // 使能通道0的接收地址	    	 
  	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_RXADDR,0x01); 
  	// 设置RF通信通道（45号通道，2.445GHz）	  
  	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_CH,45);	  
    // 设置通道0的有效数据长度（32字节）	    
  	NRF24L01_Write_Reg(SPI_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);
    // RF配置：0db增益，2Mbps速率，高功率
  	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_SETUP,0x0f);
    // 配置寄存器：上电、开启CRC（16位）、接收模式、所有中断使能
  	NRF24L01_Write_Reg(SPI_WRITE_REG+NCONFIG, 0x0f); 
    // 拉高CE，进入接收模式（等待数据）
  	Set_NRF24L01_CE;                                
}			

/**
 * @brief  将NRF24L01设置为发送模式
 * @param  无
 * @retval 无
 */
void TX_Mode(void)
{														 
	Clr_NRF24L01_CE;	    
    // 写入发送地址（5字节）
  	NRF24L01_Write_Buf(SPI_WRITE_REG+TX_ADDR,(u8*)TX_ADDRESS,TX_ADR_WIDTH);    
    // 写入接收通道0地址（用于自动应答，需与发送地址一致）	  
  	NRF24L01_Write_Buf(SPI_WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH); 
    // 使能通道0的自动应答    
  	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_AA,0x01);     
    // 使能通道0的接收地址  
  	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_RXADDR,0x01); 
    // 自动重传设置：延时500+86us，重传10次
  	NRF24L01_Write_Reg(SPI_WRITE_REG+SETUP_RETR,0x1a);
    // 设置RF通信通道（45号通道）
  	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_CH,45);       
    // RF配置：0db增益，2Mbps速率，高功率   
  	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_SETUP,0x0f);  
    // 配置寄存器：上电、开启CRC（16位）、发送模式、所有中断使能
  	NRF24L01_Write_Reg(SPI_WRITE_REG+NCONFIG,0x0e);    
    // 拉高CE（保持10us以上），启动发送
	Set_NRF24L01_CE;                                  
}	

/**
 * @brief  ANO飞控专用：AP模式发送数据包（自定义长度）
 * @param  *tx_buf: 发送数据缓冲区
 * @param  len: 发送数据长度
 * @retval 无
 */
void ANO_NRF_TxPacket_AP(uint8_t * tx_buf, uint8_t len)
{	
	Clr_NRF24L01_CE;		 // 进入StandBy I模式
	NRF24L01_Write_Buf(0xa8, tx_buf, len);  // 0xA8=WR_TX_PLOAD+NO_ACK，写入数据（无ACK）
	Set_NRF24L01_CE;                        // 启动发送
}

/**
 * @brief  ANO飞控专用：初始化NRF24L01（指定工作模式和通道）
 * @param  model: 工作模式（1=RX,2=TX,3=RX2,4=TX2）
 * @param  ch: RF通信通道（0~127）
 * @retval 无
 */
void ANO_NRF_Init(u8 model, u8 ch)
{
	Clr_NRF24L01_CE;  // 待机模式
	
	// 初始化公共参数
	NRF24L01_Write_Buf(SPI_WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH);	// 写RX地址
	NRF24L01_Write_Buf(SPI_WRITE_REG+TX_ADDR,(u8*)TX_ADDRESS,TX_ADR_WIDTH); 	// 写TX地址  
	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_AA,0x01); 	// 使能通道0自动应答 
	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_RXADDR,0x01);	// 使能通道0接收地址 
	NRF24L01_Write_Reg(SPI_WRITE_REG+SETUP_RETR,0x1a);	// 自动重传：500us延时，10次重传 
	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_CH,ch);			// 设置RF通道
	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_SETUP,0x0f); 	// RF配置：0db、2Mbps、高功率

	// 根据模式配置差异化参数
	if(model == 1)	// 普通接收模式
	{
		// 设置通道0数据长度
		NRF24L01_Write_Reg(SPI_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);							
		// 配置：接收模式、16位CRC、所有中断使能
		NRF24L01_Write_Reg(SPI_WRITE_REG + NCONFIG, 0x0f);   			
	}
	else if(model == 2)	// 普通发送模式
	{
		// 设置通道0数据长度
		NRF24L01_Write_Reg(SPI_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);							
		// 配置：发送模式、16位CRC、所有中断使能
		NRF24L01_Write_Reg(SPI_WRITE_REG + NCONFIG, 0x0e);   			
	}
	else if(model == 3)	// 扩展接收模式2
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xff);  // 清空TX FIFO
		NRF24L01_Write_Reg(FLUSH_RX,0xff);  // 清空RX FIFO
		// 配置：接收模式、16位CRC、所有中断使能
		NRF24L01_Write_Reg(SPI_WRITE_REG + NCONFIG, 0x0f);   			
		// 自定义扩展配置（ANO飞控专用）
		SPI_RW(0x50);
		SPI_RW(0x73);
		NRF24L01_Write_Reg(SPI_WRITE_REG+0x1c,0x01);
		NRF24L01_Write_Reg(SPI_WRITE_REG+0x1d,0x06);
	}
	else	// 扩展发送模式2
	{
		// 配置：发送模式、16位CRC、所有中断使能
		NRF24L01_Write_Reg(SPI_WRITE_REG + NCONFIG, 0x0e);   			
		NRF24L01_Write_Reg(FLUSH_TX,0xff);  // 清空TX FIFO
		NRF24L01_Write_Reg(FLUSH_RX,0xff);  // 清空RX FIFO
		// 自定义扩展配置（ANO飞控专用）
		SPI_RW(0x50);
		SPI_RW(0x73);
		NRF24L01_Write_Reg(SPI_WRITE_REG+0x1c,0x01);
		NRF24L01_Write_Reg(SPI_WRITE_REG+0x1d,0x06);
	}
	Set_NRF24L01_CE;  // 启动工作模式
}

/**
 * @brief  NRF24L01总初始化入口（硬件+功能初始化）
 * @param  无
 * @retval 无
 */
void NRF24L01_init(void)
{
	// 初始化硬件IO
	NRF24L01_Configuration();
	
	// LED状态指示（前灯亮，后灯灭）
	bLED_H();	 	// 前左灯亮
	aLED_H();		// 前右灯亮
	fLED_L();		// 后左灯灭
	hLED_L();		// 后右灯灭
	
	// 循环检测NRF24L01，直到初始化成功
	do
	{ 
		GetLockCode();  // 获取硬件锁码（ANO飞控专用）
		// 根据CPU ID取模设置通信通道（避免同频干扰）
		_CH = ST_CpuID % 0x7E;      
		ANO_NRF_Init(MODEL_RX2,0);  // 初始化为扩展接收模式2
		NRF_Err = 1;                // 标记NRF错误
	}while(NRF24L01_Check() == FAILED);  // 检测到NRF存在则退出循环
	NRF_Err = 0;  // 标记NRF正常
}

/**
 * @brief  ANO飞控专用：检测NRF24L01事件（接收/发送/重传）
 * @param  无
 * @retval 无
 */
void ANO_NRF_Check_Event(void)
{
	// 读取状态寄存器（获取中断事件）
	u8 sta = NRF24L01_Read_Reg(SPI_READ_REG + STATUS);   
	
	// 1. 检测接收完成事件
	if(sta & (1<<RX_DR))										
	{
		// 读取接收数据长度
		u8 rx_len = NRF24L01_Read_Reg(R_RX_PL_WID);       
		if(rx_len < 33)  // 数据长度合法（1~32字节）
		{
			// 读取接收数据到缓冲区
			NRF24L01_Read_Buf(RD_RX_PLOAD,NRF24L01_2_RXDATA,rx_len); 
			Nrf_Erro = 0;  // 清零错误计数
		}
		else  // 数据长度非法，清空FIFO
		{
			NRF24L01_Write_Reg(FLUSH_RX,0xff);
		}
	}
	
	// 2. 检测发送完成事件
	if(sta & (1<<TX_DS))
	{
		// 可添加发送完成后的处理逻辑
	}
	
	// 3. 检测最大重传事件
	if(sta & (1<<MAX_RT))
	{
		if(sta & 0x01)	// TX FIFO满
		{
			NRF24L01_Write_Reg(FLUSH_TX,0xff);  // 清空TX FIFO
		}
	}
	
	// 清除所有中断标志（写1清除）
	NRF24L01_Write_Reg(SPI_WRITE_REG + STATUS, sta);
}

/**
 * @brief  1KHz周期检测NRF24L01连接状态
 * @param  无
 * @retval 1: 已连接；0: 断开连接
 */
u8 NRF_Connect(void)
{
	static u8 Connect_flag;  // 连接状态标志（静态变量）
	
	Nrf_Erro ++;  // 超时计数递增
	if(Nrf_Erro == 1)  // 有新数据接收（超时计数重置）
	{
		// 解析2.4G接收数据（ANO飞控协议）
		ANO_DT_Data_Receive_Anl(NRF24L01_2_RXDATA,NRF24L01_2_RXDATA[3]+5);
		NRF_SSI_CNT++;  // 通信成功计数
		Connect_flag = 1;  // 标记已连接
	}
	if(Nrf_Erro >= 500)  // 500ms未接收到数据（断开连接）
	{
		Nrf_Erro = 1;    // 重置超时计数
		Connect_flag = 0;// 标记断开连接
	}
	return Connect_flag;  // 返回当前连接状态
}