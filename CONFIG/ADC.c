//	作者：咸
//	电话:18878881386
//	邮箱:3078510877@qq.com
//	日期：2026.3.6
//	版本：1.0
//  ADC数模转换器模块
#include "ALL_DEFINE.h"

__IO uint16_t ADC_ConvertedValue[2];  // ADC转换结果缓冲区（DMA存储，2个半字）

/**
 * ADC1对应的GPIO引脚配置
 */
static void ADC1_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* 1. 使能DMA1时钟（ADC转换结果通过DMA传输） */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    /* 2. 使能ADC1时钟和GPIOB时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_GPIO_ADC, ENABLE);

    /* 3. 配置PB0为模拟输入模式（ADC专用，无上下拉） */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_ADC ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;  // 模拟输入模式
	GPIO_Init(GPIO_ADC, &GPIO_InitStructure);
}

/**
 * ADC1工作模式配置（核心：DMA+连续转换）
 */
static void ADC1_Mode_Config(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;

	/* ------------------- DMA配置 ------------------- */
	// 重置DMA1通道1（ADC1默认使用DMA1通道1）
	DMA_DeInit(DMA1_Channel1);
	// DMA外设基地址：ADC1数据寄存器地址（数据源）
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;	
	// DMA内存基地址：ADC转换结果缓冲区（数据目标）
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue;
	// DMA传输方向：外设→内存（ADC→RAM）
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	// DMA缓冲区大小：2个半字（对应2次ADC转换）
	DMA_InitStructure.DMA_BufferSize = 2;
	// 外设地址不自增（ADC数据寄存器地址固定）
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	// 内存地址自增（依次存储到ADC_ConvertedValue[0]、[1]）
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	// 外设数据宽度：半字（16位）
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	// 内存数据宽度：半字（16位）
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	// DMA模式：循环模式（缓冲区满后自动从头开始）
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;		
	// DMA优先级：高优先级
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	// 禁用内存到内存传输（仅外设→内存）
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	// 初始化DMA1通道1
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	
	/* 使能DMA1通道1 */
	DMA_Cmd(DMA1_Channel1, ENABLE);

	/* ------------------- ADC1配置 ------------------- */
	// ADC模式：独立模式（单ADC工作）
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	
	// 使能扫描模式（支持多通道，此处仅1通道但保留配置）
	ADC_InitStructure.ADC_ScanConvMode = ENABLE ; 	
	// 使能连续转换模式（ADC转换完成后自动启动下一次）
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	
	// 外部触发转换：无（软件触发）
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	
	// 数据对齐方式：右对齐（低12位有效，符合常规使用习惯）
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; 	
	// 转换通道数量：1个（仅通道8）
	ADC_InitStructure.ADC_NbrOfChannel = 1;	 	
	// 初始化ADC1
	ADC_Init(ADC1, &ADC_InitStructure);
	
	/* 配置ADC时钟：PCLK2分频6倍（72MHz/6=12MHz，ADC最大时钟14MHz） */
	RCC_ADCCLKConfig(RCC_PCLK2_Div6); 
	
	/* 配置ADC1规则通道：通道8，转换顺序1，采样时间55.5个ADC时钟 */
	// 采样时间越长，精度越高，速度越慢
	ADC_RegularChannelConfig(ADC1, ADC_Channel, 1, ADC_SampleTime_55Cycles5);

	/* 使能ADC1的DMA传输功能 */
	ADC_DMACmd(ADC1, ENABLE);
	
	/* 使能ADC1 */
	ADC_Cmd(ADC1, ENABLE);
	
	/* ADC校准：重置校准寄存器 */   
	ADC_ResetCalibration(ADC1);
	/* 等待校准重置完成 */
	while(ADC_GetResetCalibrationStatus(ADC1));
	
	/* 启动ADC校准 */
	ADC_StartCalibration(ADC1);
	/* 等待校准完成 */
	while(ADC_GetCalibrationStatus(ADC1));
	
	/* 软件触发ADC转换（连续模式下，一次触发持续转换） */ 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

/**
 * ADC1初始化入口函数
 */
void ADC1_Init(void)
{
	ADC1_GPIO_Init();   // 配置GPIO
	ADC1_Mode_Config();   // 配置ADC+DMA模式
}

/************************ 电压检测相关配置 ************************/
int16_t voltage = 4000;     // 当前电池电压值（单位：1mv，初始值4000mv=4V）
#define power0 3700     // 低压阈值1：3700mv（3.7V）
#define power1 3750     // 正常电压阈值：3750mv（3.75V）


void Voltage_Check()//20HZ：建议20Hz频率调用（50ms一次）
{
	static u16 cnt0,cnt1;  // 低压计数/正常计数（防抖用）
	
	/* 第一步：计算实际电池电压并滤波 */
	// ADC转换值范围0~4095，对应参考电压3.3V，电压计算：(ADC值/4096)*3300mv
	// 乘以2：因为硬件上电池电压经过1/2分压（如10K+10K电阻分压），需还原实际电压
	// 一阶低通滤波：new = old + 0.2*(measure - old)，平滑电压波动
	voltage += 0.2f *(2 *(3300 *ADC_ConvertedValue[0]/4096) - voltage);
	
	/* 第二步：飞行状态判断（飞行时不检测低压） */
	if(ALL_flag.unlock)// ALL_flag.unlock=1表示飞行中，跳过低压判断
	{
		return;
	}
	else// 非飞行状态，执行低压判断
	{
		/* 情况1：电压低于3700mv且高于3400mv（低压范围） */
		if(voltage < power0 && voltage >3400)
		{
			cnt0++;  // 低压计数+1
			cnt1=0;  // 正常计数清零
			// 低压计数超过100次（20Hz→5秒），判定为持续低压
			if(cnt0>100)
			{
				cnt0 = 100;  // 计数上限，避免溢出
				// 低压标志置位+警告LED开启
				if(LED_warn==0)
				{
					flag.low_power=1;
					LED_warn = 1;
				}
			}
		}
		/* 情况2：电压高于3750mv（正常电压） */
		else if(voltage > power1)
		{
			cnt1++;  // 正常计数+1
			cnt0=0;  // 低压计数清零
			// 正常计数超过100次（5秒），判定为持续正常
			if(cnt1>100)
			{
				cnt1 = 100;  // 计数上限
				// 低压标志清零+警告LED关闭
				if(LED_warn==1)
				{
					flag.low_power=0;
					LED_warn = 0;
				}
			}
		}
		/* 情况3：电压在3700~3750mv之间（过渡区），清零计数 */
		else
		{
			cnt0=0;
			cnt1=0;
		}
	}
}




