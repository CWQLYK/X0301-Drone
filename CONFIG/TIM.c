//	作者：咸
//	电话:18878881386
//	邮箱:3078510877@qq.com
//	日期：2026.3.10
//	版本：1.0
//  TIM模块代码
#include "ALL_DEFINE.h"   // 硬件宏定义（如PWM引脚、定时器参数）


/**************************************************************
 * 函数名: TIM2_PWM_Config
 * 功能: 初始化TIM2的4路PWM输出（四轴飞控核心电机驱动）
 * 描述: 
 *  1. 配置GPIOA引脚为定时器复用推挽输出；
 *  2. 计算定时器预分频值，生成指定频率的PWM波；
 *  3. 配置TIM2_CH1~CH4为PWM模式1，初始占空比0（上电安全）；
 * 适用场景：四轴飞控4个电机的PWM驱动（核心功能）
 * 输入: 无
 * 输出: 无
 * 返回值: 无
 ***************************************************************/
void TIM2_PWM_Config(void)
{
   uint16_t TIM_Prescaler;  // 定时器预分频值（临时变量）
	
   // 定义定时器时基结构体（配置计数周期、预分频等）
   TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
   // 定义定时器输出比较结构体（配置PWM模式、占空比等）
   TIM_OCInitTypeDef TIM_OCInitStructure;
   // 定义GPIO初始化结构体（配置引脚模式）
   GPIO_InitTypeDef GPIO_InitStructure;

   /* 1. 使能GPIOA时钟（PWM1~PWM4对应GPIOA引脚） */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

   /* 2. 配置PWM输出引脚（GPIOA） */
   // 选择PWM1~PWM4对应的引脚（宏定义，如PWM1_io=GPIO_Pin_0）
   GPIO_InitStructure.GPIO_Pin = PWM1_io | PWM2_io | PWM3_io| PWM4_io;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  // 引脚输出速率50MHz
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    // 复用推挽输出（定时器PWM专用）
   GPIO_Init(PWM_GPA, &GPIO_InitStructure);           // 应用配置到GPIOA（PWM_GPA=GPIOA）

   /* 3. 使能TIM2定时器时钟（APB1总线，36MHz×2=72MHz） */
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

   /* 4. 计算定时器预分频值（核心参数） */
   // 公式推导：
   // PWM频率 = 定时器时钟 / [(ARR+1) × (PSC+1)]
   // TIM_Prescaler = (系统时钟 / [(ARR+1) × PWM频率]) - 1
   // SystemCoreClock：系统主频（72MHz），TIM2_PWM_MAX：ARR值，TIM2_PWM_HZ：目标PWM频率
   TIM_Prescaler = SystemCoreClock/(TIM2_PWM_MAX+1)/TIM2_PWM_HZ -1;
	
   /* 5. 配置TIM2时基参数 */
   // 自动重装值（ARR）：计数周期，0~TIM2_PWM_MAX（如1000→计数1000次溢出）
   TIM_TimeBaseStructure.TIM_Period = TIM2_PWM_MAX; 
   // 预分频值：分频后定时器时钟 = 72MHz/(PSC+1)
   TIM_TimeBaseStructure.TIM_Prescaler = TIM_Prescaler; 
   // 时钟分频系数：0=不分频（TIM_CKD_DIV1），不影响PWM频率
   TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
   // 计数模式：向上计数（0→ARR→0循环）
   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
   // 应用时基配置到TIM2
   TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

   /* 6. 配置PWM模式（通用参数，所有通道共用） */
   // PWM模式1：计数器 < CCR值时，输出高电平；≥CCR值时，输出低电平
   TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
   // 使能输出比较（PWM输出有效）
   TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
   // 初始占空比：CCR值=0（上电时电机无输出，保证安全）
   TIM_OCInitStructure.TIM_Pulse = 0;
   // 输出极性：高电平有效（计数器<CCR时输出高）
   TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

   /* 7. 配置并使能TIM2_CH1~CH4通道（4路PWM） */
   // 通道1（PWM1）
   TIM_OC1Init(TIM2, &TIM_OCInitStructure);
   TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);  // 使能CCR1预装载（修改后立即生效）
   // 通道2（PWM2）
   TIM_OC2Init(TIM2, &TIM_OCInitStructure);
   TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
   // 通道3（PWM3）
   TIM_OC3Init(TIM2, &TIM_OCInitStructure);
   TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
   // 通道4（PWM4）
   TIM_OC4Init(TIM2, &TIM_OCInitStructure);
   TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);

   /* 8. 使能TIM2自动重装寄存器（ARR）预装载 */
   TIM_ARRPreloadConfig(TIM2, ENABLE);
   /* 9. 使能TIM2定时器（PWM输出生效） */
   TIM_Cmd(TIM2, ENABLE);
}

/**************************************************************
 * 函数名: TIM3_PWM_Config
 * 功能: 初始化TIM3的2路PWM输出（飞控扩展外设驱动）
 * 描述: 
 *  1. 配置GPIOB引脚为定时器复用推挽输出（需引脚重映射）；
 *  2. 计算预分频值，生成指定频率的PWM波；
 *  3. 配置TIM3_CH1~CH2为PWM模式1，初始占空比由TIM3_DUTY指定；
 * 适用场景：扩展外设（如云台、舵机、LED调光等）
 * 输入: 无
 * 输出: 无
 * 返回值: 无
 ***************************************************************/
void TIM3_PWM_Config(void)
{
	uint16_t TIM_Prescaler;  // 定时器预分频值（临时变量）
	
	// 定义定时器时基结构体
	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
	// 定义定时器输出比较结构体
	TIM_OCInitTypeDef  				TIM_OCInitStructure;
	// 定义GPIO初始化结构体
	GPIO_InitTypeDef 					GPIO_InitStructure;
	
	/* 1. 使能外设时钟 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);    // 使能TIM3时钟（APB1）
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);   // 使能GPIOB时钟（PWM5~PWM6对应GPIOB）
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);     // 使能AFIO时钟（引脚重映射必需）
	
	/* 2. 配置TIM3引脚部分重映射（原生引脚被占用，映射到GPIOB） */
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3,ENABLE);
	
	/* 3. 配置PWM输出引脚（GPIOB） */
	GPIO_InitStructure.GPIO_Pin = PWM5_io | PWM6_io;        // 选择PWM5~PWM6对应的引脚
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;       // 输出速率50MHz
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;         // 复用推挽输出
	GPIO_Init(PWM_GPB, &GPIO_InitStructure);                // 应用配置到GPIOB（PWM_GPB=GPIOB）

	/* 4. 计算TIM3预分频值（同TIM2公式，参数独立） */
	TIM_Prescaler = SystemCoreClock/(TIM3_PWM_MAX+1)/TIM3_PWM_HZ -1;
	
	/* 5. 配置TIM3时基参数 */
	TIM_TimeBaseStructure.TIM_Period = TIM3_PWM_MAX;        // 自动重装值（ARR）
	TIM_TimeBaseStructure.TIM_Prescaler = TIM_Prescaler;     // 预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; // 时钟分频：不分频（显式写法，更规范）
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);          // 应用配置到TIM3
	
	/* 6. 配置PWM模式（扩展外设专用） */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;       // PWM模式1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 使能输出
	TIM_OCInitStructure.TIM_Pulse = TIM3_DUTY;              // 初始占空比（由宏定义，如舵机中位）
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; // 高电平有效
	
	/* 7. 配置并使能TIM3_CH1~CH4通道（仅CH1~CH2实际使用） */
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);                // 通道1（PWM5）
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);                // 通道2（PWM6）
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);                // 通道3（冗余配置，无实际引脚）
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);                // 通道4（冗余配置，无实际引脚）
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	/* 8. 使能TIM3自动重装寄存器预装载 */
	TIM_ARRPreloadConfig(TIM3, ENABLE);
	/* 9. 使能TIM3定时器（PWM输出生效） */
	TIM_Cmd(TIM3, ENABLE);
}
