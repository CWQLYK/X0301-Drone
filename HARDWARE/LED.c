//	作者：咸
//	电话:18878881386
//	邮箱:3078510877@qq.com
//	日期：2026.3.4
//	版本：1.0
//	LED模块
#include "ALL_DEFINE.h"

//---------------------------------------------------------
/* LED状态初始化配置：可通过枚举类型选择LED初始状态 */
sLED LED = {300, AllFlashLight};  // LED初始化参数：闪烁周期300ms，初始状态为全部闪烁
u8 LED_warn;                      // LED报警标志位（预留扩展：不同报警等级控制）

/**
 * LED硬件初始化（引脚配置）
 */
void LEDInit(void)	
{	
    // 定义GPIO初始化结构体（STM32标准库必备）
	GPIO_InitTypeDef GPIO_InitStructure;
	
	// 关键配置：修改AFIO映射寄存器，使能4线烧写模式
	// 作用：释放JTAG占用的引脚（如PB3/PB4等），让这些引脚可作为普通GPIO控制LED
	AFIO->MAPR = 0X02000000; 
	
	// 使能外设时钟：GPIOB（LED引脚所在端口） + AFIO（引脚复用功能）
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO , ENABLE);
	
	// 禁用JTAG，仅保留SWD烧写模式（进一步释放PB3/PB4等引脚）
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	
	// 配置LED引脚参数
  	GPIO_InitStructure.GPIO_Pin = fLED_io | hLED_io | aLED_io | bLED_io; // 所有LED对应的引脚（宏定义）
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;                    // 推挽输出模式（适合驱动LED）
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                   // 输出速率50MHz（LED无高速要求，此为通用配置）
  	GPIO_Init(LED_GPIOB, &GPIO_InitStructure); 	                      // 应用配置到GPIOB端口
}

/**
 *  LED状态控制核心函数（周期性刷新LED显示）
 */
void PilotLED() // 默认闪烁间隔300ms（可通过LED.FlashTime修改）
{
    // 静态变量：记录上一次LED状态切换的系统时间（SysTick计数），仅初始化一次
	static uint32_t LastTime = 0;

	// 周期控制：未到闪烁间隔则直接返回，避免频繁切换LED状态
	if(SysTick_count - LastTime < LED.FlashTime)
	{
		return;
	}
	else
		LastTime = SysTick_count; // 到达间隔，更新上次切换时间

	// 根据LED状态枚举值，执行不同的LED控制逻辑
	switch(LED.status)
	{
		case AlwaysOff:      // 状态1：所有LED常暗
			bLED_H();  // bLED引脚置高（假设高电平灭LED，需匹配硬件电路）
			fLED_H();  // fLED引脚置高
			hLED_H();  // hLED引脚置高
			aLED_H();  // aLED引脚置高（原代码重复bLED_H，此处修正笔误）
			//printf("LED.status:%d\r\n",1);
			break;
			
		case AllFlashLight:  // 状态2：所有LED同时闪烁
			fLED_Toggle(); // fLED引脚电平翻转（亮→灭/灭→亮）
			bLED_Toggle(); // bLED引脚电平翻转
			hLED_Toggle(); // hLED引脚电平翻转
			aLED_Toggle(); // aLED引脚电平翻转
			//printf("LED.status:%d\r\n",2);
		  break;
			
		case AlwaysOn:       // 状态3：所有LED常亮
			bLED_L();  // bLED引脚置低（假设低电平亮LED，匹配硬件电路）
			fLED_L();  // fLED引脚置低
			aLED_L();  // aLED引脚置低
			hLED_L();  // hLED引脚置低
			//printf("LED.status:%d\r\n",3);
		  break;
			
		case AlternateFlash: // 状态4：LED交替闪烁（仅单次切换，后转为全部闪烁）
			bLED_H();  // bLED灭
			fLED_L();  // fLED亮
			aLED_H();  // aLED灭
			hLED_L();  // hLED亮
			LED.status = AllFlashLight; // 切换后自动转为全部闪烁状态
			//printf("LED.status:%d\r\n",4);
			break;
			
		case WARNING:        // 状态5：警告模式（慢速闪烁）
		  fLED_Toggle();    // 所有LED翻转
		  hLED_Toggle();
			bLED_Toggle();
		  aLED_Toggle();
			LED.FlashTime = 800; // 警告闪烁周期800ms（比默认慢）
			//printf("LED.status:%d\r\n",5);
			break;
			
		case DANGEROURS:     // 状态6：危险模式（快速闪烁）
			bLED_L();    // bLED常亮
		  aLED_L();    // aLED常亮
			fLED_Toggle(); // fLED快速翻转
		  hLED_Toggle(); // hLED快速翻转
			LED.FlashTime = 70; // 危险闪烁周期70ms（高频闪烁报警）
			//printf("LED.status:%d\r\n",6);
			break;
			
		default:             // 异常状态：默认所有LED常暗
			LED.status = AlwaysOff;
			//printf("LED.status:%d\r\n",7);
			break;
	}
}

