//#include "HCSR04.h"
//#include "DELAY.h"

//float distance =0;

///**
//  *@brief  超声波避障模块的初始化 定时器TIM2  PA0是TIM2的ETR引脚(外部触发输入引脚)
//	*@param  NONE
//	*@retval NONE
//	*/
////门控信号，控制定时器TIM2的启停。echo为高电平时定时器计数，检测高电平的时长。
//void HCSR04_Init(void)
//{
//	#if MY_DRIVER
//	//寄存器开发
//	#else
//	//库函数开发
//	GPIO_InitTypeDef gpioinit;
//	TIM_TimeBaseInitTypeDef timeinit;
//	
//	//1.初始化PA0(外部触发输入引脚) 配置为浮空输入模式
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
//	gpioinit.GPIO_Pin = GPIO_Pin_0;
//	gpioinit.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//	gpioinit.GPIO_Speed = GPIO_Speed_2MHz;
//	GPIO_Init(GPIOA,&gpioinit);
//	
//	//2.初始化定时器2
//	TIM_TimeBaseStructInit(&timeinit);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
//	timeinit.TIM_Prescaler = 71;
//	timeinit.TIM_CounterMode = TIM_CounterMode_Up;
//	timeinit.TIM_Period = 65535;
//	TIM_TimeBaseInit(TIM2,&timeinit);
//	
//	//3.ETR初始化 关闭预分频 极性不反相即高电平或上升沿, 滤波系数0 ETR引脚用于触发定时器计数。
//	TIM_ETRConfig(TIM2,TIM_ExtTRGPSC_OFF,TIM_ExtTRGPolarity_NonInverted,0x00);
//  
//	//4.选择ETRF作为TRIG <==> 选择ETRF作为TIM2的触发源
//	TIM_SelectInputTrigger(TIM2, TIM_TS_ETRF);
//	
//	//选择 从模式(SlaveMode) -- 门控/触发模式  
//	//（Gated Mode）这意味着定时器的计数仅在外部触发信号有效时才进行。
//	TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Gated);
//	
//	//CNT 清零, CEN置位
//  TIM2->CNT =0x0;
//	TIM_Cmd(TIM2, ENABLE);
//	
//	#endif
//}

///**
//  *@brief  HC_SR04 超声波避障模块 trig引脚的初始化 PA2
//  *@param  NONE
//  *@retval NONE
//  */
//void HCSR04_trig_Init(void)
//{
//	#if MY_DRIVER
//	//寄存器开发
//	#else
//	//库函数开发
//	GPIO_InitTypeDef gpioinit;
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
//	gpioinit.GPIO_Pin = GPIO_Pin_2;
//	gpioinit.GPIO_Mode = GPIO_Mode_Out_PP;
//	gpioinit.GPIO_Speed = GPIO_Speed_2MHz;
//	GPIO_Init(GPIOA,&gpioinit);
//	GPIO_WriteBit(GPIOA,GPIO_Pin_0,Bit_RESET);  //设置为低电平
//	#endif
//}

///**
//  *@brief  通过trig引脚发出的脉冲信号 10uS 以上脉冲触发信号 触发测距 
//  *@param  NONE
//  *@retval NONE
//  */
//void Trig_Enable(void)
//{
//	GPIO_WriteBit(GPIOA,GPIO_Pin_2,Bit_SET);
//	delay(0x200);
//	GPIO_WriteBit(GPIOA,GPIO_Pin_2,Bit_RESET);
//}

///**
//  *@brief  EXTI1模块的初始化  PA1 引脚
//  *@param  NONE
//  *@retval NONE
//  */
//void HCSR04_EXTI1_Init(void)
//{
//	#if MY_DRIVER
//	//寄存器开发
//	#else
//	//库函数开发
//	GPIO_InitTypeDef gpioinit;
//	EXTI_InitTypeDef extiinit;
//	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO,ENABLE);
//	GPIO_StructInit(&gpioinit);
//	gpioinit.GPIO_Pin = GPIO_Pin_1;
//	gpioinit.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//	gpioinit.GPIO_Speed = GPIO_Speed_2MHz;
//	GPIO_Init(GPIOA,&gpioinit);
//	
//	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource1);
//	extiinit.EXTI_Line = EXTI_Line1;  //对应的中断标志位
//	extiinit.EXTI_Mode = EXTI_Mode_Interrupt;
//	extiinit.EXTI_Trigger = EXTI_Trigger_Falling;
//	extiinit.EXTI_LineCmd = ENABLE;
//	EXTI_Init(&extiinit);
//	//清除中断标志位
//	EXTI_ClearFlag(EXTI_Line1);
//	
//	#endif
//}

///**
//  *@brief  ECHO引脚的初始化
//  *@param  NONE
//  *@retval NONE
//  */
// void ECHO_Init(void)
//{
//	HCSR04_trig_Init();  //trig 引脚初始化 -- PA2引脚
//	HCSR04_Init();       //TIM2 初始化 -- PA0(ETR) 
//  HCSR04_EXTI1_Init(); //EXTI1模块的初始化 -- PA1引脚
//}


#include "HCSR04.h"
#include "DELAY.h"

//TRIG---------PA0
//ECHO---------PA1
uint16_t time;

/**
  *@brief  HCSR04超声波测距模块的初始化
	*@param  NONE
	*@retval NONE
	*/
void HCSR04_Init(void)
{
	#if MY_DRIVER
	//寄存器开发
	#else
	//库函数开发
	//TRIG引脚的初始化
	GPIO_InitTypeDef gpioinit;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	gpioinit.GPIO_Pin = GPIO_Pin_0;
	gpioinit.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioinit.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&gpioinit);
	
	//ECHO引脚的初始化
	gpioinit.GPIO_Pin = GPIO_Pin_1;
	gpioinit.GPIO_Mode = GPIO_Mode_IPD;
	gpioinit.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&gpioinit);
	
	GPIO_ResetBits(GPIOA,GPIO_Pin_0);
	#endif
}

/**
  *@brief  TRIG引脚发送脉冲
  *@param  NONE
  *@retval NONE
  */
void HCSR04_Start(void)
{
	GPIO_SetBits(GPIOA,GPIO_Pin_0);
	Delay_us(45);
	GPIO_ResetBits(GPIOA,GPIO_Pin_0);
	HCSR04_TIM_Init();
}

/**
  *@brief  获得time值
  *@param  NONE
  *@retval distance
  */
uint16_t HCSR04_GetValue(void)
{
	HCSR04_Start();
	Delay_ms(100);                           //0.1ms 0.0001s
	return ((time * 0.0001) * 34000) / 2;
}


/**
  *@breif  控制超声波测距模块的舵机初始化 PB7 TIM4_CH2通道
  *@param  NONE  舵机的时基配置为 72 20000 <=> 20ms
  *@retval NONE
  */
void HCSR04_SG90_Init(void)
{
	#if MY_DRIVER
	//寄存器开发
	#else
	//库函数开发
	GPIO_InitTypeDef gpioinit;
	TIM_TimeBaseInitTypeDef timeinit;
	TIM_OCInitTypeDef pwminit;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	
	GPIO_StructInit(&gpioinit);
	gpioinit.GPIO_Pin = GPIO_Pin_7;
	gpioinit.GPIO_Mode = GPIO_Mode_AF_PP;
	gpioinit.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&gpioinit);
	
	TIM_TimeBaseStructInit(&timeinit);
	timeinit.TIM_Prescaler = 72-1;
	timeinit.TIM_CounterMode = TIM_CounterMode_Up; //向上计数
	timeinit.TIM_Period = 20000-1;
	timeinit.TIM_ClockDivision = TIM_CKD_DIV1;     //不设置时钟分割
	TIM_TimeBaseInit(TIM4,&timeinit);
	TIM_ARRPreloadConfig(TIM4,ENABLE);
	
	TIM_OCStructInit(&pwminit);
	pwminit.TIM_OCMode = TIM_OCMode_PWM1;     //PWM1模式
	pwminit.TIM_Pulse = 0;
	pwminit.TIM_OCPolarity = TIM_OCPolarity_High;  //高电平为有效电平
	pwminit.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC2Init(TIM4,&pwminit);
  TIM_OC2PreloadConfig(TIM4,TIM_OCPreload_Enable);
	TIM_Cmd(TIM4,ENABLE);
	
	#endif
}

///**
//  *@brief    设置TIM4_CH2的PWM输出占空比 
//  *@param    compare
//  *@retval   NONE
//  */
//void PWM_SetCompare2(uint16_t compare)
//{
//	TIM_SetCompare2(TIM4,compare);
//}

/**
  *@brief  低电触发蜂鸣器的初始化 PA12
  *@param  NONE
  *@retval NONE
  */
void Bee_Init(void)
{
	#if MY_DRIVER
	//寄存器开发
	#else
	//库函数开发
	GPIO_InitTypeDef gpioinit;
	GPIO_StructInit(&gpioinit);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	gpioinit.GPIO_Pin = GPIO_Pin_12;
	gpioinit.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioinit.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&gpioinit);
	
	GPIO_WriteBit(GPIOA,GPIO_Pin_12,Bit_RESET);
	#endif
}
