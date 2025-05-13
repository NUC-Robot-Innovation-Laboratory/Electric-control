#include "TIM.h"
extern uint16_t time;

/**
  *@brief  TIM3的时基初始化
	*@param  psc 精度 arr 量程
	*@retval NONE
	*/
void TIM_Init(uint16_t psc,uint16_t arr)
{
	#if MY_DRIVER
	//寄存器开发
	#else
	//库函数开发
	TIM_TimeBaseInitTypeDef timebaseinit;
	TIM_TimeBaseStructInit(&timebaseinit);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	TIM_Cmd(TIM3,ENABLE);
	timebaseinit.TIM_Prescaler = psc-1;  //1000
	timebaseinit.TIM_CounterMode = TIM_CounterMode_Up;
	timebaseinit.TIM_Period = arr-1;  //72MHZ
	timebaseinit.TIM_ClockDivision = TIM_CKD_DIV1; //不进行时钟分割
	TIM_TimeBaseInit(TIM3,&timebaseinit);
	TIM_ARRPreloadConfig(TIM3,ENABLE);
	
	#endif
}



void TIM2_IRQHandler()
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET)
	{
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1) == 1)
		{
			time++;
		}
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
	}



}





/**
  *@brief  HCSR04超声波定时器配置
	*@param  NONE
	*@retval NONE
	*/
void HCSR04_TIM_Init(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);	//选择APB1总线下的定时器Timer2
	TIM_InternalClockConfig(TIM2);		//TIM2使用内部时钟
	time = 0;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;		//计数模式，此处为向上计数
	TIM_TimeBaseInitStructure.TIM_Period = 7199;		//ARR 1 = 0.0001S
	TIM_TimeBaseInitStructure.TIM_Prescaler = 0;		//PSC
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;		//高级计时器特有，重复计数
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
	
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);		//使能中断
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;		//中断通道选择
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//优先级，同上
	
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd(TIM2, ENABLE);		//打开定时器

}
