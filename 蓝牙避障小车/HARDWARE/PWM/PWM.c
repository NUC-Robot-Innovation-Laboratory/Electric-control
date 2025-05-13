#include "PWM.h"

/**
  *@brief  PWM的初始化
	*@param  pulse  占空比
	*@retval NONE
	*/
void PWM_Init(uint16_t pulse)
{
	#if MY_DRIVER
	//寄存器开发
	#else
	//库函数开发
	TIM_OCInitTypeDef pwminit;
	TIM_OCStructInit(&pwminit);
	pwminit.TIM_OCMode = TIM_OCMode_PWM1;  //设置为PWM1模式
	pwminit.TIM_Pulse = pulse-1;           //设置脉冲宽度 占空比
	pwminit.TIM_OutputState = TIM_OutputState_Disable;
	pwminit.TIM_OCNPolarity = TIM_OCNPolarity_High;   //设置高电平为有效电平
	
	TIM_OC1Init(TIM3,&pwminit);
	TIM_OC2Init(TIM3,&pwminit);
	TIM_OC3Init(TIM3,&pwminit);
	TIM_OC4Init(TIM3,&pwminit);
	
	//预装载功能
	TIM_OC1PreloadConfig(TIM3,TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM3,TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM3,TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM3,TIM_OCPreload_Enable);
	#endif
	
}



void PWM_Start(void)
{
	TIM_CCxCmd(TIM3, TIM_Channel_1, TIM_CCx_Enable);
	TIM_CCxCmd(TIM3, TIM_Channel_2, TIM_CCx_Enable);
	TIM_CCxCmd(TIM3, TIM_Channel_3, TIM_CCx_Enable);
	TIM_CCxCmd(TIM3, TIM_Channel_4, TIM_CCx_Enable);
}

void PWM_Stop(void)
{
	TIM_CCxCmd(TIM3, TIM_Channel_1, TIM_CCx_Disable);
	TIM_CCxCmd(TIM3, TIM_Channel_2, TIM_CCx_Disable);
	TIM_CCxCmd(TIM3, TIM_Channel_3, TIM_CCx_Disable);
	TIM_CCxCmd(TIM3, TIM_Channel_4, TIM_CCx_Disable);
}

void PWM_ARM_Start(void)
{
	TIM_CCxCmd(TIM2, TIM_Channel_1, TIM_CCx_Enable);
	TIM_CCxCmd(TIM2, TIM_Channel_2, TIM_CCx_Enable);
	TIM_CCxCmd(TIM2, TIM_Channel_3, TIM_CCx_Enable);
	TIM_CCxCmd(TIM2, TIM_Channel_4, TIM_CCx_Enable);
}

void PWM_ARM_Stop(void)
{
	TIM_CCxCmd(TIM2, TIM_Channel_1, TIM_CCx_Disable);
	TIM_CCxCmd(TIM2, TIM_Channel_2, TIM_CCx_Disable);
	TIM_CCxCmd(TIM2, TIM_Channel_3, TIM_CCx_Disable);
	TIM_CCxCmd(TIM2, TIM_Channel_4, TIM_CCx_Disable);
}



