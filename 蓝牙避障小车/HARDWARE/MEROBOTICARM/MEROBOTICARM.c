#include "MEROBOTICARM.h"

//对机械臂的控制就是对四个舵机的控制 PS2手柄遥控改变的是舵机的转动角度

//机械臂四个舵机
//PA0(TIM2_CH1)------------正面左舵机
//PA1(TIM2_CH2)------------正面右舵机
//PA2(TIM2_CH3)------------机械夹舵机
//PA3(TIM2_CH4)------------底部转身舵机

/**
  *@brief  对四个舵机及引脚的初始化
	*@param  NONE  舵机的时基配置为 72 20000 <=> 20ms
	*@retval NONE
	*/
void MEARM_Init(void)
{
	//库函数开发
	GPIO_InitTypeDef gpioinit;
	TIM_TimeBaseInitTypeDef timeinit;
	TIM_OCInitTypeDef pwminit;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	GPIO_StructInit(&gpioinit);
	gpioinit.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	gpioinit.GPIO_Mode =GPIO_Mode_AF_PP;
	gpioinit.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&gpioinit);
	
	TIM_TimeBaseStructInit(&timeinit);
	timeinit.TIM_CounterMode = TIM_CounterMode_Up;
	timeinit.TIM_Prescaler = 72-1;
	timeinit.TIM_Period = 20000-1;
	timeinit.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM2,&timeinit);
	TIM_ARRPreloadConfig(TIM2,ENABLE);
	
	TIM_OCStructInit(&pwminit);
	pwminit.TIM_OCMode = TIM_OCMode_PWM1;     //对于舵机采用PWM1模式
	pwminit.TIM_Pulse = 0;
	pwminit.TIM_OCPolarity = TIM_OCPolarity_High;
	pwminit.TIM_OutputState = TIM_OutputState_Enable;
	
	TIM_OC1Init(TIM2,&pwminit);
	TIM_OC2Init(TIM2,&pwminit);
	TIM_OC3Init(TIM2,&pwminit);
	TIM_OC4Init(TIM2,&pwminit);
	
	TIM_OC1PreloadConfig(TIM2,TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM2,TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM2,TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM2,TIM_OCPreload_Enable);
	
	TIM_Cmd(TIM2,ENABLE);
}

/**
  *@brief    设置TIM2_CH1,2,3,4的PWM输出占空比 
  *@param    compare
  *@retval   NONE
  */
void PWM_SetCompare1(uint16_t compare)  //PA0 正面左舵机
{
	TIM_SetCompare1(TIM2,compare);
}

void PWM_SetCompare2(uint16_t compare)  //PA1 正面右舵机
{
	TIM_SetCompare2(TIM2,compare);
}

void PWM_SetCompare3(uint16_t compare)  //PA2 机械夹舵机
{
	TIM_SetCompare3(TIM2,compare);
}

void PWM_SetCompare4(uint16_t compare)  //PA3 机械臂底部舵机
{
	TIM_SetCompare4(TIM2,compare);
}
