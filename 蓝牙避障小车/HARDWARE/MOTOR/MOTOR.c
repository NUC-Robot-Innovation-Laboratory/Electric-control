#include "MOTOR.h"

/**
  *@brief  L298N模块的初始化
	*@param  NONE
	*@retval NONE
	*/
void MOTOR_Init(void)
{
	LED_Init();
	TIM_Init(72,1000);
	PWM_Init(201);
}
/*
  前A：PA6---------AENA
       PA7---------AENB
       PB3 PB4 PB5 PB6 控制IN1 IN2 IN3 IN4
  后B：PB0--------BENA
       PB1--------BENB
			 PC0 PC1 PC3 PC2 控制IN1 IN2 IN3 IN4
 */

void MOTOR_Forward(void)	
{
	//前轮部分
	GPIO_WriteBit(GPIOB,GPIO_Pin_4|GPIO_Pin_5,Bit_SET);
	GPIO_WriteBit(GPIOB,GPIO_Pin_3|GPIO_Pin_6,Bit_RESET);
	//后轮部分
	GPIO_WriteBit(GPIOC,GPIO_Pin_0|GPIO_Pin_2,Bit_SET);
	GPIO_WriteBit(GPIOC,GPIO_Pin_1|GPIO_Pin_3,Bit_RESET);
}

void MOTOR_Backward(void)
{
	//前轮部分
	GPIO_WriteBit(GPIOB,GPIO_Pin_3|GPIO_Pin_6,Bit_SET);
	GPIO_WriteBit(GPIOB,GPIO_Pin_4|GPIO_Pin_5,Bit_RESET);
	//后轮部分
	GPIO_WriteBit(GPIOC,GPIO_Pin_1|GPIO_Pin_3,Bit_SET);
	GPIO_WriteBit(GPIOC,GPIO_Pin_0|GPIO_Pin_2,Bit_RESET);
}

void MOTOR_leftward(void)
{
	//后轮部分
	GPIO_WriteBit(GPIOB,GPIO_Pin_5,Bit_SET);
	GPIO_WriteBit(GPIOB,GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_6,Bit_RESET);
	//前轮部分
	GPIO_WriteBit(GPIOC,GPIO_Pin_2,Bit_SET);
	GPIO_WriteBit(GPIOC,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_3,Bit_RESET);
}

void MOTOR_rightward(void)
{
  //后轮部分
	GPIO_WriteBit(GPIOB,GPIO_Pin_5,Bit_SET);
	GPIO_WriteBit(GPIOB,GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_6,Bit_RESET);
	//前轮部分
	GPIO_WriteBit(GPIOC,GPIO_Pin_0,Bit_SET);
	GPIO_WriteBit(GPIOC,GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3,Bit_RESET);
}
