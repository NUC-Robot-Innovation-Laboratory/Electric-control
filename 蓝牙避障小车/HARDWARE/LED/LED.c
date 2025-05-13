#include "LED.h"


/*
  前A：PA6---------AENA
       PA7---------AENB
       PB3 PB4 PB5 PB6 控制IN1 IN2 IN3 IN4
  后B：PB0--------BENA
       PB1--------BENB
			 PC0 PC1 PC3 PC2 控制IN1 IN2 IN3 IN4
 */
 
/**
  *@brief   L298N 各个驱动模块的引脚配置
	*@param   NONE
	*@retval  NONE
	*/
void LED_Init(void)
{
	#if MY_DRIVER
	//寄存器开发
	#else
	//库函数开发  
	GPIO_InitTypeDef gpioinit;
	GPIO_StructInit(&gpioinit);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOD,ENABLE);
	
	//1.配置后轮的使能PWM引脚 PA6 PA7
	gpioinit.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	gpioinit.GPIO_Mode = GPIO_Mode_AF_PP;
	gpioinit.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&gpioinit);
	
	//2.配置后轮的控制引脚 PC0 PC1 PC2 PC3 控制IN1 IN2 IN3 IN4
	gpioinit.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	gpioinit.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioinit.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&gpioinit);
	GPIO_WriteBit(GPIOC,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3,Bit_RESET);
	//3.配置前轮的使能PWM引脚 PB0 PB1
	gpioinit.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	gpioinit.GPIO_Mode = GPIO_Mode_AF_PP;
	gpioinit.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&gpioinit);
	
	//4.配置前轮的控制引脚 PB3 PB4 PB5 PB6
	gpioinit.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6;
	gpioinit.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioinit.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&gpioinit);
	GPIO_WriteBit(GPIOB,GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6,Bit_RESET);
	
	//5.板载LED灯1 PA8
	gpioinit.GPIO_Pin = GPIO_Pin_8;
	gpioinit.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioinit.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&gpioinit);
	GPIO_WriteBit(GPIOA,GPIO_Pin_8,Bit_RESET);
	
	//6.板载LED灯2 PD2
	gpioinit.GPIO_Pin = GPIO_Pin_2;
	gpioinit.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioinit.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD,&gpioinit);
	GPIO_WriteBit(GPIOD,GPIO_Pin_2,Bit_SET);
	#endif
}
