#include "MOTOR.h"
#include "HCSR04.h"
#include "DELAY.h"
#include "OLED.h"
#include "TIM.h"
#include "Bluetooth.h"
#include "EXTI.h"
#include "MEROBOTICARM.h"

uint16_t distance;
uint16_t rx_data;
uint8_t rx_end;

#define		CLI()    __set_PRIMASK(1) //禁止INTR中断   CPU关中断 禁用全局中断
#define		STI()    __set_PRIMASK(0) //使能INTR中断   CPU开中断 打开全局中断




//500 0
//1500 90
//2500 180   1 2 3 4 5 6 7 8

int main(void)
{
  //驱动模块的初始化
	MOTOR_Init();
	
	MEARM_Init();                                                                                                                                                                                                                                                                                                                                                                                                       
	
	Bluetooth_USART1_Init();
	
	ARM_Start();
	
	Motor_Start();


	GPIO_WriteBit(GPIOD,GPIO_Pin_2,Bit_SET);	
  
	
		//OLED模块的初始化
	OLED_Init();

	OLED_Printf(1,20,OLED_6X8,"zh yyx zjl jch");
  OLED_ShowString(1, 3, "wangwangdui",OLED_6X8);
	
	/*调用OLED_Update函数，将OLED显存数组的内容更新到OLED硬件进行显示*/
	OLED_Update();
	
	while(1)
	{
		Bluetooth_MOTOR();
	}
}

/**
	*@brief  蓝牙模块的中断服务函数
	*@param  NONE
	*@retval NONE
	*/
void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1,USART_IT_RXNE)!= RESET)
	{
		rx_data = USART_ReceiveData(USART1);
		rx_end = 1;
		/*调用OLED_Update函数，将OLED显存数组的内容更新到OLED硬件进行显示*/
	  OLED_Update();
	}
	USART_ClearITPendingBit(USART1,USART_IT_RXNE);    //清除中断标志位
}
















