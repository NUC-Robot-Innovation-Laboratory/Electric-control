#include "EXTI.h"
#include "HCSR04.h"


///**
//  *@brief  NVIC模块配置
//	*@param  NONE
//	*@retval NONE
//	*/
//void HCSR04_NVIC_Init(void)
//{
//	#if MY_DRIVER
//	//CMSIS接口函数
//	#else
//	//NVIC的结构体实现
//	NVIC_InitTypeDef nvicinit;
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//	nvicinit.NVIC_IRQChannel = EXTI1_IRQn;    
//	nvicinit.NVIC_IRQChannelPreemptionPriority = 2;
//	nvicinit.NVIC_IRQChannelSubPriority = 2;
//	nvicinit.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&nvicinit);
//	
//	#endif
//}

///**
//  *@brief  EXTI1中断处理函数
//	*@param  NONE
//	*@retval NONE
//	*/
//void EXTI1_IRQHandler(void)
//{
//  __IO uint16_t count;
//  //if(EXTI_GetFlagStatus(EXTI_Line1)==SET)
//	if(EXTI_GetITStatus(EXTI_Line1)==SET)
//	{
//		count = TIM2->CNT;         //读取定时器2的CNT寄存器
//		//计算距离
//		distance = count/58.82;    //用于将计数值转换为厘米，假设计数值的单位是微秒，并且声速在空气中的大约值。
//		//计数值的单位是微秒  1厘米约等于 58.82 微秒
//		
//		//清零CNT寄存器的值为下一次测距做准备
//		TIM2->CNT = 0;
//		
//		//清除中断标志位
//		EXTI_ClearITPendingBit(EXTI_Line1);
//	}		
//}



//USART1
//PA9(TX) -------- 蓝牙RX
//PA10(RX)-------- 蓝牙TX

/**
  *@brief  蓝牙模块的NVIC初始化
	*@param  NONE
	*@retval NONE
	*/
void Bluetooth_NVIC_Init(void)
{
	
	NVIC_InitTypeDef nvicinit;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	nvicinit.NVIC_IRQChannel = USART1_IRQn;
	nvicinit.NVIC_IRQChannelPreemptionPriority = 0;
	nvicinit.NVIC_IRQChannelSubPriority = 1;
	nvicinit.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicinit);
	
}


