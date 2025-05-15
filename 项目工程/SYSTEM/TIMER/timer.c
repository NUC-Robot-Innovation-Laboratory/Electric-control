#include "timer.h"
/*
函数功能: 配置定时器1
函数参数: psc 预分频器  arr重装载值
*/
void TIMER1_Init(u16 psc,u16 arr)
{
   /*1. 开时钟*/
   RCC->APB2ENR|=1<<11; //开启定时器1的时钟
   RCC->APB2RSTR|=1<<11;//开启定时器1复位时钟
   RCC->APB2RSTR&=~(1<<11);//关闭定时器1复位时钟
   /*2. 配置核心寄存器*/
   TIM1->PSC=psc-1;
   TIM1->ARR=arr;
   TIM1->DIER|=1<<0; //开启更新中断
   STM32_SetPriority(TIM1_UP_IRQn,1,1); //设置中断优先级
   TIM1->CR1|=1<<0; //开启定时器1
}

///*
//函数功能: 定时器1的更新中断服务函数
//*/
//void TIM1_UP_IRQHandler(void)
//{
//    if(TIM1->SR&1<<0)
//    {
//      TIM1->SR&=~(1<<0);
//      USART1_RX_FLAG=1; //表示接收完毕
//      TIM1->CR1&=~(1<<0); //关闭定时器1 
//    }
//}

/*
函数功能: 配置定时器2
函数参数: psc 预分频器  arr重装载值
*/
void TIMER2_Init(u16 psc,u16 arr)
{
   /*1. 开时钟*/
   RCC->APB1ENR|=1<<0; //开启定时器2的时钟
   RCC->APB1RSTR|=1<<0;//开启定时器2复位时钟
   RCC->APB1RSTR&=~(1<<0);//关闭定时器2复位时钟
   /*2. 配置核心寄存器*/
   TIM2->PSC=psc-1;
   TIM2->ARR=arr;
   TIM2->DIER|=1<<0; //开启更新中断
   STM32_SetPriority(TIM2_IRQn,1,1); //设置中断优先级
   //TIM2->CR1|=1<<0; //开启定时器2
}

/*
函数功能: 定时器2中断服务函数
*/
void TIM2_IRQHandler(void)
{
    if(TIM2->SR&1<<0)
    {
      TIM2->SR&=~(1<<0);
      USART2_RX_FLAG=1; //表示接收完毕
      TIM2->CR1&=~(1<<0); //关闭定时器2
    }
}

/*
函数功能: 配置定时器3
函数参数: psc 预分频器  arr重装载值
*/
void TIMER3_Init(u16 psc,u16 arr)
{
   /*1. 开时钟*/
   RCC->APB1ENR|=1<<1; //开启定时器3的时钟
   RCC->APB1RSTR|=1<<1;//开启定时器3复位时钟
   RCC->APB1RSTR&=~(1<<1);//关闭定时器3复位时钟
   /*2. 配置核心寄存器*/
   TIM3->PSC=psc-1;
   TIM3->ARR=arr;
   TIM3->DIER|=1<<0; //开启更新中断
   STM32_SetPriority(TIM3_IRQn,1,1); //设置中断优先级
  // TIM3->CR1|=1<<0; //开启定时器3
}

/*
函数功能: 定时器3中断服务函数
*/
void TIM3_IRQHandler(void)
{
    if(TIM3->SR&1<<0)
    {
      TIM3->SR&=~(1<<0);
      USART3_RX_FLAG=1; //表示接收完毕
      TIM3->CR1&=~(1<<0); //关闭定时器3
    }
}

/*
函数功能: 配置定时器4
函数参数: psc 预分频器  arr重装载值
*/
void TIMER4_Init(u16 psc,u16 arr)
{
   /*1. 开时钟*/
   RCC->APB1ENR|=1<<2; //开启定时器4的时钟
   RCC->APB1RSTR|=1<<2;//开启定时器4复位时钟
   RCC->APB1RSTR&=~(1<<2);//关闭定时器4复位时钟
   /*2. 配置核心寄存器*/
   TIM4->PSC=psc-1;
   TIM4->ARR=arr;
   TIM4->DIER|=1<<0; //开启更新中断
   STM32_SetPriority(TIM4_IRQn,1,1); //设置中断优先级
   TIM4->CR1|=1<<0; //开启定时器4
}

/*
函数功能: 定时器4中断服务函数
*/
void TIM4_IRQHandler(void)
{
    if(TIM4->SR&1<<0)
    {
      LED1=!LED1;
      BEEP=!BEEP;
      TIM4->SR&=~(1<<0);
    }
}
