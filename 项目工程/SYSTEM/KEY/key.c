#include "key.h"

/*
函数功能:按键初始化
硬件连接:WEK_UP=PA0  按下高电平
k1=PC5 k2=PA15 按下低电平--IO口默认为低电平，需要上拉

*/
void KEY_Init(void)
{
    //开时钟
    RCC->APB2ENR|=1<<2;
    RCC->APB2ENR|=1<<4;
    
    //配置模式
    GPIOA->CRL&=0xFFFFFFF0;
    GPIOA->CRL|=0x00000008;
    
    GPIOA->CRH&=0x0FFFFFFF;
    GPIOA->CRH|=0x80000000;
        
    GPIOC->CRL&=0xFF0FFFFF;
    GPIOC->CRL|=0x00800000;
    
    GPIOC->ODR|=1<<5;
    GPIOA->ODR|=1<<15;
    
}


/*
函数功能:函数扫描函数
k1=PC5 k2=PA15 按下低电平--IO口默认为低电平，需要上拉
*/
u8 KEY_Scan()
{
    //按下为高电平 PA0
   if(KEY0)
   {
       delay_ms(50);
       if(KEY0)
       {
          while(KEY0){}
          return 1;
       }
   }
   
   //按下为低电平 PC5
   if(KEY1==0)
   {
       delay_ms(50);
       if(KEY1==0)
       {
          while(KEY1==0){}
          return 2;
       }
   }
   
    //按下为低电平 PA15
   if(KEY2==0)
   {
       delay_ms(50);
       if(KEY2==0)
       {
          while(KEY2==0){}
          return 3;
       }
   }
   
   
   return 0;
}


