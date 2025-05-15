#include "stm32f10x.h"

void delay(void);

int main()
{
	//RCC->APB2ENR |= (1<<4);
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	//GPIOC->CRH |= (1<<20) | (1<<21);
	GPIOC->CRH |= GPIO_CRH_MODE13;
	GPIOC->CRH &= ~(1<<22) | (1<<23);
	GPIOC->ODR &= ~(1<<13);
	//GPIOC->ODR |= (1<<13);
	
	while(1)
	{
		GPIOC->ODR &= ~(1<<13);
		delay();
		GPIOC->ODR |= (1<<13);
		delay();
	}
}

void delay(void)
{
	long i=0;
	for(i=0;i<1000000;i++)
	{}
}
