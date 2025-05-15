#include "delay.h"

void delay_ms(u32 time)//1ms
{
	u32 i,j,k;
	for(i=0;i<time;i++)
		for(j=0;j<100;j++)
			for(k=0;k<100;k++);
}

void delay_us(u32 time)//1us
{
	u32 i,j;
	for(i=0;i<time;i++)
		for(j=0;j<10;j++);
}


void DelayMs(u32 time)
{
   u32 i,j,k;
	for(i=0;i<time;i++)
		for(j=0;j<100;j++)
			for(k=0;k<100;k++);
}

void DelayUs(u32 time)
{
   u32 i,j;
	for(i=0;i<time;i++)
		for(j=0;j<10;j++);
}
