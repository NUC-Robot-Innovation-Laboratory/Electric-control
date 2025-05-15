#ifndef _TIMER_H
#define _TIMER_H
#include "stm32f10x.h"
#include "sys.h"
#include "usart.h"

void TIMER1_Init(u16 psc,u16 arr);
void TIMER2_Init(u16 psc,u16 arr);
void TIMER3_Init(u16 psc,u16 arr);
void TIMER4_Init(u16 psc,u16 arr);
#endif
