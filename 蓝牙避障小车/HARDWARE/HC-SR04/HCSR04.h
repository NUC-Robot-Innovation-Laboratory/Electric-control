#ifndef __HCSR04__H
#define __HCSR04__H
#include "SYS.h"
#include "TIM.h"

//extern float distance; 

//void HCSR04_Init(void);
//void HCSR04_trig_Init(void);
//void Trig_Enable(void);
//void HCSR04_EXTI1_Init(void);
//void ECHO_Init(void);

void HCSR04_Init(void);
void HCSR04_Start(void);
uint16_t HCSR04_GetValue(void);

void HCSR04_SG90_Init(void);
void Bee_Init(void);
//void PWM_SetCompare2(uint16_t compare);


#endif
