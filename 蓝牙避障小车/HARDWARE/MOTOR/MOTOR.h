#ifndef __MOTOR_H
#define __MOTOR_H


#include "SYS.h"
#include "LED.h"
#include "TIM.h"
#include "PWM.h"

#define		PWM_SetRatio(x)		(TIM3->CCR1=(x),TIM3->CCR2=(x),TIM3->CCR3=(x),TIM3->CCR4=(x))
#define		Motor_Start()			PWM_Start()			//start motor without changing moving direction														
#define		Motor_Stop()			PWM_Stop()
#define		Motor_SetSpeed(x)		PWM_SetRatio(x)  //x in the range of  [300, 900]

void MOTOR_Init(void);
void MOTOR_Forward(void);
void MOTOR_Backward(void);
void MOTOR_leftward(void);
void MOTOR_rightward(void);

#endif
