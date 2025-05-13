#ifndef __PWM_H
#define __PWM_H

#include "SYS.h"

void PWM_Init(uint16_t pulse);  //占空比

void PWM_Start(void);
void PWM_Stop(void);
void PWM_ARM_Start(void);
void PWM_ARM_Stop(void);


#endif
