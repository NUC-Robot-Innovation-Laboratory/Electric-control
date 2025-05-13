#ifndef __MEROBOTICARM__H
#define __MEROBOTICARM__H

#include "SYS.h"

#define		ARM_Start()			PWM_ARM_Start()			//start motor without changing moving direction														
#define		ARM_Stop()			PWM_ARM_Stop()

void MEARM_Init(void);
void PWM_SetCompare1(uint16_t compare);
void PWM_SetCompare2(uint16_t compare);
void PWM_SetCompare3(uint16_t compare);
void PWM_SetCompare4(uint16_t compare);



#endif
