#ifndef _KEY_H
#define _KEY_H
#include "stm32f10x.h"
#include "sys.h"
#include "delay.h"

#define KEY0 PAin(0)
#define KEY1 PCin(5)
#define KEY2 PAin(15) 

//º¯ÊıÉùÃ÷
void KEY_Init(void);
u8 KEY_Scan(void);

#endif
