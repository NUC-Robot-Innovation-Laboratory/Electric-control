#ifndef _DS18B20_H_
#define _DS18B20_H_

#include "stm32f10x.h"
#include "delay.h"
#include "sys.h"

//IO方向设置  DAT---PB3 
#define DS18B20_IO_IN()  {GPIOB->CRL&=0xFFFF0FFF;GPIOB->CRL|=0x00008000;}
#define DS18B20_IO_OUT() {GPIOB->CRL&=0xFFFF0FFF;GPIOB->CRL|=0x00003000;}
////IO操作函数											   
#define	DS18B20_DQ_OUT PBout(3) 
#define	DS18B20_DQ_IN  PBin(3)  
   	
u8 DS18B20_Init(void);//初始化DS18B20
short DS18B20_Get_Temp(void);//获取温度
void DS18B20_Start(void);//开始温度转换
void DS18B20_Write_Byte(u8 dat);//写入一个字节
u8 DS18B20_Read_Byte(void);//读出一个字节
u8 DS18B20_Read_Bit(void);//读出一个位
u8 DS18B20_Check(void);//检测是否存在DS18B20
void DS18B20_Rst(void);//复位DS18B20    

#endif
