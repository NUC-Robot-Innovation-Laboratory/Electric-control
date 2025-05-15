#include "stm32f10x.h"                  // Device header
#include "led.h"                        //led外设 RCT6自己的led灯
#include "delay.h"                      //延时和中断
#include "key.h" 
#include "usart.h"                      //串口
#include "timer.h"                      //定时器
#include "esp8266.h"                    //ESP8266的配置
#include "ds18b20.h"                    //DS18B20水温传感器
//#include "oled.h"                       //SPI协议 OLED显示屏驱动
#include <string.h>
#include <stdlib.h>
#include <iostream>
using namespace std;
//温度值
u16 DS18B20=0;
char DS18B20_str[10];
float temp_data;       //温度数据
float avg_tempture;    //当前的温度
float temperatures[10];  //记录10次温度值后使用PID算法得到较准确的值

//临时数据存放缓冲区
char tmp_buff[100];
//温度加热的上限的阈值
int temp_max=30;

int ESP8266_ConnectState=0;//Esp8266的连接状态 1表示连接 0表示断开
//JTAG模式设置，用于设置JTAG的模式
//mode:jtag swd模式设置:00 全使能:01 使能swd:10 全关闭
#define SWD_ENABLE       0x01   //SWD使能
#define JTAG_SWD_ENABLE  0x00   //JTAG模式 SWD使能
#define JTAG_SWD_DISABLE 0x02   //JTAG模式 SWD失能

void JTAG_Set(u8 mode)
{
	u32 temp;
	temp=mode;
	temp<<=25;
	RCC->APB2ENR|=1<<0;     //开启辅助时钟
	AFIO->MAPR&=0xF8FFFFFF; //清除MAPR的[26:24]
	AFIO->MAPR|=temp;      //设置JTAG模式
}



//PID算法的实现

int main(void)
{
   while(1)
   {
	   
	   
	   
   }	   
}
