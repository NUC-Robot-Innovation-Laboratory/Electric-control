#ifndef _ESP8266_H
#define _ESP8266_H
#include "stm32f10x.h"
#include "usart.h"
#include "delay.h"
//º¯ÊýÉùÃ÷
u8 ESP8266_Init(void);
u8 ESP8266_SendCmd(char *cmd);
u8 ESP8266_AP_TCP_Server_Mode(char *ssid,char *pass,u16 port);
u8 ESP8266_ServerSendData(u8 id,u8 *data,u16 len);
u8 ESP8266_STA_TCP_Client_Mode(char *ssid,char *pass,char *ip,u16 port,u8 flag);
u8 ESP8266_ClientSendData(u8 *data,u16 len);
#endif
