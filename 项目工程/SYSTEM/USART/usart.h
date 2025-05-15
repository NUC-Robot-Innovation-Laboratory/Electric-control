#ifndef _USART_H
#define _USART_H
#include "stm32f10x.h"
#include <stdio.h>
#include <stdarg.h>
#include "sys.h"
#include <string.h>

#define USART1_RX_LENGTH 100
extern u8 USART1_RX_BUFFER[USART1_RX_LENGTH]; //保存接收数据的缓冲区
extern u32 USART1_RX_CNT;  //当前接收到的数据长度
extern u8 USART1_RX_FLAG; //1表示数据接收完毕 0表示没有接收完毕

#define USART2_RX_LENGTH 500
extern u8 USART2_RX_BUFFER[USART2_RX_LENGTH]; //保存接收数据的缓冲区
extern u32 USART2_RX_CNT;  //当前接收到的数据长度
extern u8 USART2_RX_FLAG; //1表示数据接收完毕 0表示没有接收完毕

#define USART3_RX_LENGTH 500
extern u8 USART3_RX_BUFFER[USART3_RX_LENGTH]; //保存接收数据的缓冲区
extern u32 USART3_RX_CNT;  //当前接收到的数据长度
extern u8 USART3_RX_FLAG; //1表示数据接收完毕 0表示没有接收完毕

void USART5_Init(u32 baud);
void USART1_Init(u32 baud);
void USART2_Init(u32 baud);
void USART3_Init(u32 baud);
void USARTx_StringSend(USART_TypeDef *USARTx,char *str);
void USARTx_DataSend(USART_TypeDef *USARTx,u8 *data,u32 len);
void USARTx_SendOneByte(USART_TypeDef *USARTx,u8 data);

#endif
