#include "Bluetooth.h"

extern uint8_t rx_end;
extern uint16_t rx_data;


//USART1
//PA9(TX) -------- À¶ÑÀRX
//PA10(RX)-------- À¶ÑÀTX

/**
  *@brief  À¶ÑÀÄ£¿é´®¿Ú1µÄ³õÊ¼»¯
	*@param  NONE
	*@retval NONE
	*/
void Bluetooth_USART1_Init(void)
{
	GPIO_InitTypeDef gpioinit;
	USART_InitTypeDef usartinit;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	
	//ÅäÖÃTXÒı½Å PA10  PA10(RX)
	gpioinit.GPIO_Pin = GPIO_Pin_10;
	gpioinit.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpioinit.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&gpioinit);

   //ÅäÖÃRXÒı½Å PA9
  gpioinit.GPIO_Pin = GPIO_Pin_9;
	gpioinit.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioinit.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&gpioinit);
	
	//´®¿ÚÍ¨ĞÅUSART1
	usartinit.USART_BaudRate = 115200;
	usartinit.USART_WordLength =USART_WordLength_8b;
	usartinit.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usartinit.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
	usartinit.USART_Parity = USART_Parity_No;
	usartinit.USART_StopBits = USART_StopBits_1;
	
	USART_Init(USART1,&usartinit);
	Bluetooth_NVIC_Init();
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);    //Ê¹ÄÜ½ÓÊÕÖĞ¶Ï
	USART_Cmd(USART1,ENABLE);
}

void Bluetooth_MOTOR(void)
{
	if(rx_end)
	{
		rx_end = 0;
		GPIO_WriteBit(GPIOD,GPIO_Pin_2,Bit_RESET);
		if(rx_data == 'F')  //Ç°½ø
		{
			Motor_Start();
			MOTOR_Forward();
			Motor_SetSpeed(1500);
			GPIO_WriteBit(GPIOD,GPIO_Pin_2,Bit_SET);
		}
		else if(rx_data == 'B')  //ºóÍË
		{
			Motor_Start();
			MOTOR_Backward();
			Motor_SetSpeed(1800);
			GPIO_WriteBit(GPIOD,GPIO_Pin_2,Bit_RESET);
		}
		else if(rx_data == 'L')  //×ó×ª
		{
			Motor_Start();
			MOTOR_leftward();
			Motor_SetSpeed(1800);   //ç°åœ¨æ”¹å·¦è½¬çš„é€Ÿåº¦æ”¹è¿™ä¸ªæ•°å€¼å¤§å°  åŠ ä¸€ä¸‹è¿™æ®µè¯  å¹…åº¦è¿˜å°å°±å¾€å¤§è°ƒ   å¹…åº¦å¤ªå¤§å°±å¾€å°è°ƒ
		}
		else if(rx_data == 'R')  //ÓÒ×ª
		{
			Motor_Start();
			MOTOR_rightward();
			Motor_SetSpeed(1800);
		}
		else if(rx_data == 'S')   //É²³µ
		{
			Motor_Stop();
		}
		else if(rx_data == '4')   //»úĞµ±Ûµ×²¿Ïò×ó×ª
		{
			ARM_Start();
			MEARM_Init();
			PWM_SetCompare4(500);
		}
		else if(rx_data ==  '6')   //»úĞµ±Ûµ×²¿ÏòÓÒ×ª
		{
			ARM_Start();
			MEARM_Init();
			PWM_SetCompare4(1800);
		}
		else if(rx_data == '8')   //¼Ğ×Ó´ò¿ª  //1800->1000   ¿ª-ºÏ
		{
			ARM_Start();
			MEARM_Init();
			PWM_SetCompare3(1200);
		}
		else if(rx_data == '2')   //¼Ğ×Ó±ÕºÏ
		{
			ARM_Start();
			PWM_SetCompare3(700);
		}
		else if(rx_data == '7')   //×ó¶æ»úÏÂ½µ(Ç°Éì)   //600->1500   ½µ-Éı
		{
			ARM_Start();
			PWM_SetCompare1(600);
		}
		else if(rx_data == '1')   //×ó¶æ»úÉÏÉı(ºóÍË)
		{
			ARM_Start();
			PWM_SetCompare1(1500);
		}
		else if(rx_data == '9')   //ÓÒ¶æ»úÏÂ½µ(Ç°Éì)  //2500->1800  ½µ-Éı
		{
			ARM_Start();
			PWM_SetCompare2(2500);
		}
		else if(rx_data == '3')   //ÓÒ¶æ»úÉÏÉı(ºóÍË)
		{
			ARM_Start();
			PWM_SetCompare2(1800);
		}
	}
}
