/*********************************************************  
*											PS2手柄头文件
*										手柄接口接线介绍
										SPI2
										PB14 ------------- MISO DO
										PB15 ------------- MOSI DI  配置为输入上拉模式 数据输入线
										PB12 ------------- CS (片选线 低电平有效 如果不想用必须接地)
										PB13 ------------- SCK(CLK)
**********************************************************/

#include "PS2.h"


#define DELAY_TIME  Delay_us(5); 
u16 Handkey;																							 //用于存储按键值
u8 Comd[2]={0x01,0x42};																		 //0x01开始命令，0x42请求数据（根据数据手册）
u8 Data[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //存储接收到的数据
u16 MASK[]={
    PSB_SELECT,
    PSB_L3,
    PSB_R3 ,
    PSB_START,
    PSB_PAD_UP,
    PSB_PAD_RIGHT,
    PSB_PAD_DOWN,
    PSB_PAD_LEFT,
    PSB_L2,
    PSB_R2,
    PSB_L1,
    PSB_R1 ,
    PSB_GREEN,
    PSB_RED,
    PSB_BLUE,
    PSB_PINK
	};	//按键值与按键明
 
/*****************************************************
* @function     PS2手柄引脚初始化
* @param        无
* @file         PS2.c
* @author       hui
* @version      V0.1
* @date         2020.7.29
* @brief        PS2手柄初始化
*****************************************************/
void PS2_Init(void)
{
	#if MY_DRIVER
	//寄存器开发 兼PS2测试程序未修改版
	RCC->APB2ENR|=1<<2;     //使能PORTA时钟
	GPIOA->CRL&=0XFFFF000F; //PA1 2 3推挽输出   
	GPIOA->CRL|=0X00003330;   

	GPIOA->CRL&=0XFFFFFFF0; 
	GPIOA->CRL|=0X00000008; //PA0 设置成输入	默认下拉   	
  #else
	//库函数开发
	
	GPIO_InitTypeDef gpioinit;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_StructInit(&gpioinit);
	//配置通信的引脚 PB12 PB13 PB14
	gpioinit.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14;
	gpioinit.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioinit.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&gpioinit);
	
	//配置MOSI PB15引脚为输入下拉模式
	gpioinit.GPIO_Pin = GPIO_Pin_15;
	gpioinit.GPIO_Mode = GPIO_Mode_IPU;
	gpioinit.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&gpioinit);
	
  #endif	
}


/*****************************************************
* @function     向手柄发送指令
* @param        指令CMD 为8位
* @file         PS2.c
* @author       hui
* @version      V0.1
* @date         2020.7.29
* @brief        无返回值
* 该函数通过控制信号线和时钟线将8位命令数据发送到手柄，
* 并通过数据输入线读取返回数据（或确认发送的数据）。
*****************************************************/
void PS2_Cmd(u8 CMD)
{
	volatile u16 ref=0x01;      //是一个16位的整型变量，用于遍历每一位数据。
	Data[1] = 0;                //用于存储接收到的数据。
	//循环通过 ref 从 0x01 开始，每次左移一位，
	//直到 0x0100（即256）。这样每次循环处理 CMD 的一位。结束后处理了8位刚好是一条命令的长度
	for(ref=0x01;ref<0x0100;ref<<=1)
	{
		//1.按位发送:
		if(ref&CMD)  //检查 CMD 的当前位是否为1。
		{
			GPIO_WriteBit(GPIOB,GPIO_Pin_14,Bit_SET);   //设置控制线为高 输出一位控制位   MISO PB14  输出高电平
		}
		else
		{
			GPIO_WriteBit(GPIOB,GPIO_Pin_14,Bit_RESET); //设置控制线为低   MISO PB14  输出低电平
		}   
		
		//2.时钟信号
		//时钟信号的高低切换，确保数据同步,在时钟线上生成脉冲。
		GPIO_WriteBit(GPIOB,GPIO_Pin_13,Bit_SET);    //时钟拉高

		//在时钟信号的高低切换之间加入延时，确保稳定性。
		Delay_us(5);                                 //延时5us

		GPIO_WriteBit(GPIOB,GPIO_Pin_13,Bit_RESET);  //时钟拉低
		
		Delay_us(5);                                 //延时5us
		
		GPIO_WriteBit(GPIOB,GPIO_Pin_13,Bit_SET);    //时钟拉高
		
		//3.数据接收 读取数据输入线的状态，并将有效数据位保存到 Data[1] 中
		if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15))   //检查数据输入线 DI 的状态。 PB15     
			Data[1] = ref|Data[1];   //如果 DI 为高，则将 ref 的当前位设置到 Data[1] 中 
	}
	Delay_us(16);    //在发送完所有位之后，添加额外的延时（16微秒）
}


/*****************************************************
* @function     手柄模式：①红灯模式 ②绿灯模式
* @param        指令CMD
* @file         PS2.c
* @author       hui
* @version      V0.1
* @date         2020.7.29
* @brief        返回值：0（红灯模式），1（绿灯模式）
*								0x41=模拟绿灯，0x73=模拟红灯
*****************************************************/
u8 PS2_RedLight(void)
{
	#if MY_DRIVER
	//寄存器开发
	CS_L;
	PS2_Cmd(Comd[0]);  //开始命令
	PS2_Cmd(Comd[1]);  //请求数据
	CS_H;
	if( Data[1] == 0X73)   
		return 0 ;
	else 
		return 1;
	#else
	//库函数开发
	GPIO_WriteBit(GPIOB,GPIO_Pin_12,Bit_RESET); //把片选线拉低
	PS2_Cmd(Comd[0]);                           //Comd[0] = 0x01开始命令，
	PS2_Cmd(Comd[1]);                           //Comd[1] = 0x42请求数据
	GPIO_WriteBit(GPIOB,GPIO_Pin_12,Bit_SET);   //把片选线拉高
	if( Data[1] == 0x73)                        //0x73=模拟红灯
		return 0;
	else                                        //0x41=模拟绿灯
		return 1;
	#endif
}


/*****************************************************
* @function     读取手柄数据
* @param        无
* @file         PS2.c
* @author       hui
* @version      V0.1
* @date         2020.7.29
* @brief        无返回值
*****************************************************/
void PS2_ReadData(void)
{
	volatile u8 byte=0;
	volatile u16 ref=0x01;
	GPIO_WriteBit(GPIOB,GPIO_Pin_12,Bit_RESET);         //拉低片选线
	PS2_Cmd(Comd[0]);  					//开始命令
	PS2_Cmd(Comd[1]); 				  //请求数据
	for(byte=2;byte<9;byte++)   //开始接受数据
	{
		for(ref=0x01;ref<0x100;ref<<=1)     //从第一位开始遍历数据 遍历8位
		{
			GPIO_WriteBit(GPIOB,GPIO_Pin_12,Bit_SET);
			Delay_us(5);
			GPIO_WriteBit(GPIOB,GPIO_Pin_12,Bit_RESET);
			Delay_us(5);
			GPIO_WriteBit(GPIOB,GPIO_Pin_13,Bit_SET);     //拉高时钟线
		      if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15))                                    //如果输入引脚读取到了数据
		      Data[byte] = ref|Data[byte];              ////如果 DI 为高，则将 ref 的当前位设置到 Data[1] 中 
		}
        Delay_us(16);
	}
	GPIO_WriteBit(GPIOB,GPIO_Pin_12,Bit_SET);
}

 
/*****************************************************
* @function     处理读取出来的手柄数据
* @param        无
* @file         PS2.c
* @author       hui
* @version      V0.1
* @date         2020.7.29
* @brief        只有一个按键按下时按下为0， 未按下为1
*****************************************************/
u8 PS2_DataKey()
{
	u8 index;

	PS2_ClearData();                  //清理数据
	PS2_ReadData();                   //读取数据

	Handkey=(Data[4]<<8)|Data[3];     //这是16个按键  按下为0， 未按下为1 
	// 合并读取到的数据，Data[3] 和 Data[4] 分别是低字节和高字节
	for(index=0;index<16;index++)     //遍历16个按键
	{	    
		if((Handkey&(1<<(MASK[index]-1)))==0) 
			return index+1;
	}
	return 0;          //没有任何按键按下
}


/*****************************************************
* @function     获取手柄遥感模拟值：范围0~256
* @param        button
* @file         PS2.c
* @author       hui
* @version      V0.1
* @date         2020.7.29
* @brief        返回按键值
*****************************************************/
u8 PS2_AnologData(u8 button)
{
	return Data[button];
}


/*****************************************************
* @function     清除数据缓冲区
* @param        无
* @file         PS2.c
* @author       hui
* @version      V0.1
* @date         2020.7.29
* @brief        
*****************************************************/
void PS2_ClearData()
{
	u8 index;
	for(index=0;index<9;index++)
		Data[index]=0x00;
}


/*****************************************************
* @function     手柄震动
* @param        motor1:右侧小震动电机，0x00关，其他开
*								motor2:左侧大震动电机，0x40~0xFF，电机开，值越大，震动越大
* @file         PS2.c
* @author       hui
* @version      V0.1
* @date         2020.7.29
* @brief        
*****************************************************/
void PS2_Vibration(u8 motor1, u8 motor2)
{             
	GPIO_WriteBit(GPIOB,GPIO_Pin_12,Bit_RESET);  //拉低片选信号线
	Delay_us(16);
  PS2_Cmd(0x01);    //开始命令  0x01是数据开始位
	PS2_Cmd(0x42); 		//请求数据
	PS2_Cmd(0X00);
	PS2_Cmd(motor1);  //右侧小震动电机
	PS2_Cmd(motor2);  //左侧小震动电机
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);        
  GPIO_WriteBit(GPIOB,GPIO_Pin_12,Bit_SET);	
	Delay_us(16);  
}


/*****************************************************
* @function     小幅度震动
*****************************************************/
void PS2_ShortPoll(void)
{
	GPIO_WriteBit(GPIOB,GPIO_Pin_12,Bit_RESET); 
	Delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x42);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x00);
	GPIO_WriteBit(GPIOB,GPIO_Pin_12,Bit_SET);
	Delay_us(16);	
}


/*****************************************************
* @function     PS2配置
*****************************************************/
void PS2_EnterConfing(void)
{
    CS_L;
	Delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01);
	PS2_Cmd(0x00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	delay_us(16);
}


/*****************************************************
* @function     发送模式设置
*****************************************************/
void PS2_TurnOnAnalogMode(void)
{
	CS_L;
	PS2_Cmd(0x01);  
	PS2_Cmd(0x44);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01); 	//analog=0x01;digital=0x00  软件设置发送模式
	PS2_Cmd(0x03);  //Ox03锁存设置，即不可通过按键“MODE”设置模式。
									//0xEE不锁存软件设置，可通过按键“MODE”设置模式。
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	delay_us(16);
}


/*****************************************************
* @function    振动设置
*****************************************************/
void PS2_VibrationMode(void)
{
	CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x4D);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0X01);
	CS_H;
	delay_us(16);	
}


/*****************************************************
* @function    完成并保存配置
*****************************************************/
void PS2_ExitConfing(void)
{
    CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	CS_H;
	delay_us(16);
}


/*****************************************************
* @function    手柄配置初始化
*****************************************************/
void PS2_SetInit(void)
{
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_EnterConfing();			//进入配置模式
	PS2_TurnOnAnalogMode();	//“红绿灯”配置模式，并选择是否保存
	//PS2_VibrationMode();	//开启震动模式
	PS2_ExitConfing();		  //完成并保存配置
}
