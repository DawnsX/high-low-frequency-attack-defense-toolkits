/*
* Copyright (c) 2015,Yuanjian
* All rights reserved.
*
* 文件名称：board.c
* 文件标识：
* 摘 要：
*
* 当前版本：0.0
* 作 者：   袁舰
* 完成日期：2015年7月17日
*
* 取代版本：
* 原作者:
* 完成日期：
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
#include "board.h"
#include "clock.h"
#include "led.h"
#include "jianPrt.h"
#include "Basic_rf.h"
#include "Hal_rf.h"
#include <stdio.h>
#include <string.h>
#include <myMacros.h>
#include <Key.h>


#include "data.h"
#include<stdlib.h>

/***********************************************************************************
* GLOBAL VARIABLES
*/


extern TASK_COMPONENTS TaskComps[];

extern uint8 ChannelTable[];

uint8 Uart0Rx[1000] = {0};
uint8 Uart0Tx[1000] = {0};
uint8 Uart1Rx[1000] = {0};
uint8 Uart1Tx[1000] = {0};


uint8 *Rx0 = Uart0Rx;
uint8 *Rx1 = Uart1Rx;


enum BuadRate_status buadrate = SLOW9600;
enum ConnectMode connectMode = DIRECT;

struct
{
	uint8 length ;
	uint8 idFirst;
	uint8 idSecond;
	uint8 cmd;
	uint8 numSatellites;
	uint8 signalStrength;
	uint8 lon[4];
	uint8 lat[4];
	uint8 lonShift[4];
	uint8 latShift[4];
}config ={
	21, 	//length
	0xaa,
	0xbb,
	0x05,	//cmd;
	32,		//numSatellites;
	1,		//signalStrength;
	{0x14,0xc0,0xcf,0x33},//everest
	{0x6c,0x9d,0xae,0x10},
	{0},
	{0}
} ;
struct
{
	uint8 numSatellites;
	uint8 signalStrength;
	uint8 lon[4];
	uint8 lat[4];
	uint32 lonShift;
	uint32 latShift;
}hubConfig ={
	32,		//numSatellites;
	1,		//signalStrength;
	{0xCC,0xCF,0xCB,0x54},//japan
	{0x56,0xDA,0xA4,0x19},
	10000000,
	10000000
} ;

/******************************************参数**********************************************************/

extern uint16 Rx0Count;
/*********************************************************************************************************
** 函数名称: Rx0Back
** 功能描述: 串口0收到整包数据后触发，指针回到接收缓存的起始位置
** 输　入:	 无
** 输　出:   无
** 全局变量: Rx0Count		该包数据的字符总数
** 调用模块: 
**
** 作　者: 袁舰
** 日　期: 2015年7月17日
**-------------------------------------------------------------------------------------------------------
** 修改人:
** 日　期:
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
void Rx0Back(void)
{
	uint16 count;
	count = Rx0Count;
	Rx0Count = 0;

	//指针回零
	Rx0 = Uart0Rx;
#ifdef DATA_HUB
	//Uart包解析
	Uart0FrameDecode(count);
#else
	Uart0FrameDecode_Node(count);
#endif
}


/******************************************参数**********************************************************/

extern uint8 checkInterval;
/*********************************************************************************************************
** 函数名称: Uart0FrameDecode
** 功能描述: 串口0的数据解析，即收到MCU数据的解析，修改
** 输　入:	 count				收到数据数量
** 输　出:   无
** 全局变量: checkInterval		两包数据时间间隔，速率不同间隔也不同
** 调用模块: 
**
** 作　者: 袁舰
** 日　期: 2015年7月17日
**-------------------------------------------------------------------------------------------------------
** 修改人:
** 日　期:
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
void Uart0FrameDecode(uint16 count)
{    
	switch(buadrate)
	{
		case SLOW9600:				//待机状态
			 memcpy(Uart1Tx,Uart0Rx,count);
			 if(memcmp(Uart1Tx,data1,28) == 0)
			 {
			 	buadrate = FAST115200;
				checkInterval = Interval_115200;				
				UART1_SendData(Uart1Tx,count);
				ChangeBaudrate();
			 }
			 else
			 {
			 	/*DMA通道3，uart1，DMA通道4,uart0*/
				UARTSendDMAWithSrc(3,count,Uart1Tx);
			 }
			 break;
		case FAST115200:
			 memcpy(Uart1Tx,Uart0Rx,count);
			 /*DMA通道3，uart1，DMA通道4,uart0*/
			 UARTSendDMAWithSrc(3,count,Uart1Tx);
			 break;
		default:
		{
		}
			
	}
}


/*********************************************************************************************************
** 函数名称: BackSpace
** 功能描述: 删除输入的字符
** 输　入:	 count					字符总数
**       	 *buffer				数组指针
** 输　出:   
** 全局变量: 
** 调用模块: 
**
** 作　者: 袁舰
** 日　期: 2015年7月17日
**-------------------------------------------------------------------------------------------------------
** 修改人:
** 日　期:
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
void BackSpace(int16 *count,uint8 *buffer)
{
	if(*count >= 0)
	{
		if(*count == 0)
		{
			buffer[*count] = 0;
			//UART1_SendByte('1');
		}
		else
		{
			//UART1_SendByte('2');
			*count = *count - 1;
			buffer[*count] = 0;
			UART0_SendByte(0x08);
			UART0_SendByte(0x20);
			UART0_SendByte(0x08);
		}
	}
}


/*********************************************************************************************************
** 函数名称: CheckInputStr
** 功能描述: 检查用户输入的经纬度格式是否正确
** 输　入:	 *buffer				数组指针
** 输　出:   0						格式错误
**			 1						格式正确
** 全局变量: 
** 调用模块: 
**
** 作　者: 袁舰
** 日　期: 2015年7月27日
**-------------------------------------------------------------------------------------------------------
** 修改人:
** 日　期:
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
uint8 CheckInputStr(uint8 *buffer)
{
	uint8 pointCount = 0;
	if(buffer[0] == '-')
	{
		if(buffer[1] >= '1' && buffer[1] <= '9')
		{
			for(uint8 i = 2;i < 12;i ++)
			{
				if(buffer[i] == '.')
				{
					pointCount++;
					if(i > 4)
					{
						return 0;
					}

					if(buffer[i+8] != 0)
					{
						return 0;
					}
				}

				if(!((buffer[i] >= '0' && buffer[i] <= '9') || buffer[i] == '.' || buffer[i] == 0))
				{
					return 0;
				}
				
			}
			if(pointCount > 1)
			{
				return 0;
			}
			else if( pointCount == 0)
			{
				if(buffer[4] != 0)
				{
					return 0;
				}
			}
		}
		else
		{
			return 0;
		}

		if(buffer[12] != 0)
		{
			return 0;
		}
		
		return 1;
	}
	else if (buffer[0] >= '0' && buffer[0] <= '9')
	{
		for(uint8 i = 1;i < 11;i ++)
		{
			if(buffer[i] == '.')
			{
				pointCount++;
				if(i > 3)
				{
					return 0;
				}
				
				if(buffer[i+8] != 0)
				{
					printf("1\r\n");
					return 0;
				}
			}

			if(!((buffer[i] >= '0' && buffer[i] <= '9') || buffer[i] == '.' || buffer[i] == 0 ))
			{
				printf("2\r\n");
				return 0;
			}
			
		}
		
		if(pointCount > 1)
		{
			return 0;
		}
		else if( pointCount == 0)
		{
			if(buffer[3] != 0)
			{
				return 0;
			}
		}
		
		if(buffer[11] != 0)
		{
			return 0;
		}

		return 1;
	}
	else
	{
		return 0;
	}

}


/*********************************************************************************************************
** 函数名称: Str2long
** 功能描述: 转换字符串到数字(包括小数点的识别,该数乘以10000000)
** 输　入:   *buffer				数组指针
** 输　出:   
** 全局变量: 
** 调用模块: 
**
** 作　者: 袁舰
** 日　期: 2015年7月27日
**-------------------------------------------------------------------------------------------------------
** 修改人:
** 日　期:
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
long int Str2long(uint8 *buffer)
{
	char *p = NULL;
	long int head;
	long int tail;
	
		
		//得到整数部分，指针p指向缓存中后一个字符
		head = strtol((char const *)buffer,&p,10);

		//处理小数部分
		if(*p == '.')
		{	
			tail = atol(p+1);
			if(*(p+1+7) != 0)
			{
				return 0;
			}
			for(int8 i=6; i >= 0 ; i-- )
			{
				if((*(p+1+i)) == 0)
				{
					tail = tail * 10;
				}
				else
				{
					break;
				}
			}
			//printf("%ld\r\n",tail);
			if(head < 0)
			{
				head = head*10000000 - tail;
			}
			else
			{
				head = head*10000000 + tail;
			}
		}
		else
		{
			head = head*10000000;
		}
		

		return head;

}

/*********************************************************************************************************
** 函数名称: ProcessCmd
** 功能描述: 处理用户的输入指令
** 输　入:   *buffer				数组指针
** 输　出:   
** 全局变量: 
** 调用模块: 
**
** 作　者: 袁舰
** 日　期: 2015年7月27日
**-------------------------------------------------------------------------------------------------------
** 修改人:
** 日　期:
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
void ProcessCmd(uint8 *buffer)
{
	static uint8 menuLevel = 0;
	uint32 temp = 0 ;
	switch(menuLevel)
	{
		case 0:
			if(buffer[1] == 0)  
			{
				switch (buffer[0])
				{
					case '1': //Amount of satellites
						MESSAGE1();
						menuLevel = 1;
						break;
		            case '2': //Longitude
						MESSAGE2();
						menuLevel = 2;
						break;
		            case '3': //Latitude
						MESSAGE3();
						menuLevel = 3;
						break;
					case '4': //Latitude
						MESSAGE4();
						menuLevel = 4;
						break;
					case '5': //Latitude
						MESSAGE5();
						menuLevel = 5;
						break;
					default:
					{
						printf("\r\n\r\nsyntax error!\r\n");
						MESSAGE0();
					}
				} 
			}
			else
			{
				printf("\r\n\r\nsyntax error!\r\n");
				MESSAGE0();
			}
			break;
		case 1:  //Amount of satellites
			if(buffer[0] >= '0' && buffer[0] <= '9' && buffer[1] == 0 )
			{
				config.numSatellites = buffer[0]-'0';
				MESSAGE1();
			}
			else if(buffer[0] >='0' && buffer[0] <= '9' \
				 && buffer[1] >='0' && buffer[1] <= '9' && buffer[2] == 0)
			{
				config.numSatellites = 10*(buffer[0]-'0')+ buffer[1] - '0';
				MESSAGE1();
			}
			else if(memcmp(buffer,"back",4) == 0)
			{
				menuLevel = 0;
				MESSAGE0();
			}
			else
			{
				printf("\r\n\r\nsyntax error!\r\n");
				MESSAGE1();
			}
			break;
		case 2:  //Longitude
			if(memcmp(buffer,"back",4) == 0)
			{
				menuLevel = 0;
				MESSAGE0();
				break;
			}
			//if((buffer[0] >='0' && buffer[0] <='9') || buffer[0] == '-' )
			if(CheckInputStr(buffer))
			{
				temp = Str2long(buffer);

				if((long int)temp>1800000000 || (long int)temp < -1800000000)
				{
					printf("\r\n\r\nsyntax error!\r\n");
					MESSAGE2();
					break;
				}
				
				config.lon[0] = (uint8)(temp & 0xff); 
				config.lon[1] = (uint8)((temp>>8)&0xff);
				config.lon[2] = (uint8)((temp>>16)&0xff);
				config.lon[3] = (uint8)((temp>>24)&0xff);
				MESSAGE2();
				break;
		
			}
			else
			{
				printf("\r\n\r\nsyntax error!\r\n");
				MESSAGE2();
			}


			break;
		case 3:	//latitude
			if(memcmp(buffer,"back",4) == 0)
			{
				menuLevel = 0;
				MESSAGE0();
				break;
			}
			//if((buffer[0] >='0' && buffer[0] <='9') || buffer[0] == '-' )
			if(CheckInputStr(buffer))
			{
				temp = Str2long(buffer);
				
				if((long int)temp>900000000 || (long int)temp < -900000000)
				{
					printf("\r\n\r\nsyntax error!\r\n");
					MESSAGE3();
					break;
				}
				
				config.lat[0] = (uint8)(temp & 0xff); 
				config.lat[1] = (uint8)((temp>>8)&0xff);
				config.lat[2] = (uint8)((temp>>16)&0xff);
				config.lat[3] = (uint8)((temp>>24)&0xff);

				MESSAGE3();
			}
			else
			{
				printf("\r\n\r\nsyntax error!\r\n");
				MESSAGE3();
			}

			break;
		case 4:  //Longitude Shift
			if(memcmp(buffer,"back",4) == 0)
			{
				menuLevel = 0;
				MESSAGE0();
				break;
			}
			//if((buffer[0] >='0' && buffer[0] <='9') || buffer[0] == '-' )
			if(CheckInputStr(buffer))
			{
				temp = Str2long(buffer);
				
				config.lonShift[0] = (uint8)(temp & 0xff); 
				config.lonShift[1] = (uint8)((temp>>8)&0xff);
				config.lonShift[2] = (uint8)((temp>>16)&0xff);
				config.lonShift[3] = (uint8)((temp>>24)&0xff);
				MESSAGE4();
			}
			else
			{
				printf("\r\n\r\nsyntax error!\r\n");
				MESSAGE4();
			}
			break;
		case 5:  //latitude Shift
			if(memcmp(buffer,"back",4) == 0)
			{
				menuLevel = 0;
				MESSAGE0();
				break;
			}
			//if((buffer[0] >='0' && buffer[0] <='9') || buffer[0] == '-' )
			if(CheckInputStr(buffer))
			{
				temp = Str2long(buffer);
				
				config.latShift[0] = (uint8)(temp & 0xff); 
				config.latShift[1] = (uint8)((temp>>8)&0xff);
				config.latShift[2] = (uint8)((temp>>16)&0xff);
				config.latShift[3] = (uint8)((temp>>24)&0xff);
				MESSAGE5();
			}
			else
			{
				printf("\r\n\r\nsyntax error!\r\n");
				MESSAGE5();
			}

			break;
		default:
		{
		}
			
	}
	memset(buffer,0,INPUTLENGTH);
}


/******************************************参数**********************************************************/

uint8 inputBuffer[INPUTLENGTH] = {0};//用户输入缓存
/*********************************************************************************************************
** 函数名称: InputProcess
** 功能描述: 处理用户的每个按键输入。(回车开始处理整个输入)
** 输　入:   *buffer				数组指针
** 输　出:   
** 全局变量: 
** 调用模块: 
**
** 作　者: 袁舰
** 日　期: 2015年7月27日
**-------------------------------------------------------------------------------------------------------
** 修改人:
** 日　期:
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
void InputProcess(uint8 *buffer)
{
	static int16 count = 0;
	//static int16 pos = 0;
	//static int16 posMax = 0;
	switch(*buffer)
	{
		case 0x08: //(back space)

			BackSpace(&count,inputBuffer);

			break;
		case 0x7f: //(Del)
			
			break;
		case 0x0d: //(Enter)
				UART0_SendByte(0x0d);
				UART0_SendByte(0x0a);
				count = 0;
				ProcessCmd(inputBuffer);
			break;
		case 27: //(Direction)

			break;
		default:
			{
				if(count + 1 < INPUTLENGTH)
				{
					if((*buffer>=0x20)&&(*buffer<=127))		// printable??
					{	
						UART0_SendByte(*buffer);				// echo back and store it
						inputBuffer[count] = *buffer;
						count++;
						//UART1_SendByte('3');
					}
				}
			}
	}
}



/*********************************************************************************************************
** 函数名称: Uart0FrameDecode_Node
** 功能描述: 串口0的数据解析
** 输　入:	 count				收到数据数量
** 输　出:   无
** 全局变量:
** 调用模块: 
**
** 作　者: 袁舰
** 日　期: 2015年7月20日
**-------------------------------------------------------------------------------------------------------
** 修改人:
** 日　期:
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
void Uart0FrameDecode_Node(uint16 count)
{  
//printf("%d",*Uart0Rx);
//printf("%d",*(Uart0Rx+1));
//printf("%d",*(Uart0Rx+2));
//printf("%d",*(Uart0Rx+3));
//memset(Uart0Rx,0,50);

	InputProcess(Uart0Rx);



	//Str2long(test);


	//PrintLeft(10,5,test);
	//printf("\r\n%d\r\n",posLeft(3,test));

}


/******************************************参数**********************************************************/

extern uint16 Rx1Count;
/*********************************************************************************************************
** 函数名称: Rx1Back
** 功能描述: 串口1收到整包数据后触发
** 输　入:	 无
** 输　出:   无
** 全局变量: Rx1Count		该包数据的字符总数
** 调用模块: 
**
** 作　者: 袁舰
** 日　期: 2015年7月17日
**-------------------------------------------------------------------------------------------------------
** 修改人:
** 日　期:
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
void Rx1Back(void)
{ 
	uint16 count;
  	count = Rx1Count;
  	Rx1Count = 0;
  	Rx1 = Uart1Rx;
	//Uart包解析
	Uart1FrameDecode(count);
}


/*********************************************************************************************************
** 函数名称: ChangeNumofSatellites
** 功能描述: 改变卫星数量
** 输　入:	 num  	卫星的数量
**      	 *data	指向给MCU处理GPS数据包的指针
** 输　出:   无
** 全局变量: 
** 调用模块: 
**
** 作　者: 袁舰
** 日　期: 2015年7月27日
**-------------------------------------------------------------------------------------------------------
** 修改人:
** 日　期:
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
void ChangeNumofSatellites(uint8 num,uint8 *data)
{
	if(*data == 0xb5 && *(data+1) == 0x62 && *(data+2) == 0x01 && *(data + 3) == 0x06 && *(data+4)  == 0x34)
	{
		data[53] = num;
		checkSum(data+Length(data)-2,data+Length(data)-1,data);
	}

}


/*********************************************************************************************************
** 函数名称: ChangePositon
** 功能描述: 改变卫星数量
** 输　入:	 *src		指向给MCU处理GPS数据包的指针
**      	 *replace	指向替换坐标的指针(总8字节,经度和纬度)
** 输　出:   无
** 全局变量: 
** 调用模块: 
**
** 作　者: 袁舰
** 日　期: 2015年7月27日
**-------------------------------------------------------------------------------------------------------
** 修改人:
** 日　期:
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
void ChangePositon(uint8 *src,uint8 *replace)
{
	uint16 pos ;
	pos = Length(src + Length(src+Length(src)) + Length(src)) + Length(src+Length(src)) + Length(src);
	memcpy(src+pos + 10,replace,8);
	checkSum(src +pos +Length(src +pos) - 2,src +pos +Length(src +pos) - 1,src+pos);
}


/******************************************参数**********************************************************/

//uint8 ConnectMode = 2;
/*********************************************************************************************************
** 函数名称: Uart1FrameDecode
** 功能描述: 串口1的数据解析，即收到GPS模块数据的解析，修改
** 输　入:	 count				收到数据数量
** 输　出:   无
** 全局变量: ConnectMode		状态量 模式切换
** 调用模块: 
**
** 作　者: 袁舰
** 日　期: 2015年7月17日
**-------------------------------------------------------------------------------------------------------
** 修改人:
** 日　期:
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
void Uart1FrameDecode(uint16 count)
{	
	memcpy(Uart0Tx,Uart1Rx,count);

	switch(connectMode)
	{
		case DIRECT:				//初始状态
			 UARTSendDMAWithSrc(4,count,Uart0Tx);
			 break;
		case REPLACE:

			if(buadrate == FAST115200)
			{	
				ChangeNumofSatellites(hubConfig.numSatellites,dataAll);
				ChangePositon(dataAll,&hubConfig.lon[0]);
				//ChangePositon(dataAll,japan);
				UARTSendDMAWithSrc(4,390,(uint8 *)dataAll);
			}
			else
			{
				UARTSendDMAWithSrc(4,count,Uart0Tx);
			}
			break;
		case SHIFT:
			PositionAdjustableShift(Uart0Tx ,hubConfig.lonShift,hubConfig.latShift);
			
			UARTSendDMAWithSrc(4,count,Uart0Tx);

			break;
		default:
		{
		}
			
	}

}


/*********************************************************************************************************
** 函数名称: putchar
** 功能描述: 重定向系统的printf函数
** 输　入:	 无
** 输　出:   无
** 全局变量: 无
** 调用模块: 
**
** 作　者: 袁舰
** 日　期: 2015年7月17日
**-------------------------------------------------------------------------------------------------------
** 修改人:
** 日　期:
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
#if 1
int putchar(int c){  
    //if(c== '\n'){  
    //  while(!UTX0IF);  
    //  UTX0IF = 0;  
    //  U0DBUF = '\r';  
    //} 
#if 0
    while(!UTX0IF);  
    UTX0IF = 0;  
    return (U0DBUF = c);  
#endif
	U0DBUF = c;
	while(!UTX0IF);  
    UTX0IF = 0;  
    return (c);  
}  
#endif


/*********************************************************************************************************
** 函数名称: Timer4_Init
** 功能描述: 初始化用于系统的定时器(1ms 产生中断)
** 输　入:	 无
** 输　出:   无
** 全局变量: 无
** 调用模块: 
**
** 作　者: 袁舰
** 日　期: 2015年7月17日
**-------------------------------------------------------------------------------------------------------
** 修改人:
** 日　期:
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
void Timer4_Init(void)
{
	

   uint8 timer4Period = 30;
   TIMER34_INIT(4); 
   timer4Period = SetTimer34Period(4, 1000);
   if(timer4Period != 0)
   	{
      TIMER34_ENABLE_OVERFLOW_INT(4,INT_ON);
	  T4CCTL0|=0x04;
      INT_ENABLE(INUM_T4, INT_ON);
      TIMER4_RUN(TRUE);
	 }

}


/*********************************************************************************************************
** 函数名称: UART_Init
** 功能描述: 初始化串口
** 输　入:	 无
** 输　出:   无
** 全局变量: 无
** 调用模块: 
**
** 作　者: 袁舰
** 日　期: 2015年7月17日
**-------------------------------------------------------------------------------------------------------
** 修改人:
** 日　期:
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
void UART_Init(void)
{
	P2DIR &= ~0xC0;//端口0的外设优先级，依次UART0,UART1，定时器1
  	P2DIR |= 0x00;//输入

	/*初始化串口0*/
	IO_PER_LOC_USART0_AT_PORT0_PIN2345();
#ifdef DATA_HUB
	UART_SETUP(0, 9600, HIGH_STOP);
#else
	UART_SETUP(0, 115200, HIGH_STOP);
#endif
	UTX0IF = 0;
	URX0IF = 0;//清除串口接收中断标志
	URX0IE = 1;//接收中断使能
	U0CSR |=0x40;

#ifdef DATA_HUB
	/*初始化串口1*/
	IO_PER_LOC_USART1_AT_PORT1_PIN4567();
	UART_SETUP(1, 9600, HIGH_STOP);

	UTX1IF = 0;//清除串口发送中断标志
	URX1IF = 0;//清除串口接收中断标志
	URX1IE = 1;//接收中断使能
	U1CSR |=0x40;

#endif

}


/*********************************************************************************************************
** 函数名称: ChangeBaudrate
** 功能描述: 改变UART0,UART1的波特率到115200(权宜的简单操作)
** 输　入:	 无
** 输　出:   无
** 全局变量: 无
** 调用模块: 
**
** 作　者: 袁舰
** 日　期: 2015年7月17日
**-------------------------------------------------------------------------------------------------------
** 修改人:
** 日　期:
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
void ChangeBaudrate(void)
{ 
		U0GCR  = 11;
		U0BAUD = 216;

		U1GCR  = 11;
		U1BAUD = 216;
}


/*********************************************************************************************************
** 函数名称: UART0_SendByte
** 功能描述: UART0发送字节
** 输　入:	 data			发送字节
** 输　出: 无
** 全局变量: 无
** 调用模块: 
**
** 作　者: 袁舰
** 日　期: 2015年7月17日
**-------------------------------------------------------------------------------------------------------
** 修改人:
** 日　期:
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
void UART0_SendByte(uint8 data)
{
	U0CSR &= ~BIT1;
	U0DBUF = data;                  //发送字节数据  
    while(!(U0CSR & BIT1));         //等待发送数据寄存器为空                 
    U0CSR &= ~BIT1;
}


/*********************************************************************************************************
** 函数名称: UART1_SendByte
** 功能描述: UART1发送字节
** 输　入:	 data			发送字节
** 输　出: 无
** 全局变量: 无
** 调用模块: 
**
** 作　者: 袁舰
** 日　期: 2015年7月17日
**-------------------------------------------------------------------------------------------------------
** 修改人:
** 日　期:
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/ 
void UART1_SendByte(uint8 data)
{
	U1CSR &= ~BIT1;
	U1DBUF = data;                  //发送字节数据  
    while(!(U1CSR & BIT1));         //等待发送数据寄存器为空                 
    U1CSR &= ~BIT1;
} 


/*********************************************************************************************************
** 函数名称: UART0_SendData
** 功能描述: UART0发送字符串
** 输　入:	 *pbuff			待发送字符串首地址
**			 len			字符串长度
** 输　出: 无
** 全局变量: 无
** 调用模块: 
**
** 作　者: 袁舰
** 日　期: 2015年7月17日
**-------------------------------------------------------------------------------------------------------
** 修改人:
** 日　期:
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/  
void UART0_SendData(uint8 *pbuff, uint16 len)  
{  
    uint16 i;
	//发送前清空标志，避免错误
	U0CSR &= ~BIT1;
	for(i = 0;i < len;i ++)  
	{  
	    U0DBUF = pbuff[i];                  //发送字节数据  
	    while(!(U0CSR & BIT1));         //等待发送数据寄存器为空                 
	    U0CSR &= ~BIT1;  
	}      
}

/*********************************************************************************************************
** 函数名称: UART1_SendData
** 功能描述: UART1发送字符串
** 输　入:	 *pbuff			待发送字符串首地址
**			 len			字符串长度
** 输　出: 无
** 全局变量: 无
** 调用模块: 
**
** 作　者: 袁舰
** 日　期: 2015年7月17日
**-------------------------------------------------------------------------------------------------------
** 修改人:
** 日　期:
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
void UART1_SendData(uint8 *pbuff, uint16 len)  
{  
    uint16 i; 
	//发送前清空标志，避免错误
	U1CSR &= ~BIT1;
	for(i = 0;i < len;i ++)  
	{  
	    U1DBUF = pbuff[i];                  //发送字节数据  
	    while(!(U1CSR & BIT1));         //等待发送数据寄存器为空                 
	    U1CSR &= ~BIT1;  
	}      
}  


/*********************************************************************************************************
** 函数名称: ChangeGpsData
** 功能描述: 替代数据包中相应数据
** 输　入:   *src   		数据包的地址
**           *replace 		替代字段的起始地址
**           count   		替代字段的长度
** 输　出:   无
** 全局变量: 无
** 调用模块: 
**
** 作　者: 袁舰
** 日　期: 2015年7月17日
**-------------------------------------------------------------------------------------------------------
** 修改人:
** 日　期:
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
uint16 ChangeGpsData(uint8 *src, uint8 *replace,uint16* count)  
{  
   uint16 pos =  0;
 
   if(*src == 0xb5 && *(src+1) == 0x62 && *(src+2) == 0x01 && *(src + 3) == 0x06)
   {
	switch(*(replace+3))
	{
		case 06:
			if(*(src+4)  == 0x34) 
			{
#if 1
			 memcpy(src,replace,60);
			 checkSum(src+Length(src)-2,src+Length(src)-1,src);
			 //UART1_SendByte('a');
#else 
#endif
			}
			break;
		case 35:
			//pos = Length(src);
			break;
		case 03:
			pos = Length(src+Length(src)) + Length(src);
			break;
		case 02:
			pos = Length(src + Length(src+Length(src)) + Length(src)) + Length(src+Length(src)) + Length(src);
			memmove(src + pos + Length(replace), src + pos + *(src+pos+4)+8,*count- pos - (*(src+pos+4)+8));
			memcpy(src+pos,replace,Length(replace));
			
			
			//UART1_SendByte(*count>>8);
			//UART1_SendByte(*count&0xff);
			break;
		case 04:
			pos = Length(src + Length(src+Length(src)) + Length(src)) + Length(src+Length(src)) + Length(src);
			pos = pos + Length(src+pos);
			memcpy(src+pos,dop,Length((uint8 *)dop));
			break;
		case 12:
			break;
		case 21:
			break;
		default:
			;
	}
	
   }
   return 0;
}

/*********************************************************************************************************
** 函数名称: Length
** 功能描述: 计算每包数据中，以0xb5 0x62开头字段的长度
** 输　入:   src   数据包中字段的地址
** 输　出: 无
** 全局变量: 无
** 调用模块: 
**
** 作　者: 袁舰
** 日　期: 2015年7月17日
**-------------------------------------------------------------------------------------------------------
** 修改人:
** 日　期:
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
uint16  Length(uint8 *src)
{
	return *(src+4) + 8;
}

/*********************************************************************************************************
** 函数名称: PositionAdjustableShift
** 功能描述: 对获得的经纬度进行相对偏移
** 输　入:   src   数据包的地址
**			 lon   经度(已经被转换为二进制补码，10000000(十进制)表示一度)
**           lat   纬度
** 输　出: 无
** 全局变量: 无
** 调用模块: 
**
** 作　者: 袁舰
** 日　期: 2015年7月17日
**-------------------------------------------------------------------------------------------------------
** 修改人:
** 日　期:
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
void PositionAdjustableShift(uint8 *src ,unsigned long int lon,unsigned long int lat)
{
	unsigned long int num ;
	//unsigned char i = 0;
	uint16 offset;

	/*确定该包数据是GPS模块发出的数据包*/
	if(*src == 0xb5 && *(src+1) == 0x62 && *(src+2) == 0x01 && *(src + 3) == 0x06)
	{   /*计算出0x01 0x02字段的偏移*/
		offset = Length(src + Length(src+Length(src)) + Length(src)) + Length(src+Length(src)) + Length(src);
		src = src +offset;
		
		/*计算经度*/
		num = lon +((unsigned long int)src[13]<<24)+((unsigned long int)src[12]<<16)+((unsigned long int)src[11]<<8)+src[10];
		src[10] =  (uint8)(num&0xff);
		src[11] =  (uint8)((num>>8)&0xff);
		src[12] =  (uint8)((num>>16)&0xff);
		src[13] =  (uint8)((num>>24)&0xff);
		
		/*计算纬度*/
		num = lat + ((unsigned long int)src[17]<<24)+((unsigned long int)src[16]<<16)+((unsigned long int)src[15]<<8)+src[14];
		src[14] =  (uint8)(num&0xff);
		src[15] =  (uint8)((num>>8)&0xff);
		src[16] =  (uint8)((num>>16)&0xff);
		src[17] =  (uint8)((num>>24)&0xff);

		/*计算校验和*/
		checkSum(src+34,src+35,src);
	}
}


/*********************************************************************************************************
** 函数名称: checkSum
** 功能描述: 设置T3或者T4的间隔时间
** 输　入: 无
** 输　出: 无
** 全局变量: 无
** 调用模块: 
**
** 作　者: 袁舰
** 日　期: 2015年7月17日
**-------------------------------------------------------------------------------------------------------
** 修改人:
** 日　期:
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
void checkSum(uint8 *CK_A,uint8 *CK_B,uint8 *buffer)
{
	uint8 i;
	uint8 n;
	uint16 CA = 0,CB = 0;
	n = *(buffer+4) + 4 ;
	for(i = 0;i < n ;i++)
	{
		CA = CA + *(buffer+i+2);
		CB = CB + CA;
	}
	*CK_A = (uint8)(CA & 0xFF);
	*CK_B = (uint8)(CB & 0xff);
}


/*********************************************************************************************************
** 函数名称: SetTimer34Period
** 功能描述: 设置T3或者T4的间隔时间
** 输　入: 	 timer  定时器
**           period 间隔时间(单位us)
** 输　出:   间隔时间
** 全局变量: 无
** 调用模块: 
**
** 作　者: 袁舰
** 日　期: 2015年7月17日
**-------------------------------------------------------------------------------------------------------
** 修改人:
** 日　期:
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
uint8 SetTimer34Period(uint8 timer, uint32 period){

		uint8 div = 0;		

		if(CC2530_GET_TICKSPD() > 5) { // Checking that the period is not too short.
			if( (period < 2*(CC2530_GET_TICKSPD()-5)) && (period != 0) ){
			   return 0;
		    }
		}

		if(period == 0){  // If period is 0, max period length and max prescaler
				div = 7;  // division is used.
				period = 255;
		} else {
				period = ((period*32) >> CC2530_GET_TICKSPD());// Determining how many timer ticks the period consist of
				while(period > 255){			  // If the period is too long, the prescaler division is
						period = (period >> 1);   // increased.
						div++;
						if(div > 7){			  // If the period is too long when using max prescaler division,
								return 0;		  // 0 is returned.
						}
				}
		}
		if(timer == 4){
				// Timer 4 selected
				T4CTL |= (div << 5);			  // Setting prescaler value
				//T4CTL |= (5 << 5);
				T4CC0 = (uint8) period;			  // Setting timer value.
				//T4CC0 = (uint8) 200;
				//return 250;
		} else if(timer == 3){
				// Timer 3 selected
				T3CTL |= (div << 5);			  // Setting prescaler value
				T3CC0 = (uint8) period;			  // Setting timer value.
		} else {return 0;}
		
		return period;


}


/*********************************************************************************************************
** 函数名称: BoardInit
** 功能描述: 板级外设初始化
** 输　入: 无
** 输　出: 无
** 全局变量: 无
** 调用模块: 
**
** 作　者: 袁舰
** 日　期: 2015年7月17日
**-------------------------------------------------------------------------------------------------------
** 修改人:
** 日　期:
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
void BoardInit(void)
{	
	INTERRUPT_OFF();
	
	//时钟初始化
    clockSetMainSrc(CLOCK_SRC_XOSC);

	//T4初始化
	Timer4_Init();

	//USART初始化
	UART_Init();

	DMAInit();
	
#ifdef  DATA_HUB
	LedTestInit();
#else
	MESSAGE0();
#endif
	LedInit();
	

	// Interrupt Priority
	INIT_INTERRUPT_PRIORITY( );

	INTERRUPT_ON(); 

}


