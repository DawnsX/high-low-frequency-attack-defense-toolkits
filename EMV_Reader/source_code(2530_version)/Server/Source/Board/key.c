/***********************************************************************************
    Filename: Key.c

***********************************************************************************/
#include "key.h"
#include "Led.h"
//#include "board.h"
#include <stdio.h>
#include <myMacros.h>
#include "Hal_rf.h"
#include "basic_rf.h"
#include <string.h>

extern TASK_COMPONENTS TaskComps[];
extern uint8 ChannelTable[];

/***********************************************************************************
* @fn		   KeyInit
*
* @brief	   初始化KEY相关IO口

*
* @param	   none
*
* @return	   none
*/
void KeyInit(void)
{
	MCU_IO_INPUT(0, 1, MCU_IO_PULLUP);
	//P0SEL |= BV(7);
	//MCU_IO_INPUT(0, 7, MCU_IO_PULLDOWN);
	//APCFG =  BV(1);
	//PICTL |=0x01;	//P0口下降沿产生中断
	//P0IEN |=0x02;	//P01口中断使能
	//IEN1 |= 0X20;	// 开P0口总中断
	//P0IFG |= 0x00;     //清中断标志
}

/***********************************************************************************
* @fn		   checkKey
*
* @brief	   检测按键状态
*
* @param	   none
*
* @return	   none
*/
enum Key_status stKey;
uint16 KeyTimes = 0;
#define IS_KEYDN (MCU_IO_GET(0, 1) == 0)
uint8 SendStatus = 1;
//uint8 RowVal;
//#define IS_KEYDN	((RowVal & (BIT5+BIT6+BIT7+BIT8)) != (BIT5+BIT6+BIT7+BIT8))
//INT8U cntBitXOut=0;
void checkKey(void)
{	//MCU_IO_TGL(0, 0);
	//halLedToggle_3();
	//CollectionTime();
	//读入
	//RowVal = (((GPIOC->IDR)&(BIT5+BIT6+BIT7+BIT8)) | ((GPIOC->ODR)&(BIT0+BIT1+BIT2+BIT3+BIT4)));
	switch(stKey)
	{
		case STKEY_IDLE:				//待机状态
			if(IS_KEYDN)
			{
				KeyTimes = 0;
				stKey = STKEY_DETDN;	//延时20ms之后再测，计数清零
				return;
			}
		#if 0
			//没有键按下，退出了休眠。这是因为本程序没有检测到下降沿，
			//却触发了中断退出了休眠。以下程序让它回到休眠,回到集合提醒
			else if(Info.ChipState == CHIP_WAKEUP_KEY)
			{
				TaskComps[7].Run = 1;
				TaskComps[7].Timer = 5;
				TaskComps[7].ItvTime = 5;

				TaskComps[10].Run = 1;
				TaskComps[10].Timer = 10;
				TaskComps[10].ItvTime = 10;
				
				Info.ChipState = ChIP_SLEEP;
			}
		#endif
			break;

		case STKEY_DETDN:				//20ms之后再确认按下
			if(++KeyTimes>=20)
			{
				if(IS_KEYDN)			//20ms之后仍是低
				{
					//TaskComps[10].Run = 0;//关闭蜂鸣器
					//TaskComps[10].Timer = 0;
					//TaskComps[10].ItvTime = 0;

					
					//printf("open_rf\r\n");
					//RF_RxModeOn();
					//DeInfo.InNet = CHECK_IF_ENTER_NET;
					KeyTimes = 0;
					stKey = STKEY_CNT;	//计数清零，进入按键计时状态
					return;
				}

				//误动作，回到IDLE
				stKey = STKEY_IDLE;

				//进入休眠 Sleep()
				//TaskComps[7].Run = 1;
				//TaskComps[7].Timer = 5;
				//TaskComps[7].ItvTime = 5;
				//Info.ChipState = ChIP_SLEEP;
			}

			//继续等待20ms
			break;

		case STKEY_CNT:
		{
			if(!IS_KEYDN)				//如果按键弹起,只有这一个出口
			{
				KeyTimes = 0;
				stKey = STKEY_IDLE;
				 
				if(SendStatus == 0)
				{
					Send( 0x01);//直连
					SendStatus = 1;
				}
				else if(SendStatus == 1)
				{
					Send( 0x02);//阻断
					SendStatus = 2;
				}
				else if(SendStatus == 2)
				{
					Send( 0x03);//替换固定坐标
					SendStatus = 3;
				}
				else if(SendStatus == 3)//偏移
				{
					Send(0x04);
					SendStatus = 0;
				}
				//stKey = STKEY_DETUP;	//20ms之后才认为结束，需要计数清零
				return; 			
			}

			//按键仍是按着状态,且大于5s，继续循环
			if(++KeyTimes>=5000)
			{
				//Info.Channel_now = 0; //保存当前频道
				//RF_SetLogicalChannel(0);//按下按键进入零频道
#ifndef DATA_HUB
				//Send_IEEE_Address();
#endif
				//其它先不管，等待按键结束退出
				//printf("close_rf\r\n");
			    //RF_RxModeOff();

				 SendConfig();
				UART0_SendByte('a');
				KeyTimes = 0;
				stKey = STKEY_DETUP;
				return;
			}		

			//继续等待
		}
		break;

		case STKEY_DETUP:
		{
			if(++KeyTimes >= 20)	//延时20ms再检测
			{
				if(IS_KEYDN)		//如果还是按键，继续等高
				{
					KeyTimes = 0;
					return;
				}


				//if(TaskComps[7].ItvTime == 0)//如果没进入休眠则进入休眠
				//{
				//	TaskComps[7].Run = 1;
				//	TaskComps[7].Timer = 5;
				//	TaskComps[7].ItvTime = 5;

				//	Info.Channel_now = ChannelTable[0]; //保存当前频道
				//	RF_SetLogicalChannel(ChannelTable[0]);//进入默认频道

				//	Info.ChipState = ChIP_SLEEP;
				//}
				
				//完全结束
				KeyTimes = 0;
				stKey = STKEY_IDLE;


				//osalSetEvent (EVENT_KEY); 
				return;
			}	

			//继续等待20ms
		}
		break;

		default:	//其它未知状态
		{
			while(1)
			{
				halLedToggle_1();
				halLedToggle_2();
				
			}
		}
		//break;
	}
	

	
	#if 0
	if(stKey == STKEY_DETDN)	//20ms读回的算是
	{			
		if(RowVal == 0x001f)	//不是这个按键,最长4ms能检测到，最短20+4+20按键可安全
		{
			ptb('$');
			stKey = STKEY_IDLE;
			return;
		}

		BELL_ON;						//有效按键
		KeyVal = RowVal;		//获取键值
		KeyTimes = 0;
		stKey = STKEY_CNT;
		
		return;
	}

	if(RowVal==0x001F)	//检测到回高了
	{
		if(stKey == STKEY_IDLE)
		{			
			return;
		}
		else if(stKey == STKEY_CNT)
		{			
			KeyTimes = 0;
			stKey = STKEY_DETUP;
			return;
		}
		else if(stKey == STKEY_DETUP)
		{
			if(++KeyTimes == 20)	//已经20ms了
			{
				//完全结束
				KeyTimes = 0;
				stKey = STKEY_IDLE;
				BELL_OFF;
								
				osalSetEvent (EVENT_KEY);	
				return;
			}
		}
	}
	else if(stKey == STKEY_IDLE)
	{
		if(++KeyTimes == 20)
		{
			//ptb('&');
			BELL_OFF;
			stKey = STKEY_DETDN;			//算是开始
		}
	}	
	else if(stKey == STKEY_CNT)
	{
		KeyTimes++;

		if(++KeyTimes > 1000)
		{
			KeyTimes = 0;					//立刻就赋值为0
			if(KeyVal == KEY_MENU)
			{
				KeyVal = KEY_MENU_LONG;		//换为长键
				BELL_OFF;								
				osalSetEvent (EVENT_KEY);	
				return;
			}
			//其它仍按正常处理
		}
	}
	#endif
}




