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
* @brief	   ��ʼ��KEY���IO��

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
	//PICTL |=0x01;	//P0���½��ز����ж�
	//P0IEN |=0x02;	//P01���ж�ʹ��
	//IEN1 |= 0X20;	// ��P0�����ж�
	//P0IFG |= 0x00;     //���жϱ�־
}

/***********************************************************************************
* @fn		   checkKey
*
* @brief	   ��ⰴ��״̬
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
	//����
	//RowVal = (((GPIOC->IDR)&(BIT5+BIT6+BIT7+BIT8)) | ((GPIOC->ODR)&(BIT0+BIT1+BIT2+BIT3+BIT4)));
	switch(stKey)
	{
		case STKEY_IDLE:				//����״̬
			if(IS_KEYDN)
			{
				KeyTimes = 0;
				stKey = STKEY_DETDN;	//��ʱ20ms֮���ٲ⣬��������
				return;
			}
		#if 0
			//û�м����£��˳������ߡ�������Ϊ������û�м�⵽�½��أ�
			//ȴ�������ж��˳������ߡ����³��������ص�����,�ص���������
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

		case STKEY_DETDN:				//20ms֮����ȷ�ϰ���
			if(++KeyTimes>=20)
			{
				if(IS_KEYDN)			//20ms֮�����ǵ�
				{
					//TaskComps[10].Run = 0;//�رշ�����
					//TaskComps[10].Timer = 0;
					//TaskComps[10].ItvTime = 0;

					
					//printf("open_rf\r\n");
					//RF_RxModeOn();
					//DeInfo.InNet = CHECK_IF_ENTER_NET;
					KeyTimes = 0;
					stKey = STKEY_CNT;	//�������㣬���밴����ʱ״̬
					return;
				}

				//�������ص�IDLE
				stKey = STKEY_IDLE;

				//�������� Sleep()
				//TaskComps[7].Run = 1;
				//TaskComps[7].Timer = 5;
				//TaskComps[7].ItvTime = 5;
				//Info.ChipState = ChIP_SLEEP;
			}

			//�����ȴ�20ms
			break;

		case STKEY_CNT:
		{
			if(!IS_KEYDN)				//�����������,ֻ����һ������
			{
				KeyTimes = 0;
				stKey = STKEY_IDLE;
				 
				if(SendStatus == 0)
				{
					Send( 0x01);//ֱ��
					SendStatus = 1;
				}
				else if(SendStatus == 1)
				{
					Send( 0x02);//���
					SendStatus = 2;
				}
				else if(SendStatus == 2)
				{
					Send( 0x03);//�滻�̶�����
					SendStatus = 3;
				}
				else if(SendStatus == 3)//ƫ��
				{
					Send(0x04);
					SendStatus = 0;
				}
				//stKey = STKEY_DETUP;	//20ms֮�����Ϊ��������Ҫ��������
				return; 			
			}

			//�������ǰ���״̬,�Ҵ���5s������ѭ��
			if(++KeyTimes>=5000)
			{
				//Info.Channel_now = 0; //���浱ǰƵ��
				//RF_SetLogicalChannel(0);//���°���������Ƶ��
#ifndef DATA_HUB
				//Send_IEEE_Address();
#endif
				//�����Ȳ��ܣ��ȴ����������˳�
				//printf("close_rf\r\n");
			    //RF_RxModeOff();

				 SendConfig();
				UART0_SendByte('a');
				KeyTimes = 0;
				stKey = STKEY_DETUP;
				return;
			}		

			//�����ȴ�
		}
		break;

		case STKEY_DETUP:
		{
			if(++KeyTimes >= 20)	//��ʱ20ms�ټ��
			{
				if(IS_KEYDN)		//������ǰ����������ȸ�
				{
					KeyTimes = 0;
					return;
				}


				//if(TaskComps[7].ItvTime == 0)//���û�����������������
				//{
				//	TaskComps[7].Run = 1;
				//	TaskComps[7].Timer = 5;
				//	TaskComps[7].ItvTime = 5;

				//	Info.Channel_now = ChannelTable[0]; //���浱ǰƵ��
				//	RF_SetLogicalChannel(ChannelTable[0]);//����Ĭ��Ƶ��

				//	Info.ChipState = ChIP_SLEEP;
				//}
				
				//��ȫ����
				KeyTimes = 0;
				stKey = STKEY_IDLE;


				//osalSetEvent (EVENT_KEY); 
				return;
			}	

			//�����ȴ�20ms
		}
		break;

		default:	//����δ֪״̬
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
	if(stKey == STKEY_DETDN)	//20ms���ص�����
	{			
		if(RowVal == 0x001f)	//�����������,�4ms�ܼ�⵽�����20+4+20�����ɰ�ȫ
		{
			ptb('$');
			stKey = STKEY_IDLE;
			return;
		}

		BELL_ON;						//��Ч����
		KeyVal = RowVal;		//��ȡ��ֵ
		KeyTimes = 0;
		stKey = STKEY_CNT;
		
		return;
	}

	if(RowVal==0x001F)	//��⵽�ظ���
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
			if(++KeyTimes == 20)	//�Ѿ�20ms��
			{
				//��ȫ����
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
			stKey = STKEY_DETDN;			//���ǿ�ʼ
		}
	}	
	else if(stKey == STKEY_CNT)
	{
		KeyTimes++;

		if(++KeyTimes > 1000)
		{
			KeyTimes = 0;					//���̾͸�ֵΪ0
			if(KeyVal == KEY_MENU)
			{
				KeyVal = KEY_MENU_LONG;		//��Ϊ����
				BELL_OFF;								
				osalSetEvent (EVENT_KEY);	
				return;
			}
			//�����԰���������
		}
	}
	#endif
}




