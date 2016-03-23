/*
* Copyright (c) 2015,Yuanjian
* All rights reserved.
*
* 文件名称：basic_rf.c
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

/***********************************************************************************
* INCLUDES
*/
#include "hal_rf.h"
#include "basic_rf.h"
#include "string.h"
#include "key.h"
#include "types.h"
#include "jianPrt.h"
#include "myMacros.h"
#include "led.h"
#include <stdio.h>
#include "board.h"
#include "clrc663.h"

/***********************************************************************************
* GLOBAL VARIABLES
*/

static uint8 rxMpdu[BASIC_RF_PACKET_SIZE + RF_RX_FCS_SIZE] = {0};
extern uint8 ChannelTable[15];

uint8 groupID[2] = {0x00,0x01};

static const uint8 RFPowerTable[] =
{
  0x25,
  0x75,
  0xD5
};

extern TASK_COMPONENTS TaskComps[];


//总共为0-15频道，0为配置频道，频道表首位为默认频道
uint8 ChannelTable[15] = {15,1,2,3,4,5,6,7,8,9,10,11,12,13,14};

uint8 sendBeginFlag = END ;
/**************************************************************************************
* FunctionName	 : PrepareToTx()
* Description	 : 写入数据到FIFO中，准备发送
* EntryParameter : None
* ReturnValue	 : None
**************************************************************************************/

uint8 SimplePacket[4] = {3,0x00,0x01,51};

uint8 connectStatus = DISCONNECTED ;

uint8 pairId = 0x56;


extern uint8 dataBuffer[11][128];

static void PrepareToTx( void )
{
	uint8 * p;
    uint8 i;
    uint8 txBufLen;
	/* flush FIFO of any previous transmit that did not go out */
    ISFLUSHTX();

	//p = &pPacket.LEN;
	p = SimplePacket;

	/* get number of bytes in the packet (does not include the length byte) */
	txBufLen = *p;

	/*
	*  Write the length byte to the FIFO.  This length does *not* include the length field
	*  itself but does include the size of the FCS (generically known as RX metrics) which
	*  is generated automatically by the radio.
	*/
	RFD = txBufLen + RF_RX_FCS_SIZE;

	/* write packet bytes to FIFO */
	for (i=0; i<txBufLen; i++)
	{
		p++;
		RFD = *p;
	}

}

static void PreToTx( uint8 *buffer )
{
	uint8 * p;
    uint8 i;
    uint8 txBufLen;
	/* flush FIFO of any previous transmit that did not go out */
    ISFLUSHTX();

	//p = &pPacket.LEN;
	p = buffer;

	/* get number of bytes in the packet (does not include the length byte) */
	txBufLen = *p;

	/*
	*  Write the length byte to the FIFO.  This length does *not* include the length field
	*  itself but does include the size of the FCS (generically known as RX metrics) which
	*  is generated automatically by the radio.
	*/
	RFD = txBufLen + RF_RX_FCS_SIZE;

	/* write packet bytes to FIFO */
	for (i=0; i<txBufLen; i++)
	  {
		p++;
		RFD = *p;
	  }

}

/**************************************************************************************************
 * @fn          MRFI_SetRFPwr
 *
 * @brief       Set RF output power.
 *
 * @param       level - Power level
 *
 * @return      none
 **************************************************************************************************
 */
void SetRFPwr(uint8 level)
{

  /* make sure radio is off before changing power level */
  RF_RxModeOff();

  TXPOWER = RFPowerTable[level];

  /* Put the radio back in RX state, if it was in RX before channel change */
  RF_RxModeOn();
}


/***********************************************************************************
* @fn          basicRfRxFrmDoneIsr
*
* @brief       Interrupt service routine for received frame from radio
*              (either data or acknowlegdement)
*
* @param       rxi - file scope variable info extracted from the last incoming
*                    frame
*              txState - file scope variable that keeps tx state info
*
* @return      none
*/
//uint8 count  = 0;
static void basicRfRxFrmDoneIsr(void)
{
	//uint8 numBytes,i,*p = rxMpdu;
	//INTERRUPT_OFF();
	// Clear interrupt and disable new RX frame done interrupt
   // halRfDisableRxInterrupt();
	uint8 i,*p = rxMpdu;

    /*
     * Pend on frame completion. First timne through this always passes.
     * Later, though, it is possible that the Rx FIFO has bytes but we
     * havn't received a complete frame yet.
     */
    //while (!(RFIRQF0 & IRQ_FIFOP)) ;

	while(RFIRQF0 & IRQ_FIFOP)
    {
      /* Check for Rx overflow. Checking here means we may flush a valid frame */
		if (FIFOP() && (!FIFO()))
		{
        	/* flush receive FIFO to recover from overflow (per datasheet, flush must be done twice) */
        	FLUSH_RX_FIFO();
        	break;
		}

		/* clear interrupt flag so we can detect another frame later. */
    	RFIRQF0 &= ~IRQ_FIFOP;

		rxMpdu[0] = RFD & PHY_PACKET_SIZE_MASK;//读取LEN
		//printf("\r\nLenth",rxMpdu[0]);

		//如果超出了最大可接收长度
		if(rxMpdu[0] > RF_MAX_RX_SIZE)
		{
			for (i=0; i<rxMpdu[0]; i++)
        	{
          		/* read and discard bytes from FIFO */
          		RFD;
        	}
		}
		else
		{
			for (i=0; i<rxMpdu[0]; i++)
        	{
          		p++;
          		*p = RFD;
        	}
			//TaskComps[2].Run = 1;
			//count++;
			//printf("\r\n%d\r\n",rxMpdu[0]);
#if 1
#ifdef DATA_HUB
			if(rxMpdu[1] == pairId && rxMpdu[2] < 101)
			{
				StoreData(dataBuffer,rxMpdu);
				//printf("store %d\r\n",rxMpdu[2]);
			}
			else if(rxMpdu[1] == pairId)
			{
				TaskComps[2].Run = 1;  //处理收到的包
			}
#else

			TaskComps[2].Run = 1;  //处理收到的包
#endif
#endif
		}

	}

}

//长度(1 byte);  配对码(1 byte); 组号(1 byte); 包中数据片序号(1 byte);数据片总量(1 byte);
void StoreData(uint8 storeBuff[][128],uint8 *inputBuff)
{
	if((inputBuff[4]-1) != inputBuff[3])//片序号(从0开始，因此片总量减1)是否达到片总量
	{
		memcpy(storeBuff[inputBuff[3]],inputBuff,inputBuff[0]);
		storeBuff[inputBuff[3]][0] -=  2;
	    //TaskComps[3].Run = 1;
	}
	else
	{
		memcpy(storeBuff[inputBuff[3]],inputBuff,inputBuff[0]);
		storeBuff[inputBuff[3]][0] -=  2;
		//通知手机，已经有一个完整数据包到达
		TaskComps[5].Run = 1;
	}
}

/***********************************************************************************
* GLOBAL FUNCTIONS
*/

/***********************************************************************************
* @fn          basicRfInit
*
* @brief       Initialise basic RF datastructures. Sets channel, short address and
*              PAN id in the chip and configures interrupt on packet reception
*
* @param       pRfConfig - pointer to BASIC_RF_CONFIG struct.
*                          This struct must be allocated by higher layer
*              txState - file scope variable that keeps tx state info
*              rxi - file scope variable info extracted from the last incoming
*                    frame
*
* @return      none
*/
void basicRfInit (void)
{
    halRfInit();

	INTERRUPT_OFF();

	// Set channel
	//RF_SetLogicalChannel(ChannelTable[1]);
	RF_SetLogicalChannel(15);

	//设置发射功率
	//SetRFPwr(3);

	// Set up receive interrupt (received data or acknowlegment)
	halRfRxInterruptConfig(basicRfRxFrmDoneIsr);

	INTERRUPT_OFF();


	RF_RxModeOn();//打开无线接收模式



	INTERRUPT_ON();


}


uint8 * BuildHead(uint8 * data,uint8 *txBuff,uint8 gId,uint8 id,uint8 partAmount)
{
	*(txBuff+1) =  pairId;//配对序号
	*(txBuff+2) = gId;//组序号
	*(txBuff+3) = id;//个序号
	*(txBuff+4) = partAmount;//包内数据片总量
	memcpy(txBuff+5,data,data[0] + 1);
	*txBuff = 4 + data[0] + 1;
	return txBuff;
}

void SendData(uint8 *sendBuff)
{

	PreToTx(sendBuff);

    // Wait until the transceiver is idle
    halRfWaitTransceiverReady();


    ISTXON(); // Sending
	sendBeginFlag = BEGIN;
    // Waiting for transmission to finish
    while(!(RFIRQF1 & IRQ_TXDONE) );

    RFIRQF1 = ~IRQ_TXDONE;
}




void SendHeartBeat(void)
{
	uint8 heartPackage[5] = {0x04,0,0xff,0xaa,0xbb};
	heartPackage[1] = pairId;
	SendData(heartPackage);

	//printf("test\r\n");
	//Send( 50);
}

void SendConfirm(void)
{
	uint8 confirmPackage[5] = {0x04,0,0xff,0x99,0xbb};
	confirmPackage[1] = pairId;

	SendData(confirmPackage);
	//printf("send Confirm\r\n");
	//Send( 50);
}


void ConnectStatusDisconnect(void)
{
	connectStatus = DISCONNECTED;
	//printf("DISCONNECTED\r\n");
	#ifdef DATA_HUB
	TaskComps[6].Run = 0;
	TaskComps[6].Timer= 0;
	TaskComps[6].ItvTime = 0;

	#else
	TaskComps[9].Run = 0;
	TaskComps[9].Timer= 0;
	TaskComps[9].ItvTime = 0;
	#endif
}

uint8 ReturnConnectStatus(void)
{
	return connectStatus;
}

/**************************************************************************************
* FunctionName   : Frame_Decode()
* Description    : RF包解析
* EntryParameter : None
* ReturnValue    : None
**************************************************************************************/
extern uint8 transmitCount ;
void ReceivedFrameDecode(void)
{
#ifdef DATA_HUB
	if(rxMpdu[1] == pairId && rxMpdu[2] == 0xEE && rxMpdu[3] == 0xCC && rxMpdu[4] == 0xDD)
	{
		/*
		for(uint8 j = 0;j<= rxMpdu[0];j++)
		{
			printf("%x,",rxMpdu[j]);
		}
		printf("\r\n");
		*/
		TaskComps[4].Timer = 3000;
		connectStatus = CONNECTED;
		//printf("reget\r\n");
   		if(TaskComps[6].ItvTime == 0)
   		{
		TaskComps[6].Timer= 200;
		TaskComps[6].ItvTime = 200;
   		}
	}




#else
/*
	for(uint8 j = 0;j<= rxMpdu[0];j++)
	{
		printf("%x,",rxMpdu[j]);
	}
	printf("\r\n");
*/
	if(rxMpdu[1] == pairId && rxMpdu[2] == 0xFF && rxMpdu[3] == 0xaa && rxMpdu[4] == 0xbb)
	{
		rxMpdu[0] = 4;
		rxMpdu[2] = 0xEE;
		rxMpdu[3] = 0xcc;
		rxMpdu[4] = 0xdd;
		connectStatus = CONNECTED;
		TaskComps[4].Timer = 3000;
		//printf("get2\r\n");
		if(TaskComps[9].ItvTime == 0)
   		{
			TaskComps[9].Timer= 200;
			TaskComps[9].ItvTime = 200;
		}
		SendData(rxMpdu);
	}
	else if(rxMpdu[1] == pairId && rxMpdu[2] == 0xFF && rxMpdu[3] == 0x99 && rxMpdu[4] == 0xbb)
	{
		//printf("confirm \r\n");
		TaskComps[6].Run = 0;
		TaskComps[6].Timer = 0;
		TaskComps[6].ItvTime = 0;

		TaskComps[7].Timer = 100;//初始化协议
		CleanBuff(dataBuffer);

		transmitCount = 0;

	}

#endif
	memset(rxMpdu,0x0,sizeof(rxMpdu));

}
/***********************************************************************************
* @fn          basicRfReceiveOn
*
* @brief       Turns on receiver on radio
*
* @param       txState - file scope variable
*
* @return      none
*/
void basicRfReceiveOn(void)
{
    //txState.receiveOn = TRUE;
    halRfReceiveOn();
}


/***********************************************************************************
* @fn          basicRfReceiveOff
*
* @brief       Turns off receiver on radio
*
* @param       txState - file scope variable
*
* @return      none
*/
void basicRfReceiveOff(void)
{
    //txState.receiveOn = FALSE;
    halRfReceiveOff();
}

/**************************************************************************************************
 * @fn          Mrfi_RxModeOn
 *
 * @brief       Put radio into receive mode.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void RF_RxModeOn(void)
{
  /* send strobe to enter receive mode */
  ISRXON();

  /* enable receive interrupts */
  RFIRQM0 |= IM_FIFOP;


}


/**************************************************************************************************
 * @fn          Mrfi_RxModeOff
 *
 * @brief       -
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void RF_RxModeOff(void)
{
  /*disable receive interrupts */
  RFIRQM0 &= ~IM_FIFOP;

  /* turn off radio */
  ISRFOFF();

  /* flush the receive FIFO of any residual data */
  FLUSH_RX_FIFO();

  /* clear receive interrupt */
  RFIRQF0 = ~IRQ_FIFOP;
}







/***********************************************************************************

***********************************************************************************/

