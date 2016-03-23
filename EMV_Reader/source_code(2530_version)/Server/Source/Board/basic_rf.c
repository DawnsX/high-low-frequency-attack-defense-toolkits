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

/***********************************************************************************
* GLOBAL VARIABLES
*/

static uint8 rxMpdu[BASIC_RF_PACKET_SIZE + RF_RX_FCS_SIZE] = {0};
extern uint8 ChannelTable[15];



static const uint8 RFPowerTable[] =
{
  0x25,
  0x75,
  0xD5
};

extern TASK_COMPONENTS TaskComps[];


//总共为0-15频道，0为配置频道，频道表首位为默认频道
uint8 ChannelTable[15] = {15,1,2,3,4,5,6,7,8,9,10,11,12,13,14};


/**************************************************************************************
* FunctionName	 : PrepareToTx()
* Description	 : 写入数据到FIFO中，准备发送
* EntryParameter : None
* ReturnValue	 : None
**************************************************************************************/

uint8 SimplePacket[4] = {3,0xaa,0xbb,0};


extern  struct {
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
}config;

extern struct
{
	uint8 numSatellites;
	uint8 signalStrength;
	uint8 lon[4];
	uint8 lat[4];
	uint32 lonShift;
	uint32 latShift;
}hubConfig;

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
			TaskComps[2].Run = 1;
	
		}
		
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
	//RF_SetLogicalChannel(ChannelTable[0]);
	RF_SetLogicalChannel(0);
	
	//设置发射功率
	//SetRFPwr(3);
	
	// Set up receive interrupt (received data or acknowlegment)
	halRfRxInterruptConfig(basicRfRxFrmDoneIsr);
	
	INTERRUPT_OFF();

	
	RF_RxModeOn();//打开无线接收模式



	INTERRUPT_ON();


}
void Send( uint8 cmd)
{

	
	SimplePacket[3] = cmd;

	

	PrepareToTx();

    // Wait until the transceiver is idle
    halRfWaitTransceiverReady();
	

    ISTXON(); // Sending

    // Waiting for transmission to finish
   //while(!(RFIRQF1 & IRQ_TXDONE) );

    //RFIRQF1 = ~IRQ_TXDONE;
	
	
	//printf("sending\n");

}

void SendConfig( void)
{

	
	//SimplePacket[3] = cmd;

	

	PreToTx(&config.length);

    // Wait until the transceiver is idle
    halRfWaitTransceiverReady();
	

    ISTXON(); // Sending

    // Waiting for transmission to finish
   //while(!(RFIRQF1 & IRQ_TXDONE) );

    //RFIRQF1 = ~IRQ_TXDONE;
	
	
	//printf("sending\n");

}

void ProcessConfig(uint8 *buffer)
{
	hubConfig.numSatellites=buffer[4];
	hubConfig.signalStrength=buffer[5];
	memcpy(hubConfig.lon,buffer+6,8);
	hubConfig.lonShift = (uint32)buffer[14] + (((uint32)buffer[15])<<8) \
		+ (((uint32)buffer[16])<<16) + (((uint32)buffer[17])<<24);
	hubConfig.latShift = (uint32)buffer[18] + (((uint32)buffer[19])<<8) \
		+ (((uint32)buffer[20])<<16) + (((uint32)buffer[21])<<24);
}
/**************************************************************************************
* FunctionName   : Frame_Decode()
* Description    : RF包解析
* EntryParameter : None
* ReturnValue    : None
**************************************************************************************/
extern enum ConnectMode connectMode;
void ReceivedFrameDecode(void)
{
#ifdef DATA_HUB
	if(rxMpdu[1] == 0xaa && rxMpdu[2] == 0xbb)
	{  
	  switch (rxMpdu[3])
	  {
		case 0x01://直连
			halLedToggle_2();
			EA = 0;
			//UTX0IF = 1;
			URX1IE = 1;//打开uart接收中断
			URX0IE = 1;
			connectMode = DIRECT;
			EA = 1;
			Send(0x01);
			break;
		case 0x02://阻断
			halLedToggle_3();
			EA= 0;
			URX1IE = 0;//关闭uart接收
			URX0IE = 0;
			EA = 1;
			Send(0x02);
			//UART0_SendData("xxx",4);
			break;
		case 0x03://固定坐标
			halLedToggle_3();
			EA= 0;
			//UTX0IF = 1;
			URX1IE = 1;//关闭uart接收
			URX0IE = 1;
			connectMode = REPLACE;
			EA = 1;
			Send(0x03);
			break;
		case 0x04://偏移量
			halLedToggle_3();
			EA= 0;
			//UTX0IF = 1;
			URX1IE = 1;//关闭uart接收
			URX0IE = 1;
			connectMode = SHIFT;
			EA = 1;
			Send(0x04);
			break;
		case 0x05://设置
			halLedToggle_3();
			//UART0_SendData(rxMpdu,25);
			ProcessConfig(rxMpdu);
			Send(0x05);
			break;
		default:
			;
		
	  }
	}

#else
	if(rxMpdu[1] == 0xaa && rxMpdu[2] == 0xbb)
	{  
	  switch (rxMpdu[3])
	  {
		case 0x01://直连
			//halLedToggle_2();
			halLedClear(1);
			halLedClear(2);
			break;
		case 0x02://阻断
			halLedSet(1);
			halLedClear(2);
			break;
		case 0x03://固定坐标
			halLedClear(1);
			halLedSet(2);
			break;
		case 0x04://偏移
			halLedSet(1);
			halLedSet(2);
			break;
		case 0x05://设置
			halLedSet(1);
			halLedSet(2);
			break;
		default:
			;
		
	  }
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

