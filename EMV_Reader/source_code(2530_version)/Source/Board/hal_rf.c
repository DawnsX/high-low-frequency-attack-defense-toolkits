/***********************************************************************************

  Filename:       hal_rf.c

  Description:    CC2430 radio interface.

***********************************************************************************/

/***********************************************************************************
* INCLUDES
*/
#include "hal_rf.h"
#include "Board.h"
#include "string.h"
#include "types.h"
#include "ioCC2530.h"
#include <stdio.h>

#include "jianPrt.h"
#include "Basic_rf.h"
#include<string.h>




/***********************************************************************************
* GLOBAL DATA
*/
#if INCLUDE_PA==2591
	static const menuItem_t pPowerSettings[] =
	{
	  "0dBm", HAL_RF_TXPOWER_0_DBM,
	  "13dBm", HAL_RF_TXPOWER_13_DBM,
	  "16dBm", HAL_RF_TXPOWER_16_DBM,
	  "18dBm", HAL_RF_TXPOWER_18_DBM,
	  "20dBm", HAL_RF_TXPOWER_20_DBM
	};
#else
	static const menuItem_t pPowerSettings[] =
	{
	  "-3dBm", HAL_RF_TXPOWER_MIN_3_DBM,
	  "0dBm", HAL_RF_TXPOWER_0_DBM,
	  "4dBm", HAL_RF_TXPOWER_4_DBM
	};
#endif


const menu_t powerMenu =
{
  pPowerSettings,
  N_ITEMS(pPowerSettings)
};

/***********************************************************************************
* LOCAL DATA
*/
static ISR_FUNC_PTR pfISR= NULL;
static uint8 rssiOffset = RSSI_OFFSET;
uint8 aExtendedAddress[8] = {0};
halDMADesc_t dmaCh0;
halDMADesc_t dmaCh1234[4];

extern uint8 ChannelTable[];




/***********************************************************************************
* LOCAL FUNCTIONS
*/


/***********************************************************************************
* GLOBAL FUNCTIONS
*/

/***********************************************************************************
* @fn      halRfInit
*
* @brief   Power up, sets default tuning settings, enables autoack, enables random
*          generator.
*
* @param   none
*
* @return  SUCCESS always (for interface compatibility)
*/
uint8 halRfInit(void)
{
    //uint8 i;

    // turning on power to analog part of radio and waiting for voltage regulator.
    //RFPWR = 0x04;
    //while( RFPWR & 0x10 );

    // Setting for AUTO CRC and AUTOACK
    FRMCTRL0 |= (AUTO_CRC | AUTO_ACK);
    //MDMCTRL0L &= ~(AUTO_CRC | AUTO_ACK);//关闭CRC 和AUTOACK
    FRMCTRL0 &= ~AUTO_ACK;//关闭AUTOACK

	//空闲信道评估模式11
	//MDMCTRL0L |= BV(6);
	//MDMCTRL0L &= ~ BV(6);
	//MDMCTRL0L |= BV(7);

	/* disable address filtering */
  	FRMFILT0 &= ~FRAME_FILTER_EN;
    // 禁止硬件地址解码
    //SRCMATCH &= ~ BV(0);
    // Turning on AUTO_TX2RX
    //FSMTC1 = ((FSMTC1 & (~AUTO_TX2RX_OFF & ~RX2RX_TIME_OFF))  | ACCEPT_ACKPKT);

    // Turning off abortRxOnSrxon.
    //FSMTC1 &= ~0x20;

    // Set FIFOP threshold to maximum
    FIFOPCTRL = 127;
    // tuning adjustments for optimal radio performance; details available in datasheet */
    //RXCTRL0H = 0x32;
    //RXCTRL0L = 0xF5;

    // Recommended RX settings
    TXFILTCFG = 0x09;
    AGCCTRL1 = 0x15;
    FSCAL1 = 0x00;
#ifdef DATA_HUB
	halPaLnaInit();
	halRfSetTxPower(HAL_RF_TXPOWER_20_DBM);
	halRfSetTxPower(HAL_RF_TXPOWER_0_DBM);
#endif

#if 0

    // Turning on receiver to get output from IF-ADC
    ISRXON();
    McuWaitUs(1);

    // Enable random generator
    ADCCON1 &= ~0x0C;

    for(i = 0 ; i < 32 ; i++)
    {
        RNDH = ADCTSTH;
        // Clock random generator
        ADCCON1 |= 0x04;
    }
    ISRFOFF();

#endif
#if 0


		/* ------------------------------------------------------------------
	   *	Initialize Random Seed Value
	   *   -------------------------------
	   */

	  /* turn on radio power, pend for the power-up delay */
	  //RFPWR &= ~RREG_RADIO_PD;
	 // while((RFPWR & ADI_RADIO_PD));

	  /*
	   *  Set radio for infinite reception.  Once radio reaches this state,
	   *  it will stay in receive mode regardless RF activity.
	   */
	  FRMCTRL0 = (FRMCTRL0 & ~RX_MODE_MASK) | RX_MODE_INFINITE_RX;

	  /* turn on the receiver */
	  ISRXON();

	/* Wait for RSSI to be valid. Once valid, radio is stable and random bits
   * can be read.
   */
  MRFI_RSSI_VALID_WAIT();
	//printf("YES");
	  /*
	   *  Wait for radio to reach infinite reception state.  Once it does,
	   *  The least significant bit of ADTSTH should be pretty random.
	   */
	 // while (FSMSTATE != FSM_FFCTRL_STATE_RX_INF)

	  /* put 16 random bits into the seed value */
	  {
		uint16 rndSeed;
		uint8  i;

		rndSeed = 0;

		for(i=0; i<16; i++)
		{
		  /* use most random bit of analog to digital receive conversion to populate the random seed */
		  rndSeed = (rndSeed << 1) | (RFRND & 0x01);
		}

		/*
		 *	The seed value must not be zero.  If it is, the pseudo random sequence will be always be zero.
		 *	There is an extremely small chance this seed could randomly be zero (more likely some type of
		 *	hardware problem would cause this).  To solve this, a single bit is forced to be one.  This
		 *	slightly reduces the randomness but guarantees a good seed value.
		 */
		rndSeed |= 0x0080;

		/*
		 *	Two writes to RNDL will set the random seed.  A write to RNDL copies current contents
		 *	of RNDL to RNDH before writing new the value to RNDL.
		 */
		RNDL = rndSeed & 0xFF;
		RNDL = rndSeed >> 8;


	  }
#endif
	/* take receiver out of infinite reception mode; set back to normal operation */
	FRMCTRL0 = (FRMCTRL0 & ~RX_MODE_MASK) | RX_MODE_NORMAL;

	//读取IEEE地址
	//read_IEEE_address();


    halRfEnableRxInterrupt();

    return SUCCESS;
}
/***********************************************************************************
* @fn      halRfSetPower
*
* @brief   Set TX output power
*
* @param   uint8 power - power level: TXPOWER_MIN_4_DBM, TXPOWER_0_DBM,
*                        TXPOWER_4_DBM
*
* @return  uint8 - SUCCESS or FAILED
*/
uint8 halRfSetTxPower(uint8 power)
{
    uint8 n;

    switch(power)
    {
#if INCLUDE_PA==2591
    case HAL_RF_TXPOWER_0_DBM: n = CC2530_91_TXPOWER_0_DBM; break;
    case HAL_RF_TXPOWER_13_DBM: n = CC2530_91_TXPOWER_13_DBM; break;
    case HAL_RF_TXPOWER_16_DBM: n = CC2530_91_TXPOWER_16_DBM; break;
    case HAL_RF_TXPOWER_18_DBM: n = CC2530_91_TXPOWER_18_DBM; break;
    case HAL_RF_TXPOWER_20_DBM: n = CC2530_91_TXPOWER_20_DBM; break;
#else
    case HAL_RF_TXPOWER_MIN_3_DBM: n = CC2530_TXPOWER_MIN_3_DBM; break;
    case HAL_RF_TXPOWER_0_DBM: n = CC2530_TXPOWER_0_DBM; break;
    case HAL_RF_TXPOWER_4_DBM: n = CC2530_TXPOWER_4_DBM; break;
#endif
    default:
        return FAILED;
    }

    // Set TX power
    TXPOWER = n;

    return SUCCESS;
}

/***********************************************************************************
* @fn		   DMAInit
*
* @brief	   初始化DMA，并取得保存在FLASH中的组号和ID

*
* @param	   none
*
* @return	   none
*/


extern uint8 Uart0Tx[300];
void DMAInit(void)
{
	halDMADesc_t *ch;
	//CC2530采用特殊结构初始化DMA，因此把DMA初始位置传送给DMA配置寄存器
	HAL_DMA_SET_ADDR_DESC1234( dmaCh1234);
#if 0
	// Setup Tx by DMA.
	ch = HAL_DMA_GET_DESC1234( 3 );//初始化通道3，为uart1

	//Source Address
	HAL_DMA_SET_SOURCE( ch, Uart1Tx);

	// The start address of the destination.
	HAL_DMA_SET_DEST( ch, 0x70F9 );//UART1 TX

    HAL_DMA_SET_LEN(ch,30);//固定长度 6

	// Using the length field to determine how many bytes to transfer.
	HAL_DMA_SET_VLEN( ch, HAL_DMA_VLEN_USE_LEN );

	// One byte is transferred each time.
	HAL_DMA_SET_WORD_SIZE( ch, HAL_DMA_WORDSIZE_BYTE );

	// The bytes are transferred 1-by-1 on Tx Complete trigger.
	HAL_DMA_SET_TRIG_MODE( ch, HAL_DMA_TMODE_SINGLE );
	HAL_DMA_SET_TRIG_SRC( ch, 17);//TX1发送完成
	//HAL_DMA_SET_TRIG_SRC( ch, DMATRIG_TX );

	// The source address is incremented by 1 byte after each transfer.
	HAL_DMA_SET_SRC_INC( ch, HAL_DMA_SRCINC_1 );

	// The destination address is constant - the Tx Data Buffer.
	HAL_DMA_SET_DST_INC( ch, HAL_DMA_DSTINC_0 );

	// The DMA Tx done is serviced by ISR in order to maintain full thruput.
	HAL_DMA_SET_IRQ( ch, HAL_DMA_IRQMASK_DISABLE );

	// Xfer all 8 bits of a byte xfer.
	HAL_DMA_SET_M8( ch, HAL_DMA_M8_USE_8_BITS );

	// DMA has highest priority for memory access.
	HAL_DMA_SET_PRIORITY( ch, HAL_DMA_PRI_HIGH );
#endif
#if 0
	DMAARM = ((1 << 3) & 0x1F);
	asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
	asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
	asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
	asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
	asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
	asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
	asm("NOP");asm("NOP");asm("NOP");

    HAL_DMA_MAN_TRIGGER( 3 );
#endif

	// Setup Tx by DMA.
	ch = HAL_DMA_GET_DESC1234( 4 );//初始化通道4，为uart0

	//Source Address
	HAL_DMA_SET_SOURCE( ch, Uart0Tx);

	// The start address of the destination.
	HAL_DMA_SET_DEST( ch, 0x70C1);//UART0 TX

    HAL_DMA_SET_LEN(ch,20);//固定长度 6

	// Using the length field to determine how many bytes to transfer.
	HAL_DMA_SET_VLEN( ch, HAL_DMA_VLEN_USE_LEN );

	// One byte is transferred each time.
	HAL_DMA_SET_WORD_SIZE( ch, HAL_DMA_WORDSIZE_BYTE );

	// The bytes are transferred 1-by-1 on Tx Complete trigger.
	HAL_DMA_SET_TRIG_MODE( ch, HAL_DMA_TMODE_SINGLE );
	HAL_DMA_SET_TRIG_SRC( ch, 15);//TX0发送完成
	//HAL_DMA_SET_TRIG_SRC( ch, DMATRIG_TX );

	// The source address is incremented by 1 byte after each transfer.
	HAL_DMA_SET_SRC_INC( ch, HAL_DMA_SRCINC_1 );

	// The destination address is constant - the Tx Data Buffer.
	HAL_DMA_SET_DST_INC( ch, HAL_DMA_DSTINC_0 );

	// The DMA Tx done is serviced by ISR in order to maintain full thruput.
	HAL_DMA_SET_IRQ( ch, HAL_DMA_IRQMASK_DISABLE );

	// Xfer all 8 bits of a byte xfer.
	HAL_DMA_SET_M8( ch, HAL_DMA_M8_USE_8_BITS );

	// DMA has highest priority for memory access.
	HAL_DMA_SET_PRIORITY( ch, HAL_DMA_PRI_HIGH );
#if 0
	DMAARM |= ((1 << 4) & 0x1F);
	asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
	asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
	asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
	asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
	asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
	asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
	asm("NOP");asm("NOP");asm("NOP");

	HAL_DMA_MAN_TRIGGER( 4 );
#endif

	//NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();
	//NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();
	//HAL_ENTER_CRITICAL_SECTION(intState);
#if 0
	EA = 0;
    HAL_DMA_ARM_CH(4);
    do
    {
      asm("NOP");
    } while (!HAL_DMA_CH_ARMED(4));
    HAL_DMA_CLEAR_IRQ(4);
    HAL_DMA_MAN_TRIGGER(4);
    //HAL_EXIT_CRITICAL_SECTION(intState);
    EA = 1;
#endif
	#if 0
	NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();
	NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();
	DMAARM |= ((1 << 4) & 0x1F);
	asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
	asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
	asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
	asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
	asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
	asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
	asm("NOP");asm("NOP");asm("NOP");
	HAL_DMA_MAN_TRIGGER( 4 );
	#endif
}
void UARTSendDMA(uint8 DMACh,uint16 len)
{
	HAL_DMA_SET_LEN(HAL_DMA_GET_DESC1234( DMACh ), len);
	EA = 0;
    HAL_DMA_ARM_CH(DMACh);
    do
    {
      asm("NOP");
    } while (!HAL_DMA_CH_ARMED(DMACh));
    //HAL_DMA_CLEAR_IRQ(DMACh);
    HAL_DMA_MAN_TRIGGER(DMACh);
	EA = 1;
}
void UARTSendDMAWithSrc(uint8 DMACh,uint16 len,uint8 *src)
{
	HAL_DMA_SET_SOURCE(HAL_DMA_GET_DESC1234( DMACh ), src);
	HAL_DMA_SET_LEN(HAL_DMA_GET_DESC1234( DMACh ), len);
	EA = 0;
    HAL_DMA_ARM_CH(DMACh);
    do
    {
      asm("NOP");
    } while (!HAL_DMA_CH_ARMED(DMACh));
    //HAL_DMA_CLEAR_IRQ(DMACh);
    HAL_DMA_MAN_TRIGGER(DMACh);
	EA = 1;
}


/***********************************************************************************
* @fn      halRfGetChipId
*
* @brief   Get chip id
*
* @param   none
*
* @return  uint8 - result
*/
uint8 halRfGetChipId(void)
{
    return CHIPID;
}


/***********************************************************************************
* @fn      halRfGetChipVer
*
* @brief   Get chip version
*
* @param   none
*
* @return  uint8 - result
*/
uint8 halRfGetChipVer(void)
{
    return CHVER;
}

/***********************************************************************************
* @fn      halRfGetRandomByte
*
* @brief   Return random byte
*
* @param   none
*
* @return  uint8 - random byte
*/
uint8 halRfGetRandomByte(void)
{
    // Clock the random generator
    ADCCON1 |= 0x04;

    return RNDH;

}

uint8 GetRandomByte(void)
{
  /* clock the random generator to get a new random value */
  ADCCON1 = (ADCCON1 & ~RCTRL_BITS) | RCTRL_CLOCK_LFSR;

  /* return newly randomized value from hardware */
  return(RNDH);
}
void GetRandomByte_test(void)
{
	uint8 num ;
	ADCCON1 = (ADCCON1 & ~RCTRL_BITS) | RCTRL_CLOCK_LFSR;
	//ADCCON1 |= 0x04;
	/* return newly randomized value from hardware */
	num = RNDH;

	printf("num_%d\r\n ",num);
  //return(RNDH);
}

/***********************************************************************************
* @fn      halRfGetRssiOffset
*
* @brief   Return RSSI Offset
*
* @param   none
*
* @return  uint8 - RSSI offset
*/
uint8 halRfGetRssiOffset(void)
{
  return rssiOffset;
}

/***********************************************************************************
* @fn      halRfSetChannel
*
* @brief   Set RF channel in the 2.4GHz band. The Channel must be in the range 11-26,
*          11= 2005 MHz, channel spacing 5 MHz.
*
* @param   channel - logical channel number
*
* @return  none
*/
void halRfSetChannel(uint8 channel)
{
    uint16 freqMHz;

    freqMHz= 2405 + ((channel - MIN_CHANNEL) * CHANNEL_SPACING); // Calculate frequency
    freqMHz -= (uint32)2048;              // Subtract; datasheet sect 14.16

    //FSCTRLL = LOBYTE(freqMHz);
   // FSCTRLH &= ~0x03;
    //FSCTRLH |= (HIBYTE(freqMHz) & 0x03);
}

/**************************************************************************************************
 * @fn          RF_SetLogicalChannel
 *
 * @brief       Set logical channel.
 *
 * @param       chan - logical channel number
 *
 * @return      none
 **************************************************************************************************
 */
void RF_SetLogicalChannel(uint8 chan)
{
  //uint8 phyChannel;


  /* make sure radio is off before changing channels */
  RF_RxModeOff();

  /* convert logical channel number into physical channel number */
  //phyChannel = mrfiLogicalChanTable[chan];

  /* write frequency value of new channel */
//#ifndef FREQUENCY_HOPPING
 // FSCTRLL = FREQ_2405MHZ + (5 * (phyChannel - 11));
//#else
  /* frequency hopping requires more channels than are available using 802.15.4
   * so don't do any calculations, just jam the MHz offset from 2400MHz into the
   * control register
   */
//  FSCTRLL = FREQ_2405MHZ + phyChannel;    // non 802.15.4 channels for FHSS code
  FREQCTRL = FREQ_2405MHZ + (5 * chan);
//#endif

  /* Put the radio back in RX state, if it was in RX before channel change */
  //if(mrfiRadioState == MRFI_RADIO_STATE_RX)
  //{
    RF_RxModeOn();
  //}
}

/***********************************************************************************
* @fn      halRfSetShortAddr
*
* @brief   Write short address to chip
*
* @param   none
*
* @return  none
*/
void halRfSetShortAddr(uint16 shortAddr)
{
    //SHORTADDRL= LOBYTE(shortAddr);
    //SHORTADDRH= HIBYTE(shortAddr);
}


/***********************************************************************************
* @fn      halRfSetPanId
*
* @brief   Write PAN Id to chip
*
* @param   none
*
* @return  none
*/
void halRfSetPanId(uint16 panId)
{
    //PANIDL= LOBYTE(panId);
    //PANIDH= HIBYTE(panId);
}


/***********************************************************************************
* @fn      halRfWriteTxBuf
*
* @brief   Write to TX buffer
*
* @param   uint8* pData - buffer to write
*          uint8 length - number of bytes
*
* @return  none
*/
void halRfWriteTxBuf(uint8* pData, uint8 length)
{
    uint8 i;

    ISFLUSHTX();          // Making sure that the TX FIFO is empty.

    RFIRQF1 = ~IRQ_TXDONE;   // Clear TX done interrupt

    // Insert data
    for(i=0;i<length;i++){
        RFD = pData[i];
		//prt("%d  ",pData[i]);
    }

}


/***********************************************************************************
* @fn      halRfAppendTxBuf
*
* @brief   Write to TX buffer
*
* @param   uint8* pData - buffer to write
*          uint8 length - number of bytes
*
* @return  none
*/
void halRfAppendTxBuf(uint8* pData, uint8 length)
{
    uint8 i;

    // Insert data
    for(i=0;i<length;i++){
        RFD = pData[i];
    }
}


/***********************************************************************************
* @fn      halRfReadRxBuf
*
* @brief   Read RX buffer
*
* @param   uint8* pData - data buffer. This must be allocated by caller.
*          uint8 length - number of bytes
*
* @return  none
*/
void halRfReadRxBuf(uint8* pData, uint8 length)
{
    while (length>0) {
        *pData++= RFD;
        length--;
		//prt("   %d",45);
    }
}



/***********************************************************************************
* @fn      halRfTransmit
*
* @brief   Transmit frame with Clear Channel Assessment.
*
* @param   none
*
* @return  uint8 - SUCCESS or FAILED
*/
uint8 halRfTransmit(void)
{
    uint8 status;

    ISTXON(); // Sending

    // Waiting for transmission to finish
    while(!(RFIRQF1 & IRQ_TXDONE) );

    RFIRQF1 = ~IRQ_TXDONE;
    status= SUCCESS;

    // TBD: use CCA
    return status;
}



/***********************************************************************************
* @fn      halRfReceiveOn
*
* @brief   Turn receiver on
*
* @param   none
*
* @return  none
*/
void halRfReceiveOn(void)
{
    FLUSH_RX_FIFO();
    ISRXON();
}

/***********************************************************************************
* @fn      halRfReceiveOff
*
* @brief   Turn receiver off
*
* @param   none
*
* @return  none
*/
void halRfReceiveOff(void)
{
    ISRFOFF();
    FLUSH_RX_FIFO();
}


/***********************************************************************************
* @fn      halRfDisableRxInterrupt
*
* @brief   Clear and disable RX interrupt.
*
* @param   none
*
* @return  none
*/
void halRfDisableRxInterrupt(void)
{
    // disable RXPKTDONE interrupt
    //RFIRQM0 &= ~BV(6);

	 /*disable receive interrupts */
     RFIRQM0 &= ~IM_FIFOP;
    // disable general RF interrupts
    IEN2 &= ~BV(0);
}


/***********************************************************************************
* @fn      halRfEnableRxInterrupt
*
* @brief   Enable RX interrupt.
*
* @param   none
*
* @return  none
*/
void halRfEnableRxInterrupt(void)
{
    // enable RXPKTDONE interrupt
    //RFIRQM0 |= BV(6);
    /* enable receive interrupts */
  	RFIRQM0 |= IM_FIFOP;
    // enable general RF interrupts
    IEN2 |= BV(0);
}


/***********************************************************************************
* @fn      halRfRxInterruptConfig
*
* @brief   Configure RX interrupt.
*
* @param   none
*
* @return  none
*/
void halRfRxInterruptConfig(ISR_FUNC_PTR pf)
{
    uint8 x;
    HAL_INT_LOCK(x);
    pfISR= pf;
    HAL_INT_UNLOCK(x);
}

/***********************************************************************************
* @fn      halRfWaitTransceiverReady
*
* @brief   Wait until the transciever is ready (SFD inactive).
*
* @param   none
*
* @return  none
*/
void halRfWaitTransceiverReady(void)
{
    while (FSMSTAT1 & (BV(1) | BV(5) ));
}

/************************************************************************************
 * @fn          macMcuRfIsr
 *
 * @brief       Interrupt service routine that handles FIFOP interrupts.
 *
 * @param       none
 *
 * @return      none
 */

HAL_ISR_FUNCTION( macMcuRfIsr, RF_VECTOR )
{
    //uint8 rfim;
    uint8 x;

    HAL_INT_LOCK(x);
	/* enable receive interrupts */
  	//RFIRQM0 |= IM_FIFOP;
	//if( RFIRQF0 & IRQ_RXPKTDONE )
	if( RFIRQF0 & IM_FIFOP)
    {
        (pfISR)();                  // Execute the custom ISR
         S1CON= 0;                   // Clear general RF interrupt flag
        RFIRQF0&= ~IM_FIFOP;   // Clear RXPKTDONE interrupt
    }
	//RFIF = 0xFF;
    HAL_INT_UNLOCK(x);
}

/**************************************************************************************************
 * @fn          HalFlashRead
 *
 * @brief       This function reads 'cnt' bytes from the internal flash.
 *
 * input parameters
 *
 * @param       pg - A valid flash page number.
 * @param       offset - A valid offset into the page.
 * @param       buf - A valid buffer space at least as big as the 'cnt' parameter.
 * @param       cnt - A valid number of bytes to read.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void HalFlashRead(uint8 pg, uint16 offset, uint8 *buf, uint16 cnt)
{
  // Calculate the offset into the containing flash bank as it gets mapped into XDATA.
  uint8 *pData = (uint8 *)(offset + 0x8000) +
                 ((pg % 16) * 2048);
  uint8 memctr = MEMCTR;  // Save to restore.

#if (!defined HAL_OAD_BOOT_CODE) && (!defined HAL_OTA_BOOT_CODE)
  //halIntState_t is;
#endif
EA = 0;
  pg /= 16;  // Calculate the flash bank from the flash page.

#if (!defined HAL_OAD_BOOT_CODE) && (!defined HAL_OTA_BOOT_CODE)
  //HAL_ENTER_CRITICAL_SECTION(is);
#endif

  // Calculate and map the containing flash bank into XDATA.
  MEMCTR = (MEMCTR & 0xF8) | pg;

  while (cnt--)
  {
    *buf++ = *pData++;
  }

  MEMCTR = memctr;
EA = 1;
#if (!defined HAL_OAD_BOOT_CODE) && (!defined HAL_OTA_BOOT_CODE)
  //HAL_EXIT_CRITICAL_SECTION(is);
#endif
}
/**************************************************************************************************
 * @fn          HalFlashWrite
 *
 * @brief       This function writes 'cnt' bytes to the internal flash.
 *
 * input parameters
 *
 * @param       addr - Valid HAL flash write address: actual addr / 4 and quad-aligned.
 * @param       buf - Valid buffer space at least as big as 'cnt' X 4.
 * @param       cnt - Number of 4-byte blocks to write.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void HalFlashWrite(uint16 addr, uint8 *buf, uint16 cnt)
{
//#if (defined HAL_DMA) && (HAL_DMA == TRUE)
//EA = 0;
  halDMADesc_t *ch = HAL_DMA_GET_DESC0();

  HAL_DMA_SET_SOURCE(ch, buf);
  HAL_DMA_SET_DEST(ch, &FWDATA);
  HAL_DMA_SET_VLEN(ch, HAL_DMA_VLEN_USE_LEN);
  HAL_DMA_SET_LEN(ch, (cnt * HAL_FLASH_WORD_SIZE));
  HAL_DMA_SET_WORD_SIZE(ch, HAL_DMA_WORDSIZE_BYTE);
  HAL_DMA_SET_TRIG_MODE(ch, HAL_DMA_TMODE_SINGLE);
  HAL_DMA_SET_TRIG_SRC(ch, HAL_DMA_TRIG_FLASH);
  HAL_DMA_SET_SRC_INC(ch, HAL_DMA_SRCINC_1);
  HAL_DMA_SET_DST_INC(ch, HAL_DMA_DSTINC_0);
  // The DMA is to be polled and shall not issue an IRQ upon completion.
  HAL_DMA_SET_IRQ(ch, HAL_DMA_IRQMASK_DISABLE);
  HAL_DMA_SET_M8( ch, HAL_DMA_M8_USE_8_BITS);
  HAL_DMA_SET_PRIORITY(ch, HAL_DMA_PRI_HIGH);
  HAL_DMA_CLEAR_IRQ(HAL_NV_DMA_CH);
  HAL_DMA_ARM_CH(HAL_NV_DMA_CH);

  FADDRL = (uint8)addr;
  FADDRH = (uint8)(addr >> 8);

  MEMCTR |= 0x08;

  FCTL |= 0x02;         // Trigger the DMA writes.
  while (FCTL & 0x80);  // Wait until writing is done.

  MEMCTR &= ~0x08;
 //EA = 1;
//#endif
}

/**************************************************************************************************
 * @fn          HalFlashErase
 *
 * @brief       This function erases the specified page of the internal flash.
 *
 * input parameters
 *
 * @param       pg - A valid flash page number to erase.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void HalFlashErase(uint8 pg)
{
  FADDRH = pg * (HAL_FLASH_PAGE_SIZE / HAL_FLASH_WORD_SIZE / 256);
  FCTL |= 0x01;
}

/*********************************************************************
 * @fn      osal_memcmp
 *
 * @brief
 *
 *   Generic memory compare.
 *
 * @param   src1 - source 1 addrexx
 * @param   src2 - source 2 address
 * @param   len - number of bytes to compare
 *
 * @return  TRUE - same, FALSE - different
 */
uint8 osal_memcmp( const void *src1, const void *src2, unsigned int len )
{
  //const uint8 GENERIC *pSrc1;
 // const uint8 GENERIC *pSrc2;
  const uint8 *pSrc1;
  const uint8 *pSrc2;

  pSrc1 = src1;
  pSrc2 = src2;

  while ( len-- )
  {
    if( *pSrc1++ != *pSrc2++ )
      return FALSE;
  }
  return TRUE;
}

/*********************************************************************
 * @fn      osal_memcpy
 *
 * @brief
 *
 *   Generic memory copy.
 *
 *   Note: This function differs from the standard memcpy(), since
 *         it returns the pointer to the next destination uint8. The
 *         standard memcpy() returns the original destination address.
 *
 * @param   dst - destination address
 * @param   src - source address
 * @param   len - number of bytes to copy
 *
 * @return  pointer to end of destination buffer
 */
void *osal_memcpy( void *dst, const void  *src, unsigned int len )
{
  uint8 *pDst;
  const uint8 *pSrc;

  pSrc = src;
  pDst = dst;

  while ( len-- )
    *pDst++ = *pSrc++;

  return ( pDst );
}



/***********************************************************************************
* @fn      halRfSetGain
*
* @brief   Set gain mode - only applicable for units with CC2590/91.
*
* @param   uint8 - gain mode
*
* @return  none
*/
void halRfSetGain(uint8 gainMode)
{
    if (gainMode==HAL_RF_GAIN_LOW) {
        HAL_PA_LNA_RX_LGM();
        rssiOffset = RSSI_OFFSET_LNA_LOWGAIN;
    } else {
        HAL_PA_LNA_RX_HGM();
        rssiOffset = RSSI_OFFSET_LNA_HIGHGAIN;
    }
}

static void halPaLnaInit(void)
{
#if INCLUDE_PA==2591
    // Initialize CC2591 to RX high gain mode
    static uint8 fFirst= TRUE;

    if(fFirst) {
        AGCCTRL1  = 0x15;
        FSCAL1 = 0x0;
        RFC_OBS_CTRL0 = 0x68;
        RFC_OBS_CTRL1 = 0x6A;
        OBSSEL1 = 0xFB;
        OBSSEL4 = 0xFC;
        P0DIR |= 0x80;
        halRfSetGain(HAL_RF_GAIN_HIGH);
    }

#else // do nothing
#endif
}

/***********************************************************************************
* LOCAL FUNCTIONS
*/



/***********************************************************************************

***********************************************************************************/
