/***********************************************************************************

  Filename:     hal_rf.h

  Description:  HAL radio interface header file

***********************************************************************************/


#ifndef HAL_RF_H
#define HAL_RF_H

/***********************************************************************************
* INCLUDES
*/
#include <types.h>

#define	INCLUDE_PA 2591

/***********************************************************************************
* TYPEDEFS
*/

/***********************************************************************************
* CONSTANTS AND DEFINES
*/

	/* RFPWR */
#define ADI_RADIO_PD    BV(4)
#define RREG_RADIO_PD   BV(3)

/* MDMCTRL1L */
#define MDMCTRL1L_RESET_VALUE         0x00
#define RX_MODE(x)                    ((x) << 0)
#define RX_MODE_INFINITE_RECEPTION    RX_MODE(2)
#define RX_MODE_NORMAL_OPERATION      RX_MODE(0)

/* FSMSTATE */
#define FSM_FFCTRL_STATE_RX_INF       31      /* infinite reception state - not documented in datasheet */

/* ADCCON1 */
#define RCTRL1                        BV(3)
#define RCTRL0                        BV(2)
#define RCTRL_BITS                    (RCTRL1 | RCTRL0)
#define RCTRL_CLOCK_LFSR              RCTRL0


// Chip ID's
#define HAL_RF_CHIP_ID_CC1100               0x00
#define HAL_RF_CHIP_ID_CC1110               0x01
#define HAL_RF_CHIP_ID_CC1111               0x11
#define HAL_RF_CHIP_ID_CC2420               0x00//0x02 comment: datasheet is inconsistent
#define HAL_RF_CHIP_ID_CC2500               0x80
#define HAL_RF_CHIP_ID_CC2510               0x81
#define HAL_RF_CHIP_ID_CC2511               0x91
#define HAL_RF_CHIP_ID_CC2550               0x82
#define HAL_RF_CHIP_ID_CC2520               0x84
#define HAL_RF_CHIP_ID_CC2430               0x85
#define HAL_RF_CHIP_ID_CC2431               0x89
#define HAL_RF_CHIP_ID_CC2530               0xA5
#define HAL_RF_CHIP_ID_CC2531               0xB5
#define HAL_RF_CHIP_ID_CC2540               0x8D

// CC2590/91 gain modes
#define HAL_RF_GAIN_LOW                     0
#define HAL_RF_GAIN_HIGH                    1

// IEEE 802.15.4 defined constants (2.4 GHz logical channels)
#define MIN_CHANNEL 				        11    // 2405 MHz
#define MAX_CHANNEL                         26    // 2480 MHz
#define CHANNEL_SPACING                     5     // MHz

/***********************************************************************************
* CONSTANTS AND DEFINES
*/
	// Chip revision
#define REV_A                      0x01
#define CHIPREVISION              REV_A
	
	// CC2530 RSSI Offset
#define RSSI_OFFSET                               73
#define RSSI_OFFSET_LNA_HIGHGAIN                  79
#define RSSI_OFFSET_LNA_LOWGAIN                   67
	
	// Various radio settings
//#define AUTO_ACK                   0x20
//#define AUTO_CRC                   0x40
	
	// TXPOWER values
#if CC2530_PG1
#define CC2530_TXPOWER_MIN_3_DBM   0x88 
#define CC2530_TXPOWER_0_DBM       0x32
#define CC2530_TXPOWER_4_DBM       0xF7
#else
#define CC2530_TXPOWER_MIN_3_DBM   0xB5 
#define CC2530_TXPOWER_0_DBM       0xD5
#define CC2530_TXPOWER_4_DBM       0xF5
#endif

/* FSCTRLL */
#define FREQ_2405MHZ    0x0B

/* RFIM */
//#define IM_TXDONE       BV(6)
//#define IM_FIFOP        BV(5)
//#define IM_SFD          BV(4)

// Output power programming (TXCTRLL)
#define PA_LEVEL_MASK           0x1F

// RF status flags
/*#define TX_ACTIVE_FLAG          0x10
#define FIFO_FLAG               0x08
#define FIFOP_FLAG              0x04
#define SFD_FLAG                0x02
#define CCA_FLAG                0x01
*/

/* FSMSTAT1 */
#define FIFO_FLAG            BV(7)
#define FIFOP_FLAG           BV(6)
#define SFD_FLAG             BV(5)
#define SAMPLED_CCA_FLAG     BV(3)


// Radio status states
#define TX_ACTIVE()             (FSMSTAT1 & SAMPLED_CCA_FLAG)
#define FIFO()                  (FSMSTAT1 & FIFO_FLAG)
#define FIFOP()                 (FSMSTAT1 & FIFOP_FLAG)
#define SFD()                   (FSMSTAT1 & SFD_FLAG)
#define CCA()                   (FSMSTAT1 & SAMPLED_CCA_FLAG)

// Various radio settings
#define ADR_DECODE              0x08
#define AUTO_CRC                0x20
#define AUTO_ACK                0x10
#define AUTO_TX2RX_OFF          0x08
#define RX2RX_TIME_OFF          0x04
#define ACCEPT_ACKPKT           0x01

// RF interrupt flags
#define IRQ_TXDONE                 0x02
#define IRQ_RXPKTDONE              0x40

// RF interrupt flags
/*
#define IRQ_RREG_ON             0x80
#define IRQ_TXDONE              0x40
#define IRQ_FIFOP               0x20
#define IRQ_SFD                 0x10
#define IRQ_CCA                 0x08
#define IRQ_CSP_WT              0x04
#define IRQ_CSP_STOP            0x02
#define IRQ_CSP_INT             0x01
*/

/* RFIRQF1 */
//#define IRQ_TXDONE      BV(1)

/* RFIRQF0 */
#define IRQ_FIFOP       BV(2)
#define IRQ_SFD         BV(1)

/* RFIRQM1 */
#define IM_TXDONE       BV(1)

/* RFIRQM0 */
#define IM_FIFOP        BV(2)
#define IM_SFD          BV(1)

/* FRMFILT0 */
#define FRAME_FILTER_EN     BV(0)

/* FRMCTRL0 */
#define RX_MODE_MASK        (BV(2) | BV(3))
#define RX_MODE_NORMAL      (0x00 << 2)
#define RX_MODE_INFINITE_RX (0x2 << 2)

/* Wait for RSSI to be valid. */
#define MRFI_RSSI_VALID_WAIT()  while(!(RSSISTAT & BV(0)));

// RF status flags

// Selected strobes
#define ISRXON()                st(RFST = 0xE3;)
#define ISTXON()                st(RFST = 0xE9;)
#define ISTXONCCA()             st(RFST = 0xEA;)
#define ISRFOFF()               st(RFST = 0xEF;)
#define ISFLUSHRX()             st(RFST = 0xEC;)
#define ISFLUSHTX()             st(RFST = 0xEE;)

#define FLUSH_RX_FIFO()         st( ISFLUSHRX(); ISFLUSHRX(); )


// CC2590-CC2591 support
#if INCLUDE_PA==2591

// Support for PA/LNA
#define HAL_PA_LNA_INIT()

// Select CC2591 RX high gain mode 
#define HAL_PA_LNA_RX_HGM() st( uint8 i; P0_7 = 1; for (i=0; i<8; i++) asm("NOP"); )

// Select CC2591 RX low gain mode
#define HAL_PA_LNA_RX_LGM() st( uint8 i; P0_7 = 0; for (i=0; i<8; i++) asm("NOP"); )

// TX power lookup index
#define HAL_RF_TXPOWER_0_DBM          0
#define HAL_RF_TXPOWER_13_DBM         1
#define HAL_RF_TXPOWER_16_DBM         2
#define HAL_RF_TXPOWER_18_DBM         3
#define HAL_RF_TXPOWER_20_DBM         4

// TX power values
#define CC2530_91_TXPOWER_0_DBM       0x25
#define CC2530_91_TXPOWER_13_DBM      0x85
#define CC2530_91_TXPOWER_16_DBM      0xA5
#define CC2530_91_TXPOWER_18_DBM      0xC5
#define CC2530_91_TXPOWER_20_DBM      0xE5

#else // dummy macros when not using CC2591

#define HAL_PA_LNA_INIT()
#define HAL_PA_LNA_RX_LGM()
#define HAL_PA_LNA_RX_HGM()

#define HAL_RF_TXPOWER_MIN_3_DBM  0
#define HAL_RF_TXPOWER_0_DBM      1
#define HAL_RF_TXPOWER_4_DBM	  2

#endif

/***********************************************************************************
* FLASH
*/
	// Use LEN for transfer count
#define HAL_DMA_VLEN_USE_LEN            0x00
	// Transfer the first byte + the number of bytes indicated by the first byte
#define HAL_DMA_VLEN_1_P_VALOFFIRST     0x01
	// Transfer the number of bytes indicated by the first byte (starting with the first byte)
#define HAL_DMA_VLEN_VALOFFIRST         0x02
	// Transfer the first byte + the number of bytes indicated by the first byte + 1 more byte
#define HAL_DMA_VLEN_1_P_VALOFFIRST_P_1 0x03
	// Transfer the first byte + the number of bytes indicated by the first byte + 2 more bytes
#define HAL_DMA_VLEN_1_P_VALOFFIRST_P_2 0x04
	
#define HAL_DMA_WORDSIZE_BYTE           0x00 /* Transfer a byte at a time. */
#define HAL_DMA_WORDSIZE_WORD           0x01 /* Transfer a 16-bit word at a time. */
	
#define HAL_DMA_TMODE_SINGLE            0x00 /* Transfer a single byte/word after each DMA trigger. */
#define HAL_DMA_TMODE_BLOCK             0x01 /* Transfer block of data (length len) after each DMA trigger. */
#define HAL_DMA_TMODE_SINGLE_REPEATED   0x02 /* Transfer single byte/word (after len transfers, rearm DMA). */
#define HAL_DMA_TMODE_BLOCK_REPEATED    0x03 /* Transfer block of data (after len transfers, rearm DMA). */
	
#define HAL_DMA_TRIG_NONE           0   /* No trigger, setting DMAREQ.DMAREQx bit starts transfer. */
#define HAL_DMA_TRIG_PREV           1   /* DMA channel is triggered by completion of previous channel. */
#define HAL_DMA_TRIG_T1_CH0         2   /* Timer 1, compare, channel 0. */
#define HAL_DMA_TRIG_T1_CH1         3   /* Timer 1, compare, channel 1. */
#define HAL_DMA_TRIG_T1_CH2         4   /* Timer 1, compare, channel 2. */
#define HAL_DMA_TRIG_T2_COMP        5   /* Timer 2, compare. */
#define HAL_DMA_TRIG_T2_OVFL        6   /* Timer 2, overflow. */
#define HAL_DMA_TRIG_T3_CH0         7   /* Timer 3, compare, channel 0. */
#define HAL_DMA_TRIG_T3_CH1         8   /* Timer 3, compare, channel 1. */
#define HAL_DMA_TRIG_T4_CH0         9   /* Timer 4, compare, channel 0. */
#define HAL_DMA_TRIG_T4_CH1        10   /* Timer 4, compare, channel 1. */
#define HAL_DMA_TRIG_ST            11   /* Sleep Timer compare. */
#define HAL_DMA_TRIG_IOC_0         12   /* Port 0 I/O pin input transition. */
#define HAL_DMA_TRIG_IOC_1         13   /* Port 1 I/O pin input transition. */
#define HAL_DMA_TRIG_URX0          14   /* USART0 RX complete. */
#define HAL_DMA_TRIG_UTX0          15   /* USART0 TX complete. */
#define HAL_DMA_TRIG_URX1          16   /* USART1 RX complete. */
#define HAL_DMA_TRIG_UTX1          17   /* USART1 TX complete. */
#define HAL_DMA_TRIG_FLASH         18   /* Flash data write complete. */
#define HAL_DMA_TRIG_RADIO         19   /* RF packet byte received/transmit. */
#define HAL_DMA_TRIG_ADC_CHALL     20   /* ADC end of a conversion in a sequence, sample ready. */
#define HAL_DMA_TRIG_ADC_CH0       21   /* ADC end of conversion channel 0 in sequence, sample ready. */
#define HAL_DMA_TRIG_ADC_CH1       22   /* ADC end of conversion channel 1 in sequence, sample ready. */
#define HAL_DMA_TRIG_ADC_CH2       23   /* ADC end of conversion channel 2 in sequence, sample ready. */
#define HAL_DMA_TRIG_ADC_CH3       24   /* ADC end of conversion channel 3 in sequence, sample ready. */
#define HAL_DMA_TRIG_ADC_CH4       25   /* ADC end of conversion channel 4 in sequence, sample ready. */
#define HAL_DMA_TRIG_ADC_CH5       26   /* ADC end of conversion channel 5 in sequence, sample ready. */
#define HAL_DMA_TRIG_ADC_CH6       27   /* ADC end of conversion channel 6 in sequence, sample ready. */
#define HAL_DMA_TRIG_ADC_CH7       28   /* ADC end of conversion channel 7 in sequence, sample ready. */
#define HAL_DMA_TRIG_ENC_DW        29   /* AES encryption processor requests download input data. */
#define HAL_DMA_TRIG_ENC_UP        30   /* AES encryption processor requests upload output data. */
	
#define HAL_DMA_SRCINC_0         0x00 /* Increment source pointer by 0 bytes/words after each transfer. */
#define HAL_DMA_SRCINC_1         0x01 /* Increment source pointer by 1 bytes/words after each transfer. */
#define HAL_DMA_SRCINC_2         0x02 /* Increment source pointer by 2 bytes/words after each transfer. */
#define HAL_DMA_SRCINC_M1        0x03 /* Decrement source pointer by 1 bytes/words after each transfer. */
	
#define HAL_DMA_DSTINC_0         0x00 /* Increment destination pointer by 0 bytes/words after each transfer. */
#define HAL_DMA_DSTINC_1         0x01 /* Increment destination pointer by 1 bytes/words after each transfer. */
#define HAL_DMA_DSTINC_2         0x02 /* Increment destination pointer by 2 bytes/words after each transfer. */
#define HAL_DMA_DSTINC_M1        0x03 /* Decrement destination pointer by 1 bytes/words after each transfer. */
	
#define HAL_DMA_IRQMASK_DISABLE  0x00 /* Disable interrupt generation. */
#define HAL_DMA_IRQMASK_ENABLE   0x01 /* Enable interrupt generation upon DMA channel done. */
	
#define HAL_DMA_M8_USE_8_BITS    0x00 /* Use all 8 bits for transfer count. */
#define HAL_DMA_M8_USE_7_BITS    0x01 /* Use 7 LSB for transfer count. */
	
#define HAL_DMA_PRI_LOW          0x00 /* Low, CPU has priority. */
#define HAL_DMA_PRI_GUARANTEED   0x01 /* Guaranteed, DMA at least every second try. */
#define HAL_DMA_PRI_HIGH         0x02 /* High, DMA has priority. */
#define HAL_DMA_PRI_ABSOLUTE     0x03 /* Highest, DMA has priority. Reserved for DMA port access.. */
	
#define HAL_DMA_MAX_ARM_CLOCKS   45   // Maximum number of clocks required if arming all 5 at once.

// Flash is partitioned into 8 banks of 32 KB or 16 pages.
#define HAL_FLASH_PAGE_PER_BANK    16
// Flash is constructed of 128 pages of 2 KB.
#define HAL_FLASH_PAGE_SIZE        2048
#define HAL_FLASH_WORD_SIZE        4

// Used by DMA macros to shift 1 to create a mask for DMA registers.
#define HAL_NV_DMA_CH              0

// Bit fields of the 'lenModeH'
#define HAL_DMA_LEN_V     0xE0
#define HAL_DMA_LEN_H     0x1F

// Bit fields of the 'ctrlA'
#define HAL_DMA_WORD_SIZE 0x80
#define HAL_DMA_TRIG_MODE 0x60
#define HAL_DMA_TRIG_SRC  0x1F

// Bit fields of the 'ctrlB'
#define HAL_DMA_SRC_INC   0xC0
#define HAL_DMA_DST_INC   0x30
#define HAL_DMA_IRQ_MASK  0x08
#define HAL_DMA_M8        0x04
#define HAL_DMA_PRIORITY  0x03

typedef struct {
  uint8 srcAddrH;
  uint8 srcAddrL;
  uint8 dstAddrH;
  uint8 dstAddrL;
  uint8 xferLenV;
  uint8 xferLenL;
  uint8 ctrlA;
  uint8 ctrlB;
} halDMADesc_t;

#define HAL_DMA_SET_ADDR_DESC0( a ) \
  st( \
    DMA0CFGH = (uint8)( (uint16)(a) >> 8 );  \
    DMA0CFGL = (uint8)( (uint16)(a) & 0xFF );       \
  )

#define HAL_DMA_SET_ADDR_DESC1234( a ) \
  st( \
    DMA1CFGH = (uint8)( (uint16)(a) >> 8 );  \
    DMA1CFGL = (uint8)( (uint16)(a) & 0xFF );       \
  )

#define HAL_DMA_GET_DESC0()           &dmaCh0

#define HAL_DMA_GET_DESC1234( a )     (dmaCh1234+((a)-1))

#define HAL_DMA_ARM_CH( ch )           DMAARM = (0x01 << (ch))

#define HAL_DMA_CH_ARMED( ch )        (DMAARM & (0x01 << (ch)))

#define HAL_DMA_ABORT_CH( ch )         DMAARM = (0x80 | (0x01 << (ch)))
#define HAL_DMA_MAN_TRIGGER( ch )      DMAREQ = (0x01 << (ch))
#define HAL_DMA_START_CH( ch )         HAL_DMA_MAN_TRIGGER( (ch) )

#define HAL_DMA_CLEAR_IRQ( ch )        DMAIRQ = ~( 1 << (ch) )

#define HAL_DMA_CHECK_IRQ( ch )       (DMAIRQ & ( 1 << (ch) ))

// Macro for quickly setting the source address of a DMA structure.
#define HAL_DMA_SET_SOURCE( pDesc, src ) \
  st( \
    pDesc->srcAddrH = (uint8)((uint16)(src) >> 8); \
    pDesc->srcAddrL = (uint8)( (uint16)(src) & 0xFF ); \
  )

// Macro for quickly setting the destination address of a DMA structure.
#define HAL_DMA_SET_DEST( pDesc, dst ) \
  st( \
    pDesc->dstAddrH = (uint8)((uint16)(dst) >> 8); \
    pDesc->dstAddrL = (uint8)( (uint16)(dst) & 0xFF ); \
  )

// Macro for quickly setting the number of bytes to be transferred by the DMA,
// max length is 0x1FFF.
#define HAL_DMA_SET_LEN( pDesc, len ) \
  st( \
    pDesc->xferLenL = (uint8)( (uint16)(len) & 0xFF); \
    pDesc->xferLenV &= ~HAL_DMA_LEN_H; \
    pDesc->xferLenV |= (uint8)((uint16)(len) >> 8); \
  )

#define HAL_DMA_GET_LEN( pDesc ) \
  (((uint16)(pDesc->xferLenV & HAL_DMA_LEN_H) << 8) | pDesc->xferLenL)

#define HAL_DMA_SET_VLEN( pDesc, vMode ) \
  st( \
    pDesc->xferLenV &= ~HAL_DMA_LEN_V; \
    pDesc->xferLenV |= (vMode << 5); \
  )

#define HAL_DMA_SET_WORD_SIZE( pDesc, xSz ) \
  st( \
    pDesc->ctrlA &= ~HAL_DMA_WORD_SIZE; \
    pDesc->ctrlA |= (xSz << 7); \
  )

#define HAL_DMA_SET_TRIG_MODE( pDesc, tMode ) \
  st( \
    pDesc->ctrlA &= ~HAL_DMA_TRIG_MODE; \
    pDesc->ctrlA |= (tMode << 5); \
  )

#define HAL_DMA_GET_TRIG_MODE( pDesc ) ((pDesc->ctrlA >> 5) & 0x3)

#define HAL_DMA_SET_TRIG_SRC( pDesc, tSrc ) \
  st( \
    pDesc->ctrlA &= ~HAL_DMA_TRIG_SRC; \
    pDesc->ctrlA |= tSrc; \
  )

#define HAL_DMA_SET_SRC_INC( pDesc, srcInc ) \
  st( \
    pDesc->ctrlB &= ~HAL_DMA_SRC_INC; \
    pDesc->ctrlB |= (srcInc << 6); \
  )

#define HAL_DMA_SET_DST_INC( pDesc, dstInc ) \
  st( \
    pDesc->ctrlB &= ~HAL_DMA_DST_INC; \
    pDesc->ctrlB |= (dstInc << 4); \
  )

#define HAL_DMA_SET_IRQ( pDesc, enable ) \
  st( \
    pDesc->ctrlB &= ~HAL_DMA_IRQ_MASK; \
    pDesc->ctrlB |= (enable << 3); \
  )

#define HAL_DMA_SET_M8( pDesc, m8 ) \
  st( \
    pDesc->ctrlB &= ~HAL_DMA_M8; \
    pDesc->ctrlB |= (m8 << 2); \
  )

#define HAL_DMA_SET_PRIORITY( pDesc, pri ) \
  st( \
    pDesc->ctrlB &= ~HAL_DMA_PRIORITY; \
    pDesc->ctrlB |= pri; \
  )
	


/***********************************************************************************
* GLOBAL FUNCTIONS
*/

// Generic RF interface
uint8 halRfInit(void);
void DMAInit(void);
void read_IEEE_address(void);
void HalFlashRead(uint8, uint16, uint8 *, uint16 );
void HalFlashWrite(uint16 , uint8 * , uint16 );
void HalFlashErase(uint8);
uint8 osal_memcmp( const void *src1, const void *src2, unsigned int len );
void *osal_memcpy( void *dst, const void  *src, unsigned int len );

uint8 halRfTransmit(void);
//void  halRfSetGain(uint8 gainMode);     // With CC2590/91 only

uint8 halRfGetChipId(void);
uint8 halRfGetChipVer(void);
uint8 halRfGetRandomByte(void);
uint8  RandomBackoffDelay(void);
uint8 halRfGetRssiOffset(void);

void  halRfWriteTxBuf(uint8* pData, uint8 length);
void  halRfReadRxBuf(uint8* pData, uint8 length);
void  halRfWaitTransceiverReady(void);

void  halRfReceiveOn(void);
void  halRfReceiveOff(void);
void  halRfDisableRxInterrupt(void);
void  halRfEnableRxInterrupt(void);
void  halRfRxInterruptConfig(ISR_FUNC_PTR pfISR);



// IEEE 802.15.4 specific interface
void  halRfSetChannel(uint8 channel);
void  halRfSetShortAddr(uint16 shortAddr);
void  halRfSetPanId(uint16 PanId);

void RF_SetLogicalChannel(uint8);
uint8 GetRandomByte(void);
void RandomSeedValueModeChange(void);
void GetRandomByte_test(void);

void halPaLnaInit(void);
uint8 halRfSetTxPower(uint8 power);
void UARTSendDMA(uint8 DMACh,uint16 len);
void UARTSendDMAWithSrc(uint8 DMACh,uint16 len,uint8 *src);




#endif

/***********************************************************************************
 
***********************************************************************************/
