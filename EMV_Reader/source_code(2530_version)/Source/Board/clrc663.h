/*
* Copyright (c) 2015,Yuanjian
* All rights reserved.
*
* 文件名称：board.h
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


#ifndef CLRC663_H
#define CLRC663_H

//#define IC_CS   P0_4
#define IC_CS   P1_4
#if 0
#define IC_SCK  P0_5
#define IC_MOSI P0_6
#define IC_MISO P1_0
#define IC_REST P0_1
#define BUZZER P1_2//低电平叫
#else
#define IC_SCK  P1_5
#define IC_MOSI P1_6
#define IC_MISO P1_7
#define IC_REST P1_2

#endif
void GenericApp_ReadCardMessage( void );
void ReaderInit(unsigned int readermode);
void Antenna_Switch(char on_or_off);
void ClearSameUID(void);
void ReceiveData(uint8 *head);

void PrtReceivedData(void);

//char *ArrayArray(const char *strSrc, uint8 lengthSrc,const char *str,uint8 length);
uint8 *ArrayArray(const uint8  * strSrc, uint8 lengthSrc,const uint8 *str,uint8 length);


uint8 *MagneticTrack2(uint8 *Src, uint8 length,uint8 *Dest,uint8 *currentPos);
uint8 *FromDate(uint8 *Src, uint8 length,uint8 *Dest,uint8* curPos);

//uint8 ItemAmount(uint8 * buffer);
uint8 ItemAmount(uint8 buffer[][128]);
void CleanBuff(uint8 buffer[][128]);
void TransmitBuff(void);

void IfSend (void);
uint8 DataUnitsAmount(void);
void GetPackageUart(void);

uint8  WaitRxEnd(void);

unsigned char ReadRegister2(unsigned char Address);
void ISO14443_A_Init(void);
void CardTest(void);

void TLVdecode(uint8 * tlvPos,uint8 packageLength);

void DoubleBytesTagProcess(uint8* curTagPos,uint8 **currentDst);
void SingleByteTagProcess(uint8* curTagPos,uint8 **currentDst);
uint8 DataCollectComplete(void);
void HardwareReset(void);
void CleanBuff2(uint8 buffer[][128],uint8 row);


//CLRC663 命令码和寄存器地址定义

/*******************************************************************************************/
/*            register     addredss                   function                             */
/*******************************************************************************************/
#define       Command 		        0x00	//Starts and stops command execution
#define       HostCtrl 		        0x01	//Host control register
#define       FIFOControl 	        0x02	//Control register of the FIFO
#define       WaterLevel 	        0x03	//Level of the FIFO underflow and overflow warning
#define       FIFOLength	        0x04	// Length of the FIFO
#define       FIFOData		        0x05	// Data In/Out exchange register of FIFO buffer
#define       IRQ0 			0x06	//Interrupt register 0
#define       IRQ1 			0x07	//Interrupt register 1
#define       IRQ0En		        0x08	// Interrupt enable register 0
#define       IRQ1En 		        0x09	//Interrupt enable register 1
#define       Error			0x0A	// Error bits showing the error status of the last command execution
#define       Status		        0x0B	// Contains status of the communication
#define       RxBitCtrl 	        0x0C	//Control register for anticollision adjustments for bit oriented protocols
#define       RxColl		        0x0D	// Collision position register
#define       TControl		        0x0E	// Control of Timer 0..3
#define       T0Control		        0x0F	// Control of Timer0
#define       T0ReloadHi 	        0x10	//High register of the reload value of Timer0
#define       T0ReloadLo		0x11	// Low register of the reload value of Timer0
#define       T0CounterValHi	        0x12	// Counter value high register of Timer0
#define       T0CounterValLo	        0x13	// Counter value low register of Timer0
#define       T1Control 		0x14	//Control of Timer1
#define       T1ReloadHi 		0x15	//High register of the reload value of Timer1
#define       T1ReloadLo		0x16	// Low register of the reload value of Timer1
#define       T1CounterValHi    	0x17	//Counter value high register of Timer1
#define       T1CounterValLo    	0x18	//Counter value low register of Timer1
#define       T2Control			0x19	// Control of Timer2
#define       T2ReloadHi 		0x1A	//High byte of the reload value of Timer2
#define       T2ReloadLo 		0x1B	//Low byte of the reload value of Timer2
#define       T2CounterValHi 	        0x1C	//Counter value high byte of Timer2
#define       T2CounterValLo	        0x1D	// Counter value low byte of Timer2
#define       T3Control 		0x1E	//Control of Timer3
#define       T3ReloadHi		0x1F	// High byte of the reload value of Timer3
#define       T3ReloadLo 		0x20	//Low byte of the reload value of Timer3
#define       T3CounterValHi    	0x21	//Counter value high byte of Timer3
#define       T3CounterValLo    	0x22	//Counter value low byte of Timer3
#define       T4Control 		0x23	//Control of Timer4
#define       T4ReloadHi 		0x24	//High byte of the reload value of Timer4
#define       T4ReloadLo 		0x25	//Low byte of the reload value of Timer4
#define       T4CounterValHi 	        0x26	//Counter value high byte of Timer4
#define       T4CounterValLo    	0x27	//Counter value low byte of Timer4
#define       DrvMod 			0x28	//Driver mode register
#define       TxAmp 			0x29	//Transmitter amplifier register
#define       DrvCon 			0x2A	//Driver configuration register
#define       Txl 			0x2B	//Transmitter register
#define       TxCrcPreset 		0x2C	//Transmitter CRC control register, preset value
#define       RxCrcPreset 		0x2D	//Receiver CRC control register, preset value
#define       TxDataNum 		0x2E	//Transmitter data number register
#define       TxModWidth 		0x2F	//Transmitter modulation width register
#define       TxSym10BurstLen   	0x30	//Transmitter symbol 1 + symbol 0 burst length register
#define       TXWaitCtrl 		0x31	//Transmitter wait control
#define       TxWaitLo			0x32	// Transmitter wait low
#define       FrameCon			0x33	// Transmitter frame control
#define       RxSofD 			0x34	//Receiver start of frame detection
#define       RxCtrl 			0x35	//Receiver control register
#define       RxWait 			0x36	//Receiver wait register
#define       RxThreshold 		0x37	//Receiver threshold register
#define       Rcv			0x38	// Receiver register
#define       RxAna 			0x39	//Receiver analog register
//#define     RFU 			0x3A	//
#define       SerialSpeed 		0x3B	//Serial speed register
#define       LFO_Trimm			0x3C	// Low-power oscillator trimming register
#define       PLL_Ctrl			0x3D	// IntegerN PLL control register, for microcontroller clock output adjustment
#define       PLL_DivOut 		0x3E	//IntegerN PLL control register, for microcontroller clock output adjustment
#define       LPCD_QMin 		0x3F	//Low-power card detection Q channel minimum threshold
#define       LPCD_QMax 		0x40	//Low-power card detection Q channel maximum threshold
#define       LPCD_IMin 		0x41	//Low-power card detection I channel minimum threshold
#define       LPCD_I_Result 	        0x42	//Low-power card detection I channel result register
#define       LPCD_Q_Result      	0x43	//Low-power card detection Q channel result register
#define       PadEn 	                0x44	//PIN enable register
#define       PadOut 			0x45	//PIN out register
#define       PadIn 			0x46	//PIN in register
#define       SigOut			0x47	// Enables and controls the SIGOUT Pin
#define       TxBitMod			0x48	// Transmitter bit mode register
//#define     RFU 			0x49	//
#define       TxDataCon			0x4A	// Transmitter data configuration register
#define       TxDataMod			0x4B	// Transmitter data modulation register
#define       TxSymFreq			0x4C	// Transmitter symbol frequency
#define       TxSym0H 			0x4D	//Transmitter symbol 0 high register
#define       TxSym0L 			0x4E	//Transmitter symbol 0 low register
#define       TxSym1H 			0x4F	//Transmitter symbol 1 high register
#define       TxSym1L 			0x50	//Transmitter symbol 1 low register
#define       TxSym2 			0x51	//Transmitter symbol 2 register
#define       TxSym3 			0x52	//Transmitter symbol 3 register
#define       TxSym10Len 		0x53	//Transmitter symbol 1 + symbol 0 length register
#define       TxSym32Len 		0x54	//Transmitter symbol 3 + symbol 2 length register
#define       TxSym10BurstCtrl  	0x55	//Transmitter symbol 1 + symbol 0 burst control register
#define       TxSym10Mod 		0x56	//Transmitter symbol 1 + symbol 0 modulation register
#define       TxSym32Mod 		0x57	//Transmitter symbol 3 + symbol 2 modulation register
#define       RxBitMod 			0x58	//Receiver bit modulation register
#define       RxEofSym 			0x59	//Receiver end of frame symbol register
#define       RxSyncValH 		0x5A	//Receiver synchronisation value high register
#define       RxSyncValL 		0x5B	//Receiver synchronisation value low register
#define       RxSyncMod 		0x5C	//Receiver synchronisation mode register
#define       RxMod 			0x5D	//Receiver modulation register
#define       RxCorr 			0x5E	//Receiver correlation register
#define       FabCal 			0x5F	//Calibration register of the receiver, calibration performed at production
#define       Version 			0x7F	//Version and subversion register
/*******************************************************************************************/
/*           command      command code                   function                          */
/*******************************************************************************************/
#define       Idle              0x00    //no action, cancels current command execution
#define       LPCD              0x01    //low-power card detection
#define       LoadKey           0x02    //reads a MIFARE key (size of 6 bytes) from FIFO buffer and puts it into Key buffer
#define       MFAuthent         0x03    //performs the MIFARE standard authentication in MIFARE read/write mode only
#define       AckReq            0x04    //performs a query, an Ack and a Req-Rn for ISO/IEC18000-3 mode 3/ EPC Class-1 HF
#define       Receive           0x05    //activates the receive circuit
#define       Transmit          0x06    // transmits data from the FIFO buffer
#define       Transceive        0x07    //transmits data from the FIFO buffer and automatically activates the receiver after transmission finished
#define       WriteE2           0x08    // gets one byte from FIFO buffer and writes it to the internal EEPROM, valid address range are the addresses of the MIFARE Key area
#define       WriteE2Page       0x09    //gets up to 64 bytes (one EEPROM page) from the FIFO buffer and writes it to the EEPROM, valid page addressrange are the pages of the MIFARE Key Area
#define       ReadE2            0x0A    //reads data from the EEPROM and copies it into the FIFO buffer, valid address range are the addresses of the MIFARE Key area
#define       LoadReg           0x0C    //reads data from the internal EEPROM and initializes the CLRC663 registers. EEPROM address needs to be within EEPROM sector 2
#define       LoadProtocol      0x0D    //reads data from the internal EEPROM and initializes the CLRC663 registers needed for a Protocol change
#define       LoadKeyE2         0x0E    //copies a key of the EEPROM into the key buffer
#define       StoreKeyE2        0x0F    //stores a MIFARE key (size of 6 bytes) into the EEPROM
#define       ReadRNR           0x1C    //Copies bytes from the Random Number generator intothe FIFO until the FiFo is full
#define       Soft Reset        0x1F    //resets the CLRC663
/*******************************************************************************************/
/*           mode                   code                           */
/*******************************************************************************************/
#define      ISO14443_A            0x00

/*******************************************************************************************/
/*          status code */
/*******************************************************************************************/
//#define       SUCCESS     0x00

#ifndef SUCCESS
#define SUCCESS 0
#endif
#define       ERROR       0x01
#define       ON          0x01
#define       OFF         0x00

#define 	RX_END		  0x00
#define 	RX_TIMEOUT	  0x01
#define 	RX_ERROR	  0x02


#define 	WAIT




















































#endif
