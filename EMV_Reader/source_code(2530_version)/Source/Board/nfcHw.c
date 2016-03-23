/*
* Copyright (c) 2015,Yuanjian
* All rights reserved.
*
* 文件名称：nfcHw.c
* 文件标识：
* 摘 要：
*
* 当前版本：0.0
* 作 者：   袁舰
* 完成日期：2015年12月21日
*
* 取代版本：
* 原作者:
* 完成日期：
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
#include "nfcHw.h"
void SPIinit(void)
{
		PERCFG |= 0x02;        // map USART1 to its alternative 2 location. P1_4: SSN, P1_5: SCK, P1_6: MOSI, P1_7: MISO
        P1SEL |= 0xE0;        // P1_5, P1_6, and P1_7 are peripherals

        U1BAUD = 0x00; U1GCR |= 0x11;        // Set baud rate to max (system clock frequency / 8)
        U1CSR &= ~0xA0;        // SPI Master Mode
        U1GCR &= ~0xC0;
		U1GCR |= 0x20;        // MSB

		/*  IC_CS P1_4 , P1_4 is GPIO (SSN)*/
		P1DIR |= 1<<4;
		P1INP |= 1<<4;
		P1SEL &= ~(1<<4);
}

void SPIWriteByte(uint8 data)
{
    U1DBUF = data;
	while (!(U1CSR&0x02));
	U1CSR &= 0xFD;
}

unsigned char SPIReadByte(void)
{
	unsigned char temp;
   	U1DBUF = 0x00;
   //U1DBUF = 0xff;
  	while (!(U1CSR&0x02));
  	U1CSR &= 0xFD;
	temp = U1DBUF;
	return temp;
}

unsigned char ReadRegister(unsigned char Address)
{
    unsigned char ucAddr;
    unsigned char ucResult=0;
	IC_CS = 0;
    ucAddr = (Address<<1)|0x01;//地址变换，SPI的读写地址有要求
	SPIWriteByte(ucAddr);
	ucResult=SPIReadByte();
	IC_CS = 1;
    return ucResult;
}

void WriteRegister(unsigned char Address, unsigned char value)
{
    unsigned char ucAddr;
    Address <<= 1;
    ucAddr = (Address&0xFE);//1111 1110
    IC_CS = 0;
	SPIWriteByte(ucAddr);
	SPIWriteByte(value);
   	IC_CS = 1;
}
//initialize the clrc663 to work with  ISO14443A

void ISO14443_A_Init(void)
{
	/* Configure T0 */
	WriteRegister(T0Control,0x98);           //Starts at the end of Tx. Stops after Rx of first data. Auto-reloaded. 13.56 MHz input clock.
	/* Configure T1 and cascade it with T0 */
	WriteRegister(T1Control,0x92);          //Starts at the end of Tx. Stops after Rx of first data. Input clock - cascaded with Timer-0.
	/* Configure T2 for LFO AutoTrimm */
	WriteRegister(T2Control,0x20);          //Set Timer-2, T2Control_Reg:  Timer used for LFO trimming
	/* T2 reload value for LFO AutoTrimm*/
	WriteRegister(T2ReloadHi,0x03);         //Set Timer-2 reload value (T2ReloadHi_Reg and T2ReloadLo_Reg)
  	WriteRegister(T2ReloadLo,0xFF);         //

	/* Configure T3 (for LPCD/ AutoTrimm) */
	WriteRegister(T3Control,0x00);          // Not started automatically. Not reloaded. Input clock 13.56 MHz

	/* Set FiFo-Size and Waterlevel */
	 WriteRegister(FIFOControl,0x10);   //SIZE 255

  	WriteRegister(WaterLevel,0xFE);        //Set WaterLevel =(FIFO length -1),cause fifo length has been set to 255=0xff,so water level is oxfe
 	 WriteRegister(RxBitCtrl,0x80);         //RxBitCtrl_Reg(0x0c)	Received bit after collision are replaced

	   WriteRegister(RxSofD,0x00);            //作为被动设备时使用


  WriteRegister(Rcv,0x12);               //


}

