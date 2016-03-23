/*
* Copyright (c) 2015,Yuanjian
* All rights reserved.
*
* 文件名称：clrc663.c
* 文件标识：
* 摘 要：
*
* 当前版本：0.0
* 作 者：   袁舰
* 完成日期：2015年9月1日
*
* 取代版本：
* 原作者:
* 完成日期：
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/

#include "types.h"
#include "clrc663.h"
#include "ioCC2530.h"
#include "stdio.h"
#include "string.h"
#include "myMacros.h"
#include "basic_rf.h"



extern TASK_COMPONENTS TaskComps[];

uint16 tempErrorCode;
uint8 address = 0x55;

//function declaration
void Delay_I_1us(unsigned int k);
void Interface_Init(void);
void SPIWriteByte(uint8 infor);
unsigned char SPIReadByte(void);
unsigned char ReadRegister(unsigned char Address);
void WriteRegister(unsigned char Address, unsigned char value);
void SetBitMask(unsigned char reg,unsigned char mask);
void ClearBitMask(unsigned char reg,unsigned char mask);
void Antenna_Switch(char on_or_off);
uint8 RequestA(char* ATQA);
uint8 TypeA_Card_Select(char* uniqueID);
void ReaderInit(unsigned int readermode);

uint8 common_buffer[256];

uint8 I_block_PCB=0x02;//每次发出一个APDU将此PDB的最低位（block_number）取反加到apdu的前面
uint8 ATS[256] ,SAK ,RATS[]={0xE0,0x80};//8 means the PCD can processs a maxmum of 256 byte,tell the PICC do not exceed this number
//char ATQA[2],UID[5],UIDCPY[5];
char ATQA[5],UID[5],UIDCPY[5];

uint16 UID_length,iterator,received_length;
uint16 tempErrorCode;

uint8 dataBuffer[11][128] = {0};
uint8 RfTxBuff[128] = {0};


//initalize the SPI interface and shutdown the buzzer

void Interface_Init(void)
{
#if 0
	/*  IC_CS P0_4 */
P1DIR |= 1<<4;
P1INP |= 1<<4;
P1SEL &= ~(1<<4);

 /* IC_SCK	P0_5 */
P1DIR |= 1<<5;
P1INP |= 1<<5;
P1SEL &= ~(1<<5);

   /* IC_MOSI P0_6 */
P1DIR |= 1<<6;
P1INP |= 1<<6;
P1SEL &= ~(1<<6);

   /* IC_MISO P1_7 */
P1DIR &= ~(1<<7);
P1INP &= ~(1<<7);
P1SEL &= ~(1<<7);

   /* IC_REST P1_2 */
P1DIR |= 1<<2;
P1INP |= 1<<2;
P1SEL &= ~(1<<2);

   /* IRQ P1_3 */
P1DIR &= ~(1<<3);
P1INP &= ~(1<<3);
P1SEL &= ~(1<<3);


IC_REST = 0;
DelayMs(2);
IC_REST = 1;
DelayMs(2);
IC_REST = 0;
DelayMs(2);
	 /* buzzer 初始化蜂鸣器*/
//P1DIR |= 1<<2;
//P1INP |= 1<<2;
//P1SEL &= ~(1<<2);
//BUZZER=1;//关闭蜂鸣器
IC_SCK = 0;
IC_CS = 1;
#else
#if 0
/*  IC_CS P0_4 */
P1DIR |= 1<<4;
P1INP |= 1<<4;
P1SEL &= ~(1<<4);

 /* IC_SCK	P0_5 */
P1DIR |= 1<<5;
P1INP |= 1<<5;
P1SEL &= ~(1<<5);

   /* IC_MOSI P0_6 */
P1DIR |= 1<<6;
P1INP |= 1<<6;
P1SEL &= ~(1<<6);

   /* IC_MISO P1_7 */
P1DIR &= ~(1<<7);
P1INP &= ~(1<<7);
P1SEL &= ~(1<<7);

   /* IC_REST P1_2 */
P1DIR |= 1<<2;
P1INP |= 1<<2;
P1SEL &= ~(1<<2);

   /* IRQ P1_3 */
P1DIR &= ~(1<<3);
P1INP &= ~(1<<3);
P1SEL &= ~(1<<3);


IC_REST = 0;
DelayMs(2);
IC_REST = 1;
DelayMs(2);
IC_REST = 0;
DelayMs(2);
	 /* buzzer 初始化蜂鸣器*/
//P1DIR |= 1<<2;
//P1INP |= 1<<2;
//P1SEL &= ~(1<<2);
//BUZZER=1;//关闭蜂鸣器
IC_SCK = 0;
IC_CS = 1;
//while(1);
#endif
#if 1
		 PERCFG |= 0x02;        // map USART1 to its alternative 2 location. P1_4: SSN, P1_5: SCK, P1_6: MOSI, P1_7: MISO
        P1SEL |= 0xE0;        // P1_5, P1_6, and P1_7 are peripherals
        //P1SEL &= ~0x10;        // P1_4 is GPIO (SSN)
        //P1DIR |= 0x10;        // SSN is set as output

        U1BAUD = 0x00; U1GCR |= 0x11;        // Set baud rate to max (system clock frequency / 8)
        U1CSR &= ~0xA0;        // SPI Master Mode
//        U1CSR &= ~0x80; U1CSR |= 0x20;        // SPI Slave Mode
        U1GCR &= ~0xC0;
        //U1GCR |= 0x40;

		U1GCR |= 0x20;        // MSB

		/*  IC_CS P0_4 */
   P1DIR |= 1<<4;
   P1INP |= 1<<4;
   P1SEL &= ~(1<<4);

		      /* IC_REST P1_2 */
   P1DIR |= 1<<2;
   P1INP |= 1<<2;
   P1SEL &= ~(1<<2);

   IC_REST = 0;
   DelayMs(2);
   IC_REST = 1;
   DelayMs(2);
   IC_REST = 0;
   DelayMs(2);

#endif
#endif


}

void SPIWriteByte(uint8 infor)
{
#if 0
    unsigned int counter;
    for(counter=0;counter<8;counter++)
    {

      if(infor&0x80)
        IC_MOSI = 1;
      else
        IC_MOSI = 0;
      //Delay_I_1us(3);
	  DelayHalfUs(3);
      IC_SCK = 1;
      //Delay_I_1us(1);
      DelayHalfUs(1);
      IC_SCK = 0;
      //Delay_I_1us(3);
      DelayHalfUs(3);
      infor <<= 1;
    }
#else

   // P1_4 = 0;        // SSN LOW
    U1DBUF = infor; while (!(U1CSR&0x02)); U1CSR &= 0xFD;
	//printf("e\n");
	// P1_4 = 1;
#endif

}

unsigned char SPIReadByte(void)
{
#if 0
  unsigned int counter;
  unsigned char SPI_Data=0;
  for(counter=0;counter<8;counter++)
  {
    SPI_Data<<=1;
    IC_SCK = 1;
    //Delay_I_1us(3);
	DelayHalfUs(3);
    if(IC_MISO == 1)
      SPI_Data |= 0x01;
      //Delay_I_1us(2);
      DelayHalfUs(2);
      IC_SCK = 0;
      //Delay_I_1us(3);
      DelayHalfUs(3);

  }
  return SPI_Data;
#else



   unsigned char temp;
  // P1_4 = 0;		 // SSN LOW
   //U1DBUF = 0x00;
   U1DBUF = 0xff;
  while (!(U1CSR&0x02));
  U1CSR &= 0xFD;

   temp = U1DBUF;
   //P1_4 = 1;		// SSN high
    //printf("dd%dff\r\n",temp);

	return temp;
#endif


}
/////////////////////////////////////////////////////////////////////
//功    能：读CLRC663寄存器
//参数说明：Address[IN]:寄存器地址
//返    回：读出的值
/////////////////////////////////////////////////////////////////////
unsigned char ReadRegister(unsigned char Address)
{
    unsigned char ucAddr;
    unsigned char ucResult=0;
	//DelayMs(5);
	//DelayUs(500);
	IC_CS = 0;
    ucAddr = (Address<<1)|0x01;//地址变换，SPI的读写地址有要求
	SPIWriteByte(ucAddr);
	ucResult=SPIReadByte();
	IC_CS = 1;
    return ucResult;
}

unsigned char ReadRegister2(unsigned char Address)
{
    unsigned char ucAddr;
    unsigned char ucResult=0;
	//DelayMs(5);
	//DelayUs(500);
	IC_CS = 0;
    ucAddr = (Address<<1)|0x01;//地址变换，SPI的读写地址有要求
	SPIWriteByte(ucAddr);
	ucResult=SPIReadByte();
	IC_CS = 1;
    return ucResult;
}

/////////////////////////////////////////////////////////////////////
//功    能：写CLRC663寄存器
//参数说明：Address[IN]:寄存器地址
//          value[IN]:写入的值
/////////////////////////////////////////////////////////////////////
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

/////////////////////////////////////////////////////////////////////
//功    能：置clrc663寄存器位
//参数说明：reg[IN]:寄存器地址
//          mask[IN]:置位值
/////////////////////////////////////////////////////////////////////
void SetBitMask(unsigned char reg,unsigned char mask)
{
    char tmp = 0x0;
    tmp = ReadRegister(reg);
    WriteRegister(reg,tmp | mask);  // set bit mask
}

/////////////////////////////////////////////////////////////////////
//功    能：清clrc663寄存器位
//参数说明：reg[IN]:寄存器地址
//          mask[IN]:清位值
/////////////////////////////////////////////////////////////////////
void ClearBitMask(unsigned char reg,unsigned char mask)
{
    char tmp = 0x0;
    tmp = ReadRegister(reg);
    WriteRegister(reg, tmp & ~mask);  // clear bit mask
}

//turn on or shutdown antenna
//para:on/off 1/0
void Antenna_Switch(char on_or_off)
{
if(on_or_off)WriteRegister(DrvMod,0x89);//>  FieldOn
else  WriteRegister(DrvMod,0x81);       //>  FieldOff
}















//initialize the clrc663 to work with  ISO14443A

void ISO14443_A_Init(void)
{
   IC_REST = 0;
   DelayMs(2);

//> =============================================
//> RC663 ApplyProtocolSettings:  ISO14443A=01
//> =============================================
  //> Configure Timers
  WriteRegister(T0Control,0x98);           //Starts at the end of Tx. Stops after Rx of first data. Auto-reloaded. 13.56 MHz input clock.
  WriteRegister(T1Control,0x92);          //Starts at the end of Tx. Stops after Rx of first data. Input clock - cascaded with Timer-0.
  WriteRegister(T2Control,0x20);          //Set Timer-2, T2Control_Reg:  Timer used for LFO trimming
  WriteRegister(T2ReloadHi,0x03);         //Set Timer-2 reload value (T2ReloadHi_Reg and T2ReloadLo_Reg)
  WriteRegister(T2ReloadLo,0xFF);         //
  WriteRegister(T3Control,0x00);          // Not started automatically. Not reloaded. Input clock 13.56 MHz
  //WriteRegister(FIFOControl,0x90);        //Set FifoControl_Reg, Fifo size=255 bytes. Flush FIFO
  WriteRegister(FIFOControl,0x10);

  WriteRegister(WaterLevel,0xFE);        //Set WaterLevel =(FIFO length -1),cause fifo length has been set to 255=0xff,so water level is oxfe
  WriteRegister(RxBitCtrl,0x80);         //RxBitCtrl_Reg(0x0c)	Received bit after collision are replaced with 1.
  WriteRegister(DrvMod,0x80);            //DrvMod reg(0x28), Tx2Inv=1,Inverts transmitter 1 at TX1 pin
  WriteRegister(TxAmp,0x00);             // TxAmp_Reg(0x29),output amplitude  0: TVDD -100 mV(maxmum)
  WriteRegister(DrvCon,0x01);            // TxCon register (address 2Ah),TxEnvelope
  WriteRegister(Txl,0x05);               //
  WriteRegister(RxSofD,0x00);            //
  WriteRegister(Rcv,0x12);               //
//> =============================================
//>  LoadProtocol( bTxProtocol=0, bRxProtocol=0)
//> =============================================
  WriteRegister(Command,Idle);           // Terminate any running command.
  WriteRegister(FIFOControl,0xB0);       // Flush_FiFo,low alert
  //WriteRegister(FIFOControl,0x30);


  WriteRegister(IRQ0,0x7F);             // Clear all IRQ 0,1 flags
  WriteRegister(IRQ1,0x7F);             //
  //> Write in Fifo: Tx and Rx protocol numbers(0,0)
  WriteRegister(FIFOData,0x00);         //
  WriteRegister(FIFOData,0x00);         //
  ////allows the Idle interrupt request means the command is finished and set gobal irq
  WriteRegister(IRQ0En,0x10);          // Idle interrupt(Command terminated), RC663_BIT_IDLEIRQ=0x10
  WriteRegister(IRQ1En,0x40);         //bit 6Set to logic 1, it allows the global interrupt request (indicated by the bit
                                      //GlobalIrq) to be propagated to the interrupt pin
  WriteRegister(Command,LoadProtocol);    // Start RC663 command "Load Protocol"=0x0d
  while(!(ReadRegister2(IRQ1)&0x40)); //Wait untill global interrupt set,meanning the command has done execution

  WriteRegister(IRQ0,0x00);              //Disable Irq 0,1 sources
  WriteRegister(IRQ1,0x00);              //


  WriteRegister(FIFOControl,0xB0);       // Flush_FiFo
  //WriteRegister(FIFOControl,0x30);

// Apply RegisterSet
//
//> Configure CRC-16 calculation, preset value(0x6363) for Tx&Rx

  WriteRegister(TxCrcPreset,0x18);           //means preset value is 6363,and uses CRC 16,but CRC is not automaticlly apended to the data
  WriteRegister(RxCrcPreset,0x18);           //


  WriteRegister(TxDataNum,0x08);             //
  WriteRegister(TxModWidth,0x20);            // Length of the pulse modulation in carrier clks+1
  WriteRegister(TxSym10BurstLen,0x00);       // Symbol 1 and 0 burst lengths = 8 bits.
  WriteRegister(FrameCon,0xCF);             // Start symbol=Symbol2, Stop symbol=Symbol3
  WriteRegister(RxCtrl,0x04);               // Set Rx Baudrate 106 kBaud

  WriteRegister(RxThreshold,0x32);          // Set min-levels for Rx and phase shift
  WriteRegister(RxAna,0x00);
  WriteRegister(RxWait,0x90);             // Set Rx waiting time
  WriteRegister(TXWaitCtrl,0xC0);
  WriteRegister(TxWaitLo,0x0B);
  WriteRegister(T0ReloadHi,0x08);         // Set Timeout. Write T0,T1 reload values(hi,Low)
  WriteRegister(T0ReloadLo,0xD8);
  WriteRegister(T1ReloadHi,0x00);
  WriteRegister(T1ReloadLo,0x00);

  WriteRegister(DrvMod,0x81);               // Write DrvMod register

  //> MIFARE Crypto1 state is further disabled.
  WriteRegister(Status,0x00);

  Antenna_Switch(ON);

  TaskComps[8].Run = 1; //启动cardtest任务
  TaskComps[8].Timer= 20;
  TaskComps[8].ItvTime = 20;
}

















//
uint8 RequestA(char* atqa)
{
  atqa[0]=0;
  atqa[1]=0;
  //> =============================================
//>  I14443p3a_Sw_RequestA
//> =============================================
  WriteRegister(TXWaitCtrl,0xC0);       //  TxWaitStart at the end of Rx data
  WriteRegister(TxWaitLo,0x0B);         // Set min.time between Rx and Tx or between two Tx
  //tempErrorCode=ReadRegister(Error);
  WriteRegister(T0ReloadHi,0x08);       //> Set timeout for this command cmd. Init reload values for timers-0,1
  WriteRegister(T0ReloadLo,0x94);
  WriteRegister(T1ReloadHi,0x00);
  WriteRegister(T1ReloadLo,0x00);
  WriteRegister(IRQ0,0x08);  //why ? ,传送完成中断，TxIrq Set, when data transmission is completed, which is immediately after the last bit is sent. Can only be reset if Set is cleared.
  WriteRegister(RxWait,0x90);    // bit9,If set to 1, the RxWait time is RxWait x(0.5/DBFreq).  bit0--bit6,Defines the time after sending, where every input is ignored
  WriteRegister(TxDataNum,0x0F);
//> ---------------------
//> Send the ReqA command
//> ---------------------
  WriteRegister(Command,Idle);           // Terminate any running command.
  WriteRegister(FIFOControl,0xB0);       // Flush_FiFo
  //WriteRegister(FIFOControl,0x30);


  WriteRegister(IRQ0,0x7F);           // Clear all IRQ 0,1 flags
  WriteRegister(IRQ1,0x7F);           //

  //WriteRegister(IRQ0En,0x84);//打开接收中断
  //WriteRegister(IRQ0En,0x84);//打开接收中断
  //printf("En_%x\r\n",ReadRegister(IRQ0En));
  DelayMs(10);
  WriteRegister(FIFOData,0x26);       //Write ReqA=26(wake up all the idle card ,not sleeping，0x52) into FIFO
  WriteRegister(Command,Transceive);        // Start RC663 command "Transcieve"=0x07. Activate Rx after Tx finishes.
#ifdef WAIT
	if(WaitRxEnd() == RX_ERROR)
	return RX_ERROR;
#else

  WriteRegister(IRQ0En,0x18);         // Wait until the command is finished. Enable IRQ sources.
  WriteRegister(IRQ1En,0x42);         //
  while(!(ReadRegister2(IRQ1)&0x40)); //Wait untill global interrupt set
  WriteRegister(IRQ0En,0x00);        //
  WriteRegister(IRQ1En,0x00);        //
#endif
  iterator=0;
  //printf("out\r\n");
 // DelayMs(20);

  iterator=ReadRegister2(FIFOLength); //read FIFO length
  if(iterator == 0)
  {
  	//printf("no data\r\n");
	return RX_END;
  }
  for(uint16 i=0;i<iterator;i++)
  atqa[i]=ReadRegister2(FIFOData);   // Read FIFO, expecting: the two byte ATQA,we could tell from ATQA how long the UID is ,4、7、10 byte?,then decide wether to go through  the subsquent anti-collision procedure
  //ReadRegister(Error);             //see if error occured
  return RX_END;
}






//
uint8 TypeA_Card_Select(char* uniqueID)
{
       // Get UID, Apply cascade level-1
  WriteRegister(TxDataNum,0x08);       //BIT3 If cleared - it is possible to send a single symbol pattern.If set - data is sent.
  WriteRegister(RxBitCtrl,0x00);       //
  WriteRegister(Command,Idle);           // Terminate any running command.
  WriteRegister(FIFOControl,0xB0);       // Flush_FiFo
  //WriteRegister(FIFOControl,0x30);

  WriteRegister(IRQ0,0x7F);              // Clear all IRQ 0,1 flags
  WriteRegister(IRQ1,0x7F);             //
  WriteRegister(FIFOData,0x93);                 //Write "Select" cmd into FIFO (SEL=93, NVB=20,cascade level-1)
  WriteRegister(FIFOData,0x20);                 //字节计数=2
  WriteRegister(Command,Transceive);          //Start tranceive command
#ifdef WAIT
	WaitRxEnd();
#else

  WriteRegister(IRQ0En,0x18);           //Enable Irqs 0,1
  WriteRegister(IRQ1En,0x42);           //Enable the global IRQ to be propagated to the IRQ pin
  while(!(ReadRegister(IRQ1)&0x40));    // Wait until the command is finished
  WriteRegister(IRQ0En,0x00);                    //Disable IRQ0 interrupt sources
  WriteRegister(IRQ0En,0x00);
  ReadRegister(IRQ0);                    // Read IRQ 0,1 Status register
  ReadRegister(IRQ1);       //
#endif

  iterator=0;
  iterator=ReadRegister(FIFOLength);//read FIFO length
  for(uint16 i=0;i<iterator;i++)
  uniqueID[i]=ReadRegister(FIFOData);  // Read FIFO,Expected - Complete UID (one PICC in HF)


  //now we got UID ,we continue to use this UID to select the card
  //this command needs CRC appended
  WriteRegister(TxCrcPreset,0x19);          //preset value is6363,use crc16,crc is apended to the data stream
  WriteRegister(RxCrcPreset,0x19);          //

  WriteRegister(Command,Idle);           // Terminate any running command.
  WriteRegister(FIFOControl,0xB0);       // Flush_FiFo
  //WriteRegister(FIFOControl,0x30);

  WriteRegister(IRQ0,0x7F);              // Clear all IRQ 0,1 flags
  WriteRegister(IRQ1,0x7F);             //
  WriteRegister(FIFOData,0x93);          //select
  WriteRegister(FIFOData,0x70);          //字节计数=7
  for(uint16 i=0;i<iterator;i++)
          WriteRegister(FIFOData,uniqueID[i]);  // Read FIFO,Expected - Complete UID (one PICC in HF)
  WriteRegister(Command,Transceive);          //Start tranceive command ,expecting to receive SAK ,select acknowlegement
#ifdef WAIT
	WaitRxEnd();
#else

  WriteRegister(IRQ0En,0x18);           //Enable Irqs 0,1
  WriteRegister(IRQ1En,0x42);           //Enable the global IRQ to be propagated to the IRQ pin
  WriteRegister(IRQ1,0x00);
  while(!(ReadRegister(IRQ1)&0x40));    // Wait until the command is finished
#endif
  SAK=ReadRegister(FIFOData);  // Read FIFO,Expecting SAK,here wo should next level of anti-collision
                                      //if SAK's bit2=1,then UID is not finished yet
  //结束防冲突环.Here we assuming the UID is 4 bytes ,so just finish the anti-collision loop
  //next we send the RATS,after RATS ,we can continue to send APDU

  WriteRegister(TxCrcPreset,0x19);          //preset value is6363,use crc16,crc is apended to the data stream
  WriteRegister(RxCrcPreset,0x19);          //
  WriteRegister(Command,Idle);           // Terminate any running command.
  WriteRegister(FIFOControl,0xB0);       // Flush_FiFo
  //WriteRegister(FIFOControl,0x30);
  WriteRegister(IRQ0,0x7F);              // Clear all IRQ 0,1 flags
  WriteRegister(IRQ1,0x7F);             //
  WriteRegister(FIFOData,0xE0);          //RATS,E0为命令，
  WriteRegister(FIFOData,0x80);          //5为FSDI(5对应于64,8对应于256)， 1为CID（card ID），为此卡在此次通信中的临时ID
  WriteRegister(Command,Transceive);          //Start tranceive command ,expecting to receive SAK ,select acknowlegement
#ifdef WAIT
  WaitRxEnd();
#else
  WriteRegister(IRQ0En,0x18);           //Enable Irqs 0,1
  WriteRegister(IRQ1En,0x42);           //Enable the global IRQ to be propagated to the IRQ pin
  while(!(ReadRegister(IRQ1)&0x40));    // Wait until the command is finished
  DelayMs(5);
#endif
  iterator=0;
  iterator=ReadRegister(FIFOLength);//read FIFO length
  for(uint16 i=0;i<iterator;i++)
      ATS[i]=ReadRegister(FIFOData);  // Read FIFO,Expecting ATS,after receiving the ATS the PICC enters protocol state,ready to process APDUs
                                      //
  return tempErrorCode= ReadRegister(Error);
}

#define PHHAL_HW_RC663_REG_IRQ1                 0x07
#define PHHAL_HW_RC663_REG_IRQ0EN               0x08
#define PHHAL_HW_RC663_REG_IRQ1EN               0x09

#define PHHAL_HW_RC663_BIT_TXIRQ                0x08
#define PHHAL_HW_RC663_BIT_RXIRQ                0x04


#define PHHAL_HW_RC663_BIT_IRQPUSHPULL          0x80
#define PHHAL_HW_RC663_BIT_GLOBALIRQ            0x40

#define PHHAL_HW_RC663_BIT_IRQINV               0x80
#define PHHAL_HW_RC663_BIT_IDLEIRQ              0x10

#define PHHAL_HW_RC663_BIT_TIMER1IRQ            0x02
#define PHHAL_HW_RC663_BIT_HIALERTIRQ           0x40
#define PHHAL_HW_RC663_BIT_EMDIRQ               0x01
#define PHHAL_HW_RC663_REG_IRQ0                 0x06




void phhalHw_Rc663_WaitIrq(
								  uint8 bIrq0WaitFor,
								  uint8 bIrq1WaitFor,
								  uint8  pIrq0Reg,
								  uint8  pIrq1Reg
								  )
 {
	 uint8 bIrq0EnReg;
	 uint8 bIrq1EnReg;
	 uint8 bRegister;


	 /* Enable Irqs if requested */
		 /* Read IRQEn registers */
		 bIrq0EnReg = ReadRegister(0x08);//PHHAL_HW_RC663_REG_IRQ0EN
		 bIrq1EnReg = ReadRegister(0x09);//PHHAL_HW_RC663_REG_IRQ1EN
		 //PH_CHECK_SUCCESS_FCT(statusTmp, phhalHw_ReadRegister(pDataParams, PHHAL_HW_RC663_REG_IRQ1EN, &bIrq1EnReg));

		 /* Enable IRQ0 interrupt sources */
		 //bIrq0EnReg &= PHHAL_HW_RC663_BIT_IRQINV;
		 bIrq0EnReg &= 0x80;

		 /* Set wait IRQs */
    	//bIrq0WaitFor = PHHAL_HW_RC663_BIT_TXIRQ | PHHAL_HW_RC663_BIT_IDLEIRQ;
    	//bIrq1WaitFor = PHHAL_HW_RC663_BIT_TIMER1IRQ;

		//bIrq0WaitFor = 0x08|0x10;
		//bIrq1WaitFor = 0x02;

		 bIrq0EnReg |= bIrq0WaitFor;
		 WriteRegister(PHHAL_HW_RC663_REG_IRQ0EN, bIrq0EnReg);

		 /* Enable IRQ1 interrupt sources */
		 bIrq1EnReg &= PHHAL_HW_RC663_BIT_IRQPUSHPULL;
		 bIrq1EnReg |= PHHAL_HW_RC663_BIT_GLOBALIRQ | bIrq1WaitFor;
		 WriteRegister( PHHAL_HW_RC663_REG_IRQ1EN, bIrq1EnReg);


	 /* wait until an IRQ occurs */
	 do
	 {

			 bRegister = ReadRegister (PHHAL_HW_RC663_REG_IRQ1);
			 printf("cycle\r\n");

	 }
	 while (!(bRegister & PHHAL_HW_RC663_BIT_GLOBALIRQ));



	 /* Clear IRQ0 interrupt sources */
	 bIrq0EnReg &= PHHAL_HW_RC663_BIT_IRQINV;
	 WriteRegister(PHHAL_HW_RC663_REG_IRQ0EN, bIrq0EnReg);

	 /* Clear IRQ1 interrupt sources */
	 bIrq1EnReg &= PHHAL_HW_RC663_BIT_IRQPUSHPULL;
	 WriteRegister(PHHAL_HW_RC663_REG_IRQ1EN, bIrq1EnReg);
#if 0
	 /* return IRQ1 status */
	 if (pIrq1Reg)
	 {
		 *pIrq1Reg = bRegister;
	 }

	 /* return Irq0 status */
	 if (pIrq0Reg)
	 {
		 PH_CHECK_SUCCESS_FCT(statusTmp, phhalHw_ReadRegister(pDataParams, PHHAL_HW_RC663_REG_IRQ0, pIrq0Reg));
	 }

	 return PH_ADD_COMPCODE(PH_ERR_SUCCESS, PH_COMP_HAL);
#endif
 }








 //APDUs needs CRC16 appended
//
uint8 TranceiveAPDU(uint8 *send_Buffer,uint8 send_Length,uint8 *receive_Buffer,uint16 *receive_Length)
{


  WriteRegister(Command,Idle);           // Terminate any running command.
  WriteRegister(FIFOControl,0xB0);       // Flush_FiFo
  //WriteRegister(FIFOControl,0x30);
  WriteRegister(IRQ0,0x7F);           // Clear all IRQ 0,1 flags
  WriteRegister(IRQ1,0x7F);           //
  WriteRegister(FIFOData,I_block_PCB);  //set I_block header
   //WriteRegister(FIFOData,0x01); //set CID
  for(iterator=0;iterator<send_Length;iterator++)
    WriteRegister(FIFOData,send_Buffer[iterator]);  //attention!!!,if use 512 byte fifo this needs changing
  WriteRegister(Command,Transceive);        // Start RC663 command "Transcieve"=0x07. Activate Rx after Tx finishes.
#ifdef WAIT
	WaitRxEnd();
#else
  WriteRegister(IRQ0En,0x18);         // Wait until the command is finished. Enable IRQ sources.
  WriteRegister(IRQ1En,0x42);         //
  while(!(ReadRegister(IRQ1)&0x40)); //Wait untill global interrupt set
  WriteRegister(IRQ1En,0x00);        //
  DelayMs(65);
#endif


  *receive_Length=ReadRegister(FIFOLength);
  //ReadRegister(FIFOData);
  for(iterator=0;iterator<*receive_Length;iterator++)
          receive_Buffer[iterator]=ReadRegister(FIFOData);   // Read FIFO, expecting: the two byte ATQA,we could tell from ATQA how long the UID is ,4、7、10 byte?,then decide wether to go through  the subsquent anti-collision procedure
 I_block_PCB=((I_block_PCB&0x01)?0x02:0x03);//change I block number
  return tempErrorCode=ReadRegister(Error); //see if error occured





}

 uint8 TranceiveAPDU2(uint8 *send_Buffer,uint8 send_Length,uint8 *receive_Buffer,uint16 *receive_Length)
{


  WriteRegister(Command,Idle);           // Terminate any running command.
  WriteRegister(FIFOControl,0xB0);       // Flush_FiFo
  //WriteRegister(FIFOControl,0x30);
  WriteRegister(IRQ0,0x7F);           // Clear all IRQ 0,1 flags
  WriteRegister(IRQ1,0x7F);           //
   WriteRegister(FIFOData,I_block_PCB);  //set I_block header
   //WriteRegister(FIFOData,0x01); //set CID
  for(iterator=0;iterator<send_Length;iterator++)
    WriteRegister(FIFOData,send_Buffer[iterator]);  //attention!!!,if use 512 byte fifo this needs changing
  WriteRegister(Command,Transceive);        // Start RC663 command "Transcieve"=0x07. Activate Rx after Tx finishes.
  WriteRegister(IRQ0En,0x18);         // Wait until the command is finished. Enable IRQ sources.
  WriteRegister(IRQ1En,0x42);         //
  while(!(ReadRegister(IRQ1)&0x40)); //Wait untill global interrupt set
  WriteRegister(IRQ1En,0x00);        //
  //DelayMs(80);
  DelayMs(580);
  *receive_Length=ReadRegister(FIFOLength);
  //ReadRegister(FIFOData);
  for(iterator=0;iterator<*receive_Length;iterator++)
          receive_Buffer[iterator]=ReadRegister(FIFOData);   // Read FIFO, expecting: the two byte ATQA,we could tell from ATQA how long the UID is ,4、7、10 byte?,then decide wether to go through  the subsquent anti-collision procedure
 I_block_PCB=((I_block_PCB&0x01)?0x02:0x03);//change I block number
  return tempErrorCode=ReadRegister(Error); //see if error occured
}
//传送get process option的等待状态
uint8 TranceiveWaitPO(uint8 *send_Buffer,uint8 send_Length,uint8 *receive_Buffer,uint16 *receive_Length)
{


  WriteRegister(Command,Idle);           // Terminate any running command.
  WriteRegister(FIFOControl,0xB0);       // Flush_FiFo
  //WriteRegister(FIFOControl,0x30);


  WriteRegister(IRQ0,0x7F);           // Clear all IRQ 0,1 flags
  WriteRegister(IRQ1,0x7F);           //
  for(iterator=0;iterator<send_Length;iterator++)
    WriteRegister(FIFOData,send_Buffer[iterator]);  //attention!!!,if use 512 byte fifo this needs changing
  WriteRegister(Command,Transceive);        // Start RC663 command "Transcieve"=0x07. Activate Rx after Tx finishes.
  WriteRegister(IRQ0En,0x18);         // Wait until the command is finished. Enable IRQ sources.
  WriteRegister(IRQ1En,0x42);         //
  while(!(ReadRegister(IRQ1)&0x40)); //Wait untill global interrupt set
  WriteRegister(IRQ1En,0x00);        //
  DelayMs(50);
  //Delay_I_1us(50000);//motherfucker
    //Delay_I_1us(50000);//motherfucker
  *receive_Length=ReadRegister(FIFOLength);
  for(iterator=0;iterator<*receive_Length;iterator++)
          receive_Buffer[iterator]=ReadRegister(FIFOData);   // Read FIFO, expecting: the two byte ATQA,we could tell from ATQA how long the UID is ,4、7、10 byte?,then decide wether to go through  the subsquent anti-collision procedure
  return tempErrorCode=ReadRegister(Error); //see if error occured
}









//
char Send_Halt_A(){
//> =============================================
//> Send HaltA cmd
//> =============================================
  //> HaltA command needs CRC-16 appended to the data stream.

  WriteRegister(TxCrcPreset,0x19);          //preset value is6363,use crc16,crc is apended to the data stream
  WriteRegister(RxCrcPreset,0x19);          //

  WriteRegister(RxBitCtrl,0x00);
  WriteRegister(IRQ0En,0x00);    //Disable IRQ0 interrupt sources
  WriteRegister(IRQ0En,0x00);
  WriteRegister(T0ReloadHi,0x3E);       // Set T0,T1 reload values
  WriteRegister(T0ReloadLo,0x58);       //
  WriteRegister(T1ReloadHi,0x00);       //
  WriteRegister(T1ReloadLo,0x00);       //
  WriteRegister(Command,Idle);           // Terminate any running command.
  WriteRegister(FIFOControl,0xB0);       // Flush_FiFo
  WriteRegister(IRQ0,0x7F);       // Clear all IRQ 0,1 flags
  WriteRegister(IRQ1,0x7F);       //
  WriteRegister(FIFOData,0x50); //> Write command data into FIFO
  WriteRegister(FIFOData,0x00);
  WriteRegister(Command,Transceive);       // Start RC663 command "Transcieve"=0x07. Activate Rx after Tx finishes.
  WriteRegister(IRQ0En,0x18);       // Wait until the command is finished. Enable IRQ sources.
  WriteRegister(IRQ1En,0x42);       //
  while(!(ReadRegister(IRQ1)&0x40));            //Wait untill global interrupt set
  WriteRegister(IRQ0En,0x00);                           //Disable IRQ0 interrupt sources
  WriteRegister(IRQ0En,0x00);
  WriteRegister(IRQ0,0x40);
  WriteRegister(IRQ0En,0x54);                     // Wait until the command is finished. Enable IRQ sources.
  WriteRegister(IRQ1En,0x42);                   //
  while(!(ReadRegister(IRQ1)&0x40));    //Wait untill global interrupt set.
  WriteRegister(IRQ0En,0x00);           //Disable IRQ0 interrupt sources
  WriteRegister(IRQ0En,0x00);
  WriteRegister(Command,Idle);           // Terminate any running command.
  WriteRegister(FIFOControl,0xB0);       // Flush_FiFo
  WriteRegister(TControl,0x03);                 // Stop timers
  WriteRegister(TxDataNum,0x08);                //  Enable bit3,DataEn ,If set - data is sent.
  return tempErrorCode=ReadRegister(Error); //see if error occured
}

//para :readermode ,is the mode you want the reader to work in
void ReaderInit(unsigned int readermode)
{
      Interface_Init();

      switch(readermode)
      {
          case ISO14443_A:ISO14443_A_Init();
                            break;
      }
}

volatile uint8 send_token=0,record_num=1;//只有持有令牌(token)的才能发送数据
uint8 commonbuffer[256]  = {0};
uint8 SELECT_PPSE[] = {0x00,0xA4,0x04,0x00,0x0E,0x32,0x50,0x41,0x59,0x2E,0x53,0x59,0x53,0x2E,0x44,0x44,0x46,0x30,0x31,0x00};
uint8 SELECT_APP1[]={  0x00,0xA4,0x04,0x00,0x08,0xA0,0x00,0x00,0x03,0x33,0x01,0x01,0x01,0x00};
uint8 SELECT_APP1_CPY[]={  0x00,0xA4,0x04,0x00,0x08,0xA0,0x00,0x00,0x03,0x33,0x01,0x01,0x01,0x00};

uint8 SELECT_APP1_LENGTH = 14;


uint8 SELECT_APP2[]={  0x00,0xA4,0x04,0x00,0x08,0xA0,0x00,0x00,0x03,0x33,0x01,0x01,0x02,0x00}
	,SELECT_APP3[]={ 0x00,0xA4,0x04,0x00,0x08,0xA0,0x00,0x00,0x03,0x33,0x01,0x01,0x03,0x00}
	//,GET_PROCESS_OPTION[]={0x80,0xA8,0x00,0x00,0x23,0x83,0x21,0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x56,0x00,0x00,0x00,0x00,0x00,0x01,0x56,0x13,0x09,0x25,0x00,0x00,0x00,0x00,0x00,0x00}

	,GET_PROCESS_OPTION[]={0x80,0xA8,0x00,0x00,0x23,0x83,0x21,0xA6,0x20,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x50,0x00,0x00,0x00,0x00,0x00,0x09,0x78,0x15,0x04,0x02,0x00,0xD7,0x58,0x73,0x14,0x00}

	,GET_RECORD_FORMAT[]={0x0A, 0x01,0x80,0xCA,0x9F,0x4F,0x00}
	//,READ_RECORD_1[]={0x00,0xB2,0x00,0x5C,0x00}
	,READ_RECORD_1[]={0x00,0xB2,0x00,0x5C,0x2d}
	,READ_RECORD_2[]={0x00,0xB2,0x01,0x0C,0x00}
	,READ_RECORD_3[]={0x00,0xB2,0x01,0x14,0x00}
	,READ_RECORD_4[]={0x00,0xB2,0x04,0x1c,0x00}
	,READ_RECORD_5[]={0x00,0xB2,0x01,0x0c,0x00}
	,WAIT_PROCESS_OPTION[]={0xF2,0x01};

uint8 READ_RECORD_test[]={0x00,0xB2,0x00,0x5C,0x00};
uint8 flag = 0 ;
extern uint8 groupID[];
extern uint8 cardID[30];
void ClearSameUID(void)
{
	memset(cardID,0,sizeof(cardID));
	//flag = 0;
}
uint8 aidTag[5] = {0xa0,0,0,3,0x33};

uint8 tarck2tag[2] = {0x57,0};
uint8 Idtag[3]={0x9f ,0x61,0};
uint8 Nametag1[3]={0x9f ,0x0b,0};
uint8 Nametag2[3]={0x5f ,0x20,0};
uint8 fromDateTag[3] = {0x5F,0x25,0};
uint8 endDateTag[3] = {0x5f,0x24,0};
uint8 endDateTag2[2] = {0x5f,0x26};//失效日期，修改了TAG



uint8 items = 0;

uint8 Fivedata[120] = {0};
uint8 *curPos  = Fivedata+1;

uint8 transmitCount = 0;



void CardTest(void)
{
	//return;
    if( RequestA(ATQA) == RX_ERROR)
	{
		HardwareReset();
		//printf("reset\r\n");
		TaskComps[8].Run = 0;//结束本函数
  		TaskComps[8].Timer= 0;
  		TaskComps[8].ItvTime = 0;

		TaskComps[7].Run = 1;//重新开始
		return;
	};
    if(ATQA[0]==0 &&ATQA[1]==0)//无卡
	{
      send_token=0;
	  //printf("NO CARD\r\n");
	  ClearSameUID();
      return;
    }
	else//有卡
	{
		//return;
		TaskComps[1].Run = 1;//启动读卡流程 GenericApp_ReadCardMessage
		//printf("YES CARD\r\n");
		//结束本函数
		TaskComps[8].Run = 0;
  		TaskComps[8].Timer= 0;
  		TaskComps[8].ItvTime = 0;
	}
}

void GenericApp_ReadCardMessage( void )
{
	uint16 receive_Length=0;
	uint8 *pos;

	if(send_token== 0)
	{
		//printf("0\r\n");
		TypeA_Card_Select(UID);
		send_token=1;//继续执行下一条命令
	}

	if(send_token==1)
	{
		//printf("1\r\n");
		TranceiveAPDU(SELECT_PPSE,sizeof(SELECT_PPSE),commonbuffer,&receive_Length);
		if((receive_Length!=0)&&(commonbuffer[receive_Length-1]==0 &&commonbuffer[receive_Length-2]==0x90))
		{
			pos = ArrayArray(commonbuffer, receive_Length,aidTag,3);
			if(pos != NULL)
			{
				memcpy(&SELECT_APP1[5],(pos),8);
				SELECT_APP1[13] = 0;
				SELECT_APP1_LENGTH = 14;
		#if 0
			if(*(pos+1) == 7)
			{
				memcpy(&SELECT_APP1[5],(pos+2),7);
				SELECT_APP1[12] = 0;
				SELECT_APP1_LENGTH = 13;
			}
			else if(*(pos+1) == 8)
			{
				memcpy(&SELECT_APP1[5],(pos+2),8);
				SELECT_APP1[13] = 0;
				SELECT_APP1_LENGTH = 14;
			}
		#endif
			}
			else
			{
				memcpy(SELECT_APP1,SELECT_APP1_CPY,sizeof(SELECT_APP1_CPY));
				SELECT_APP1_LENGTH = 14;

			}
		}
		else
		{
			//printf("end\r\n");
			goto end;
		}
		memset(commonbuffer,0, receive_Length);
		send_token=2;
  	}

	if(send_token== 2)
	{
		//printf("2\r\n");
		TranceiveAPDU(SELECT_APP1,SELECT_APP1_LENGTH,commonbuffer,&receive_Length);
		if((receive_Length!=0)&&(commonbuffer[receive_Length-1]==0&&commonbuffer[receive_Length-2]==0x90))
		{

		}
		 else
		{
			goto end;
		}
		memset(commonbuffer,0, receive_Length);

		//CleanBuff(dataBuffer);
	    send_token=4;
    }

	if(send_token== 4)
	{
		//printf("4\r\n");
		record_num = 1;
		for(;record_num<=10;record_num++)
		{
			READ_RECORD_2[2]=record_num;
			TranceiveAPDU(READ_RECORD_2,sizeof(READ_RECORD_2),commonbuffer,&receive_Length);
			if((receive_Length!=0)&&(commonbuffer[receive_Length-1]==0 &&commonbuffer[receive_Length-2]==0x90))
			{
				TLVdecode(commonbuffer+1,receive_Length - 3);
				if(DataCollectComplete() == 1)
				{
					goto collectted;
				}
			}
			else
			{
				//printf("No_NUM_%d\r\n",record_num);
				memset(commonbuffer,0, receive_Length);
				break;
			}
		}
		send_token= 3;
	}

	if(send_token== 3)
	{
		//printf("3.0\r\n");
		record_num = 1;
		for(;record_num<=10;)
		{
			READ_RECORD_3[2]=record_num;
			TranceiveAPDU(READ_RECORD_3,sizeof(READ_RECORD_3),commonbuffer,&receive_Length);
			if((receive_Length!=0)&&(commonbuffer[receive_Length-1]==0 &&commonbuffer[receive_Length-2]==0x90))
			{
				//printf("Yes_NUM_%d\r\n",record_num);
				TLVdecode(commonbuffer+1,receive_Length - 3);
				if(DataCollectComplete() == 1)
				{
					goto collectted;
				}
				//printf("can\r\n");

			}
			else
		    {
				//printf("No_NUM_%d\r\n",record_num);
				memset(commonbuffer,0, receive_Length);
				break;
		    }

			memset(commonbuffer,0, receive_Length);
			record_num++;
		}
		//把数据拷贝到待发送数组中
		//memcpy(dataBuffer[0],Fivedata,Fivedata[0]+1);
		record_num = 1;
		send_token= 5;

    }

	if(send_token== 5)
	{
		record_num = 1;
		for(;record_num<=10;)
		{
			READ_RECORD_4[2]=record_num;
			TranceiveAPDU(READ_RECORD_4,sizeof(READ_RECORD_4),commonbuffer,&receive_Length);
			//TranceiveAPDU(READ_RECORD_4,sizeof(READ_RECORD_3),commonbuffer,&receive_Length);
			if((receive_Length!=0)&&(commonbuffer[receive_Length-1]==0 &&commonbuffer[receive_Length-2]==0x90))
			{
				//printf("Yes_NUM_%d\r\n",record_num);
				TLVdecode(commonbuffer+1,receive_Length - 3);
				if(DataCollectComplete() == 1)
				{
					goto collectted;
				}
			}
			else
		    {
				//printf("No_NUM_%d\r\n",record_num);
				memset(commonbuffer,0, receive_Length);
				break;
		    }

			memset(commonbuffer,0, receive_Length);
			record_num++;
		}
		//把数据拷贝到待发送数组中
		memcpy(dataBuffer[0],Fivedata,Fivedata[0]+1);
		record_num = 1;
		send_token= 6;
	}
collectted:
	if(send_token== 6)
	{
		//printf("6.0\r\n");
		record_num = 1;
		for(;record_num<=10;)
		{
			READ_RECORD_1[2]=record_num;
			TranceiveAPDU(READ_RECORD_1,sizeof(READ_RECORD_1),commonbuffer,&receive_Length);
			if((receive_Length!=0)&&(commonbuffer[receive_Length-1]==0 &&commonbuffer[receive_Length-2]==0x90))
			{
				//把数据拷贝到待发送数组中
				dataBuffer[record_num][0] =       receive_Length - 3;
				memcpy(dataBuffer[record_num]+1,commonbuffer+1,receive_Length -3);
		    }
		   else
		   {
		   		CleanBuff2(dataBuffer,record_num);
				record_num = 1;
				memset(commonbuffer,0, receive_Length);
				break;
		   }
			   memset(commonbuffer,0, receive_Length);
			   record_num++;
		}

		//TaskComps[5].Run = 1;//打印读到的数据
		//printf("ItemAmount__%d\r\n",ItemAmount(dataBuffer));
		items = ItemAmount(dataBuffer);

		IfSend();
		record_num = 1;
		send_token= 0;
	}

end:
	memset(Fivedata,0,sizeof(Fivedata));

	memset(commonbuffer,0, sizeof(commonbuffer));
	//SendOverFlag();
	send_token= 0;//数据输出完毕，再次初始化
	record_num= 1;
	curPos  = Fivedata+1;

	Idtag[2] = 0;
	Nametag1[2] = 0;
	Nametag2[2] = 0;
    fromDateTag[2] = 0;
	endDateTag[2] = 0;
	tarck2tag[1] = 0;

	//Send_Halt_A();
    Antenna_Switch(OFF);              // Write DrvMod register
    IC_REST = 0;
   	DelayMs(2);
    IC_REST = 1;

	TaskComps[7].Timer = 200; //初始化协议
}

void ReceiveData(uint8 *head)
{
	if(head[1] == groupID[0] && head[2] == groupID[1])
	{
		if((head[4]&0x7f) == 0)
		{
          	memcpy(dataBuffer[head[3]-1],head,head[0]-2 +1);
			dataBuffer[head[3]-1][0] -=   2;
		}
		else if((head[4]&0x7f)  == 1)
		{
			memcpy(dataBuffer[head[3]-1],head,head[0]-2 +1);
			dataBuffer[head[3]-1][0] -=   2;
		}
		else if((head[4]&0x7f)  == 2)
		{
			memcpy(&dataBuffer[head[3]-1][125],&head[5],head[0]-2 +1);
			dataBuffer[head[3]-1][0] +=   (head[0] - 6);
		}
	}

}
uint8 cardID[30] = {0};
uint8 itemAmount = 0;

void IfSend (void)
{
	if(ReturnConnectStatus() != CONNECTED)
	{
		printf("discon\r\n");
		return;
	}
	//printf("con\r\n");
	uint8 *tmp = NULL;
	uint8 tmpItemAmount = 0;

	tmp = ArrayArray(dataBuffer[0],dataBuffer[0][0],tarck2tag,1);
	if(tmp != NULL)
	{	//相同卡号
		if(memcmp(cardID,tmp,*(tmp+1)+2) == 0)
		{
			tmpItemAmount = DataUnitsAmount();
			if(tmpItemAmount > itemAmount)
			{
				TaskComps[6].Run = 1;//发送数据
				TaskComps[6].Timer = 100;//发送数据
				TaskComps[6].ItvTime = 100;
				itemAmount = tmpItemAmount;
				//printf("SEND SAME\r\n");
			}
			//printf("NO SEND\r\n");
			CleanBuff(dataBuffer);
			//printf("UNIT%d\r\n",tmpItemAmount);
#if 0
				printf("same card\r\n");

				for(uint8 j = 0;j <=20;j++)
				{
					printf("%x,",cardID[j]);
				}
				printf("\r\n");
#endif
		}
		else//具备不同卡号，处理
		{
			memcpy(cardID,tmp,*(tmp+1) + 2);
			TaskComps[6].Run = 1;//发送数据
			TaskComps[6].Timer = 100;//发送数据
			TaskComps[6].ItvTime = 100;
			itemAmount = DataUnitsAmount();
			//printf("SEND DIFF\r\n");
		}
	}


#if 0
	ArrayArray()
	for(uint8 i = 0; i <= 10; i++)
	{
		if(dataBuffer[i][0] != 0)
		{
			for(uint8 j = 0;j <=dataBuffer[i][0];j++)
			{
				printf("%x,",dataBuffer[i][j]);
			}
			printf("\r\n");
#ifdef DATA_HUB
			dataBuffer[i][0]   = 0;
#endif
		}
		else
		{
			break;
		}
	}
#endif

}
uint8 DataUnitsAmount(void)
{
	uint8 *tmp;
	uint8 tmpItemAmount = 0;
	//身份证号
	tmp = ArrayArray(dataBuffer[0], dataBuffer[0][0],Idtag,2);
	if(tmp != NULL)
	{
		tmpItemAmount++;
	}

	//名字
	tmp = ArrayArray(dataBuffer[0], dataBuffer[0][0],Nametag1,2);
	if(tmp != NULL)
	{
		tmpItemAmount++;
	}

	//失效日期
	tmp = ArrayArray(dataBuffer[0], dataBuffer[0][0],endDateTag2,2);
	if(tmp != NULL)
	{
		tmpItemAmount++;
	}

	tmp = ArrayArray(dataBuffer[0], dataBuffer[0][0],fromDateTag,2);
	if(tmp != NULL)
	{
		tmpItemAmount++;
	}

	for(uint8 i = 1; i <= 10; i++)
	{
		if(dataBuffer[i][0] != 0)
		{
			tmpItemAmount++;

		}
		else
		{
			break;
		}
	}

	return tmpItemAmount;

}
void PrtReceivedData(void)
{
//#ifdef DATA_HUB
	for(uint8 i = 0; i <= 10; i++)
	{
		if(dataBuffer[i][0] != 0)
		{
			for(uint8 j = 0;j<=dataBuffer[i][0];j++)
			//for(uint8 j = 0;j<=50;j++)
			{
				printf("%x,",dataBuffer[i][j]);
			}
			printf("\r\n");
#ifdef DATA_HUB
			dataBuffer[i][0]   = 0;
#endif
		}
		else
		{
			break;
		}
	}
	//for(i = 0 ; i)
//#endif

	printf("AT2\r\n");

}

void GetPackageUart(void)
{
	//for(uint8 seq = 0; dataBuffer[];)
	//uint8 flag = 0;
	if(dataBuffer[0][4] > 0x0b || dataBuffer[0][4] == 0)
	{
		//接收错误相应动作
		CleanBuff(dataBuffer);
		return;
	}
	else
	{
		for(uint8 seq = 0; seq < dataBuffer[0][4]; seq++)
		{
			if(dataBuffer[seq][3] != seq)
			{
				//flag = 1
				//接收错误相应动作
				CleanBuff(dataBuffer);

				return;
			}

		}
		SendConfirm();
		printf("AT+GET");
	}

}
#if 0
void TransmitBuff(void)
{
	static uint8 seq = 0;
	static uint8 gId = 0;
	if(dataBuffer[seq][0] ==0 || seq > 10)
	{
		//结束本函数
		{
			TaskComps[6].Run = 0;
			TaskComps[6].Timer = 0;
			TaskComps[6].ItvTime = 0;
		}
		seq = 0;
		CleanBuff(dataBuffer);
		return;
	}
	BuildHead(dataBuffer[seq],RfTxBuff,gId,seq,11);
	SendData(RfTxBuff);
	seq++;
}
#else

void TransmitBuff(void)
{
	static uint8 seq = 0;
	static uint8 gId = 0;



	TaskComps[7].Timer = 0;//不要初始化协议

	for(;dataBuffer[seq][0] !=0 && seq <= 10;)
	//for(;seq <= 10;)
	{
		#if 0
		if(dataBuffer[seq][0] ==0 || seq > 10)
		{
			//结束本函数
			{
				//TaskComps[6].Run = 0;
				//TaskComps[6].Timer = 0;
				//TaskComps[6].ItvTime = 0;
			}
			seq = 0;
			CleanBuff(dataBuffer);
			return;
		}
		#endif
		//printf("TRUE SEND2.1 %d\r\n",dataBuffer[seq][0]);
		BuildHead(dataBuffer[seq],RfTxBuff,gId,seq,items);
		SendData(RfTxBuff);
		//SendData(test);
		//printf("TRUE SEND1.1 \r\n");
		seq++;
	}
	seq = 0;
	if(gId++ == 100) gId = 0;
	//printf("gid %d\r\n",gId);
	if(transmitCount ++ == 3)
	{
		transmitCount  = 0;
		TaskComps[6].Run = 0;
		TaskComps[6].Timer = 0;
		TaskComps[6].ItvTime = 0;

		TaskComps[7].Timer = 100;//初始化协议
		CleanBuff(dataBuffer);

	}

}

#endif
uint8 ItemAmount(uint8 buffer[][128])
{
        uint8 count;
	for( count = 0; count <= 10; count++)
	{
		if(buffer[count][0] == 0)
		{
			break;
		}
	}
	return count;
}
void CleanBuff(uint8 buffer[][128])
{
	uint8 row;
	for( row = 0; row <= 10; row++)
	{
		memset(buffer[row],0,128);
	}

}

void CleanBuff2(uint8 buffer[][128],uint8 row)
{
	//uint8 row;
	for( ; row <= 10; row++)
	{
		memset(buffer[row],0,128);
	}

}

/*********************************************************************************************************
** 函数名称: ArrayArray
** 功能描述: 从一个数组中查找另一个数组
** 输　入: strSrc 		源数组指针
		   lengthSrc	源数组长度
		   str			查找的数组指针
		   length		查找的数组长度
** 输　出: 找到第一个数组在源数组位置指针
** 全局变量: 无
** 调用模块:
**
** 作　者: 袁舰
** 日　期: 2015年11月27日
**-------------------------------------------------------------------------------------------------------
** 修改人:
** 日　期:
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/

uint8 *ArrayArray(const uint8  * strSrc, uint8 lengthSrc,const uint8 *str,uint8 length)
{
    const uint8 *s = strSrc;
    const uint8 *t = str;
    // uint8 *s = ( uint8 *)strSrc;
    // uint8 *t = ( uint8 *)str;
		  uint8 lengthCpy = length;

    for (; lengthSrc > 0; ++ strSrc, --lengthSrc)
    {
       for (s = strSrc, t = str;length > 0 && *s == *t; ++s, ++t,--length)
            ;
        if (length == 0)
        {
            return (uint8 *) strSrc;
        }
		else
		{
			length = lengthCpy;
		}
    }
    return NULL;
}

#if 0
char *ArrayArray(const char *strSrc, uint8 lengthSrc,const char *str,uint8 length)
{
    const char *s = strSrc;
    const char *t = str;

    for (; lengthSrc > 0; ++ strSrc, --lengthSrc)
    {
       for (s = strSrc, t = str;length > 0 && *s == *t; ++s, ++t,--length)
            ;
        if (length == 0)
            return (char *) strSrc;
    }
    return NULL;
}
#endif
/*********************************************************************************************************
** 函数名称: ArrayArray
** 功能描述: 从一个数组中查找另一个数组
** 输　入: strSrc 		源数组指针
		   lengthSrc	源数组长度
		   str			查找的数组指针
		   length		查找的数组长度
** 输　出: 找到第一个数组在源数组位置指针
** 全局变量: 无
** 调用模块:
**
** 作　者: 袁舰
** 日　期: 2015年11月27日
**-------------------------------------------------------------------------------------------------------
** 修改人:
** 日　期:
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
uint8 *MagneticTrack2(uint8 *Src, uint8 length,uint8 *Dest,uint8 *currentPos)
{
	uint8 *tmp;
	//uint8 *currentPos;
	uint8 check;
	uint8 trackLength = 0;
	uint8 cardIDlength = 0;
	uint8 tmpLenght = 0;

	tmp = ArrayArray(Src, length,tarck2tag,1);
	if(tmp != NULL)
	{
		*(Dest+1) = 0x57;//磁道2的ID，现在表示卡号字段
			printf("CARD ID\r\n");
		//写入卡号
		while(trackLength < 15)
		{
			check = *(tmp+2+trackLength);
			if(((check & 0xf0)>>4) >= 0 && ((check & 0xf0)>>4) <= 9)
			{
				*(Dest + 3 + cardIDlength) = ((check & 0xf0)>>4);
				cardIDlength += 1;
			}
			else
			{
				break;
			}

			if((check & 0x0f) >= 0 && (check & 0x0f) <= 9)
			{
				*(Dest + 3 + cardIDlength) = (check & 0x0f);
				cardIDlength += 1;
			}
			else
			{
				break;
			}
			trackLength += 1;
		}
		*(Dest+2) = cardIDlength;
		//字段的总长度
		*Dest = *(Dest + 2) + *Dest + 2 +4;

		//失效日期字段
		*(Dest + 3 + cardIDlength) = 0x5f;
		*(Dest + 4 + cardIDlength) = 0x24;
#if 1
		if(cardIDlength % 2)
		{
			*(Dest + 5 + cardIDlength) = *(tmp+3+trackLength);
			*(Dest + 6 + cardIDlength) = *(tmp+4+trackLength);
			//printf("XXX\r\n");
		}
		else
		{
			//printf("YYY\r\n");
			*(Dest + 5 + cardIDlength) = ((*(tmp+2+trackLength) & 0x0f) << 4)|((*(tmp+3+trackLength) & 0xf0)>>4);
			*(Dest + 6 + cardIDlength) = ((*(tmp+3+trackLength) & 0x0f) << 4)|((*(tmp+4+trackLength) & 0xf0)>>4);
		}
#endif
		//cardIDlength += 4;
		currentPos = (Dest + 7 + cardIDlength);

	}
	//身份证号
	tmp = ArrayArray(Src, length,Idtag,2);
	if(tmp != NULL && *(tmp+3) != 0x20)
	{
		//printf("ID\r\n");
		*currentPos = 0x9f;
		*(currentPos+1) = 0x61;
		while(*(tmp+tmpLenght + 3) != 0x20 && *(tmp+tmpLenght + 3) != 0x0)
		{
			*(currentPos + 3 +tmpLenght) = *(tmp+tmpLenght + 3);
			tmpLenght++;
		}
		*(currentPos+2) = tmpLenght;
		*Dest = *(currentPos+2) + *Dest + 3;

		currentPos = currentPos + *(currentPos+2) + 3;
	}

	//名字
	tmp = ArrayArray(Src, length,Nametag1,2);
	if(tmp != NULL && *(tmp + 3) != 0x20)
	{
		//printf("NAME1\r\n");
		memcpy(currentPos,tmp,*(tmp+2) +3);
		currentPos = currentPos + *(tmp+2) +3;
		*Dest = *Dest + *(tmp+2) +3;
	}
	else
	{
		tmp = ArrayArray(Src, length,Nametag2,2);

		if(tmp != NULL && *(tmp + 3) != 0x20)
		{
			//printf("NAME2\r\n");
			memcpy(currentPos,tmp,*(tmp+2) +3);
			currentPos = currentPos + *(tmp+2) +3;
			*Dest = *Dest + *(tmp+2) +3;
		}
	}

	return  currentPos;
}

uint8 *FromDate(uint8 *Src, uint8 length,uint8 *Dest,uint8* curPos)
{
	uint8 *tmp = NULL;
	uint8 flag = 0;
	tmp = ArrayArray(Src, length,fromDateTag,2);
	if(tmp != NULL )
	{
		printf("fromTag\r\n");
		memcpy(curPos,tmp,*(tmp+2)+3);
		*Dest = *Dest + *(tmp+2)+3;
		curPos += *(tmp+2)+3;

		flag = 2;

	}
#if 1
	tmp = ArrayArray(Src, length,endDateTag,2);
	if(tmp != NULL)
	{
		printf("endTag\r\n");
		memcpy(curPos,tmp,*(tmp+2)+3);
		*Dest = *Dest + *(tmp+2)+3;
		*(curPos+1) = 0x26;//修改Tag

		curPos += *(tmp+2)+3;

		flag = 1;
	}
#endif
	if(flag == 1 || flag == 2)
	{
		return curPos;
	}
	return NULL;
}

uint8  WaitRxEnd(void)
{
	uint8 msCount = 0;
	uint8 Irq0Status = 0;
	//WriteRegister(IRQ0,0x7f);
	//WriteRegister(IRQ1,0x7f);

	//WriteRegister(IRQ0En,0x84);//打开接收中断
	//Irq0Status = ReadRegister(IRQ0);
	while( msCount < 65)
	{

		Irq0Status = ReadRegister(IRQ0);


		if(Irq0Status & 0x02)
		{
			WriteRegister(IRQ0,0x7f);
			WriteRegister(IRQ1,0x7f);
			return RX_ERROR;
		}

		if(Irq0Status & 0x04)
		{
			WriteRegister(IRQ0,0x7f);
			WriteRegister(IRQ1,0x7f);
			return RX_END;
		}
		DelayMs(1);
		//printf("IRQ0 %x\r\n",Irq0Status);
		msCount++;
	};
	//printf("count_%x\r\n",msCount);

	WriteRegister(IRQ0,0x7f);
	WriteRegister(IRQ1,0x7f);



	//if(msCount < 65)
	//{
		//WriteRegister(IRQ0En,0x00);
	//	return RX_END;
	//}
	//else
	//{
		//WriteRegister(IRQ0En,0x00);
		return RX_TIMEOUT;
	//}

}

void TLVdecode(uint8 * tlvPos,uint8 packageLength)
{
	uint8 currentIndex = 0;
	uint8 currentStatus = 'T';
	uint8 valueSize = 0;
	//printf("wholelength %x",packageLength);
	while(currentIndex < packageLength)
	{
		switch (currentStatus)
		{
			case 'T':
				if ( (*(tlvPos + currentIndex) & 0x20) != 0x20)//单一结构
				{

					  if ( (*(tlvPos + currentIndex) & 0x1f) == 0x1f)//tag两字节
					  {

							 //tagIndex++;

							 //printf("current_1vlaue %x %x\r\n",*(tlvPos + currentIndex),*(tlvPos + currentIndex + 1));
                            DoubleBytesTagProcess(tlvPos + currentIndex,&curPos);
							currentIndex += 2;//指针位置加2
							//printf("single two tag \r\n");

							 //解析length域

							   //解析value域

					  }
					 else//tag单字节

					  {

							//printf("single one tag \r\n");
							// printf("current_2vlaue %x\r\n",*(tlvPos + currentIndex));
							SingleByteTagProcess(tlvPos + currentIndex,&curPos);
							 //DoubleBytesTagProcess(uint8* curTagPos,curPos);
							currentIndex++;//指针位置加1
						  //解析length域

							//解析value域

					  }
					 currentStatus = 'L';

				}
				else//复合结构
				{

						//printf("constructed tag \r\n");

					  if ( (*(tlvPos + currentIndex) & 0x1f) == 0x1f)//tag两字节
					  {

							 //tagIndex++;
							currentIndex += 2;//指针位置加2
							//printf("constructed two tag \r\n");


							 //解析length域

							   //解析value域

					  }
					 else//tag单字节
					 {

							//printf("constructed one tag \r\n");
							currentIndex++;//指针位置加1
						  //解析length域

							//解析value域

					  }


					  	if ( (*(tlvPos + currentIndex) & 0x80) != 0x80)//b6~b0的值就是value域的长度
						{
							//valueSize = *(tlvPos + currentIndex);
							currentIndex ++;

						}
						else//b6~b0的值指示后面有几个字节表示value长度
						{
							uint8 lengthSize = 0;
							lengthSize = *(tlvPos + currentIndex) & 0x7f;
							#if 0
							for (uint8 index = 0; index < lengthSize; index++) // 计算Length域的长度
							{
								valueSize += *(tlvPos + currentIndex +index) << (index * 8);
							}
							#endif
							currentIndex += (lengthSize + 1);
						}

						currentStatus = 'T';

					}
				break;
			case 'L':
				if ( (*(tlvPos + currentIndex) & 0x80) != 0x80)//b6~b0的值就是value域的长度
				{
					valueSize = *(tlvPos + currentIndex);
					currentIndex += (valueSize + 1);
					//printf("valuesize %x\r\n",valueSize);
				}
				else//b6~b0的值指示后面有几个字节表示value长度
				{
					uint8 lengthSize = 0;
					lengthSize = *(tlvPos + currentIndex) & 0x7f;
					for (uint8 index = 0; index < lengthSize; index++) // 计算Length域的长度
					{
						valueSize += *(tlvPos + currentIndex +index) << (index * 8);
					}
					currentIndex = valueSize + currentIndex + lengthSize + 1;
				}
					currentStatus = 'T';
				break;

			default:

		};
		//printf("currentIndex %x\r\n",currentIndex);
		//break;
	}

}
void DoubleBytesTagProcess(uint8* curTagPos,uint8 **currentDst)
{
	uint8* currentPos = *currentDst;
	if(Idtag[2] == 0)
	{
		if(*curTagPos == Idtag[0] && *(curTagPos+1) == Idtag[1])
		{
			uint8 tmpLenght = 0;
			*currentPos = 0x9f;
			*(currentPos+1) = 0x61;
			//printf("ID ");
			while(*(curTagPos+tmpLenght + 3) != 0x20 && *(curTagPos+tmpLenght + 3) != 0x0 && tmpLenght < *(curTagPos + 2))
			{
				*(currentPos + 3 +tmpLenght) = *(curTagPos+tmpLenght + 3);
				//printf("%x ",*(curTagPos+tmpLenght + 3));
				tmpLenght++;
			}
			//printf("\r\nlength %x\r\n",tmpLenght);

		*(currentPos+2) = tmpLenght;

		//*Dest = *(currentPos+2) + *Dest + 3;
		Fivedata[0] += (*(currentPos+2) + 3);
		//currentPos = currentPos + *(currentPos+2) + 3;
		*currentDst = *currentDst + *(currentPos+2) + 3;

			Idtag[2] = 1;
		return;
		}
	}

	if(Nametag1[2] == 0)
	{
		if(*curTagPos == Nametag1[0] && *(curTagPos+1) == Nametag1[1])
		{
			memcpy(currentPos,curTagPos,*(curTagPos+2) +3);
			//currentPos = currentPos + *(tmp+2) +3;
			//*Dest = *Dest + *(tmp+2) +3;
			Fivedata[0] += (*(curTagPos+2) + 3);
			*currentDst = *currentDst + *(currentPos+2) + 3;

			Nametag1[2] = 1;
			return;
		}
	}

	if(Nametag2[2] == 0)
	{
		if(*curTagPos == Nametag2[0] && *(curTagPos+1) == Nametag2[1])
		{
			memcpy(currentPos,curTagPos,*(curTagPos+2) +3);
			//currentPos = currentPos + *(tmp+2) +3;
			//*Dest = *Dest + *(tmp+2) +3;
			Fivedata[0] += (*(curTagPos+2) + 3);
			*currentDst = *currentDst + *(currentPos+2) + 3;

			Nametag2[2] = 1;
			return;
		}
	}

	if(fromDateTag[2] == 0)
	{
		if(*curTagPos == fromDateTag[0] && *(curTagPos+1) == fromDateTag[1])
		{
			memcpy(currentPos,curTagPos,*(curTagPos+2)+3);
			Fivedata[0] += (*(curTagPos+2) + 3);
			*currentDst = *currentDst + *(currentPos+2) + 3;
			//*Dest = *Dest + *(tmp+2)+3;
			//curPos += *(tmp+2)+3;

            fromDateTag[2] = 1;

			return;
		}
	}

	if(endDateTag[2] == 0)
	{
		if(*curTagPos == endDateTag[0] && *(curTagPos+1) == endDateTag[1])
		{
			memcpy(currentPos,curTagPos,*(curTagPos+2)+3);

			*(currentPos+1) = 0x26;//修改Tag
			Fivedata[0] += (*(curTagPos+2) + 3);


			*currentDst = *currentDst + *(currentPos+2) + 3;

			endDateTag[2] = 1;
			return;
		}
	}

}

void SingleByteTagProcess(uint8* curTagPos,uint8 **currentDst)
{
	uint8* currentPos = *currentDst;
	uint8 trackLength = 0;
	uint8 cardIDlength = 0;
	uint8 check;
	if(tarck2tag[1] == 0)
	{
		if(*curTagPos == tarck2tag[0] )
		{
			*currentPos = 0x57;//磁道2的ID，现在表示卡号字段
			//printf("CARD ID\r\n");
		//写入卡号
		while(trackLength < 15)
		{
			check = *(curTagPos+2+trackLength);
			if(((check & 0xf0)>>4) >= 0 && ((check & 0xf0)>>4) <= 9)
			{
				*(currentPos + 2 + cardIDlength) = ((check & 0xf0)>>4);
				cardIDlength += 1;
			}
			else
			{
				break;
			}

			if((check & 0x0f) >= 0 && (check & 0x0f) <= 9)
			{
				*(currentPos + 2 + cardIDlength) = (check & 0x0f);
				cardIDlength += 1;
			}
			else
			{
				break;
			}
			trackLength += 1;
		}
		*(currentPos+1) = cardIDlength;
		//字段的总长度
		//*Dest = *(Dest + 2) + *Dest + 2 +4;


		//失效日期字段
		*(currentPos + 2 + cardIDlength) = 0x5f;
		*(currentPos + 3 + cardIDlength) = 0x24;

#if 1
		if(cardIDlength % 2)
		{
			*(currentPos + 4 + cardIDlength) = *(curTagPos+3+trackLength);
			*(currentPos + 5 + cardIDlength) = *(curTagPos+4+trackLength);
			//printf("XXX\r\n");
		}
		else
		{
			//printf("YYY\r\n");
			*(currentPos + 4 + cardIDlength) = ((*(curTagPos+2+trackLength) & 0x0f) << 4)|((*(curTagPos+3+trackLength) & 0xf0)>>4);
			*(currentPos + 5 + cardIDlength) = ((*(curTagPos+3+trackLength) & 0x0f) << 4)|((*(curTagPos+4+trackLength) & 0xf0)>>4);
		}
#endif
		//cardIDlength += 4;
		//currentPos = (Dest + 7 + cardIDlength);

		Fivedata[0] += (*(currentPos+1) +   2 + 4);

		*currentDst = *currentDst + *(currentPos+1) + 2 +4;

		tarck2tag[1] = 1;

			return;
		}

	}
}

uint8 DataCollectComplete(void)
{
	if(tarck2tag[1] == 1 && Idtag[2] == 1 && fromDateTag[2] == 1 && endDateTag[2] == 1 && (Nametag1[2] == 1 || Nametag2[2] == 1))
	{
		memcpy(dataBuffer[0],Fivedata,Fivedata[0]+1);
		send_token = 6;
		return 1;
	}
	else
	{
		return 0;
	}

}
void HardwareReset(void)
{
	IC_REST = 0;
   	DelayMs(2);
    IC_REST = 1;
	DelayMs(10);
	IC_REST = 0;
   	DelayMs(2);
}
