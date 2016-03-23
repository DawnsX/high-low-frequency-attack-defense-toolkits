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
#ifndef BOARD_H
#define BOARD_H

#include "jianPrt.h"
#include "types.h"

//#define DATA_HUB


/***********************************************************************************
 * GLOBAL FUNCTIONS
 */
 
void BoardInit(void);
uint8 SetTimer34Period(uint8, uint32);
void Timer4_Init(void);
void UART_Init(void);
void UART0_SendByte(uint8 data);
void UART1_SendByte(uint8 data);
void UART0_SendData(uint8 *pbuff, uint16 len);
void UART1_SendData(uint8 *pbuff, uint16 len);  
void Uart0FrameDecode(uint16);
void Uart1FrameDecode(uint16 );
void Rx0Back(void);
void Rx1Back(void);

void ChangeBaudrate(void);
uint16 ChangeGpsData(uint8 *src, uint8 *replace,uint16 *count);
uint16  Length(uint8 *src);
void checkSum(uint8 *CK_A,uint8 *CK_B,uint8 *buffer);
void PositionAdjustableShift(uint8 *src ,unsigned long int lon,unsigned long int lat);
void ParamGuide(void);
void Uart0FrameDecode_Node(uint16 count);



/************************************************************************************/
/*
*  This macro is for use by other macros to form a fully valid C statement.
*  Without this, the if/else conditionals could show unexpected behavior.
*
*  For example, use...
*    #define SET_REGS()  st( ioreg1 = 0; ioreg2 = 0; )
*  instead of ...
*    #define SET_REGS()  { ioreg1 = 0; ioreg2 = 0; }
*  or
*    #define  SET_REGS()    ioreg1 = 0; ioreg2 = 0;
*  The last macro would not behave as expected in the if/else construct.
*  The second to last macro will cause a compiler error in certain uses
*  of if/else construct
*
*  It is not necessary, or recommended, to use this macro where there is
*  already a valid C statement.  For example, the following is redundant...
*    #define CALL_FUNC()   st(  func();  )
*  This should simply be...
*    #define CALL_FUNC()   func()
*
* (The while condition below evaluates false without generating a
*  constant-controlling-loop type of warning on most compilers.)
*/
#define st(x)      do { x } while (__LINE__ == -1)


/************************************************************************************/
/*
中断优先级设定
*/

#define INIT_INTERRUPT_PRIORITY( )                                                                        \
	st(IP1 = 0x03;IP0 = 0x0E;)

/******************************************************************************
*******************  USART-UART specific functions/macros   *******************
******************************************************************************/
// The macros in this section simplify UART operation.
#define BAUD_E(baud, clkDivPow) (     \
    (baud==2400)   ?  6  +clkDivPow : \
    (baud==4800)   ?  7  +clkDivPow : \
    (baud==9600)   ?  8  +clkDivPow : \
    (baud==14400)  ?  8  +clkDivPow : \
    (baud==19200)  ?  9  +clkDivPow : \
    (baud==28800)  ?  9  +clkDivPow : \
    (baud==38400)  ?  10 +clkDivPow : \
    (baud==57600)  ?  10 +clkDivPow : \
    (baud==76800)  ?  11 +clkDivPow : \
    (baud==115200) ?  11 +clkDivPow : \
    (baud==153600) ?  12 +clkDivPow : \
    (baud==230400) ?  12 +clkDivPow : \
    (baud==307200) ?  13 +clkDivPow : \
    0  )


#define BAUD_M(baud) (      \
    (baud==2400)   ?  59  : \
    (baud==4800)   ?  59  : \
    (baud==9600)   ?  59  : \
    (baud==14400)  ?  216 : \
    (baud==19200)  ?  59  : \
    (baud==28800)  ?  216 : \
    (baud==38400)  ?  59  : \
    (baud==57600)  ?  216 : \
    (baud==76800)  ?  59  : \
    (baud==115200) ?  216 : \
    (baud==153600) ?  59  : \
    (baud==230400) ?  216 : \
    (baud==307200) ?  59  : \
  0)

//USART0，IO口设定
#define IO_PER_LOC_USART0_AT_PORT0_PIN2345() do { PERCFG = (PERCFG&~0x01)|0x00; }while (0)

//USART1，IO口设定
#define IO_PER_LOC_USART1_AT_PORT1_PIN4567() do { PERCFG |= 0x02; }while (0)


// Macro for setting up a UART transfer channel. The macro sets the appropriate
// pins for peripheral operation, sets the baudrate, and the desired options of
// the selected uart. _uart_ indicates which uart to configure and must be
// either 0 or 1. _baudRate_ must be one of 2400, 4800, 9600, 14400, 19200,
// 28800, 38400, 57600, 76800, 115200, 153600, 230400 or 307200. Possible
// options are defined below.
//
// Example usage:
//
//      UART_SETUP(0,115200,HIGH_STOP);
//
// This configures uart 0 for contact with "hyperTerminal", setting:
//      Baudrate:           115200
//      Data bits:          8
//      Parity:             None
//      Stop bits:          1
//      Flow control:       None
//

#define UART_SETUP(uart, baudRate, options)      \
   do {                                          \
      if ((options) & FLOW_CONTROL_ENABLE){      \
         if((uart) == 0){      /* USART0       */\
            if(PERCFG & 0x01){ /* Alt 2        */\
               P1SEL |= 0x3C;                    \
            } else {           /* Alt 1        */\
               P0SEL |= 0x3C;                    \
            }                                    \
         }                                       \
         else {                /* USART1       */\
            if(PERCFG & 0x02){ /* Alt 2        */\
               P1SEL |= 0xF0;                    \
            } else {           /* Alt 1        */\
               P0SEL |= 0x3C;                    \
            }                                    \
         }                                       \
      }                                          \
      else{                    /* Flow Ctrl Dis*/\
         if((uart) == 0){      /* USART0       */\
            if(PERCFG & 0x01){ /* Alt 2        */\
               P1SEL |= 0x30;                    \
            } else {           /* Alt 1        */\
               P0SEL |= 0x0C;                    \
            }                                    \
         }                                       \
         else {                /* USART1       */\
            if(PERCFG & 0x02){ /* Alt 2        */\
               P1SEL |= 0xC0;                    \
            } else {           /* Alt 1        */\
               P0SEL |= 0x30;                    \
            }                                    \
         }                                       \
      }                                          \
                                                 \
      U##uart##GCR = BAUD_E((baudRate), CC2530_GET_CLKSPD()); \
      U##uart##BAUD = BAUD_M(baudRate);          \
                                                 \
      U##uart##CSR |= 0x80;                      \
                                                 \
      U##uart##UCR |= ((options) | 0x80);        \
                                                 \
      if((options) & TRANSFER_MSB_FIRST){        \
         U##uart##GCR |= 0x20;                   \
      }                                          \
   } while(0)


// Options for UART_SETUP macro
#define FLOW_CONTROL_ENABLE         0x40
#define FLOW_CONTROL_DISABLE        0x00
#define EVEN_PARITY                 0x20
#define ODD_PARITY                  0x00
#define NINE_BIT_TRANSFER           0x10
#define EIGHT_BIT_TRANSFER          0x00
#define PARITY_ENABLE               0x08
#define PARITY_DISABLE              0x00
#define TWO_STOP_BITS               0x04
#define ONE_STOP_BITS               0x00
#define HIGH_STOP                   0x02
#define LOW_STOP                    0x00
#define HIGH_START                  0x01
#define TRANSFER_MSB_FIRST          0x80
#define TRANSFER_MSB_LAST           0x00


/******************************************************************************
*******************       Interrupt functions/macros        *******************
******************************************************************************/

// Macros which simplify access to interrupt enables, interrupt flags and
// interrupt priorities. Increases code legibility.

//******************************************************************************

#define INT_ON   1
#define INT_OFF  0
#define INT_SET  1
#define INT_CLR  0

// Global interrupt enables
#define INT_GLOBAL_ENABLE(on) EA=(!!on)

#define DISABLE_ALL_INTERRUPTS() (IEN0 = IEN1 = IEN2 = 0x00)

#define INUM_RFERR 0
#define INUM_ADC   1
#define INUM_URX0  2
#define INUM_URX1  3
#define INUM_ENC   4
#define INUM_ST    5
#define INUM_P2INT 6
#define INUM_UTX0  7
#define INUM_DMA   8
#define INUM_T1    9
#define INUM_T2    10
#define INUM_T3    11
#define INUM_T4    12
#define INUM_P0INT 13
#define INUM_UTX1  14
#define INUM_P1INT 15
#define INUM_RF    16
#define INUM_WDT   17

#define NBR_OF_INTERRUPTS 18

//Macro for enabling overflow interrupt
#define TIMER34_ENABLE_OVERFLOW_INT(timer,val) \
   (T##timer##CTL =  (val) ? T##timer##CTL | 0x08 : T##timer##CTL & ~0x08)


	// Macro used together with the INUM_* constants
	// to enable or disable certain interrupts.
	// Example usage:
	//	 INT_ENABLE(INUM_RFERR, INT_ON);
	//	 INT_ENABLE(INUM_URX0, INT_OFF);
	//	 INT_ENABLE(INUM_T1, INT_ON);
	//	 INT_ENABLE(INUM_T2, INT_OFF);
#define INT_ENABLE(inum, on)                        \
	   do { 											\
		  if	  (inum==INUM_RFERR) { RFERRIE = on; }	\
		  else if (inum==INUM_ADC)	 { ADCIE   = on; }	\
		  else if (inum==INUM_URX0)  { URX0IE  = on; }	\
		  else if (inum==INUM_URX1)  { URX1IE  = on; }	\
		  else if (inum==INUM_ENC)	 { ENCIE   = on; }	\
		  else if (inum==INUM_ST)	 { STIE    = on; }	\
		  else if (inum==INUM_P2INT) { (on) ? (IEN2 |= 0x02) : (IEN2 &= ~0x02); } \
		  else if (inum==INUM_UTX0)  { (on) ? (IEN2 |= 0x04) : (IEN2 &= ~0x04); } \
		  else if (inum==INUM_DMA)	 { DMAIE   = on; }	\
		  else if (inum==INUM_T1)	 { T1IE    = on; }	\
		  else if (inum==INUM_T2)	 { T2IE    = on; }	\
		  else if (inum==INUM_T3)	 { T3IE    = on; }	\
		  else if (inum==INUM_T4)	 { T4IE    = on; }	\
		  else if (inum==INUM_P0INT) { P0IE    = on; }	\
		  else if (inum==INUM_UTX1)  { (on) ? (IEN2 |= 0x08) : (IEN2 &= ~0x08); } \
		  else if (inum==INUM_P1INT) { (on) ? (IEN2 |= 0x10) : (IEN2 &= ~0x10); } \
		  else if (inum==INUM_RF)	 { (on) ? (IEN2 |= 0x01) : (IEN2 &= ~0x01); } \
		  else if (inum==INUM_WDT)	 { (on) ? (IEN2 |= 0x20) : (IEN2 &= ~0x20); } \
	   } while (0)

// Macros for turning timers on or off
#define TIMER1_RUN(value)      (T1CTL = (value) ? T1CTL|0x02 : T1CTL&~0x03)
#define TIMER2_RUN(value)      (T2CNF  = (value) ? T2CNF|0x01  : T2CNF&~0x01)
#define MAC_TIMER_RUN(value)   do{ TIMER2_RUN(value); }while(0)  //MAC-timer == timer 2
#define TIMER3_RUN(value)      (T3CTL = (value) ? T3CTL|0x10 : T3CTL&~0x10)
#define TIMER4_RUN(value)      (T4CTL = (value) ? T4CTL|0x10 : T4CTL&~0x10)

// Macro for initialising timer 3 or 4
#define TIMER34_INIT(timer)   \
   do {                       \
   	  T##timer##CTL   = 0x06; \
   	  T##timer##CC0   = 0x00; \
      T##timer##CCTL1 = 0x00; \
      T##timer##CC1   = 0x00; \
   } while (0)




/* the TimeoutTerminator type is the prototype used for functions that test
 * semaphores and are passed to the RF_WaitTimeoutUsec and RF_WaitTimeoutMS
 * functions.
 */


enum BuadRate_status{SLOW9600,FAST115200};
enum ConnectMode{DIRECT,REPLACE,SHIFT};

#define Interval_9600 		5
#define Interval_115200 	2
#define INPUTLENGTH  		256  
#define INPUTLENGTHMASK  	(INPUTLENGTH - 1)  


#define MESSAGE0()			st(printf("\r\nMenu:\r\n");\
							printf("1.Amount of satellites\r\n");\
							printf("2.Longitude\r\n");\
							printf("3.Latitude\r\n");\
							printf("4.Longitude offset\r\n");\
							printf("5.Latitude offset\r\n");\
							printf("tips:input the sequence number of item you want to modify.\r\n");\
							printf("input:");\
							)

#define MESSAGE1() 			st(printf("\r\nPlease input the number of satellites. (from 0 to 49)\r\n");\
							printf("input \"back\", go to main menu.\r\n");\
							printf("input:");\
							)
#if 0
#define MESSAGE2()			st(printf("Signal strength\r\n");\
							printf("1.High\r\n");\
							printf("2.Middle\r\n");\
							printf("3.Low\r\n");\
							printf("input \"back\", go to main menu.\r\n");\
							printf("input:");)
#endif
#define MESSAGE2()			st(printf("\r\nInput longitude.(from -180° to 180°)\r\n");\
							printf("Example:35.214237\r\n");\
							printf("Input \"back\", go to main menu.\r\n");\
							printf("input:");)
							
#define MESSAGE3()			st(printf("\r\nInput latitude.(from -90° to 90°)\r\n");\
							printf("Example:31.768036\r\n");\
							printf("Input \"back\", go to main menu.\r\n");\
							printf("input:");)

#define MESSAGE4()			st(printf("\r\nInput longitude offset.(the result should from -180° to 180°)\r\n");\
							printf("Example:-5\r\n");\
							printf("Input \"back\", go to main menu.\r\n");\
							printf("input:");)
							
#define MESSAGE5()			st(printf("\r\nInput latitude offset.(the result should from -90° to 90°)\r\n");\
							printf("Example:5\r\n");\
							printf("Input \"back\", go to main menu.\r\n");\
							printf("input:");)



#endif

