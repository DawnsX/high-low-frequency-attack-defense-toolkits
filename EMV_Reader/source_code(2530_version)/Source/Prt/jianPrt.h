#ifndef _JIANPRT_H
#define _JIANPRT_H
//#include "stm32f10x.h"
//#include "jianMacros.h"
#include "stdarg.h"

#include "types.h"
#include "ioCC2530.h"


typedef uint8 INT8U;
typedef uint16 INT16U;
typedef uint32 INT32U;

typedef int8 INT8S;
typedef int16 INT16S;
typedef int32 INT32S;



#if 1
#define TXD      (P1_6)
#define TXD_HIGH (TXD =  1)
#define TXD_LOW  (TXD =  0)
//#define TXD_TOG  (TXD =  1)
#endif

#if 1
//#define TXF      (TIM2->SR)
//#define TXF_ON   (TXF & TIM_FLAG_Update)
//#define TXF_OFF  (TXF &= ~TIM_FLAG_Update)
#define TIM_FLAG_Update 0x01
#define TXF      (TIMIF)
#define TXF_ON   (TXF & TIM_FLAG_Update)
#define TXF_OFF  (TXF &= ~TIM_FLAG_Update)

#endif

#if 1
#define RXF		 (GPIOA->IDR)
//#define RXF_ON	 (RXF & BIT12)	//双线时配置
#define RXF_ON	 (RXF & BIT11)		//单线时配置
#endif

//static void basicRfRxFrmDoneIsr_test2(void);

void PrtGPIOInit(void);
void PrtTimer3_Init(void);

void  ptb(INT8U c);
void  ptc(INT8U c);
void  pts(INT8U *s);
INT8U RByte(void);
char hex2cha(char hex);
void prt(INT8U *fmt,...);



#endif






