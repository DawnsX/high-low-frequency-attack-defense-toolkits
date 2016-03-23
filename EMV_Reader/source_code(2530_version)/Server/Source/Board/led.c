/***********************************************************************************
    Filename: led.c

***********************************************************************************/
#include "Led.h"
#include "types.h"
#include "basic_rf.h"
#include "board.h"
#include "ioCC2530.h"
#include <stdio.h>
#include "myMacros.h"


extern TASK_COMPONENTS TaskComps[];
/***********************************************************************************
* @fn		   LedInit
*
* @brief	   初始化LED相关IO口及充电信号输入
*
* @param	   none
*
* @return	   none
*/
void LedInit(void)
{
    MCU_IO_OUTPUT(0, 4, 0);	//四个LED，IO口选择为输出模式
    MCU_IO_OUTPUT(0, 5, 1);
	MCU_IO_OUTPUT(0, 6, 1);
	MCU_IO_OUTPUT(1, 2, 0);
	
	//MCU_IO_SET_HIGH_PREP(1, 1);//四个LED，IO口输出为高，即等灭
	//MCU_IO_SET_HIGH_PREP(1, 4);

	//MCU_IO_INPUT(1, 3, MCU_IO_PULLUP);//BAT_CHRG
	//MCU_IO_INPUT(1, 2, MCU_IO_PULLUP);//BAT_DONE

}

/***********************************************************************************
* @fn      halLedSet
*
* @brief   Turn LED on.
*
* @param   uint8 id - id of LED
*
* @return  none
*/
void halLedSet(uint8 id)
{
    if (id==1)
        MCU_IO_SET_LOW_PREP(0, 5);
	if (id==2)
        MCU_IO_SET_LOW_PREP(0, 6);
    //if (id==3)
     //   MCU_IO_SET_LOW_PREP(2, 0);
	//if (id==4)
    //    MCU_IO_SET_LOW_PREP(1, 2);
}

/***********************************************************************************
* @fn      halLedClear
*
* @brief   Turn LED off.
*
* @param   uint8 id - id of LED
*
* @return  none
*/
void halLedClear(uint8 id)
{
    if (id==1)
        MCU_IO_SET_HIGH_PREP(0, 5);
	if (id==2)
        MCU_IO_SET_HIGH_PREP(0, 6);
    //if (id==3)
    //    MCU_IO_SET_HIGH_PREP(2, 0);
	//if (id==4)
    //    MCU_IO_SET_HIGH_PREP(1, 2);
}
/***********************************************************************************
* @fn      halLedToggle
*
* @brief   Change state of LED. If on, turn it off. Else turn on.
*
* @param   uint8 id - id of LED
*
* @return  none
*/
void halLedToggle(uint8 id)
{
    if (id==1)
        MCU_IO_TGL_PREP(1, 1);
	if (id==2)
        MCU_IO_TGL_PREP(1, 4);
    if (id==3)
        MCU_IO_TGL_PREP(2, 0);
	if (id==4)
        MCU_IO_TGL_PREP(1, 2);
}

void halLedToggle_1(void)
{
	MCU_IO_TGL_PREP(0, 4);
	MCU_IO_TGL_PREP(1, 2);
	//printf("OK2");
	//manTrigger();
}

void halLedToggle_2(void)
{
	MCU_IO_TGL_PREP(0, 5);
	//printf("OK3");
}

void halLedToggle_3(void)
{
	MCU_IO_TGL_PREP(0, 6);
}

void halLedToggle_4(void)
{
	MCU_IO_TGL_PREP(1, 2);
}

void LedTestInit(void)  
{  
    P1SEL &= ~(0x01<<3);  //GPIO功能，而不是外设
	P1DIR &= ~(0x01<<3);  //输入
	P1INP |= (0x01<<3);	  //三态
    //P1DIR |= 0X01;  
         
     
    P1IEN |= (0X01<<3);     //P10设置为中断方式  
    PICTL |= 0X02;     //下降沿触发  
    //EA = 1;  
    IEN2 |= 0x10;  
    P1IFG  = 0; 
	P1IF = 0;  
}  


