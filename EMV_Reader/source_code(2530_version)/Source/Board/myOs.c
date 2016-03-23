/***********************************************************************************
  Filename:     myOs.c

  Description:  时间片选轮询法的主要函数

***********************************************************************************/

#include "types.h"
#include "myMacros.h"

extern TASK_COMPONENTS TaskComps[];

/**************************************************************************************
* FunctionName   : TaskRemarks()
* Description    : 任务标志处理
* EntryParameter : None
* ReturnValue    : None
**************************************************************************************/
void TaskRemarks(void)
{
    uint8 i;

    for (i=0; i<TASKS_MAX; i++)          // 逐个任务时间处理
    {
         if (TaskComps[i].Timer)          // 时间不为0
        {
            TaskComps[i].Timer--;         // 减去一个节拍
            if (TaskComps[i].Timer == 0)       // 时间减完了
            {
                 TaskComps[i].Timer = TaskComps[i].ItvTime;       // 恢复计时器值，从新下一次
                 TaskComps[i].Run = 1;           // 任务可以运行
            }
        }
   }
}

/**************************************************************************************
* FunctionName   : TaskProcess()
* Description    : 任务处理
* EntryParameter : None
* ReturnValue    : None
**************************************************************************************/
void TaskProcess(void)
{
    uint8 i;

    for (i=0; i<TASKS_MAX; i++)           // 逐个任务时间处理
    {
         if (TaskComps[i].Run)           // 时间不为0
        {
        	 TaskComps[i].Run = 0;          // 标志清0
             TaskComps[i].TaskHook();         // 运行任务
        }
    }
}


#pragma optimize=none
void DelayUs(uint16 usec)
{
    usec >>= 1;
    while (usec--)
    {
       	asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
        //NOP();
    }
}
#pragma optimize=none
void DelayMs(uint16 msec)
{
    while (msec--)
    {
        DelayUs(1000);
    }
}


#pragma optimize=none
void DelayHalfUs(uint16 usec)
{
    usec >>= 1;
    while (usec--)
    {
       	asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		//asm("nop");
        //NOP();
    }
}





