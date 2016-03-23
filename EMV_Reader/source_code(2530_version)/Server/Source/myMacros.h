/***********************************************************************************
  Filename:     myMacros.h
  
  Description:  常用宏及结构体定义

***********************************************************************************/
#include "board.h" 

#ifdef DATA_HUB
#define TASKS_MAX 4
#else
#define TASKS_MAX 4
#endif

void TaskRemarks(void);
void TaskProcess(void);


// 任务结构
typedef struct _TASK_COMPONENTS
{
    uint8 Run;                 // 程序运行标记：0-不运行，1运行
    uint8 Timer;              // 计时器
    uint8 ItvTime;              // 任务运行间隔时间
    void (*TaskHook)(void);    // 要运行的任务函数
} TASK_COMPONENTS;       // 任务定义


