/***********************************************************************************
  Filename:     myOs.c

  Description:  ʱ��Ƭѡ��ѯ������Ҫ����

***********************************************************************************/

#include "types.h"
#include "myMacros.h"

extern TASK_COMPONENTS TaskComps[];

/**************************************************************************************
* FunctionName   : TaskRemarks()
* Description    : �����־����
* EntryParameter : None
* ReturnValue    : None
**************************************************************************************/
void TaskRemarks(void)
{
    uint8 i;

    for (i=0; i<TASKS_MAX; i++)          // �������ʱ�䴦��
    {
         if (TaskComps[i].Timer)          // ʱ�䲻Ϊ0
        {
            TaskComps[i].Timer--;         // ��ȥһ������
            if (TaskComps[i].Timer == 0)       // ʱ�������
            {
                 TaskComps[i].Timer = TaskComps[i].ItvTime;       // �ָ���ʱ��ֵ��������һ��
                 TaskComps[i].Run = 1;           // �����������
            }
        }
   }
}

/**************************************************************************************
* FunctionName   : TaskProcess()
* Description    : ������
* EntryParameter : None
* ReturnValue    : None
**************************************************************************************/
void TaskProcess(void)
{
    uint8 i;

    for (i=0; i<TASKS_MAX; i++)           // �������ʱ�䴦��
    {
         if (TaskComps[i].Run)           // ʱ�䲻Ϊ0
        {
        	 TaskComps[i].Run = 0;          // ��־��0
             TaskComps[i].TaskHook();         // ��������
        }
    }   
}










