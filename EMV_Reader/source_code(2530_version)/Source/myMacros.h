/***********************************************************************************
  Filename:     myMacros.h

  Description:  ���ú꼰�ṹ�嶨��

***********************************************************************************/
#include "board.h"

#ifdef DATA_HUB
#define TASKS_MAX 7
#else
#define TASKS_MAX 10
#endif

void TaskRemarks(void);
void TaskProcess(void);


// ����ṹ
typedef struct _TASK_COMPONENTS
{
    uint16 Run;                  // �������б�ǣ�0-�����У�1����
    uint16 Timer;              	 // ��ʱ��
    uint16 ItvTime;              // �������м��ʱ��
    void (*TaskHook)(void);      // Ҫ���е�������
} TASK_COMPONENTS;       // ������


