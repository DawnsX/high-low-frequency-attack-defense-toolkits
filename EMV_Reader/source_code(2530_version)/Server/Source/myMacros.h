/***********************************************************************************
  Filename:     myMacros.h
  
  Description:  ���ú꼰�ṹ�嶨��

***********************************************************************************/
#include "board.h" 

#ifdef DATA_HUB
#define TASKS_MAX 4
#else
#define TASKS_MAX 4
#endif

void TaskRemarks(void);
void TaskProcess(void);


// ����ṹ
typedef struct _TASK_COMPONENTS
{
    uint8 Run;                 // �������б�ǣ�0-�����У�1����
    uint8 Timer;              // ��ʱ��
    uint8 ItvTime;              // �������м��ʱ��
    void (*TaskHook)(void);    // Ҫ���е�������
} TASK_COMPONENTS;       // ������


