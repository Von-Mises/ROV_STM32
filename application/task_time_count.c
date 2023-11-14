 /**
 * ***************************@copyright (C) Copyright  2023  ZJU***************************
 * @file   task_time_count.c/h
 * @brief  ��������״̬��ʾ����
 * @author Qiqi Li(������)
 * @version 1.0
 * @date 2023-11-8
 * 
 * @verbatim:
 * ==============================================================================
 *                                                                               
 * ==============================================================================
 * @endverbatim
 * ***************************@copyright (C) Copyright  2023  ZJU***************************
 */
#include "task_time_count.h"
#include "lwip/api.h"
#include "string.h"
#include "socket_send.h"

void task_time_count_task(void const * argument)
{
	char *RunTimeInfo = NULL;
	RunTimeInfo = (char *)pvPortMalloc(400);
	if (RunTimeInfo == NULL)
	{
		printf("No memory in this task\n");
	}
	memset(RunTimeInfo, 0, 400);
	
	char *TaskInfo = NULL;
	TaskInfo = (char *)pvPortMalloc(400);
	if (TaskInfo == NULL)
	{
		printf("No memory in this task\n");
	}
	memset(TaskInfo, 0, 400);
	
	while(1)
	{
		vTaskGetRunTimeStats(RunTimeInfo);
		vTaskList(TaskInfo);
//		socket_printf("%s\r\n","TaskName RunTime RunTimePercent");
//		socket_printf("%s\r\n", RunTimeInfo);
		printf("������\t\t����ʱ��\t������ռ�ٷֱ�\r\n");
		printf("%s\r\n", RunTimeInfo);
		
		printf("������\t\t״̬\t���ȼ�\tʣ��ջ\t�������\r\n");
		printf("%s\r\n", TaskInfo);
		
		free(RunTimeInfo);
		free(TaskInfo);
		
		vTaskDelay(3000);
	}
}

