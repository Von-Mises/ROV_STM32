 /**
 * ***************************@copyright (C) Copyright  2023  ZJU***************************
 * @file   task_time_count.c/h
 * @brief  任务运行状态显示任务
 * @author Qiqi Li(李琪琪)
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
		printf("任务名\t\t运行时间\t运行所占百分比\r\n");
		printf("%s\r\n", RunTimeInfo);
		
		printf("任务名\t\t状态\t优先级\t剩余栈\t任务序号\r\n");
		printf("%s\r\n", TaskInfo);
		
		free(RunTimeInfo);
		free(TaskInfo);
		
		vTaskDelay(3000);
	}
}

