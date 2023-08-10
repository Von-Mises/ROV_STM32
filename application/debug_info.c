 /**
 * ***************************@copyright (C) Copyright  2023  ZJU***************************
 * @file   debug_info.c/h
 * @brief  运行信息上传
 * @author Qiqi Li(李琪琪)
 * @version 1.0
 * @date 2023-08-03
 * 
 * @verbatim:
 * ==============================================================================
 *                                                                               
 * ==============================================================================
 * @endverbatim
 * ***************************@copyright (C) Copyright  2023  ZJU***************************
 */
 //使用freertos和hal库编写一个任务用来发送ROV运行信息
#include "debug_info.h"
#include "socket_send.h"
#include <string.h>
#include "main.h"
#include "cmsis_os.h"

/**
 * @brief  Function implementing the  debug_info thread.
 * @param  argument         none
 */
void debug_info_task(void const * argument)
{
	while(1)
	{
		uint8_t str[] = "ROV Program is Running!\r\n";
		msg_send(str,sizeof(str));
		osDelay(2000);
	}
}
