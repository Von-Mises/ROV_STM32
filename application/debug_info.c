 /**
 * ***************************@copyright (C) Copyright  2023  ZJU***************************
 * @file   debug_info.c/h
 * @brief  ������Ϣ�ϴ�
 * @author Qiqi Li(������)
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
 //ʹ��freertos��hal���дһ��������������ROV������Ϣ
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
