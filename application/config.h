#ifndef __CONFIG_H__
#define __CONFIG_H__

#include "socket_send.h"




#define OS							1					//是否在有操作系统情况下执行
#define IMU_DEBUG				0					//开启IMU数据串口回传
#define DM_DEBUG				0					//开启深度计数据串口回传
#define IP_DEBUG				0					//是否开启网络数据回显

#define CONSOLE_MODE		1					//是否开启TCP发送文本


#if OS
	#define ROV_Delay		vTaskDelay
#else
	#define ROV_Delay		HAL_Delay
#endif

#if OS
	#include "cmsis_os.h"
#endif

#endif
