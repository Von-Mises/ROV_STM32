#ifndef __CONFIG_H__
#define __CONFIG_H__

#include "socket_send.h"




#define OS							1					//�Ƿ����в���ϵͳ�����ִ��
#define IMU_DEBUG				0					//����IMU���ݴ��ڻش�
#define DM_DEBUG				0					//������ȼ����ݴ��ڻش�
#define IP_DEBUG				0					//�Ƿ����������ݻ���

#define CONSOLE_MODE		1					//�Ƿ���TCP�����ı�


#if OS
	#define ROV_Delay		vTaskDelay
#else
	#define ROV_Delay		HAL_Delay
#endif

#if OS
	#include "cmsis_os.h"
#endif

#endif
