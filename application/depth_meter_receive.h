/**
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  * @file       depth_meter_receive.c/h
  * @brief      ������ȼƷ��͹���������,����DMA���䷽ʽ��ԼCPU
  *             ��Դ�����ô��ڿ����ж�������������ͬʱ�ṩһЩ��������DMA������
  *             �ķ�ʽ��֤�Ȳ�ε��ȶ��ԡ�
  * @note       ��������ͨ�������ж�����������freeRTOS����
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     DEC-21-2022     Qiqi Li(������)    1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  */

#ifndef DEPTH_METER_RECEIVE_H
#define DEPTH_METER_RECEIVE_H

#include "struct_typedef.h"


#define DM_FRAME_LENGTH 	32
#define DM_CMD_LENGTH 		16
	

/* ----------------------- Data Struct ------------------------------------- */



/* ----------------------- extern Function ----------------------------------- */

extern void DM_init(void);
extern void DM_reset(void);
extern float get_depth_data(void);

#endif
