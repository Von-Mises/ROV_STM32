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

#ifndef ALTIMETER_RECEIVE_H
#define ALTIMETER_RECEIVE_H

#include "struct_typedef.h"


#define AM_FRAME_LENGTH 	30
#define AM_DATA_LENGTH 		15
#define AM_CMD_LENGTH 		15
#define AM_AIR_VELOCITY		340
#define AM_WATER_VELOCITY   1500

/* ----------------------- Data Struct ------------------------------------- */



/* ----------------------- extern Function ----------------------------------- */

extern void AM_init(void);
extern float get_height_data(void);
extern void AM_get_data(void);

#endif
