/**
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  * @file       tem_hum_task.c/h
  * @brief      ��ʪ�ȴ��������ݴ�������         
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     DEC-7-2022     HaoLion(������)    1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  */

#ifndef TEM_HUM_TASK_H
#define TEM_HUM_TASK_H

#include "struct_typedef.h"




/* ----------------------- Data Struct ------------------------------------- */



/* ----------------------- Extern Function ----------------------------------- */
/**
  * @brief          Data of temperature and humidity processing
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          ��ʪ�����ݴ���
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void tem_hum_task(void const * argument);

extern float get_cabin_humidity(void);

extern float get_cabin_temperature(void);

#endif
