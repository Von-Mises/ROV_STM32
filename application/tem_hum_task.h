/**
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  * @file       tem_hum_task.c/h
  * @brief      温湿度传感器数据处理任务         
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     DEC-7-2022     HaoLion(郝亮亮)    1. done
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
  * @brief          温湿度数据处理
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void tem_hum_task(void const * argument);

extern float get_cabin_humidity(void);

extern float get_cabin_temperature(void);

#endif
