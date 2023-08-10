/**
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  * @file       water_level_task.c/h
  * @brief      水位计检测任务         
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

#ifndef WATER_LEVEL_TASK_H
#define WATER_LEVEL_TASK_H

#include "struct_typedef.h"




/* ----------------------- Data Struct ------------------------------------- */



/* ----------------------- Extern Function ----------------------------------- */
/**
  * @brief          water level ADC
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          采样水位计
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void water_level_task(void const * argument);

extern float get_water_level_percentage(void);

extern float get_cpu_temperature(void);
#endif
