/**
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  * @file       pwm_task.c/h
  * @brief      有关PWM波的设置任务在这里         
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     DEC-14-2022     HaoLion(郝亮亮)    1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  */

#ifndef PWM_TASK_H
#define PWM_TASK_H

#include "struct_typedef.h"




/* ----------------------- Data Struct ------------------------------------- */



/* ----------------------- Extern Function ----------------------------------- */
/**
  * @brief          servo_motor and light PWM output
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          舵机和灯光PWM输出
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void pwm_task(void const * argument);

#endif
