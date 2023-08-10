/**
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  * @file       pwm_task.c/h
  * @brief      �й�PWM������������������         
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     DEC-14-2022     HaoLion(������)    1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  */

#include <stdio.h>
#include "bsp_pwm.h"
#include "pwm_task.h"
#include "comunication.h"
#include "cmsis_os.h"

/**
  * @brief          servo_motor and light PWM output
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          ����͵ƹ�PWM���
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void pwm_task(void const * argument)
{
	const ext_control_cmd_t *rov_pwm_Ctrl = get_ctrl_cmd();
	uint16_t steering_gear_angle_pwm;//���pwmֵ����Χ��125~625
	uint16_t brightness_pwm;//������pwm��Χ��275~475
	pwm_init();
	while(1)
	{
		/*������ӳ�䵽��Ӧwm��Χ����*/
		steering_gear_angle_pwm = (uint16_t)(rov_pwm_Ctrl->steering_gear_angle*25/9.0+375);  
		brightness_pwm = (uint16_t)(rov_pwm_Ctrl->brightness*2+275);
		servo_pwm_set(steering_gear_angle_pwm);
		bright_pwm_set(brightness_pwm);
		osDelay(500);
	}
}


