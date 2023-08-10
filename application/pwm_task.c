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
  * @brief          舵机和灯光PWM输出
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void pwm_task(void const * argument)
{
	const ext_control_cmd_t *rov_pwm_Ctrl = get_ctrl_cmd();
	uint16_t steering_gear_angle_pwm;//舵机pwm值，范围：125~625
	uint16_t brightness_pwm;//照明灯pwm范围：275~475
	pwm_init();
	while(1)
	{
		/*将数据映射到对应wm范围区间*/
		steering_gear_angle_pwm = (uint16_t)(rov_pwm_Ctrl->steering_gear_angle*25/9.0+375);  
		brightness_pwm = (uint16_t)(rov_pwm_Ctrl->brightness*2+275);
		servo_pwm_set(steering_gear_angle_pwm);
		bright_pwm_set(brightness_pwm);
		osDelay(500);
	}
}


