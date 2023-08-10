/**
  ****************************(C) COPYRIGHT 2023 ZJU****************************
  * @file       bsp_init.c/h
  * @brief      初始化板载的各种外设，因为有些外设可能会在任务创建时候初始化，所以
  *             这里主要初始化一些中断任务的外设，比如串口、CAN等
  *             
  * @note       bsp_init()应该放在HAL库初始化代码之后
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     MAR-21-2023     HaoLion(郝亮亮)    1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 ZJU****************************
  */


#include "bsp_init.h"
#include "bsp_can.h"
#include "IMU_receive.h"
#include "thruster.h"
#include "bsp_dac.h"
#include "bsp_pulse.h"
#include "bsp_pwm.h"
#include "bsp_usart.h"
#include "depth_meter_receive.h"
#include "altimeter_receive.h"
#include "config.h"
#include "main.h"

/**
	* @brief          初始化中断任务的所有外设
	* @param[in]      none
  * @retval         none
  */
	
void bsp_init()
{
	
    usart1_tx_dma_init();
	/*IMU*/
	IMU_init();
	
	/*深度计*/
	DM_init();
	
	/*履带电机*/
	Track_Motor_Init();
	Pulse_Fb_Init();
	
	/*推进器*/
	//先初始化can,再初始化推进器
	can_filter_init();
	Thruster_Init();
	/*灯光和舵机*/
	pwm_init();
	/*Add More Devices*/
	/*高度计*/
	//AM_init();
}

