/**
  ****************************(C) COPYRIGHT 2023 ZJU****************************
  * @file       bsp_init.c/h
  * @brief      ��ʼ�����صĸ������裬��Ϊ��Щ������ܻ������񴴽�ʱ���ʼ��������
  *             ������Ҫ��ʼ��һЩ�ж���������裬���紮�ڡ�CAN��
  *             
  * @note       bsp_init()Ӧ�÷���HAL���ʼ������֮��
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     MAR-21-2023     HaoLion(������)    1. done
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
	* @brief          ��ʼ���ж��������������
	* @param[in]      none
  * @retval         none
  */
	
void bsp_init()
{
	
    usart1_tx_dma_init();
	/*IMU*/
	IMU_init();
	
	/*��ȼ�*/
	DM_init();
	
	/*�Ĵ����*/
	Track_Motor_Init();
	Pulse_Fb_Init();
	
	/*�ƽ���*/
	//�ȳ�ʼ��can,�ٳ�ʼ���ƽ���
	can_filter_init();
	Thruster_Init();
	/*�ƹ�Ͷ��*/
	pwm_init();
	/*Add More Devices*/
	/*�߶ȼ�*/
	//AM_init();
}

