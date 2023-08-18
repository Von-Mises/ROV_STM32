/**
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  * @file       tem_hum_task.c/h
  * @brief      ��ʪ�ȴ��������ݴ�������         
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     DEC-7-2022     Qiqi Li(������)    1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  */

#include <stdio.h>
#include "tem_hum_task.h"
#include "main.h"
#include "bsp_i2c.h"
#include "sth31driver.h"
#include "cmsis_os.h"
#include "socket_send.h"
#include "altimeter_receive.h"
#include "thruster.h"
#include "IMU_receive.h"

float temperature;
float humidity;

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
void tem_hum_task(void const * argument)
{
	//����IMU��Ҫ�ٸ�λһ�Σ���֪��Ϊʲô
	IMU_init();
    SHT31_Init();
    while(1)
    {
        SHT31_Read_Dat(&temperature,&humidity);

//		for (int i = 0; i < 6; i++)
//		{
//			CAN_param_request(i);
//			osDelay(5);
//		}
		/*�߶ȼ�����*/
		//AM_get_data();
		osDelay(1000);
    }
}


float get_cabin_temperature(void)
{
    return (float)(temperature);
}

float get_cabin_humidity(void)
{
    return (float)(humidity);
}

