/**
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  * @file       tem_hum_task.c/h
  * @brief      温湿度传感器数据处理任务         
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     DEC-7-2022     Qiqi Li(李琪琪)    1. done
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
  * @brief          温湿度数据处理
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void tem_hum_task(void const * argument)
{
	//这里IMU需要再复位一次，不知道为什么
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
		/*高度计数据*/
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

