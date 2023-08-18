/**
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  * @file       water_level_task.c/h
  * @brief      水位计检测任务（加入电压基准芯片后发现有问题，所以取消这个任务）     
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     DEC-7-2022     Qiqi Li(李琪琪)    1. done
  *  V1.0.1     MAR-29-2023    Qiqi Li(李琪琪)   	2. over
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  */

#include "water_level_task.h"
#include "main.h"
#include "bsp_adc.h"
#include "cmsis_os.h"

#define FULL_WATER_VOLTAGE     5.0f

fp32 water_level_voltage;
fp32 waterlevel_percentage;
fp32 cpu_temperature;

static  fp32 calc_percentage(float voltage);

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
void water_level_task(void const * argument)
{
    osDelay(1000);
    //use inner 1.2v to calbrate
    init_vrefint_reciprocal();
    while(1)
    {
        water_level_voltage = get_water_gage();
		waterlevel_percentage=calc_percentage(water_level_voltage);
		cpu_temperature = sampling_cpu_temprate();
        osDelay(100);
    }
}

static fp32 calc_percentage(float voltage)
{
    fp32 percentage;
    
	percentage = voltage/FULL_WATER_VOLTAGE;
    return percentage;
}

float get_water_level_percentage(void)
{
    return (float)(waterlevel_percentage * 100.0f);
}

float get_cpu_temperature(void)
{
    return (float)(cpu_temperature);
}

