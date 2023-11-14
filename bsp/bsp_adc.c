#include "bsp_adc.h"
#include "main.h"
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc3;

volatile uint16_t VREFINT_CAL;

volatile fp32 voltage_vdda_real = 2.5f;		//

static uint16_t adcx_get_chx_value(ADC_HandleTypeDef *ADCx, uint32_t ch)
{
    static ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ch;
    sConfig.Rank = 1;
	sConfig.SingleDiff=ADC_SINGLE_ENDED;  				//单边采集          		
	sConfig.OffsetNumber=ADC_OFFSET_NONE;
	sConfig.Offset=0;
    sConfig.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;//ADC_SAMPLETIME_2CYCLES_5;

    if (HAL_ADC_ConfigChannel(ADCx, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_ADC_Start(ADCx);

    HAL_ADC_PollForConversion(ADCx, 10);
    return (uint16_t)HAL_ADC_GetValue(ADCx);

}

/**
	* @brief        初始化AD采样
  * @param[in]      none 
  * @retval         none
  */
void init_vrefint_reciprocal(void)
{
		VREFINT_CAL = *(__IO uint16_t *)(0x1FF1E860);
		HAL_ADCEx_Calibration_Start(&hadc1,ADC_CALIB_OFFSET,ADC_SINGLE_ENDED);
		HAL_ADCEx_Calibration_Start(&hadc3,ADC_CALIB_OFFSET,ADC_SINGLE_ENDED);

}
fp32 sampling_cpu_temprate(void)
{
    uint16_t adcx = 0;
    fp32 temperate;
	uint16_t TS_CAL1;
	uint16_t TS_CAL2;
    adcx = adcx_get_chx_value(&hadc3, ADC_CHANNEL_TEMPSENSOR);
    TS_CAL1 = *(__IO uint16_t *)(0x1FF1E820);
	TS_CAL2 = *(__IO uint16_t *)(0x1FF1E840);
	temperate = ((110.0f - 30.0f) / (TS_CAL2 - TS_CAL1)) * ((adcx/3.3f*2.5f) - TS_CAL1) + 30.0f;

    return temperate;
}



uint16_t adcx = 0;
fp32 get_water_gage(void)
{
    fp32 voltage;
		
//	  voltage_vdda_real = 3.3*VREFINT_CAL/(2.0*adcx_get_chx_value(&hadc3, ADC_CHANNEL_VREFINT));
		//分压电路所以实际电压值应该×2
		adcx = adcx_get_chx_value(&hadc1, ADC_CHANNEL_2);
    voltage =  (fp32)adcx * voltage_vdda_real /65535 * 2.0f;
    return voltage;
}
