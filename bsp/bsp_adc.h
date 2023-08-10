#ifndef BSP_ADC_H
#define BSP_ADC_H
#include "struct_typedef.h"

extern void init_adc(void);
extern fp32 get_water_gage(void);
extern fp32 sampling_cpu_temprate(void);
/**
	* @brief          初始化AD采样，采集内部参考电压200次取平均
  * @param[in]      none 
  * @retval         none
  */
extern void init_vrefint_reciprocal(void);
#endif
