#ifndef BSP_PULSE_H
#define BSP_PULSE_H
#include "struct_typedef.h"


extern void Pulse_Fb_Init(void);

extern void get_crawler_motor_speed(uint16_t* pcrawler_motor_1,uint16_t* pcrawler_motor_2);

extern void track_velocity_measure(void);

#endif
