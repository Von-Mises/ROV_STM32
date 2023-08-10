#ifndef BSP_PWM_H
#define BSP_PWM_H
#include "struct_typedef.h"

#define SERVO_MID_VAL 375

extern void pwm_init(void);

extern void servo_pwm_set(uint16_t pwm);

extern void bright_pwm_set(uint16_t pwm);

#endif
