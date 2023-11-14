#ifndef BSP_TIMER_H
#define BSP_TIMER_H

extern volatile unsigned long FreeRTOSRunTimeTicks;

void TIM5_IRQHandler(void);
void Timer_Init(void);

#endif
