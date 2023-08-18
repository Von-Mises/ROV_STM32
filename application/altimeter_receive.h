/**
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  * @file       depth_meter_receive.c/h
  * @brief      解析深度计发送过来的数据,利用DMA传输方式节约CPU
  *             资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，串口
  *             的方式保证热插拔的稳定性。
  * @note       该任务是通过串口中断启动，不是freeRTOS任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     DEC-21-2022     Qiqi Li(李琪琪)    1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  */

#ifndef ALTIMETER_RECEIVE_H
#define ALTIMETER_RECEIVE_H

#include "struct_typedef.h"


#define AM_FRAME_LENGTH 	30
#define AM_DATA_LENGTH 		15
#define AM_CMD_LENGTH 		15
#define AM_AIR_VELOCITY		340
#define AM_WATER_VELOCITY   1500

/* ----------------------- Data Struct ------------------------------------- */



/* ----------------------- extern Function ----------------------------------- */

extern void AM_init(void);
extern float get_height_data(void);
extern void AM_get_data(void);

#endif
