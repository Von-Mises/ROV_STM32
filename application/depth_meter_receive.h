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

#ifndef DEPTH_METER_RECEIVE_H
#define DEPTH_METER_RECEIVE_H

#include "struct_typedef.h"


#define DM_FRAME_LENGTH 	32
#define DM_CMD_LENGTH 		16
	

/* ----------------------- Data Struct ------------------------------------- */



/* ----------------------- extern Function ----------------------------------- */

extern void DM_init(void);
extern void DM_reset(void);
extern float get_depth_data(void);

#endif
