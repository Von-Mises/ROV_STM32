/**
  ****************************(C) COPYRIGHT 2023 ZJU****************************
  * @file       parse_task.c/h
  * @brief      socket receive data slove.
  *             上位机数据下发处理线程.        
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-5-2023     Qiqi Li(李琪琪)    1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 ZJU****************************
  */
#ifndef PARSE_TASK_H
#define PARSE_TASK_H

#include "main.h"
#include "fifo.h"

#define TCP_FIFO_BUF_LENGTH 2048
//extern void referee_init(void);

extern fifo_s_t tcp_fifo;

/**
  * @brief          tcp data upacked 
  * @param[in]      data	data of frame
  * @param[in]      length of data
  * @retval         none
  */
/**
  * @brief          数据解包
  * @param[in]      data 帧原始数据
  * @param[in]      framelen 帧长度
  * @retval         none
  */
static void tcp_unpack_fifo_data(void);

/**
  * @brief          parse task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          接收数据解析任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void parse_task(void const * argument);
#endif
