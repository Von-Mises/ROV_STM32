/**
  ****************************(C) COPYRIGHT 2023 ZJU****************************
  * @file       parse_task.c/h
  * @brief      socket receive data slove.
  *             ��λ�������·������߳�.        
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-5-2023     HaoLion(������)    1. done
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
  * @brief          parse task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          �������ݽ�������
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void parse_task(void const * argument);
#endif
