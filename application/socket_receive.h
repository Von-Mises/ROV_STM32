/**
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  * @file       socket_receive.c/h
  * @brief      创建tcp线程，解析上位机数据,该任务的优先级应被置为最高         
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     DEC-7-2022     Qiqi Li(李琪琪)    1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  */

#ifndef SOCKET_RECEIVE_H
#define SOCKET_RECEIVE_H

#include "protocol.h"


#define SERVER_IP_ADDR "192.168.1.25"

#define TCP_SERVER_RECIEVE_PORT 5001

#define RECV_DATA (1024)

/* ----------------------- Data Struct ------------------------------------- */



/* ----------------------- Extern Function ----------------------------------- */

#if IP_DEBUG
/**
  * @brief          通过usart1发送网络数据
  * @param[in]      ip_data: sbus数据
  * @param[in]      dat_len: 网络包长度
  * @retval         none
  */
static void ip_to_usart1(uint8_t *ip_data,uint8_t dat_len);
#endif

/**
  * @brief          server_receive_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          TCP服务器通讯任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void server_receive_task(void const * argument);



#endif
