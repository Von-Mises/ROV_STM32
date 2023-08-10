/**
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  * @file       socket_receive.c/h
  * @brief      ����tcp�̣߳�������λ������,����������ȼ�Ӧ����Ϊ���         
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     DEC-7-2022     HaoLion(������)    1. done
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

/**
  * @brief          server_receive_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          TCP������ͨѶ����
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void server_receive_task(void const * argument);



#endif
