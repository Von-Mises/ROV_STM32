/**
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  * @file       socket_send.c/h
  * @brief      tcp��������ÿ2ms����λ������һ��״̬         
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     DEC-13-2022     HaoLion(������)    1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  */

#ifndef SOCKET_SEND_H
#define SOCKET_SEND_H

#include "protocol.h"


#define SERVER_IP_ADDR "192.168.1.25"

#define TCP_SERVER_SEND_PORT 5002

#define SEND_DATA (512)

/* ----------------------- Data Struct ------------------------------------- */



/* ----------------------- Extern Function ----------------------------------- */
/**
  * @brief          send debug info 
  * @param[in]      data	message 
  * @param[in]      length of message
  * @retval         none
  */
/**
	* @brief          ���͵�����Ϣ
  * @param[in]      data �ı���Ϣ
  * @param[in]      tlen 	�ı�����
  * @retval         none
  */
extern void msg_send(const uint8_t* data, uint16_t tlen);

/**
  * @brief          server_send_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
	* @brief          TCP��������
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void server_send_task(void const * argument);

#endif
