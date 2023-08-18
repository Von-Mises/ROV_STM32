/**
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  * @file       socket_send.c/h
  * @brief      tcp发送任务，每2ms向上位机更新一次状态         
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     DEC-13-2022     Qiqi Li(李琪琪)    1. done
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
#include "comunication.h"

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
	* @brief        发送调试信息
  * @param[in]      data 文本消息
  * @param[in]      tlen 	文本长度
  * @retval         none
  */
extern void msg_send(char* data, uint16_t tlen);

/**
  * @brief          server_send_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
	* @brief          TCP发送任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void server_send_task(void const * argument);

/**
  * @brief          deal with message that from queue
  * @param[in]      none
  * @retval         lenth of tcp send data
  */
/**
	* @brief          从消息队列读取消息，通过tcp打包发送
  * @param[in]      none 
  * @retval         tcp 发送数据的长度
  */
static uint16_t msg_consume(void);

/**
  * @brief          rov some measure data updata, such as motor speed, euler angle，status of cabin
  * @param[out]     rov_status_updata: "ext_rov_status_t" valiable point
  * @retval         none
  */
/**
  * @brief          ROV状态数据更新，包括电机速度，欧拉角度，舱内状态等
  * @param[out]     rov_status_updata:"ext_rov_status_t"变量指针.
  * @retval         none
  */
static void rov_status_update(ext_rov_status_t *rov_status_updata);

/**
  * @brief          send status of rov
  * @param[out]     none
  * @retval         lenth of tcp send data
  */
/**
	* @brief          发送rov状态
  * @param[out]     none
  * @retval         tcp 发送数据的长度
  */
static uint16_t rov_send_status(void);

/**
  * @brief          send information of rov
  * @param[out]     none
  * @retval         lenth of tcp send data
  */
/**
  * @brief          发送rov运行信息
  * @param[out]     none
  * @retval         tcp 发送数据的长度
  */
int socket_printf(char* format, ...);

#endif
