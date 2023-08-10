/**
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  * @file       socket_send.c/h
  * @brief      tcp发送任务，每10ms向上位机更新一次状态
	*							还可以支持发送文本消息便于调试，文本消息将会被发送在消息队列中
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     DEC-13-2022     HaoLion(郝亮亮)    1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  */

#include "socket_send.h"
#include "main.h"
#include "string.h"
#include "comunication.h"
#include "CRC8_CRC16.h"
#include "bsp_pulse.h"
#include "water_level_task.h"
#include "tem_hum_task.h"
#include "thruster.h"
#include "IMU_receive.h"
#include "rov_movation_task.h"
#include "depth_meter_receive.h"
#include "altimeter_receive.h"
#include "rov_behaviour.h"
#include <stdio.h>

#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/api.h"
#include <lwip/sockets.h>
#include "queue.h"

QueueHandle_t Tcp_Msg_Queue =NULL;
#define TCP_QUEUE_LEN 10 /* 队列的长度，最大可包含多少个消息 */
#define TCP_QUEUE_SIZE 4 /* 队列中每个消息大小（字节） */

//发送数据指针
char *send_data;

typedef struct{
	uint8_t len;			//数据长度
	uint8_t pdata[];			//数据指针
}queue_data_t;

ext_rov_status_t tcp_rov_status;

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
  * @brief          server_send_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
	* @brief          TCP发送任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void server_send_task(void const * argument)
{
	int sock = -1,connected;
	int send_data_len;
	struct sockaddr_in server_addr,client_addr;
	socklen_t sin_size;
	/*初始化结构体*/
	init_send_struct_data();
	/*获取Buffer指针*/
	send_data = (char *)get_text_buffer();
	
	/* 创建Msg_Queue */
	Tcp_Msg_Queue = xQueueCreate((UBaseType_t ) TCP_QUEUE_LEN,/* 消息队列的长度 */
													(UBaseType_t ) TCP_QUEUE_SIZE);/* 消息的大小 */
	if (send_data == NULL)
	{
		printf("No memory in send task\n");
		goto __exit;
	}
	sock = socket(AF_INET, SOCK_STREAM, 0);
	if (sock < 0)
	{
		printf("Socket error in send task\n");
		goto __exit;
	}
	if (Tcp_Msg_Queue == NULL)
	{
		printf("Queue create failed in send task\n");
		goto __exit;
	}
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.s_addr = INADDR_ANY;
	server_addr.sin_port = htons(TCP_SERVER_SEND_PORT);
	memset(&(server_addr.sin_zero), 0, sizeof(server_addr.sin_zero));
	/*绑定网卡信息*/
	if (bind(sock, (struct sockaddr *)&server_addr, sizeof(struct sockaddr)) == -1)
	{
		printf("Unable to bind in send task\n");
		goto __exit;
	}
	/*服务器进入监听状态*/
	if (listen(sock, 5) == -1)
	{
		printf("Listen error in send task\n");
		goto __exit;
	}
	while(1)
	{
		sin_size = sizeof(struct sockaddr_in);
		/*等待连接请求，会阻塞该线程直至建立TCP连接*/
		connected = accept(sock, (struct sockaddr *)&client_addr, &sin_size);
		printf("Send task has new client connected from (%s, %d)\n",
		inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));
		{
			int flag = 1;
			setsockopt(connected,
								IPPROTO_TCP, /* set option at TCP level */
								TCP_NODELAY, /* name of option */
								(void *) &flag, /* the cast is historical cruft */
								sizeof(int)); /* length of option value */
		}
		while (1)
		{
			send_data_len = msg_consume();
			if(send_data_len > 0)
			{
				if(write(connected,send_data,send_data_len) < 0)
					break;
			}
			osDelay(10);
			/*40ms上传一次数据*/
			rov_status_update(&tcp_rov_status);
			send_data_len = rov_send_status();
			if(write(connected,send_data,send_data_len) < 0)
					break;
			osDelay(40);
		}
		/*客户端消息发送失败，断开连接*/
		if (connected >= 0)
			closesocket(connected);
		connected = -1;
	}	
__exit:
	if (sock >= 0) closesocket(sock);
	if (send_data) free(send_data);
	if (Tcp_Msg_Queue == NULL) vQueueDelete(Tcp_Msg_Queue);
}




/**
  * @brief          send debug info 
  * @param[in]      data	message 
  * @param[in]      length of message
  * @retval         none
  */
/**
	* @brief          发送调试信息
  * @param[in]      data 文本消息
  * @param[in]      tlen 	文本长度
  * @retval         none
  */
void msg_send(const uint8_t* data, uint16_t tlen)
{
	if(Tcp_Msg_Queue == NULL)
		return;
	//创建队列内存，需要一个字节保存数据的长度，所以数据长度+1
	queue_data_t* queue_data = (queue_data_t *)pvPortMalloc(tlen+1);
	queue_data->len = tlen;
	//填入需要发送的数据
	memcpy(queue_data->pdata, data, tlen);
	//把结构体指针保存到address
	int address;
	address = (int)queue_data;
	//保存queue_data的指针到队列，不等待
	if (xQueueSend(Tcp_Msg_Queue, (void *)&address, 0) == pdFALSE)
	{
		vPortFree(queue_data);
		return;
	}
}


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
static uint16_t msg_consume()
{
	int address;
	uint16_t tcp_len;
	//等待接收队列数据
	if (xQueueReceive(Tcp_Msg_Queue, (void *)&address, 0) == pdTRUE)
	{
		//把指针转换为结构体
		queue_data_t *queue_data;
		queue_data = (queue_data_t *)address;
		tcp_len = send_data_pack(TEXT_MSG_ID,queue_data->pdata,queue_data->len);
//		printf("queue data:%s\r\n",queue_data->data);
		vPortFree(queue_data);
		return tcp_len;
	}
	else
	{
		//读取消息失败
		return 0;
	}
}

/**
  * @brief          rov some measure data updata, such as motor speed, euler angle，status of cabin
  * @param[out]     rov_status_updata: "ext_rov_status_t" valiable point
  * @retval         none
  */
/**
  * @brief          ROV状态数据更新，包括电机速度，欧拉角度，舱内状态等
  * @param[out]     rov_status_updata:"ext_rov_status_t"变量指针.3
  * @retval         none
  */
static void rov_status_update(ext_rov_status_t *rov_status_updata)
{
	uint16_t motor_speed_1,motor_speed_2;
	get_crawler_motor_speed(&motor_speed_1, &motor_speed_2);
	rov_status_updata->motors_status.track_speed[0] = motor_speed_1;
	rov_status_updata->motors_status.track_speed[1] = motor_speed_2;
	for(int i=0;i<6;i++)
	{
		rov_status_updata->motors_status.thruster_speed[i] = get_thruster_measure_point(i)->speed_rpm;
	}
	rov_status_updata->cabin_status.cabin_temperature = get_cabin_temperature();
	rov_status_updata->cabin_status.cabin_humidity = get_cabin_humidity();
	rov_status_updata->cabin_status.cabin_water_level = get_water_level_percentage();
	rov_status_updata->cabin_status.cpu_temperature = get_cpu_temperature();
	rov_status_updata->run_mode = get_rov_behaviour();
	
	rov_status_updata->IMU_data = *(get_imu_data_point());
	rov_status_updata->ROV_height = get_height_data();
	rov_status_updata->ROV_depth = get_depth_data();
	/*还未添加完*/
}

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
static uint16_t rov_send_status(void)
{
	return send_data_pack(ROV_STATUS_ID,(uint8_t*)&tcp_rov_status,sizeof(ext_rov_status_t));
}
