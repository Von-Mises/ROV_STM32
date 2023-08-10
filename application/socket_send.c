/**
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  * @file       socket_send.c/h
  * @brief      tcp��������ÿ10ms����λ������һ��״̬
	*							������֧�ַ����ı���Ϣ���ڵ��ԣ��ı���Ϣ���ᱻ��������Ϣ������
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
#define TCP_QUEUE_LEN 10 /* ���еĳ��ȣ����ɰ������ٸ���Ϣ */
#define TCP_QUEUE_SIZE 4 /* ������ÿ����Ϣ��С���ֽڣ� */

//��������ָ��
char *send_data;

typedef struct{
	uint8_t len;			//���ݳ���
	uint8_t pdata[];			//����ָ��
}queue_data_t;

ext_rov_status_t tcp_rov_status;

/**
  * @brief          deal with message that from queue
  * @param[in]      none
  * @retval         lenth of tcp send data
  */
/**
	* @brief          ����Ϣ���ж�ȡ��Ϣ��ͨ��tcp�������
  * @param[in]      none 
  * @retval         tcp �������ݵĳ���
  */
static uint16_t msg_consume(void);

/**
  * @brief          rov some measure data updata, such as motor speed, euler angle��status of cabin
  * @param[out]     rov_status_updata: "ext_rov_status_t" valiable point
  * @retval         none
  */
/**
  * @brief          ROV״̬���ݸ��£���������ٶȣ�ŷ���Ƕȣ�����״̬��
  * @param[out]     rov_status_updata:"ext_rov_status_t"����ָ��.
  * @retval         none
  */
static void rov_status_update(ext_rov_status_t *rov_status_updata);

/**
  * @brief          send status of rov
  * @param[out]     none
  * @retval         lenth of tcp send data
  */
/**
	* @brief          ����rov״̬
  * @param[out]     none
  * @retval         tcp �������ݵĳ���
  */
static uint16_t rov_send_status(void);

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
void server_send_task(void const * argument)
{
	int sock = -1,connected;
	int send_data_len;
	struct sockaddr_in server_addr,client_addr;
	socklen_t sin_size;
	/*��ʼ���ṹ��*/
	init_send_struct_data();
	/*��ȡBufferָ��*/
	send_data = (char *)get_text_buffer();
	
	/* ����Msg_Queue */
	Tcp_Msg_Queue = xQueueCreate((UBaseType_t ) TCP_QUEUE_LEN,/* ��Ϣ���еĳ��� */
													(UBaseType_t ) TCP_QUEUE_SIZE);/* ��Ϣ�Ĵ�С */
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
	/*��������Ϣ*/
	if (bind(sock, (struct sockaddr *)&server_addr, sizeof(struct sockaddr)) == -1)
	{
		printf("Unable to bind in send task\n");
		goto __exit;
	}
	/*�������������״̬*/
	if (listen(sock, 5) == -1)
	{
		printf("Listen error in send task\n");
		goto __exit;
	}
	while(1)
	{
		sin_size = sizeof(struct sockaddr_in);
		/*�ȴ��������󣬻��������߳�ֱ������TCP����*/
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
			/*40ms�ϴ�һ������*/
			rov_status_update(&tcp_rov_status);
			send_data_len = rov_send_status();
			if(write(connected,send_data,send_data_len) < 0)
					break;
			osDelay(40);
		}
		/*�ͻ�����Ϣ����ʧ�ܣ��Ͽ�����*/
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
	* @brief          ���͵�����Ϣ
  * @param[in]      data �ı���Ϣ
  * @param[in]      tlen 	�ı�����
  * @retval         none
  */
void msg_send(const uint8_t* data, uint16_t tlen)
{
	if(Tcp_Msg_Queue == NULL)
		return;
	//���������ڴ棬��Ҫһ���ֽڱ������ݵĳ��ȣ��������ݳ���+1
	queue_data_t* queue_data = (queue_data_t *)pvPortMalloc(tlen+1);
	queue_data->len = tlen;
	//������Ҫ���͵�����
	memcpy(queue_data->pdata, data, tlen);
	//�ѽṹ��ָ�뱣�浽address
	int address;
	address = (int)queue_data;
	//����queue_data��ָ�뵽���У����ȴ�
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
	* @brief          ����Ϣ���ж�ȡ��Ϣ��ͨ��tcp�������
  * @param[in]      none 
  * @retval         tcp �������ݵĳ���
  */
static uint16_t msg_consume()
{
	int address;
	uint16_t tcp_len;
	//�ȴ����ն�������
	if (xQueueReceive(Tcp_Msg_Queue, (void *)&address, 0) == pdTRUE)
	{
		//��ָ��ת��Ϊ�ṹ��
		queue_data_t *queue_data;
		queue_data = (queue_data_t *)address;
		tcp_len = send_data_pack(TEXT_MSG_ID,queue_data->pdata,queue_data->len);
//		printf("queue data:%s\r\n",queue_data->data);
		vPortFree(queue_data);
		return tcp_len;
	}
	else
	{
		//��ȡ��Ϣʧ��
		return 0;
	}
}

/**
  * @brief          rov some measure data updata, such as motor speed, euler angle��status of cabin
  * @param[out]     rov_status_updata: "ext_rov_status_t" valiable point
  * @retval         none
  */
/**
  * @brief          ROV״̬���ݸ��£���������ٶȣ�ŷ���Ƕȣ�����״̬��
  * @param[out]     rov_status_updata:"ext_rov_status_t"����ָ��.3
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
	/*��δ�����*/
}

/**
  * @brief          send status of rov
  * @param[out]     none
  * @retval         lenth of tcp send data
  */
/**
	* @brief          ����rov״̬
  * @param[out]     none
  * @retval         tcp �������ݵĳ���
  */
static uint16_t rov_send_status(void)
{
	return send_data_pack(ROV_STATUS_ID,(uint8_t*)&tcp_rov_status,sizeof(ext_rov_status_t));
}
