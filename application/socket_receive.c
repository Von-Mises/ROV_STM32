/**
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  * @file       socket_receive.c/h
  * @brief      创建tcp线程，解析上位机数据,该任务的优先级应被置为最高         
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     DEC-7-2022     HaoLion(郝亮亮)    1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  */

#include "socket_receive.h"
#include "main.h"
#include "string.h"
#include "comunication.h"
#include "CRC8_CRC16.h"
#include "config.h"
#include "bsp_usart.h"
#include "parse_task.h"

#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/api.h"
#include <lwip/sockets.h>


extern fifo_s_t tcp_fifo;

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
void server_receive_task(void const * argument)
{
	int sock = -1,connected;
	uint8_t *recv_data;
	struct sockaddr_in server_addr,client_addr;
	socklen_t sin_size;
	int recv_data_len;
	/*初始化结构体*/
	init_recieve_struct_data();
	/*申请内存*/
	recv_data = (uint8_t *)pvPortMalloc(RECV_DATA);
	if (recv_data == NULL)
	{
		printf("No memory in recive task\n");
		goto __exit;
	}
	sock = socket(AF_INET, SOCK_STREAM, 0);
	if (sock < 0)
	{
		printf("Socket error in recive task\n");
		goto __exit;
	}
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.s_addr = INADDR_ANY;
	server_addr.sin_port = htons(TCP_SERVER_RECIEVE_PORT);
	memset(&(server_addr.sin_zero), 0, sizeof(server_addr.sin_zero));
	/*绑定网卡信息*/
	if (bind(sock, (struct sockaddr *)&server_addr, sizeof(struct sockaddr)) == -1)
	{
		printf("Unable to bind in recive task\n");
		goto __exit;
	}
	/*服务器进入监听状态*/
	if (listen(sock, 5) == -1)
	{
		printf("Listen error in recive task\n");
		goto __exit;
	}
	while(1)
	{
		sin_size = sizeof(struct sockaddr_in);
		/*等待连接请求，会阻塞该线程直至建立TCP连接*/
		connected = accept(sock, (struct sockaddr *)&client_addr, &sin_size);
		printf("Recive task has new client connected from (%s, %d)\n",
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
			recv_data_len = recv(connected, recv_data, RECV_DATA, 0);
			if (recv_data_len <= 0)
				break;
//			printf("recv %d len data\n",recv_data_len);
			#if IP_DEBUG
			ip_to_usart1(recv_data,recv_data_len);
			#endif
			/*判断队列是否已满，满则清空缓存*/
			if(fifo_s_isfull(&tcp_fifo))
			{
				fifo_s_flush(&tcp_fifo);
				printf("fifo is full\n");
			}
			/*存入FIFO*/
			fifo_s_puts(&tcp_fifo,(char*)recv_data, recv_data_len);
			
		}
		/*客户端消息发送失败，断开连接*/
		if (connected >= 0)
			closesocket(connected);
		connected = -1;
	}	
__exit:
	if (sock >= 0) closesocket(sock);
	if (recv_data) free(recv_data);
}


#if IP_DEBUG
/**
  * @brief          通过usart1发送网络数据
  * @param[in]      ip_data: sbus数据
  * @param[in]      dat_len: 网络包长度
  * @retval         none
  */
static void ip_to_usart1(uint8_t *ip_data,uint8_t dat_len)
{
    static uint8_t usart_tx_buf[120];
    memcpy(usart_tx_buf , ip_data, dat_len);
		usart_tx_buf[dat_len] = '\n';
    usart1_tx_dma_enable(usart_tx_buf,dat_len+1);
}
#endif



