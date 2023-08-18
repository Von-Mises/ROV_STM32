/**
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  * @file       IMU_receive.c/h
  * @brief      解析IMU发送过来的数据,利用DMA传输方式节约CPU
  *             资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，串口
  *             的方式保证热插拔的稳定性。
  * @note       该任务是通过串口中断启动，不是freeRTOS任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     NOV-23-2022     Qiqi Li(李琪琪)    1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  */

#include "IMU_receive.h"
#include "main.h"
#include "bsp_usart.h"
#include "string.h"
#include "CRC8_CRC16.h"
#include "config.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;


//接收原始数据，为40个字节，给了80个字节长度，防止DMA传输越界
static uint8_t imu_rx_buf[2][IMU_FRAME_LENGTH];

uint8_t imu_tx_buf[IMU_CMD_LENGTH]={0};

//IMU  data 
//九轴数据
IMU_data_t imu_data;

#if IMU_DEBUG
/**
  * @brief          send sbus data by usart1, called in usart3_IRQHandle
  * @param[in]      sbus: sbus data, 18 bytes
  * @retval         none
  */
/**
  * @brief          通过usart1发送sbus数据,在usart3_IRQHandle调用
  * @param[in]      imu_data: sbus数据, 40字节
  * @retval         none
  */
static void imu_to_usart1(uint8_t *imu_data);
#endif

/**
  * @brief          IMU init
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          IMU初始化
  * @param[in]      none
  * @retval         none
  */
void IMU_init(void)
{
    usart3_init(imu_rx_buf[0], imu_rx_buf[1], IMU_FRAME_LENGTH);
	IMU_Set_Cmd(START_CMD_ID);
}


/**
  * @brief          IMU set parameter
  * @param[in]      imu_cmd_id tpye of cmd
  * @retval         none
  */
/**
  * @brief          IMU属性设置
  * @param[in]      imu_cmd_id 命令标识
  * @retval         none
  */
void IMU_Set_Cmd(imu_cmd_id_e imu_cmd_id)
{
	imu_tx_buf[0] = IMU_HEADER_SOF>>8;
	imu_tx_buf[1] = IMU_HEADER_SOF&0xff;
	imu_tx_buf[2]	= 0x04;
	imu_tx_buf[3] = imu_cmd_id;
	imu_tx_buf[4] = imu_cmd_id+0x04;
	imu_tx_buf[5] = 0xAA;
	usart3_tx_dma_enable(imu_tx_buf,IMU_CMD_LENGTH);
}

/**
  * @brief          IMU protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     imu_data: IMU data struct point
  * @retval         none
  */
/**
  * @brief          IMU协议解析
  * @param[in]      imu_frame: 原生数据指针
  * @param[out]     imu_data: IMU数据指针
  * @retval         none
  */
extern void imu_data_solve(volatile const uint8_t *imu_frame, IMU_data_t *imu_data);


/**
  * @brief          get  imu data point
  * @param[in]      none
  * @retval         imu data point
  */
/**
  * @brief          获取IMU数据指针
  * @param[in]      none
  * @retval         IMU数据指针
  */
const IMU_data_t *get_imu_data_point(void)
{
    return &imu_data;
}


/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
	if(huart3.Instance->ISR & UART_FLAG_RXNE)//接收到数据
	{
			__HAL_UART_CLEAR_PEFLAG(&huart3);
	}
	else if(USART3->ISR & UART_FLAG_IDLE)
	{
		static uint16_t this_time_rx_len = 0;
		__HAL_UART_CLEAR_IDLEFLAG(&huart3);
		SCB_InvalidateDCache_by_Addr ((uint32_t *)imu_rx_buf, 2*IMU_FRAME_LENGTH);
		if ((((DMA_Stream_TypeDef   *)hdma_usart3_rx.Instance)->CR & DMA_SxCR_CT) == RESET)
			{
					/* Current memory buffer used is Memory 0 */

					//disable DMA
					//失效DMA
					__HAL_DMA_DISABLE(&hdma_usart3_rx);

					//get receive data length, length = set_data_length - remain_length
					//获取接收数据长度,长度 = 设定长度 - 剩余长度
					this_time_rx_len = IMU_FRAME_LENGTH - ((DMA_Stream_TypeDef   *)hdma_usart3_rx.Instance)->NDTR;
						
					//reset set_data_lenght
					//重新设定数据长度
					((DMA_Stream_TypeDef   *)hdma_usart3_rx.Instance)->NDTR = IMU_FRAME_LENGTH;

					//set memory buffer 1
					//设定缓冲区1
					((DMA_Stream_TypeDef   *)hdma_usart3_rx.Instance)->CR |= DMA_SxCR_CT;
					
					//enable DMA
					//使能DMA
					__HAL_DMA_ENABLE(&hdma_usart3_rx);
					if(this_time_rx_len == IMU_DATA_LENGTH)
					{
							imu_data_solve(imu_rx_buf[0], &imu_data);
							#if IMU_DEBUG
							imu_to_usart1(imu_rx_buf[0]);
							#endif
					}	
				}
			else
			{
					/* Current memory buffer used is Memory 1 */
					//disable DMA
					//失效DMA
					__HAL_DMA_DISABLE(&hdma_usart3_rx);

					//get receive data length, length = set_data_length - remain_length
					//获取接收数据长度,长度 = 设定长度 - 剩余长度
					//this_time_rx_len = IMU_FRAME_LENGTH - ((DMA_Stream_TypeDef   *)hdma_usart3_rx.Instance)->NDTR;
					this_time_rx_len = IMU_FRAME_LENGTH - ((DMA_Stream_TypeDef   *)hdma_usart3_rx.Instance)->NDTR;

					//reset set_data_lenght
					//重新设定数据长度
					((DMA_Stream_TypeDef   *)hdma_usart3_rx.Instance)->NDTR = IMU_FRAME_LENGTH;

					//set memory buffer 0
					//设定缓冲区0
					((DMA_Stream_TypeDef   *)hdma_usart3_rx.Instance)->CR &= ~(DMA_SxCR_CT);
					
					//enable DMA
					//使能DMA
					__HAL_DMA_ENABLE(&hdma_usart3_rx);
					if(this_time_rx_len == IMU_DATA_LENGTH)
					{
							imu_data_solve(imu_rx_buf[1], &imu_data);
							#if IMU_DEBUG
							imu_to_usart1(imu_rx_buf[1]);
							#endif
					}
					
			}
		
	}
	
}


/**
  * @brief          IMU protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     imu_data: IMU data struct point
  * @retval         none
  */
/**
  * @brief          IMU协议解析
  * @param[in]      imu_frame: 原生数据指针
  * @param[out]     imu_data: IMU数据指针
  * @retval         none
  */
static void imu_data_solve(volatile const uint8_t *imu_frame, IMU_data_t *imu_data)
{
	/*数据校验*/
    if (imu_frame == NULL || imu_frame == NULL)
    {
        return;
    }
		if (((imu_frame[0]<<8)|imu_frame[1]) != IMU_HEADER_SOF )
		{
				return;
		}
		if (imu_frame[2] != 0x25)
		{
			return;
		}
		if(imu_frame[IMU_DATA_LENGTH-1] != DATA_EOF)
		{
			return;
		}
		/*数据解析*/
		imu_data->yaw = (int16_t)(((imu_frame[3]<<8) + imu_frame[4]))*0.1;						//俯仰角，单位为°
		imu_data->pitch = (int16_t)(((imu_frame[5]<<8) + imu_frame[6]))*0.1;							//偏航角
		imu_data->roll = (int16_t)(((imu_frame[7]<<8) + imu_frame[8]))*0.1;							//横滚角
		imu_data->ax = (int16_t)(((imu_frame[9]<<8) + imu_frame[10]))/16384;						//X方向加速度，单位为g
		imu_data->ay = (int16_t)(((imu_frame[11]<<8) + imu_frame[12]))/16384;						//y方向加速度
		imu_data->az = (int16_t)(((imu_frame[13]<<8) + imu_frame[14]))/16384;						//z方向加速度
		imu_data->gx = (int16_t)(((imu_frame[15]<<8) + imu_frame[16]))/32.8;						//X方向角速度，单位为°/s
		imu_data->gy = (int16_t)(((imu_frame[17]<<8) + imu_frame[18]))/32.8;						//y方向角速度
		imu_data->gz = (int16_t)(((imu_frame[19]<<8) + imu_frame[20]))/32.8;						//z方向角速度
		imu_data->mx = (int16_t)(((imu_frame[21]<<8) + imu_frame[22]))/333.3;						//X方向磁场，单位为Gs
		imu_data->my = (int16_t)(((imu_frame[23]<<8) + imu_frame[24]))/333.3;						//y方向磁场
		imu_data->mz = (int16_t)(((imu_frame[25]<<8) + imu_frame[26]))/333.3;						//z方向磁场
		imu_data->temperate = (int16_t)(((imu_frame[31]<<8) + imu_frame[32]))/333.3;		//IMU温度，单位为℃ 
}


#if IMU_DEBUG
/**
  * @brief          send sbus data by usart1, called in usart3_IRQHandle
  * @param[in]      sbus: sbus data, 18 bytes
  * @retval         none
  */
/**
  * @brief          通过usart1发送sbus数据,在usart3_IRQHandle调用
  * @param[in]      imu_data: sbus数据, 40字节
  * @retval         none
  */
void imu_to_usart1(uint8_t *imu_data)
{
    static uint8_t usart_tx_buf[42];
    static uint8_t i =0;
    usart_tx_buf[0] = 0xA6;
    memcpy(usart_tx_buf + 1, imu_data, 40);
    for(i = 0, usart_tx_buf[41] = 0; i < 41; i++)
    {
        usart_tx_buf[41] += usart_tx_buf[i];
    }
    usart1_tx_dma_enable(usart_tx_buf, 42);
}
#endif


