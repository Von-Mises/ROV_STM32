/**
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  * @file       IMU_receive.c/h
  * @brief      ����IMU���͹���������,����DMA���䷽ʽ��ԼCPU
  *             ��Դ�����ô��ڿ����ж�������������ͬʱ�ṩһЩ��������DMA������
  *             �ķ�ʽ��֤�Ȳ�ε��ȶ��ԡ�
  * @note       ��������ͨ�������ж�����������freeRTOS����
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     NOV-23-2022     Qiqi Li(������)    1. done
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


//����ԭʼ���ݣ�Ϊ40���ֽڣ�����80���ֽڳ��ȣ���ֹDMA����Խ��
static uint8_t imu_rx_buf[2][IMU_FRAME_LENGTH];

uint8_t imu_tx_buf[IMU_CMD_LENGTH]={0};

//IMU  data 
//��������
IMU_data_t imu_data;

#if IMU_DEBUG
/**
  * @brief          send sbus data by usart1, called in usart3_IRQHandle
  * @param[in]      sbus: sbus data, 18 bytes
  * @retval         none
  */
/**
  * @brief          ͨ��usart1����sbus����,��usart3_IRQHandle����
  * @param[in]      imu_data: sbus����, 40�ֽ�
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
  * @brief          IMU��ʼ��
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
  * @brief          IMU��������
  * @param[in]      imu_cmd_id �����ʶ
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
  * @brief          IMUЭ�����
  * @param[in]      imu_frame: ԭ������ָ��
  * @param[out]     imu_data: IMU����ָ��
  * @retval         none
  */
extern void imu_data_solve(volatile const uint8_t *imu_frame, IMU_data_t *imu_data);


/**
  * @brief          get  imu data point
  * @param[in]      none
  * @retval         imu data point
  */
/**
  * @brief          ��ȡIMU����ָ��
  * @param[in]      none
  * @retval         IMU����ָ��
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
	if(huart3.Instance->ISR & UART_FLAG_RXNE)//���յ�����
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
					//ʧЧDMA
					__HAL_DMA_DISABLE(&hdma_usart3_rx);

					//get receive data length, length = set_data_length - remain_length
					//��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
					this_time_rx_len = IMU_FRAME_LENGTH - ((DMA_Stream_TypeDef   *)hdma_usart3_rx.Instance)->NDTR;
						
					//reset set_data_lenght
					//�����趨���ݳ���
					((DMA_Stream_TypeDef   *)hdma_usart3_rx.Instance)->NDTR = IMU_FRAME_LENGTH;

					//set memory buffer 1
					//�趨������1
					((DMA_Stream_TypeDef   *)hdma_usart3_rx.Instance)->CR |= DMA_SxCR_CT;
					
					//enable DMA
					//ʹ��DMA
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
					//ʧЧDMA
					__HAL_DMA_DISABLE(&hdma_usart3_rx);

					//get receive data length, length = set_data_length - remain_length
					//��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
					//this_time_rx_len = IMU_FRAME_LENGTH - ((DMA_Stream_TypeDef   *)hdma_usart3_rx.Instance)->NDTR;
					this_time_rx_len = IMU_FRAME_LENGTH - ((DMA_Stream_TypeDef   *)hdma_usart3_rx.Instance)->NDTR;

					//reset set_data_lenght
					//�����趨���ݳ���
					((DMA_Stream_TypeDef   *)hdma_usart3_rx.Instance)->NDTR = IMU_FRAME_LENGTH;

					//set memory buffer 0
					//�趨������0
					((DMA_Stream_TypeDef   *)hdma_usart3_rx.Instance)->CR &= ~(DMA_SxCR_CT);
					
					//enable DMA
					//ʹ��DMA
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
  * @brief          IMUЭ�����
  * @param[in]      imu_frame: ԭ������ָ��
  * @param[out]     imu_data: IMU����ָ��
  * @retval         none
  */
static void imu_data_solve(volatile const uint8_t *imu_frame, IMU_data_t *imu_data)
{
	/*����У��*/
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
		/*���ݽ���*/
		imu_data->yaw = (int16_t)(((imu_frame[3]<<8) + imu_frame[4]))*0.1;						//�����ǣ���λΪ��
		imu_data->pitch = (int16_t)(((imu_frame[5]<<8) + imu_frame[6]))*0.1;							//ƫ����
		imu_data->roll = (int16_t)(((imu_frame[7]<<8) + imu_frame[8]))*0.1;							//�����
		imu_data->ax = (int16_t)(((imu_frame[9]<<8) + imu_frame[10]))/16384;						//X������ٶȣ���λΪg
		imu_data->ay = (int16_t)(((imu_frame[11]<<8) + imu_frame[12]))/16384;						//y������ٶ�
		imu_data->az = (int16_t)(((imu_frame[13]<<8) + imu_frame[14]))/16384;						//z������ٶ�
		imu_data->gx = (int16_t)(((imu_frame[15]<<8) + imu_frame[16]))/32.8;						//X������ٶȣ���λΪ��/s
		imu_data->gy = (int16_t)(((imu_frame[17]<<8) + imu_frame[18]))/32.8;						//y������ٶ�
		imu_data->gz = (int16_t)(((imu_frame[19]<<8) + imu_frame[20]))/32.8;						//z������ٶ�
		imu_data->mx = (int16_t)(((imu_frame[21]<<8) + imu_frame[22]))/333.3;						//X����ų�����λΪGs
		imu_data->my = (int16_t)(((imu_frame[23]<<8) + imu_frame[24]))/333.3;						//y����ų�
		imu_data->mz = (int16_t)(((imu_frame[25]<<8) + imu_frame[26]))/333.3;						//z����ų�
		imu_data->temperate = (int16_t)(((imu_frame[31]<<8) + imu_frame[32]))/333.3;		//IMU�¶ȣ���λΪ�� 
}


#if IMU_DEBUG
/**
  * @brief          send sbus data by usart1, called in usart3_IRQHandle
  * @param[in]      sbus: sbus data, 18 bytes
  * @retval         none
  */
/**
  * @brief          ͨ��usart1����sbus����,��usart3_IRQHandle����
  * @param[in]      imu_data: sbus����, 40�ֽ�
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


