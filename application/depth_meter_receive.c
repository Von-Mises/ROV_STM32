/**
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  * @file       depth_meter_receive.c/h
  * @brief      ������ȼƷ��͹���������,����DMA���䷽ʽ��ԼCPU
  *             ��Դ�����ô��ڿ����ж�������������ͬʱ�ṩһЩ��������DMA������
  *             �ķ�ʽ��֤�Ȳ�ε��ȶ��ԡ�
  * @note       ��������ͨ�������ж�����������freeRTOS����
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     DEC-21-2022     Qiqi Li(������)    1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  */

#include "depth_meter_receive.h"
#include "main.h"
#include "bsp_usart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "config.h"

extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;


//����ԭʼ���ݣ�Ϊ16���ֽڣ�����32���ֽڳ��ȣ���ֹDMA����Խ��
static uint8_t dm_rx_buf[2][DM_FRAME_LENGTH];

uint8_t dm_tx_buf[DM_CMD_LENGTH]={0};

//depth  data 
//�������
float depth;


/**
  * @brief          DM init
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          DM��ʼ��
  * @param[in]      none
  * @retval         none
  */
void DM_init(void)
{
    usart6_init(dm_rx_buf[0], dm_rx_buf[1], DM_FRAME_LENGTH);
	DM_reset();
}


/**
  * @brief          Depth meter reset
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ��ȼƸ�λ
  * @param[in]      none
  * @retval         none
  */
void DM_reset(void)
{
	uint8_t tx_len= sprintf((char*)dm_tx_buf,"!R\r\n");
	usart6_tx_dma_enable(dm_tx_buf,tx_len);
}

/**
  * @brief          DM protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @retval         none
  */
/**
  * @brief          DMЭ�����
  * @param[in]      dm_rx_buf: ԭ������ָ��
  * @retval         none
  */
static void depth_data_solve(volatile const uint8_t *dm_rx_buf);

#if DM_DEBUG
/**
  * @brief          send sbus data by usart1, called in usart3_IRQHandle
  * @param[in]      sbus: sbus data, 18 bytes
  * @retval         none
  */
/**
  * @brief          ͨ��usart1����sbus����,��usart6_IRQHandle����
  * @param[in]      dm_data: sbus����, 20�ֽ�
  * @retval         none
  */
static void dm_to_usart1(uint8_t *dm_data);
#endif

/**
  * @brief          get  depth data 
  * @param[in]      none
  * @retval         depth data 
  */
/**
  * @brief          ��ȡ�������
  * @param[in]      none
  * @retval         ���
  */
float get_depth_data(void)
{
    return depth;
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
	if(huart6.Instance->ISR & UART_FLAG_RXNE)//���յ�����
	{
			__HAL_UART_CLEAR_PEFLAG(&huart6);
	}
	else if(USART6->ISR & UART_FLAG_IDLE)
	{
		static uint16_t this_time_rx_len = 0;
		__HAL_UART_CLEAR_IDLEFLAG(&huart6);
		SCB_InvalidateDCache_by_Addr ((uint32_t *)dm_rx_buf, 2*DM_FRAME_LENGTH);
		if ((((DMA_Stream_TypeDef   *)hdma_usart6_rx.Instance)->CR & DMA_SxCR_CT) == RESET)
			{
					/* Current memory buffer used is Memory 0 */

					//disable DMA
					//ʧЧDMA
					__HAL_DMA_DISABLE(&hdma_usart6_rx);

					//get receive data length, length = set_data_length - remain_length
					//��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
					this_time_rx_len = DM_FRAME_LENGTH - ((DMA_Stream_TypeDef   *)hdma_usart6_rx.Instance)->NDTR;
						
					//reset set_data_lenght
					//�����趨���ݳ���
					((DMA_Stream_TypeDef   *)hdma_usart6_rx.Instance)->NDTR = DM_FRAME_LENGTH;

					//set memory buffer 1
					//�趨������1
					((DMA_Stream_TypeDef   *)hdma_usart6_rx.Instance)->CR |= DMA_SxCR_CT;
					
					//enable DMA
					//ʹ��DMA
					__HAL_DMA_ENABLE(&hdma_usart6_rx);
					if(*(dm_rx_buf[0]+this_time_rx_len-1) == 0x0A)
					{
							depth_data_solve(dm_rx_buf[0]);
							#if DM_DEBUG
							dm_to_usart1(dm_rx_buf[0]);
							#endif
					}
				
					
			}
			else
			{
					/* Current memory buffer used is Memory 1 */
					//disable DMA
					//ʧЧDMA
					__HAL_DMA_DISABLE(&hdma_usart6_rx);

					//get receive data length, length = set_data_length - remain_length
					//��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
					//this_time_rx_len = IMU_FRAME_LENGTH - ((DMA_Stream_TypeDef   *)hdma_usart3_rx.Instance)->NDTR;
					this_time_rx_len = DM_FRAME_LENGTH - ((DMA_Stream_TypeDef   *)hdma_usart6_rx.Instance)->NDTR;

					//reset set_data_lenght
					//�����趨���ݳ���
					((DMA_Stream_TypeDef   *)hdma_usart6_rx.Instance)->NDTR = DM_FRAME_LENGTH;

					//set memory buffer 0
					//�趨������0
					((DMA_Stream_TypeDef   *)hdma_usart6_rx.Instance)->CR &= ~(DMA_SxCR_CT);
					
					//enable DMA
					//ʹ��DMA
					__HAL_DMA_ENABLE(&hdma_usart6_rx);
					if(*(dm_rx_buf[1]+this_time_rx_len-1) == 0x0A)
					{
							depth_data_solve(dm_rx_buf[1]);
							#if DM_DEBUG
							dm_to_usart1(dm_rx_buf[1]);
							#endif
					}
					
			}
		
	}
	
}



/**
  * @brief          DM protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @retval         none
  */
/**
  * @brief          DMЭ�����
  * @param[in]      dm_rx_buf: ԭ������ָ��
  * @retval         none
  */
static void depth_data_solve(volatile const uint8_t *dm_rx_buf)
{
	uint8_t* p = (uint8_t *)dm_rx_buf;
	for(int i = 0;i<DM_FRAME_LENGTH;i++)
	{
		if(*(p+i) == 'D')
		{
			depth = atof((char const *)(p+i+2));
			break;
		}
	}
}


#if DM_DEBUG
/**
  * @brief          send sbus data by usart1, called in usart3_IRQHandle
  * @param[in]      sbus: sbus data, 18 bytes
  * @retval         none
  */
/**
  * @brief          ͨ��usart1����sbus����,��usart6_IRQHandle����
  * @param[in]      dm_data: sbus����, 20�ֽ�
  * @retval         none
  */
void dm_to_usart1(uint8_t *dm_data)
{
    static uint8_t usart_tx_buf[18];
    static uint8_t i =0;
    usart_tx_buf[0] = 0xA6;
    memcpy(usart_tx_buf + 1, dm_data, 16);
    for(i = 0, usart_tx_buf[17] = 0; i < 17; i++)
    {
        usart_tx_buf[17] += usart_tx_buf[i];
    }
    usart1_tx_dma_enable(usart_tx_buf, 18);
}
#endif


