/**
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  * @file       atimeter_receive.c/h
  * @brief      解析P30声呐发送过来的数据,解析高度计数据
  * @note       
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     May-4-2023     HaoLion(郝亮亮)    1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  */

#include "altimeter_receive.h"
#include "main.h"
#include "bsp_usart.h"
#include "config.h"

extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_uart5_tx;


//接收原始数据，为15个字节，给了30个字节长度，防止DMA传输越界
static uint8_t am_rx_buf[2][AM_FRAME_LENGTH];

uint8_t am_tx_buf[AM_CMD_LENGTH]={0};

//height  data 
//高度数据 单位：m
float height;



/**
  * @brief          Altimeter set sound velocity info
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          高度计设置声速
  * @param[in]      none
  * @retval         none
  */
static void AM_set_velocity(uint16_t velocity);

/**
  * @brief          AM init
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          AM初始化
  * @param[in]      none
  * @retval         none
  */
void AM_init(void)
{
    uart5_init(am_rx_buf[0], am_rx_buf[1], AM_FRAME_LENGTH);
	AM_set_velocity(AM_WATER_VELOCITY);
}


/**
  * @brief          Altimeter get once simple data
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          高度度计获取单次简单数据
  * @param[in]      none
  * @retval         none
  */
void AM_get_data(void)
{
		am_tx_buf[0]=0x42;//填充发送缓冲区
		am_tx_buf[1]=0x52;//填充发送缓冲区
		am_tx_buf[2]=0x00;//填充发送缓冲区
		am_tx_buf[3]=0x00;//填充发送缓冲区
		am_tx_buf[4]=0xBB;//填充发送缓冲区
		am_tx_buf[5]=0x04;//填充发送缓冲区
		am_tx_buf[6]=0x00;//填充发送缓冲区
		am_tx_buf[7]=0x00;//填充发送缓冲区
		am_tx_buf[8]=0x53;//填充发送缓冲区
		am_tx_buf[9]=0x01;//填充发送缓冲区
		uart5_tx_dma_enable(am_tx_buf,10);
}

/**
  * @brief          Altimeter set sound velocity info
  * @param[in]      velocity 设置的声速，单位：m/s 
  * @retval         none
  */
/**
  * @brief          高度计设置声速
  * @param[in]      velocity 设置的声速，单位：m/s 
  * @retval         none
  */
static void AM_set_velocity(uint16_t velocity)
{
		uint32_t velocity_mm = velocity * 1000;
		uint16_t psum = 0;
		am_tx_buf[0]=0x42;//填充发送缓冲区
		am_tx_buf[1]=0x52;//填充发送缓冲区
		am_tx_buf[2]=0x04;//填充发送缓冲区
		am_tx_buf[3]=0x00;//填充发送缓冲区
		am_tx_buf[4]=0xEA;//填充发送缓冲区
		am_tx_buf[5]=0x03;//填充发送缓冲区
		am_tx_buf[6]=0x00;//填充发送缓冲区
		am_tx_buf[7]=0x00;//填充发送缓冲区
		am_tx_buf[8]=velocity_mm & 0xff;//低八位
		am_tx_buf[9]=(velocity_mm >>8) & 0xff;
		am_tx_buf[10]=(velocity_mm >>16) & 0xff;
		am_tx_buf[11]=velocity_mm >>24;//填充发送缓冲区
		for(int i = 0; i<11 ; i++)
		{
				psum = psum + am_tx_buf[i];
		}
		am_tx_buf[12]=psum & 0xff;//校验和低八位
		am_tx_buf[13]=psum >> 8;//校验和高八位
		uart5_tx_dma_enable(am_tx_buf,14);
   }

/**
  * @brief          AM protocol resolution
  * @param[in]      am_rx_buf: raw data point
  * @retval         none
  */
/**
  * @brief          AM协议解析
  * @param[in]      am_rx_buf: 原生数据指针
  * @retval         none
  */
static void altimeter_data_solve(volatile const uint8_t *am_raw_rx_buf);
/**
  * @brief          send sbus data by usart1, called in usart3_IRQHandle
  * @param[in]      sbus: sbus data, 18 bytes
  * @retval         none
  */
/**
  * @brief          通过usart1发送sbus数据,在usart6_IRQHandle调用
  * @param[in]      dm_data: sbus数据, 20字节
  * @retval         none
  */
//static void am_to_usart1(uint8_t *dm_data);


/**
  * @brief          get  altimeter data 
  * @param[in]      none
  * @retval         altimeter data 
  */
/**
  * @brief          获取高度数据
  * @param[in]      none
  * @retval         高度
  */
float get_height_data(void)
{
    return height;
}

/**
  * @brief This function handles UART5 global interrupt.
  */
void UART5_IRQHandler(void)
{
	if(UART5->ISR & UART_FLAG_IDLE)
	{
		static uint16_t this_time_rx_len = 0;
		__HAL_UART_CLEAR_IDLEFLAG(&huart5);
		SCB_InvalidateDCache_by_Addr ((uint32_t *)am_rx_buf, 2*AM_FRAME_LENGTH);
		
		if ((((DMA_Stream_TypeDef   *)hdma_uart5_rx.Instance)->CR & DMA_SxCR_CT) == RESET)
			{
					/* Current memory buffer used is Memory 0 */
					__HAL_DMA_DISABLE(&hdma_uart5_rx);
					this_time_rx_len = AM_FRAME_LENGTH - __HAL_DMA_GET_COUNTER(huart5.hdmarx);
					__HAL_DMA_SET_COUNTER(huart5.hdmarx, AM_FRAME_LENGTH);
					//set memory buffer 1
					//设定缓冲区1
					((DMA_Stream_TypeDef   *)hdma_uart5_rx.Instance)->CR |= DMA_SxCR_CT;
					__HAL_DMA_ENABLE(&hdma_uart5_rx);
					if(this_time_rx_len == AM_DATA_LENGTH)
					{
							altimeter_data_solve(am_rx_buf[0]);
							#if AM_DEBUG
							am_to_usart1(am_rx_buf[0]);
							#endif
					}
			}
			else
			{
					
					/* Current memory buffer used is Memory 1 */
					__HAL_DMA_DISABLE(&hdma_uart5_rx);
					this_time_rx_len = AM_FRAME_LENGTH - __HAL_DMA_GET_COUNTER(huart5.hdmarx);
					__HAL_DMA_SET_COUNTER(huart5.hdmarx, AM_FRAME_LENGTH);
					//set memory buffer 1
					//设定缓冲区1
					((DMA_Stream_TypeDef   *)hdma_uart5_rx.Instance)->CR &= ~(DMA_SxCR_CT);
					__HAL_DMA_ENABLE(&hdma_uart5_rx);
					if(this_time_rx_len == AM_DATA_LENGTH)
					{
							altimeter_data_solve(am_rx_buf[1]);
							#if AM_DEBUG
							am_to_usart1(am_rx_buf[1]);
							#endif
					}
			}
		
	}
	
}

/**
  * @brief          AM protocol resolution
  * @param[in]      am_rx_buf: raw data point
  * @retval         none
  */
/**
  * @brief          AM协议解析
  * @param[in]      am_rx_buf: 原生数据指针
  * @retval         none
  */
static void altimeter_data_solve(volatile const uint8_t *am_raw_rx_buf)
{
		uint16_t psum = 0;
		if(am_raw_rx_buf[0] == 0x42 && am_raw_rx_buf[1] == 0x52)
		{
			for(int i = 0; i<13 ; i++)
			{
					psum = psum + am_raw_rx_buf[i];
			}
			if((psum&0xff)==am_raw_rx_buf[13] && (psum>>8)==am_raw_rx_buf[14])
			{
					height= am_raw_rx_buf[8]|(am_raw_rx_buf[9]<<8)|(am_raw_rx_buf[10]<<16)|(am_raw_rx_buf[11]<<24);
			}
			
		}
}




#if AM_DEBUG
/**
  * @brief          send sbus data by usart1, called in usart3_IRQHandle
  * @param[in]      sbus: sbus data, 18 bytes
  * @retval         none
  */
/**
  * @brief          通过usart1发送sbus数据,在usart6_IRQHandle调用
  * @param[in]      dm_data: sbus数据, 20字节
  * @retval         none
  */
void am_to_usart1(uint8_t *dm_data)
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


