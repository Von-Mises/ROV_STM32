#include "bsp_can.h"
#include "main.h"

extern FDCAN_HandleTypeDef hfdcan1;

void can_filter_init(void)
{
	/* USER CODE BEGIN FDCAN1_Init 2 */
	//配置RX滤波器 
	FDCAN_FilterTypeDef FDCAN1_RXFilter;
	FDCAN1_RXFilter.IdType=FDCAN_STANDARD_ID;                       //标准ID
	FDCAN1_RXFilter.FilterIndex=0;                                  //滤波器索引                   
	FDCAN1_RXFilter.FilterType=FDCAN_FILTER_MASK;                   //滤波器类型
	FDCAN1_RXFilter.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;           //过滤器0关联到FIFO0  
	FDCAN1_RXFilter.FilterID1=0x0000;                               //32位ID
	FDCAN1_RXFilter.FilterID2=0x0000;                               //如果FDCAN配置为传统模式的话，这里是32位掩码
	if(HAL_FDCAN_ConfigFilter(&hfdcan1,&FDCAN1_RXFilter)!=HAL_OK)   //滤波器初始化
	{
		Error_Handler();
	}
	HAL_FDCAN_Start(&hfdcan1);                               //开启FDCAN
	HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);
/* USER CODE END FDCAN1_Init 2 */
}
