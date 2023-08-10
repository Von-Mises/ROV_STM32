#include "bsp_can.h"
#include "main.h"

extern FDCAN_HandleTypeDef hfdcan1;

void can_filter_init(void)
{
	/* USER CODE BEGIN FDCAN1_Init 2 */
	//����RX�˲��� 
	FDCAN_FilterTypeDef FDCAN1_RXFilter;
	FDCAN1_RXFilter.IdType=FDCAN_STANDARD_ID;                       //��׼ID
	FDCAN1_RXFilter.FilterIndex=0;                                  //�˲�������                   
	FDCAN1_RXFilter.FilterType=FDCAN_FILTER_MASK;                   //�˲�������
	FDCAN1_RXFilter.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;           //������0������FIFO0  
	FDCAN1_RXFilter.FilterID1=0x0000;                               //32λID
	FDCAN1_RXFilter.FilterID2=0x0000;                               //���FDCAN����Ϊ��ͳģʽ�Ļ���������32λ����
	if(HAL_FDCAN_ConfigFilter(&hfdcan1,&FDCAN1_RXFilter)!=HAL_OK)   //�˲�����ʼ��
	{
		Error_Handler();
	}
	HAL_FDCAN_Start(&hfdcan1);                               //����FDCAN
	HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);
/* USER CODE END FDCAN1_Init 2 */
}
