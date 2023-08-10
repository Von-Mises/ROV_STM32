#include "bsp_usart.h"
#include "main.h"

extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_uart5_tx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

void usart1_tx_dma_init(void)
{

    //enable the DMA transfer for the receiver and tramsmit request
    //使能DMA串口接收和发送
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(((DMA_Stream_TypeDef   *)hdma_usart1_tx.Instance)->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }

    ((DMA_Stream_TypeDef   *)hdma_usart1_tx.Instance)->PAR = (uint32_t) & (USART1->TDR);
    ((DMA_Stream_TypeDef   *)hdma_usart1_tx.Instance)->M0AR = (uint32_t)(NULL);
    ((DMA_Stream_TypeDef   *)hdma_usart1_tx.Instance)->NDTR = 0;


}
void usart1_tx_dma_enable(uint8_t *data, uint16_t len)
{
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(((DMA_Stream_TypeDef   *)hdma_usart1_tx.Instance)->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_HISR_TCIF7);

    ((DMA_Stream_TypeDef   *)hdma_usart1_tx.Instance)->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&hdma_usart1_tx, len);

    __HAL_DMA_ENABLE(&hdma_usart1_tx);
}



void usart3_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{

    //enable the DMA transfer for the receiver and tramsmit request
    //使能DMA串口接收和发送
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAT);

    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);



    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    
    while(((DMA_Stream_TypeDef   *)hdma_usart3_rx.Instance)->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart3_rx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart3_rx, DMA_LISR_TCIF1);

    ((DMA_Stream_TypeDef   *)hdma_usart3_rx.Instance)->PAR = (uint32_t) & (USART3->RDR);
    //memory buffer 1
    //内存缓冲区1
    ((DMA_Stream_TypeDef   *)hdma_usart3_rx.Instance)->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //内存缓冲区2
    ((DMA_Stream_TypeDef   *)hdma_usart3_rx.Instance)->M1AR = (uint32_t)(rx2_buf);
    //data length
    //数据长度
    __HAL_DMA_SET_COUNTER(&hdma_usart3_rx, dma_buf_num);

    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(((DMA_Stream_TypeDef   *)hdma_usart3_rx.Instance)->CR, DMA_SxCR_DBM);

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart3_rx);


    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart3_tx);

    while(((DMA_Stream_TypeDef   *)hdma_usart3_tx.Instance)->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart3_tx);
    }

    ((DMA_Stream_TypeDef   *)hdma_usart3_tx.Instance)->PAR = (uint32_t) & (USART3->TDR);
		((DMA_Stream_TypeDef   *)hdma_usart3_tx.Instance)->M0AR = (uint32_t)(NULL);
    ((DMA_Stream_TypeDef   *)hdma_usart3_tx.Instance)->NDTR = 0;

}



void usart3_tx_dma_enable(uint8_t *data, uint16_t len)
{
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart3_tx);

    while(((DMA_Stream_TypeDef   *)hdma_usart3_tx.Instance)->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart3_tx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart3_tx, DMA_LISR_TCIF0);

    ((DMA_Stream_TypeDef   *)hdma_usart3_tx.Instance)->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&hdma_usart3_tx, len);

    __HAL_DMA_ENABLE(&hdma_usart3_tx);
}


void usart6_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{

    //enable the DMA transfer for the receiver and tramsmit request
    //使能DMA串口接收和发送
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAT);

    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);



    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart6_rx);
    
    while(((DMA_Stream_TypeDef   *)hdma_usart6_rx.Instance)->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_rx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx, DMA_LISR_TCIF1);

    ((DMA_Stream_TypeDef   *)hdma_usart6_rx.Instance)->PAR = (uint32_t) & (USART6->RDR);
    //memory buffer 1
    //内存缓冲区1
    ((DMA_Stream_TypeDef   *)hdma_usart6_rx.Instance)->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //内存缓冲区2
    ((DMA_Stream_TypeDef   *)hdma_usart6_rx.Instance)->M1AR = (uint32_t)(rx2_buf);
    //data length
    //数据长度
    __HAL_DMA_SET_COUNTER(&hdma_usart6_rx, dma_buf_num);

    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(((DMA_Stream_TypeDef   *)hdma_usart6_rx.Instance)->CR, DMA_SxCR_DBM);

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart6_rx);


    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart6_tx);

    while(((DMA_Stream_TypeDef   *)hdma_usart6_tx.Instance)->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_tx);
    }

    ((DMA_Stream_TypeDef   *)hdma_usart6_tx.Instance)->PAR = (uint32_t) & (USART6->TDR);

}



void usart6_tx_dma_enable(uint8_t *data, uint16_t len)
{
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart6_tx);

    while(((DMA_Stream_TypeDef   *)hdma_usart6_tx.Instance)->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_tx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart6_tx, DMA_HISR_TCIF6);

    ((DMA_Stream_TypeDef   *)hdma_usart6_tx.Instance)->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&hdma_usart6_tx, len);

    __HAL_DMA_ENABLE(&hdma_usart6_tx);
}


void uart5_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{

    //enable the DMA transfer for the receiver and tramsmit request
    //使能DMA串口接收和发送
    SET_BIT(huart5.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart5.Instance->CR3, USART_CR3_DMAT);

    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);



    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_uart5_rx);
    
    while(((DMA_Stream_TypeDef   *)hdma_uart5_rx.Instance)->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_uart5_rx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_uart5_rx, DMA_LISR_TCIF1);

    ((DMA_Stream_TypeDef   *)hdma_uart5_rx.Instance)->PAR = (uint32_t) & (UART5->RDR);
    //memory buffer 1
    //内存缓冲区1
    ((DMA_Stream_TypeDef   *)hdma_uart5_rx.Instance)->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //内存缓冲区2
    ((DMA_Stream_TypeDef   *)hdma_uart5_rx.Instance)->M1AR = (uint32_t)(rx2_buf);
    //data length
    //数据长度
    __HAL_DMA_SET_COUNTER(&hdma_uart5_rx, dma_buf_num);

    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(((DMA_Stream_TypeDef   *)hdma_uart5_rx.Instance)->CR, DMA_SxCR_DBM);

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_uart5_rx);


    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_uart5_tx);

    while(((DMA_Stream_TypeDef   *)hdma_uart5_tx.Instance)->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_uart5_tx);
    }

    ((DMA_Stream_TypeDef   *)hdma_uart5_tx.Instance)->PAR = (uint32_t) & (UART5->TDR);

}



void uart5_tx_dma_enable(uint8_t *data, uint16_t len)
{
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_uart5_tx);

    while(((DMA_Stream_TypeDef   *)hdma_uart5_tx.Instance)->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_uart5_tx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_uart5_tx, DMA_HISR_TCIF4);

    ((DMA_Stream_TypeDef   *)hdma_uart5_tx.Instance)->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&hdma_uart5_tx, len);

    __HAL_DMA_ENABLE(&hdma_uart5_tx);
}

