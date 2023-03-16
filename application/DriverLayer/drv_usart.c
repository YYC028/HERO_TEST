#include "drv_usart.h"
#include  "main.h"
#include  "rc_potocal.h"
extern UART_HandleTypeDef huart3;
#define USART3_RX_DATA_FRAME_LEN	(18u)	// 串口3数据帧长度
#define USART3_RX_BUF_LEN			(USART3_RX_DATA_FRAME_LEN + 6u)	// 串口3接收缓冲区长度
uint8_t usart3_dma_rxbuf[2][USART3_RX_BUF_LEN];




void DRV_UART_IRQHandler(UART_HandleTypeDef *huart)
{
    // 判断是否为空闲中断
	if( __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) &&
		__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
	{
		uart_rx_idle_callback(huart);
	}
}





void USART3_Init(void)
{
	__HAL_UART_CLEAR_IDLEFLAG(&huart3);
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
	
	// Enable the DMA transfer for the receiver request
	SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);	
	
	DMAEx_MultiBufferStart_NoIT(huart3.hdmarx, \
							    (uint32_t)&huart3.Instance->DR, \
							    (uint32_t)usart3_dma_rxbuf[0], \
							    (uint32_t)usart3_dma_rxbuf[1], \
							    USART3_RX_DATA_FRAME_LEN);
}


static void uart_rx_idle_callback(UART_HandleTypeDef* huart)
{
	/* clear idle it flag avoid idle interrupt all the time */
	__HAL_UART_CLEAR_IDLEFLAG(huart);	
	/* handle received data in idle interrupt */
		 if(huart == &huart3)
	{
		/* clear DMA transfer complete flag */
		__HAL_DMA_DISABLE(huart->hdmarx);

		/* handle dbus data dbus_buf from DMA */
		//uint32_t status = taskENTER_CRITICAL_FROM_ISR();
		if ((USART3_RX_BUF_LEN - huart->hdmarx->Instance->NDTR) == USART3_RX_DATA_FRAME_LEN)
		{
			if(huart->hdmarx->Instance->CR & DMA_SxCR_CT)
				huart->hdmarx->XferM1CpltCallback(huart->hdmarx);
			else
				huart->hdmarx->XferCpltCallback(huart->hdmarx);
		}
	

		/* restart dma transmission */
		__HAL_DMA_SET_COUNTER(huart->hdmarx, USART3_RX_BUF_LEN);
		__HAL_DMA_ENABLE(huart->hdmarx);	  
	}


  
}



static void dma_m0_rxcplt_callback(DMA_HandleTypeDef *hdma)
{
	// 将当前目标内存设置为Memory1
	hdma->Instance->CR |= (uint32_t)(DMA_SxCR_CT);
	USART3_rxDataHandler(usart3_dma_rxbuf[0]);
}


static void dma_m1_rxcplt_callback(DMA_HandleTypeDef *hdma)
{
	// 将当前目标内存设置为Memory0
	hdma->Instance->CR &= ~(uint32_t)(DMA_SxCR_CT);
	USART3_rxDataHandler(usart3_dma_rxbuf[1]);
}



static HAL_StatusTypeDef DMAEx_MultiBufferStart_NoIT(DMA_HandleTypeDef *hdma, \
                                                    uint32_t SrcAddress, \
                                                    uint32_t DstAddress, \
                                                    uint32_t SecondMemAddress, \
                                                    uint32_t DataLength)
{
    HAL_StatusTypeDef status = HAL_OK;

    /* Memory-to-memory transfer not supported in double buffering mode */
    if (hdma->Init.Direction == DMA_MEMORY_TO_MEMORY)
    {
		hdma->ErrorCode = HAL_DMA_ERROR_NOT_SUPPORTED;
		return HAL_ERROR;
    }   

	/* Set the UART DMA transfer complete callback */
	/* Current memory buffer used is Memory 1 callback */
	hdma->XferCpltCallback   = dma_m0_rxcplt_callback;
	/* Current memory buffer used is Memory 0 callback */
	hdma->XferM1CpltCallback = dma_m1_rxcplt_callback;	

	/* Check callback functions */
	if ((NULL == hdma->XferCpltCallback) || (NULL == hdma->XferM1CpltCallback))
	{
	hdma->ErrorCode = HAL_DMA_ERROR_PARAM;
	return HAL_ERROR;
	}
	
	/* Process locked */
	__HAL_LOCK(hdma);
	
	if(HAL_DMA_STATE_READY == hdma->State)
	{	
		/* Change DMA peripheral state */
		hdma->State = HAL_DMA_STATE_BUSY;

		/* Initialize the error code */
		hdma->ErrorCode = HAL_DMA_ERROR_NONE;

		/* Enable the Double buffer mode */
		hdma->Instance->CR |= (uint32_t)DMA_SxCR_DBM;

		/* Configure DMA Stream destination address */
		hdma->Instance->M1AR = SecondMemAddress;		

		/* Configure DMA Stream data length */
		hdma->Instance->NDTR = DataLength;		
		
		/* Peripheral to Memory */
		if((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
		{   
			/* Configure DMA Stream destination address */
			hdma->Instance->PAR = DstAddress;

			/* Configure DMA Stream source address */
			hdma->Instance->M0AR = SrcAddress;
		}
		/* Memory to Peripheral */
		else
		{
			/* Configure DMA Stream source address */
			hdma->Instance->PAR = SrcAddress;

			/* Configure DMA Stream destination address */
			hdma->Instance->M0AR = DstAddress;
		}		
		
		/* Clear TC flags */
		__HAL_DMA_CLEAR_FLAG (hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma));
		/* Enable TC interrupts*/
//		hdma->Instance->CR  |= DMA_IT_TC;
		
		/* Enable the peripheral */
		__HAL_DMA_ENABLE(hdma); 
	}
	else
	{
		/* Process unlocked */
		__HAL_UNLOCK(hdma);	  

		/* Return error status */
		status = HAL_BUSY;		
	}
	/* Process unlocked */
	__HAL_UNLOCK(hdma);

	return status; 	
}