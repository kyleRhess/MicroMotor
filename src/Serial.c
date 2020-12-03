#include <string.h>
#include <stdio.h>
#include "stm32f4xx.h"
#include "Serial.h"
#include "system.h"
#include "hall.h"

volatile uint8_t toggle = 0;
UART_HandleTypeDef s_UARTHandle = { .Instance = USART1 };

DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

__IO ITStatus UartReady				= SET;
__IO ITStatus UartRxCmdReady 		= RESET;

uint8_t dma_buffer_tx[FULL_RX*2];
uint8_t dma_buffer_rx[FULL_RX*2];
uint32_t dma_tx_idx = 0;
uint32_t dma_rx_idx = 0;
uint32_t dma_rx_idxLast = 0;
uint32_t dma_tx_idxLast = 0;

float pwm_freq_rx = 1.0f;

int InitSerial(uint32_t baudrate, uint32_t stopbits, uint32_t datasize, uint32_t parity)
{
	int rc = HAL_OK;

	MX_DMA_Init();

	s_UARTHandle.Init.BaudRate   	= baudrate;
	s_UARTHandle.Init.WordLength 	= datasize;
	s_UARTHandle.Init.StopBits   	= stopbits;
	s_UARTHandle.Init.Parity     	= parity;
	s_UARTHandle.Init.Mode       	= USART_MODE_TX_RX;
	s_UARTHandle.Init.HwFlowCtl 	= UART_HWCONTROL_NONE;
	s_UARTHandle.Init.OverSampling 	= UART_OVERSAMPLING_16;
	rc = HAL_UART_Init(&s_UARTHandle);

	serialODR 		= 50;
	rxIndexA 		= 0;
	rxIndexB 		= 0;
	rxBufferSwitch  = 0;
	connLoss 		= 0;
	firstSync 		= 1;
	handshakeCMD 	= 0;
	serialMSG 		= 0;

	return rc;
}

void MX_DMA_Init(void)
{
	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA2_Stream2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 2, 1);
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

	/* DMA2_Stream7_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 2, 2);
	HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
}

void rx_serial_data(uint8_t *pData, uint16_t Size)
{
	HAL_UART_Receive_DMA(&s_UARTHandle, pData, Size);
}

void tx_serial_data(uint8_t* pData, uint16_t Size)
{
	HAL_UART_Transmit_DMA(&s_UARTHandle, pData, Size);
}

static uint32_t crc32_for_byte(uint32_t r)
{
	for(int j = 0; j < 8; ++j)
		r = (r & 1? 0: (uint32_t)0xEDB88320L) ^ r >> 1;
	return r ^ (uint32_t)0xFF000000L;
}

void crc32_(const void *data, size_t n_bytes, uint32_t* crc)
{
  static uint32_t table[0x100];
  if(!*table)
    for(size_t i = 0; i < 0x100; ++i)
      table[i] = crc32_for_byte(i);
  for(size_t i = 0; i < n_bytes; ++i)
    *crc = table[(uint8_t)*crc ^ ((uint8_t*)data)[i]] ^ *crc >> 8;
}

void crc32(const void *data, size_t n_bytes, uint32_t* crc)
{
	uint32_t sum = 0;
	for(size_t i = 0; i < n_bytes; ++i)
		sum += ((uint8_t*)data)[i];
	*crc = sum;
}

//int main(int ac, char** av) {
//  FILE *fp;
//  char buf[1L << 15];
//  for(int i = ac > 1; i < ac; ++i)
//    if((fp = i? fopen(av[i], "rb"): stdin)) {
//      uint32_t crc = 0;
//      while(!feof(fp) && !ferror(fp))
//        crc32(buf, fread(buf, 1, sizeof(buf), fp), &crc);
//      if(!ferror(fp))
//        printf("%08x%s%s\n", crc, ac > 2? "\t": "", ac > 2? av[i]: "");
//      if(i)
//        fclose(fp);
//    }
//  return 0;
//}



/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  HAL_UART_IRQHandler(&s_UARTHandle);
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
}

/**
  * @brief This function handles DMA2 stream7 global interrupt.
  */
void DMA2_Stream7_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
}

static void proc_command(uint32_t command, uint32_t param)
{
	pwm_freq_rx = (float)command;
	if (command == param)
	{
		if(command == 0xAF)
		{
			if(pwm_duty < 0.0f)
			{
				pwm_inc = PWM_INC_F;
				pwm_duty += 0.5f;
			}
			else
			{
				pwm_inc = -PWM_INC_F;
				pwm_duty -= 0.5f;
			}
		}
		else
		{
			WritePin(GPIOB, GPIO_DIS_PIN, !ReadPin(GPIOB, GPIO_DIS_PIN));
		}
	}

//	switch (command)
//	{
//		case 0xabcd:
//			WritePin(GPIOB, GPIO_DIS_PIN, !ReadPin(GPIOB, GPIO_DIS_PIN));
//			break;
//		default:
//			break;
//	}
}

/* USER CODE BEGIN 1 */
/*
 * UART Interrupts
 */

void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
	if(UartHandle->hdmatx->ErrorCode == HAL_DMA_ERROR_FE)
	{
		// Since FIFO mode disabled, just ignore this error
		__NOP();
	}
	else
	{
		if(__HAL_UART_GET_FLAG(UartHandle, UART_FLAG_ORE) != RESET)
		{
			if(__HAL_UART_GET_FLAG(UartHandle, UART_FLAG_RXNE) == RESET)
				UartHandle->Instance->DR;
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	static uint32_t thiscrc = 0;
	static uint32_t rxCrc = 0;

	crc32(&dma_buffer_rx[0], FULL_RX, &thiscrc);

	rxCrc = (dma_buffer_rx[FULL_RX - 4] << 24) |
			(dma_buffer_rx[FULL_RX - 3] << 16) |
			(dma_buffer_rx[FULL_RX - 2] << 8) |
			(dma_buffer_rx[FULL_RX - 1] << 0);

	proc_command((uint32_t)dma_buffer_rx[0], (uint32_t)dma_buffer_rx[63]);

//	pwm_freq_rx = dma_buffer_rx[0]

	HAL_UART_Receive_DMA(huart, &dma_buffer_rx[0], FULL_RX);
	dma_rx_idx++;
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	__NOP();
}

void UART_DMATransmitCplt(DMA_HandleTypeDef *hdma)
{
	__NOP();
}

void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	__NOP();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	dma_tx_idx++;
	__NOP();
}



/*
 *
 */
//void USART1_IRQHandler(void)
//{
//	if(s_UARTHandle.gState != HAL_UART_STATE_BUSY_TX)
//	{
//		if(rxBufferSwitch == 0)
//		{
//			// Copy ISR buffer into RX buffer
//			uartRxA[rxIndexA++] = (uint8_t)(s_UARTHandle.Instance->DR & (uint8_t)0x00FF);
//
//			if(rxIndexA >= TX_BUFF_SZ)
//			{
//				memcpy(&uartTx, &uartRxA, TX_BUFF_SZ);
//				HAL_UART_Transmit_IT(&s_UARTHandle, uartTx, TX_BUFF_SZ);
//				rxIndexA = 0;
//				rxBufferSwitch = 1;
//			}
//		}
//		else
//		{
//			// Copy ISR buffer into RX buffer
//			uartRxB[rxIndexB++] = (uint8_t)(s_UARTHandle.Instance->DR & (uint8_t)0x00FF);
//
//			if(rxIndexB >= TX_BUFF_SZ)
//			{
//				memcpy(&uartTx, &uartRxB, TX_BUFF_SZ);
//				HAL_UART_Transmit_IT(&s_UARTHandle, uartTx, TX_BUFF_SZ);
//				rxIndexB = 0;
//				rxBufferSwitch = 0;
//			}
//		}
//
//		add_characters((uint8_t)(s_UARTHandle.Instance->DR & (uint8_t)0x00FF), 1);
//		update_display();
//
//		// Clear out the rx uart register
//		__HAL_UART_FLUSH_DRREGISTER(&s_UARTHandle);
//		__HAL_UART_CLEAR_FEFLAG(&s_UARTHandle);
//	}
//
//	UartRxCmdReady = RESET;
//
//	HAL_UART_IRQHandler(&s_UARTHandle);
//	return;
//}

//void HAL_UART_MspInit(UART_HandleTypeDef* huart)
//{
//	if(huart->Instance == USART1)
//	{
//		GPIO_InitTypeDef GPIO_InitStructureUart = {0};
//
//		// Setup all our peripherals
//		__HAL_RCC_USART1_CLK_ENABLE();
//		__HAL_RCC_GPIOB_CLK_ENABLE();
//
//		GPIO_InitStructureUart.Pin = GPIO_PIN_6 | GPIO_PIN_7;
//		GPIO_InitStructureUart.Mode = GPIO_MODE_AF_PP;
//		GPIO_InitStructureUart.Alternate = GPIO_AF7_USART1;
//		GPIO_InitStructureUart.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//		GPIO_InitStructureUart.Pull = GPIO_PULLUP;
//		HAL_GPIO_Init(GPIOB, &GPIO_InitStructureUart);
//
//		// Enable the UART interrupt
//		__HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
//
//		HAL_NVIC_SetPriority(USART1_IRQn, 4, 0);
//		HAL_NVIC_EnableIRQ(USART1_IRQn);
//	}
//}


/**
* @brief UART MSP Initialization
* This function configures the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};
	  if(huart->Instance==USART1)
	  {
	  /* USER CODE BEGIN USART1_MspInit 0 */

	  /* USER CODE END USART1_MspInit 0 */
	    /* Peripheral clock enable */
	    __HAL_RCC_USART1_CLK_ENABLE();

	    __HAL_RCC_GPIOB_CLK_ENABLE();
	    /**USART1 GPIO Configuration
	    PB6     ------> USART1_TX
	    PB7     ------> USART1_RX
	    */
	    GPIO_InitStruct.Pin 					= UART_TX_PIN | UART_RX_PIN;
	    GPIO_InitStruct.Mode 					= GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull 					= GPIO_PULLDOWN;
	    GPIO_InitStruct.Speed 					= GPIO_SPEED_FREQ_VERY_HIGH;
	    GPIO_InitStruct.Alternate 				= GPIO_AF7_USART1;
	    HAL_GPIO_Init(UART_PORT, &GPIO_InitStruct);

	    /* USART1 DMA Init */
	    /* USART1_RX Init */
	    hdma_usart1_rx.Instance 				= DMA2_Stream2;
	    hdma_usart1_rx.Init.Channel 			= DMA_CHANNEL_4;
	    hdma_usart1_rx.Init.Direction 			= DMA_PERIPH_TO_MEMORY;
	    hdma_usart1_rx.Init.PeriphInc 			= DMA_PINC_DISABLE;
	    hdma_usart1_rx.Init.MemInc 				= DMA_MINC_ENABLE;
	    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	    hdma_usart1_rx.Init.MemDataAlignment 	= DMA_MDATAALIGN_BYTE;
	    hdma_usart1_rx.Init.Mode 				= DMA_CIRCULAR;
	    hdma_usart1_rx.Init.Priority 			= DMA_PRIORITY_HIGH;
	    hdma_usart1_rx.Init.FIFOMode 			= DMA_FIFOMODE_ENABLE;
	    hdma_usart1_rx.Init.FIFOThreshold 		= DMA_FIFO_THRESHOLD_FULL;
	    hdma_usart1_rx.Init.MemBurst 			= DMA_MBURST_SINGLE;
	    hdma_usart1_rx.Init.PeriphBurst 		= DMA_PBURST_SINGLE;
	    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
	    {
	    	while(1){;;}
	    }

	    __HAL_LINKDMA(huart,hdmarx,hdma_usart1_rx);

	    /* USART1_TX Init */
	    hdma_usart1_tx.Instance 				= DMA2_Stream7;
	    hdma_usart1_tx.Init.Channel 			= DMA_CHANNEL_4;
	    hdma_usart1_tx.Init.Direction 			= DMA_MEMORY_TO_PERIPH;
	    hdma_usart1_tx.Init.PeriphInc 			= DMA_PINC_DISABLE;
	    hdma_usart1_tx.Init.MemInc 				= DMA_MINC_ENABLE;
	    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	    hdma_usart1_tx.Init.MemDataAlignment 	= DMA_MDATAALIGN_BYTE;
	    hdma_usart1_tx.Init.Mode 				= DMA_NORMAL;
	    hdma_usart1_tx.Init.Priority 			= DMA_PRIORITY_HIGH;
	    hdma_usart1_tx.Init.FIFOMode 			= DMA_FIFOMODE_DISABLE;
	    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
	    {
	    	while(1){;;}
	    }

	    __HAL_LINKDMA(huart, hdmatx, hdma_usart1_tx);

	  /* USER CODE BEGIN USART1_MspInit 1 */
	    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
	    HAL_NVIC_EnableIRQ(USART1_IRQn);
	  /* USER CODE END USART1_MspInit 1 */
	  }
}

/**
* @brief UART MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{

  if(huart->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX
    */
    HAL_GPIO_DeInit(UART_PORT, UART_TX_PIN | UART_RX_PIN);

  /* USER CODE BEGIN USART1_MspDeInit 1 */

    /* USART1 DMA DeInit */
	HAL_DMA_DeInit(huart->hdmarx);
	HAL_DMA_DeInit(huart->hdmatx);

    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE END USART1_MspDeInit 1 */
  }

}
