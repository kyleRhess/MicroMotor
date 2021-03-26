
#include "serial.h"

static uint32_t crc32_for_byte(uint32_t r);
static void crc32(const void *data, size_t n_bytes, uint32_t* crc);
static uint32_t proc_command(uint32_t command);
static void tx_ack(uint32_t *command);
static void tx_nak(void);
static void Serial_DMA_Init(void);

static UART_HandleTypeDef 	s_UARTHandle = { .Instance = USART1 };
static DMA_HandleTypeDef 	hdma_usart1_rx;
static DMA_HandleTypeDef 	hdma_usart1_tx;

static uint8_t 				uartRxBuff[RX_BUFF_SZ];
static uint8_t 				uartTxBuff[TX_BUFF_SZ];

static DataFields 			txCmdData;
static DataFields 			rxCmdData;

static uint32_t crc32_for_byte(uint32_t r)
{
	for(int j = 0; j < 8; ++j)
		r = (r & 1? 0: (uint32_t)0xEDB88320L) ^ r >> 1;
	return r ^ (uint32_t)0xFF000000L;
}

static void crc32(const void *data, size_t n_bytes, uint32_t* crc)
{
  static uint32_t table[0x100];
  if(!*table)
    for(size_t i = 0; i < 0x100; ++i)
      table[i] = crc32_for_byte(i);
  for(size_t i = 0; i < n_bytes; ++i)
    *crc = table[(uint8_t)*crc ^ ((uint8_t*)data)[i]] ^ *crc >> 8;
}

static uint32_t proc_command(uint32_t command)
{
	uint32_t rc = 0;

	switch (command)
	{
		case CMD_MOTOR_RESET:
			rc = command;
			Encoder_Reset();
			FOC_Init();
			Signal_SetMotorPos(0.0f);
			Signal_ClearMotorState(MOTOR_MODE_ENABLE);
			Signal_ClearMotorState(MOTOR_MODE_OVERCURRENT);
			Signal_ClearMotorState(MOTOR_MODE_OVERSPEED);
			break;
		case CMD_MOTOR_QUERY:
			rc = command;
			txCmdData.pwmValue 		= m_fRotorTheta;//Hall_GetRPM();// Signal_GetMotorPWM();
			txCmdData.driveMode 	= Signal_GetMotorState();
			txCmdData.rpmValue 		= m_fMechAngle / 4.0f;//pi_pos.lastInput;//Hall_GetRPM();
			txCmdData.encoderCnt 	= Clock_GetUsLast();
			break;
		case CMD_MOTOR_ENABLE:
			rc = command;
			if(!(Signal_GetMotorState() & MOTOR_MODE_OVERCURRENT) &&
			   !(Signal_GetMotorState() & MOTOR_MODE_OVERCURRENT))
			{
				Signal_SetMotorState(MOTOR_MODE_ENABLE);
			}
			break;
		case CMD_MOTOR_DISABLE:
			rc = command;
			Signal_ClearMotorState(MOTOR_MODE_ENABLE);
			break;
		case CMD_MOTOR_PWM:
			rc = command;
			memcpy(&rxCmdData, &uartRxBuff[1+4], sizeof(rxCmdData));
			Signal_SetMotorPWM(rxCmdData.pwmValue);
			break;
		case CMD_MOTOR_HOME:
			rc = command;
			Signal_SetMotorState(MOTOR_MODE_HOMING);
			break;
		case CMD_MOTOR_POSITION:
			rc = command;
			memcpy(&rxCmdData, &uartRxBuff[1+4], sizeof(rxCmdData));
			Signal_SetMotorPos(rxCmdData.posValue);
			break;
		case CMD_MOTOR_PARMS:
			rc = command;
			memcpy(&rxCmdData, &uartRxBuff[1+4], sizeof(rxCmdData));
			Signal_SetMotorPosKp(rxCmdData.posKp);
			Signal_SetMotorPosKi(rxCmdData.posKi);
			break;
		default:
			break;
	}
	return rc;
}

static void tx_ack(uint32_t *command)
{
	uint32_t txCrc = 0;

	uartTxBuff[0] = START_CHAR;
	memcpy(&uartTxBuff[1], command, 4);
	memcpy(&uartTxBuff[1+4], &txCmdData, sizeof(txCmdData));

	crc32(&uartTxBuff[0], TX_BUFF_SZ-4, &txCrc);

	memcpy(&uartTxBuff[TX_BUFF_SZ-4], &txCrc, 4);

	Serial_TxData(TX_BUFF_SZ);
}

static void tx_nak(void)
{
	uint32_t txCrc = 0;

	uartTxBuff[0] = START_CHAR;
	uint32_t nakVal = 0xFFFFAAAA;
	memcpy(&uartTxBuff[1], &nakVal, 4);

	crc32(&uartTxBuff[0], TX_BUFF_SZ-4, &txCrc);

	memcpy(&uartTxBuff[TX_BUFF_SZ-4], &txCrc, 4);

	Serial_TxData(TX_BUFF_SZ);
}

static void Serial_DMA_Init(void)
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

int Serial_InitPort(uint32_t baudrate, uint32_t stopbits, uint32_t datasize, uint32_t parity)
{
	int rc = HAL_OK;

	Serial_DMA_Init();

	s_UARTHandle.Init.BaudRate   	= baudrate;
	s_UARTHandle.Init.WordLength 	= datasize;
	s_UARTHandle.Init.StopBits   	= stopbits;
	s_UARTHandle.Init.Parity     	= parity;
	s_UARTHandle.Init.Mode       	= USART_MODE_TX_RX;
	s_UARTHandle.Init.HwFlowCtl 	= UART_HWCONTROL_NONE;
	s_UARTHandle.Init.OverSampling 	= UART_OVERSAMPLING_16;
	rc = HAL_UART_Init(&s_UARTHandle);

	return rc;
}

void Serial_RxData(uint16_t Size)
{
	HAL_UART_Receive_DMA(&s_UARTHandle, uartRxBuff, Size);
}

void Serial_TxData(uint16_t Size)
{
	HAL_UART_Transmit_DMA(&s_UARTHandle, uartTxBuff, Size);
}

void Serial_TxData2(uint8_t *txBuff, uint16_t Size)
{
	HAL_UART_Transmit_DMA(&s_UARTHandle, txBuff, Size);
}

void Serial_TxQuery(void)
{
	tx_ack((uint32_t *)CMD_MOTOR_QUERY);
}

/* USER CODE BEGIN 1 */
/*
 * UART Interrupts
 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint32_t thiscrc = 0;
	uint32_t rxCrc = 0;

	// start, data...data, checksum
	if(uartRxBuff[0] == START_CHAR)
	{
		crc32(&uartRxBuff[0], RX_BUFF_SZ-4, &thiscrc);
		rxCrc = (uartRxBuff[RX_BUFF_SZ - 4] << 24) |
				(uartRxBuff[RX_BUFF_SZ - 3] << 16) |
				(uartRxBuff[RX_BUFF_SZ - 2] << 8) |
				(uartRxBuff[RX_BUFF_SZ - 1] << 0);

		if(thiscrc == rxCrc && rxCrc != 0)
		{
			uint32_t command = 0;
			memcpy(&command, &uartRxBuff[1], 4);

			if(proc_command(command) == command)
				tx_ack(&command);
			else
				tx_nak();
		}
		else
		{
			tx_nak();
		}
	}

	HAL_UART_Receive_DMA(huart, &uartRxBuff[0], RX_BUFF_SZ);
}

#if 0
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
	__NOP();
}
#endif

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

void USART1_IRQHandler(void)
{
	HAL_UART_IRQHandler(&s_UARTHandle);
}

void DMA2_Stream2_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&hdma_usart1_rx);
}

void DMA2_Stream7_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&hdma_usart1_tx);
}

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
	    hdma_usart1_rx.Init.FIFOMode 			= DMA_FIFOMODE_DISABLE;
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
		HAL_NVIC_SetPriority(USART1_IRQn, 1, 1);
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
