#include "encoder.h"

static TIM_HandleTypeDef qTimer;


/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
void Encoder_Init(void)
{
	TIM_Encoder_InitTypeDef sConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	qTimer.Instance 			= TIM5;
	qTimer.Init.Prescaler 		= 0;
	qTimer.Init.CounterMode 	= TIM_COUNTERMODE_UP;
	qTimer.Init.Period 		= 0xFFFFFFFF;
	qTimer.Init.ClockDivision 	= TIM_CLOCKDIVISION_DIV1;
	sConfig.EncoderMode 		= TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity 		= TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection 		= TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler 		= TIM_ICPSC_DIV1;
	sConfig.IC1Filter 			= 0;
	sConfig.IC2Polarity 		= TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection 		= TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler 		= TIM_ICPSC_DIV1;
	sConfig.IC2Filter 			= 0;

	if (HAL_TIM_Encoder_Init(&qTimer, &sConfig) != HAL_OK)
	{
		while(1){;;}
	}

	sMasterConfig.MasterOutputTrigger 	= TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode 		= TIM_MASTERSLAVEMODE_DISABLE;

	if (HAL_TIMEx_MasterConfigSynchronization(&qTimer, &sMasterConfig) != HAL_OK)
	{
		while(1){;;}
	}

	HAL_TIM_Encoder_Start(&qTimer, TIM_CHANNEL_1 | TIM_CHANNEL_2);
}

void Encoder_ZInit(void)
{
	// OUT_Z pin setup
	GPIO_InitTypeDef gZPin;
	gZPin.Pin 		= ENCODER_Z_PIN;
	gZPin.Mode 		= GPIO_MODE_IT_RISING;
	gZPin.Pull 		= GPIO_NOPULL;
	gZPin.Speed 	= GPIO_SPEED_HIGH;
	HAL_GPIO_Init(ENCODER_Z_PORT, &gZPin);
	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 1);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

uint32_t Encoder_GetCounts(void)
{
	return qTimer.Instance->CNT;
}

void Encoder_Reset(void)
{
	qTimer.Instance->CNT = 0;
}

/**
* @brief This function handles EXTI line.
*/
void EXTI15_10_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(ENCODER_Z_PIN);
	HAL_GPIO_EXTI_IRQHandler(HALL_A_PIN);
	HAL_GPIO_EXTI_IRQHandler(HALL_B_PIN);
	HAL_GPIO_EXTI_IRQHandler(HALL_C_PIN);
}

/**
* @brief TIM_Encoder MSP Initialization
* This function configures the hardware resources used in this example
* @param htim_encoder: TIM_Encoder handle pointer
* @retval None
*/
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef* htim_encoder)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim_encoder->Instance==TIM5)
  {
  /* USER CODE BEGIN TIM5_MspInit 0 */

  /* USER CODE END TIM5_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM5_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM5 GPIO Configuration
    PA0-WKUP     ------> TIM5_CH1
    PA1     ------> TIM5_CH2
    */
    GPIO_InitStruct.Pin 		= ENCODER_A_PIN | ENCODER_B_PIN;
    GPIO_InitStruct.Mode 		= GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull 		= GPIO_NOPULL;
    GPIO_InitStruct.Speed 		= GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate 	= GPIO_AF2_TIM5;
    HAL_GPIO_Init(ENCODER_PORT, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM5_MspInit 1 */

  /* USER CODE END TIM5_MspInit 1 */
  }

}

/**
* @brief TIM_Encoder MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param htim_encoder: TIM_Encoder handle pointer
* @retval None
*/

void HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef* htim_encoder)
{

  if(htim_encoder->Instance==TIM5)
  {
  /* USER CODE BEGIN TIM5_MspDeInit 0 */

  /* USER CODE END TIM5_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM5_CLK_DISABLE();

    /**TIM5 GPIO Configuration
    PA0-WKUP     ------> TIM5_CH1
    PA1     ------> TIM5_CH2
    */
    HAL_GPIO_DeInit(ENCODER_PORT, ENCODER_A_PIN | ENCODER_B_PIN);

  /* USER CODE BEGIN TIM5_MspDeInit 1 */

  /* USER CODE END TIM5_MspDeInit 1 */
  }

}
