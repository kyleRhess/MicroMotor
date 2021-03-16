
#include "adc.h"

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

static uint32_t g_ADCValue1 = 0;
static float g_ADCValue2 	= 0;
static float g_ADCValue3 	= 0;
static uint32_t g_ADCBuffer[ADC_BUF_LEN];

void ADC_Init(void)
{
	ADC_ChannelConfTypeDef sConfig 		= {0};

	hadc1.Instance 						= ADC1;
	hadc1.Init.ClockPrescaler 			= ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution 				= ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode 			= ENABLE;
	hadc1.Init.ContinuousConvMode 		= DISABLE;
	hadc1.Init.DiscontinuousConvMode 	= DISABLE;
	hadc1.Init.ExternalTrigConvEdge 	= ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv 		= ADC_SOFTWARE_START;
	hadc1.Init.DataAlign 				= ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion 			= 3;
	hadc1.Init.DMAContinuousRequests 	= ENABLE;
	hadc1.Init.EOCSelection 			= ADC_EOC_SINGLE_CONV;

	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	sConfig.Channel 		= ADC_CHANNEL_2;
	sConfig.Rank 			= 1;
	sConfig.SamplingTime 	= ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	sConfig.Channel 		= ADC_CHANNEL_3;
	sConfig.Rank 			= 2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	sConfig.Channel 		= ADC_CHANNEL_15;
	sConfig.Rank 			= 3;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&g_ADCBuffer, ADC_BUF_LEN);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
#define FILT_K		0.90f
#ifdef FILT_K
#define FILT_K_1	(1 - FILT_K)
	g_ADCValue1 = g_ADCBuffer[2];
	g_ADCValue2 = g_ADCValue2*FILT_K_1 + ((float)g_ADCBuffer[0])*FILT_K;
	g_ADCValue3 = g_ADCValue3*FILT_K_1 + ((float)g_ADCBuffer[1])*FILT_K;
#else
	g_ADCValue2 = (float)g_ADCBuffer[1];
	g_ADCValue3 = (float)g_ADCBuffer[2];
#endif

	m_fCurrentA = ((((float)g_ADCValue3 * ADC_SCALE) - ADC_ZERO) * ADC_RES);
	m_fCurrentB = ((((float)g_ADCValue2 * ADC_SCALE) - ADC_ZERO) * ADC_RES);

	// EMRG stop if current gets too high
	if(m_fCurrentA > MAX_CURRENT || m_fCurrentB > MAX_CURRENT || m_fCurrentA < -MAX_CURRENT || m_fCurrentB < -MAX_CURRENT )
	{
		while(1)
		{
			Signal_SetMotorState(MOTOR_MODE_DISABLE);
		}
	}

	Run_SVM();
}
