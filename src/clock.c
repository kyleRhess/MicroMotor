#include "clock.h"

// Background timer for keeping time (25kHz)
static TIM_HandleTypeDef SamplingTimer 	= { .Instance = TIM9 };
static volatile uint32_t timeElapUs 	= 0;
static volatile uint32_t timeElapMs 	= 0;
static volatile uint32_t timeElapUsLast = 0;
static volatile uint32_t timeElapMsLast = 0;

uint32_t Clock_GetMs(void)
{
	return timeElapMs;
}

uint32_t Clock_GetMsLast(void)
{
	return timeElapMsLast;
}

void Clock_StartTimer(ClockTimer *ct, uint32_t periodMs)
{
	ct->timeRemaining 	= periodMs;
	ct->timeMsLast 		= timeElapMs;
	ct->timerActive 	= 1;
}

int Clock_UpdateTimer(ClockTimer *ct)
{
	int rc = 0;
	if((timeElapMs - ct->timeMsLast) >= ct->timeRemaining)
	{
		ct->timeMsLast = Clock_GetMs();
		rc = 1;
	}
	return rc;
}

int Clock_InitSamplingTimer(void)
{
	__HAL_RCC_TIM9_CLK_ENABLE();

	// 25 kHz = 100E6/((49+1)*(79+1)*(1))
    SamplingTimer.Init.Prescaler 		= 49;
    SamplingTimer.Init.CounterMode 		= TIM_COUNTERMODE_UP;
    SamplingTimer.Init.Period 			= 79;
    SamplingTimer.Init.ClockDivision 	= TIM_CLOCKDIVISION_DIV1;

    if(HAL_TIM_Base_Init(&SamplingTimer) != HAL_OK)
    	return HAL_ERROR;

    if(HAL_TIM_Base_Start_IT(&SamplingTimer) != HAL_OK)
    	return HAL_ERROR;

    return HAL_OK;
}

void TIM1_BRK_TIM9_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&SamplingTimer);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM9)
	{
		timeElapUs += (1000000 / 25000);

		if((timeElapUs - timeElapUsLast) >= 1000)
		{
			// 1 ms tick
			timeElapMs += 1;
			timeElapMsLast = timeElapMs;
			timeElapUsLast = timeElapUs;
		}
	}
}
