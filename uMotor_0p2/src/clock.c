

#include "clock.h"
#include "adc.h"

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

uint32_t Clock_GetUs(void)
{
	return timeElapUs;
}

uint32_t Clock_GetUsLast(void)
{
	return timeElapUsLast;
}

void Clock_StartTimer(ClockTimer *ct, uint32_t periodMs)
{
	ct->timeRemaining 	= periodMs;
	ct->timeMsLast 		= timeElapMs;
	ct->timerActive 	= 1;
}

void Clock_StartTimerUs(ClockTimerus *ct, uint32_t periodUs)
{
	ct->timeRemaining 	= periodUs;
	ct->timeUsLast 		= timeElapUs;
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

int Clock_UpdateTimerUs(ClockTimerus *ct)
{
	int rc = 0;
	if((timeElapUs - ct->timeUsLast) >= ct->timeRemaining)
	{
		ct->timeUsLast = Clock_GetUs();
		rc = 1;
	}
	return rc;
}

void TIM1_BRK_TIM9_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&SamplingTimer);
}

static int timerDivisor = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	TIM1->SR  		= 0x00;
	timeElapUs 		+= TIME_ELAP_US;
	timerDivisor++;

#ifdef DEBUG_PIN
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
#endif

	if(timerDivisor >= SVM_DIVISOR)
	{
		timerDivisor 	= 0;
		hadc1.Instance->CR2 |= ADC_CR2_SWSTART;
	}

	// 1 ms tick
	if((timeElapUs - timeElapUsLast) >= 1000)
	{
		timeElapMs += 1;
		timeElapMsLast = timeElapMs;
		timeElapUsLast = timeElapUs;
	}
}
