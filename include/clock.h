#ifndef CLOCK_H_ /* include guard */
#define CLOCK_H_

#include "stm32f4xx.h"

#define CLOCK_FREQ	25000

typedef struct ClockTimer
{
	int 		timerActive;
	uint32_t	timeMsLast;
	uint32_t	timeRemaining;
} ClockTimer;


uint32_t Clock_GetMs(void);
uint32_t Clock_GetMsLast(void);
void Clock_StartTimer(ClockTimer *ct, uint32_t periodMs);
int Clock_UpdateTimer(ClockTimer *ct);
int Clock_InitSamplingTimer(void);

#endif /* CLOCK_H_ */
