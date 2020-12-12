#ifndef CLOCK_H_ /* include guard */
#define CLOCK_H_

#include "stm32f4xx.h"

#define CLOCK_FREQ	25000

typedef struct CLOCK_TIMER
{
	int 		timerActive;
	uint32_t	timeMsLast;
	uint32_t	timeRemaining;
} CLOCK_TIMER;

uint32_t clock_GetMs(void);
uint32_t clock_GetMsLast(void);
void clock_StartTimer(CLOCK_TIMER *ct, uint32_t periodMs);
int clock_UpdateTimer(CLOCK_TIMER *ct);
int InitSamplingTimer(void);

#endif /* CLOCK_H_ */
