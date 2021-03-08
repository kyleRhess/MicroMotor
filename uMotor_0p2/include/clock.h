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

typedef struct ClockTimerus
{
	int 		timerActive;
	uint32_t	timeUsLast;
	uint32_t	timeRemaining;
} ClockTimerus;

volatile int m_bRunCurrentLoop;

uint32_t Clock_GetMs(void);
uint32_t Clock_GetMsLast(void);
uint32_t Clock_GetUs(void);
uint32_t Clock_GetUsLast(void);
void Clock_StartTimer(ClockTimer *ct, uint32_t periodMs);
void Clock_StartTimerUs(ClockTimerus *ct, uint32_t periodUs);
int Clock_UpdateTimer(ClockTimer *ct);
int Clock_UpdateTimerUs(ClockTimerus *ct);
int Clock_InitSamplingTimer(void);

#endif /* CLOCK_H_ */
