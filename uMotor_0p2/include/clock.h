#ifndef CLOCK_H_ /* include guard */
#define CLOCK_H_

#include "stm32f4xx.h"

#define CLOCK_FREQ		25000

#define LOOPF			(PWM_F * 2)/1000
#define TIME_ELAP_US	1000000/(PWM_F * 2)
#define TIME_ELAP_MS	1000/(PWM_F * 2)

#define SVM_DIVISOR		20
#define SVM_RATE		(PWM_F * 2)/SVM_DIVISOR


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


float Clock_GetTimeS(void);
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
