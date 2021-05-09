#ifndef CLOCK_H_ /* include guard */
#define CLOCK_H_

#include "system.h"
#include "stm32f4xx.h"

#define LOOPF			(PWM_F * 2)/1000
#define TIME_ELAP_US	1000000/(PWM_F * 2)
#define TIME_ELAP_MS	1000/(PWM_F * 2)

#define SVM_DIVISOR		10//20
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


uint32_t Clock_GetMs(void);
uint32_t Clock_GetUs(void);
void Clock_StartTimer(ClockTimer *ct, uint32_t periodMs);
void Clock_StartTimerUs(ClockTimerus *ct, uint32_t periodUs);
int Clock_UpdateTimer(ClockTimer *ct);
int Clock_UpdateTimerUs(ClockTimerus *ct);

#endif /* CLOCK_H_ */
