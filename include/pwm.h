#ifndef PWM_H_ /* include guard */
#define PWM_H_

#include <string.h>
#include <math.h>

#include "system.h"

#ifdef BI_POLAR
#define PWM_F		18000 	// Hz
#define DUTY_LIM_H	95.0	// %
#define DUTY_LIM_L	5.0		// %
#else
#define PWM_F		50000 	// Hz
#define DUTY_LIM_H	99.9	// %
#define DUTY_LIM_L	0.0		// %
#endif

#define CLOCK_CYCLES_PER_SECOND 100000000
#define PWM_PRESCALE			1
#define PWM_PERIOD			 	((CLOCK_CYCLES_PER_SECOND/2) / PWM_F) - 1 							// Freq = 100000000 / ((PWM_PRESCALE + 1) * (PWM_PERIOD + 1))
#define PWM_FREQ 				(CLOCK_CYCLES_PER_SECOND / ((PWM_PRESCALE + 1) * (PWM_PERIOD + 1)))
#define PULSE_NS_PER_CNT		(10 * (PWM_PRESCALE+1))  											// = (10ns per Prescaler count) + 10ns
#define CNTS_FROM_US(xxx)		((xxx * 1000) / PULSE_NS_PER_CNT)
#define MAX_RELOAD              0xFFFF

typedef struct PWM_Out
{
	TIM_HandleTypeDef 	timer;
	float 				dutyCycle;
	uint32_t 			frequency;
	uint32_t 			Channel;
	TIM_TypeDef*		TIM;
} PWM_Out;

TIM_HandleTypeDef PWM_Init(void);
void PWM_adjust_DutyCycle(float dutyCycle);
void PWM_adjust_Frequency(uint32_t newFreq);
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm);
int PWM_Init_Output(void);

#endif /* PWM_H_ */
