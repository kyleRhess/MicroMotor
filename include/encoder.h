#ifndef ENCODER_H_ /* include guard */
#define ENCODER_H_

#include <stdio.h>
#include "stm32f4xx.h"

extern TIM_HandleTypeDef q_time;

void encoder_Init(TIM_HandleTypeDef *timer);
void encoder_ZInit(void);

#endif /* ENCODER_H_ */
