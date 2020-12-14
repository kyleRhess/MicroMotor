#ifndef ENCODER_H_ /* include guard */
#define ENCODER_H_

#include <stdio.h>

#include "stm32f4xx.h"
#include "system.h"

void Encoder_Init(void);
void Encoder_ZInit(void);
uint32_t Encoder_GetCounts(void);
void Encoder_Reset(void);

#endif /* ENCODER_H_ */
