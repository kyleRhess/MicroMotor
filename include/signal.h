#ifndef SIGNAL_H_ /* include guard */
#define SIGNAL_H_

#include "system.h"

void Signal_Init(void);
void Signal_SetMotorState(int state);
void Signal_SetMotorPWM(float speed);
float Signal_GetMotorPWM(void);
uint32_t Signal_GetMotorMode(void);
void Signal_MuxUpdate(void);
void Signal_SetDirection(int dir);

#endif /* SIGNAL_H_ */
