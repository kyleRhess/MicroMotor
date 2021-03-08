#ifndef SIGNAL_H_ /* include guard */
#define SIGNAL_H_

#include "system.h"

void Signal_Init(void);
void Signal_SetMotorState(int state);
int Signal_GetMotorState();
void Signal_SetMotorPWM(float speed);
void Signal_SetMotorPos(float position);
void Signal_SetMotorPosKp(float kp);
void Signal_SetMotorPosKi(float ki);
float Signal_GetMotorPosKp();
float Signal_GetMotorPosKi();
float Signal_GetMotorPWM(void);
float Signal_GetMotorPos(void);
uint32_t Signal_GetMotorMode(void);
void Signal_SetDirection(int dir);

#endif /* SIGNAL_H_ */
