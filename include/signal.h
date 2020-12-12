#ifndef SIGNAL_H_ /* include guard */
#define SIGNAL_H_

#include "stm32f4xx.h"

#define CMD_MOTOR_ENABLE		0x00000001
#define CMD_MOTOR_DISABLE		0x00000002
#define CMD_MOTOR_PWM			0x00000004
#define CMD_MOTOR_MODE			0x00000008
#define CMD_MOTOR_QUERY			0x00000010

#define MOTOR_MODE_DISABLE		0x00000001
#define MOTOR_MODE_ENABLE		0x00000002
#define MOTOR_MODE_BIPOLAR		0x00000004
#define MOTOR_MODE_UNIPOLAR		0x00000008
#define MOTOR_MODE_FORWARD		0x00000010
#define MOTOR_MODE_BAKWARD		0x00000020

extern int signal_currentEnableState;
extern int signal_driveDirection;
extern float signal_currentPwmValue;

void signal_Init(void);
void signal_SetMotorState(int state);
void signal_SetMotorPWM(float speed);
uint32_t signal_GetMotorMode(void);
void signal_MuxUpdate(void);
void signal_SetMuxState(uint8_t mux, uint8_t state);
void signal_SetDirection(int dir);

#endif /* SIGNAL_H_ */
