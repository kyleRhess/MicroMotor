#ifndef SIGNAL_H_ /* include guard */
#define SIGNAL_H_

#include "system.h"

#define MOTOR_MODE_DEFAULT		0xF0000000
#define MOTOR_MODE_ENABLE		0x00000001
#define MOTOR_MODE_SPEED		0x00000002
#define MOTOR_MODE_REVERSING	0x00000004
#define MOTOR_MODE_OVERCURRENT	0x00000008
#define MOTOR_MODE_OVERSPEED	0x00000010
#define MOTOR_MODE_HOMING	    0x00000020
#define MOTOR_MODE_POSITION	  	0x00000040
#define MOTOR_MODE_NOHEART	    0x00000080
#define MOTOR_MODE_TORQUE	   	0x00000100
#define MOTOR_MODE_CRUISING	   	0x00000200
#define MOTOR_MODE_UNDERVOLT	0x00000400

#define MOTOR_MODE_TBD6	        0x00000400
#define MOTOR_MODE_TBD7	        0x00000800
#define MOTOR_MODE_TBD8	        0x00001000
#define MOTOR_MODE_TBD9	        0x00002000
#define MOTOR_MODE_TBD10        0x00004000
#define MOTOR_MODE_TBD11        0x00008000
#define MOTOR_MODE_TBD12        0x00010000
#define MOTOR_MODE_TBD13        0x00020000
#define MOTOR_MODE_TBD14        0x00040000
#define MOTOR_MODE_TBD15        0x00080000
#define MOTOR_MODE_TBD16        0x00100000
#define MOTOR_MODE_TBD17        0x00200000
#define MOTOR_MODE_TBD18        0x00400000
#define MOTOR_MODE_TBD19        0x00800000
#define MOTOR_MODE_TBD20        0x01000000
#define MOTOR_MODE_TBD21        0x02000000
#define MOTOR_MODE_TBD22        0x04000000
#define MOTOR_MODE_TBD23        0x08000000

void Signal_Init(void);
void Signal_ResetMotor(void);
void Signal_SetMotorState(uint32_t state);
void Signal_ClearMotorState(uint32_t state);
uint32_t Signal_GetMotorState();
void Signal_SetMotorSpeed(float speed);
void Signal_SetMotorTorque(float torque);
void Signal_SetMotorPos(float position);
void Signal_SetParam(uint8_t paramID, float paramValue);
float Signal_GetMotorSpeed(void);
float Signal_GetMotorPos(void);
float Signal_GetMotorTorque(void);
void Signal_SetHeartBeatMs(uint32_t millis);
uint32_t Signal_GetHeartBeatMs(void);

#endif /* SIGNAL_H_ */
