#ifndef SERIAL_H_ /* include guard */
#define SERIAL_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "system.h"

#define MSG_RATE_HZ(xxx) 		(SAMPLE_RATE / xxx)

#define RX_BUFF_SZ 				37
#define TX_BUFF_SZ 				37

#define START_CHAR 				0xAA

#define CMD_SET_OUT_DAT 		0x01
#define CMD_SET_OUT_RATE 		0x02
#define CMD_SET_PID_GAIN 		0x03

#define CMD_MOTOR_ENABLE		0x00000001
#define CMD_MOTOR_DISABLE		0x00000002
#define CMD_MOTOR_PWM			0x00000004
#define CMD_MOTOR_MODE			0x00000008
#define CMD_MOTOR_QUERY			0x00000010
#define CMD_MOTOR_RESET			0x00000020

#define MOTOR_MODE_DISABLE		0x00000001
#define MOTOR_MODE_ENABLE		0x00000002
#define MOTOR_MODE_BIPOLAR		0x00000004
#define MOTOR_MODE_UNIPOLAR		0x00000008
#define MOTOR_MODE_FORWARD		0x00000010
#define MOTOR_MODE_BAKWARD		0x00000020

#define MOTOR_MODE_BAKWARD		0x00000020
#define MOTOR_MODE_BAKWARD		0x00000020
#define MOTOR_MODE_BAKWARD		0x00000020
#define MOTOR_MODE_BAKWARD		0x00000020
#define MOTOR_MODE_BAKWARD		0x00000020
#define MOTOR_MODE_BAKWARD		0x00000020

#define PWM_INC_F	0.0001f

typedef enum
{
	STATUS_ENABLE = 0,
	STATUS_RPM,
	STATUS_PWM,
	STATUS_ODR,
	STATUS_PID,
	STATUS_NUM
} StatusNum;

typedef struct DataFields
{
	float		pwmValue; 	// 1
	uint32_t	driveMode;	// 2
	float		rpmValue;	// 3
	uint32_t	encoderCnt;	// 4
	uint32_t	TBD3;	// 5
	uint32_t	TBD4;	// 6
	uint32_t	TBD5;	// 7
} __attribute__((__packed__)) DataFields;

int Serial_InitPort(uint32_t baudrate, uint32_t stopbits, uint32_t datasize, uint32_t parity);
void Serial_RxData(uint16_t Size);
void Serial_TxData(uint16_t Size);

#endif /* SERIAL_H_ */
