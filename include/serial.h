#ifndef SERIAL_H_ /* include guard */
#define SERIAL_H_

#include <stdio.h>
#include <stdlib.h>
#include "stm32f4xx.h"

#define MSG_RATE_HZ(xxx) 	(SAMPLE_RATE / xxx)

#define RX_BUFF_SZ 			37
#define TX_BUFF_SZ 			37

#define START_CHAR 			0xAA
#define SERIAL_CMD_ACK 		"ACK"
#define SERIAL_CMD_NAK 		"NAK"

#define CMD_SET_OUT_DAT 	0x01
#define CMD_SET_OUT_RATE 	0x02
#define CMD_SET_PID_GAIN 	0x03

#define SERIAL_MSG_DRONE 	0x01
#define SERIAL_MSG_GPS	 	0x02

#define SERIAL_MSG_START 	0x4A
#define SERIAL_CMD_START 	0x7F
#define SERIAL_CMD_END 		0xF7

#define PWM_INC_F	0.0001f

typedef enum
{
	STATUS_ENABLE = 0,
	STATUS_RPM,
	STATUS_PWM,
	STATUS_ODR,
	STATUS_PID,
	STATUS_NUM
} STATUS;

extern uint8_t uartRx[RX_BUFF_SZ];
extern uint8_t uartTx[TX_BUFF_SZ];

uint8_t 	rxIndexA;
uint8_t 	rxIndexB;
uint8_t 	rxBufferSwitch;
uint16_t 	connLoss;
uint8_t 	firstSync;
uint16_t 	serialODR;
uint8_t 	serialMSG;
uint8_t		handshakeCMD;

struct DataMsg {
	uint8_t start;
	uint32_t dat[32];
	uint32_t cksum;
} __attribute__((__packed__));




struct DataMsg datMsg;

int InitSerial(uint32_t baudrate, uint32_t stopbits, uint32_t datasize, uint32_t parity);
void MX_DMA_Init(void);
void tx_serial_data(uint8_t *pData, uint16_t Size);
void rx_serial_data(uint8_t *pData, uint16_t Size);
void crc32_(const void *data, size_t n_bytes, uint32_t* crc);
void crc32(const void *data, size_t n_bytes, uint32_t* crc);
float toFloat(uint8_t bytes[], int startI);
void decodePIDGains(uint8_t *payload);


#endif /* SERIAL_H_ */
