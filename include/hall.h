#include <stdio.h>
#include "stm32f4xx.h"

//#define BI_POLAR
#ifndef BI_POLAR
#define UNI_POLAR
#endif

//GPIOC
#define GPIO_S0_PIN		GPIO_PIN_2
#define GPIO_S1_PIN		GPIO_PIN_0
#define GPIO_S2_PIN		GPIO_PIN_5
#define GPIO_ALEN_PIN	GPIO_PIN_1
#define GPIO_BLEN_PIN	GPIO_PIN_4

//GPIOA
#define GPIO_S3_PIN		GPIO_PIN_6

//GPIOB
#define GPIO_S4_PIN		GPIO_PIN_14
#define GPIO_S5_PIN		GPIO_PIN_12
#define GPIO_CLEN_PIN	GPIO_PIN_13
#define GPIO_DIS_PIN	GPIO_PIN_15

//MUX States
#define MUX_A				0x0A
#define MUX_B				0x0B
#define MUX_C				0x0C
#define	MUX_STATE_P_PWM		0x01
#define	MUX_STATE_M_PWM		0x02
#define	MUX_STATE_HI		0x03
#define	MUX_STATE_Z			0x04
#define	MUX_STATE_LO		0x05
#define	MUX_STATE_P_PWM_T	0x06
#define	MUX_STATE_M_PWM_T	0x07
#define	MUX_STATE_TBD4		0x08

#define READ_H(xxx)	(HALL_PORT->IDR & xxx)

#define STATE_1		READ_H(HALL_A_PIN) && !READ_H(HALL_B_PIN) && !READ_H(HALL_C_PIN)
#define STATE_2		READ_H(HALL_A_PIN) && READ_H(HALL_B_PIN) && !READ_H(HALL_C_PIN)
#define STATE_3		!READ_H(HALL_A_PIN) && READ_H(HALL_B_PIN) && !READ_H(HALL_C_PIN)
#define STATE_4		!READ_H(HALL_A_PIN) && READ_H(HALL_B_PIN) && READ_H(HALL_C_PIN)
#define STATE_5		!READ_H(HALL_A_PIN) && !READ_H(HALL_B_PIN) && READ_H(HALL_C_PIN)
#define STATE_6		READ_H(HALL_A_PIN) && !READ_H(HALL_B_PIN) && READ_H(HALL_C_PIN)

#define DRIVE_FORWARD	0
#define DRIVE_BAKWARD	1

volatile uint32_t	hall_steps;

extern volatile int hall_a_timer;
extern volatile int hall_a_state;
extern volatile int hall_a_stateLast;
extern volatile int hall_b_timer;
extern volatile int hall_b_state;
extern volatile int hall_b_stateLast;
extern volatile int hall_c_timer;
extern volatile int hall_c_state;
extern volatile int hall_c_stateLast;


void Encoder_Z_Init(void);
void Hall_Input_Init(void);
void setMuxState(uint8_t mux, uint8_t state);

uint32_t get_hall_steps(void);
void set_hall_steps(uint32_t val);
void set_direction(int dir);
