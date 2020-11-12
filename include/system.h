#ifndef SYSTEM_H_ /* include guard */
#define SYSTEM_H_

#define HALL_PORT 		GPIOC
#define HALL_A_PIN		GPIO_PIN_11
#define HALL_B_PIN		GPIO_PIN_10
#define HALL_C_PIN		GPIO_PIN_12

#define UART_PORT		GPIOB
#define UART_TX_PIN		GPIO_PIN_6
#define UART_RX_PIN		GPIO_PIN_7

#define ENCODER_PORT	GPIOA
#define ENCODER_A_PIN	GPIO_PIN_0
#define ENCODER_B_PIN	GPIO_PIN_1

#define ENCODER_Z_PORT	GPIOC
#define ENCODER_Z_PIN	GPIO_PIN_15


#define PWM_PORT		GPIOA
#define PWM_POS_PIN		GPIO_PIN_8
#define PWM_NEG_PIN		GPIO_PIN_9



#include "stm32f4xx_hal.h"



void WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
uint8_t ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

float mapVal(float x, float in_min, float in_max, float out_min, float out_max);



#endif /* SYSTEM_H_ */
