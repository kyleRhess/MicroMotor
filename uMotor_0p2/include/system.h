#ifndef SYSTEM_H_ /* include guard */
#define SYSTEM_H_

#include "clock.h"
#include "cmsis_device.h"
#include "diag/Trace.h"
#include "encoder.h"
#include "hall.h"
#include "main.h"
#include "pid.h"
#include "pwm.h"
#include "serial.h"
#include "signal.h"
#include "foc.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_conf_template.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_tim_ex.h"

#define HALL_PORT 			GPIOC
#define HALL_A_PIN			GPIO_PIN_11
#define HALL_B_PIN			GPIO_PIN_10
#define HALL_C_PIN			GPIO_PIN_12
	
#define UART_PORT			GPIOB
#define UART_TX_PIN			GPIO_PIN_6
#define UART_RX_PIN			GPIO_PIN_7
	
#define ENCODER_PORT		GPIOA
#define ENCODER_A_PIN		GPIO_PIN_0
#define ENCODER_B_PIN		GPIO_PIN_1
	
#define ENCODER_Z_PORT		GPIOC
#define ENCODER_Z_PIN		GPIO_PIN_15
	
#define PWM_PORT			GPIOA
#define PWM_POS_PIN			GPIO_PIN_8
#define PWM_NEG_PIN			GPIO_PIN_9

#define ADC_PORT			GPIOA
#define ADC_A_PIN			GPIO_PIN_3
#define ADC_B_PIN			GPIO_PIN_2
#define ADC_THROT_PIN		GPIO_PIN_0
#define ADC_BAT_PORT		GPIOC
#define ADC_BAT_PIN			GPIO_PIN_5

#define PIN_LOW_A			GPIO_PIN_8
#define PIN_LOW_B			GPIO_PIN_9
#define PIN_LOW_C			GPIO_PIN_10

#define POWER_SW_PORT		ENCODER_Z_PORT
#define POWER_SW_PIN		ENCODER_Z_PIN


void System_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
uint8_t System_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
float System_mapVal(float x, float in_min, float in_max, float out_min, float out_max);

#endif /* SYSTEM_H_ */
