#include "hall.h"

static void hall_monitor(uint8_t hall_num, int state);

static volatile uint32_t hall_steps 	= 0;
static volatile int hall_state[3] 		= {0};
static volatile int hall_stateLast[3] 	= {-1};
static float hall_currentRpmValue 		= 0.0f;


static void hall_monitor(uint8_t hall_num, int state)
{
	// nop's give a slight delay
	__NOP();
	__NOP();
	hall_state[hall_num] = state;
	if(hall_state[hall_num] != hall_stateLast[hall_num])
	{
		hall_stateLast[hall_num] = hall_state[hall_num];
		Signal_MuxUpdate();

		// inc hall state steps
		hall_steps++;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
	switch (GPIO_Pin)
	{
		case HALL_A_PIN:
			hall_monitor(HALL_A, READ_H(HALL_A_PIN) && HALL_A_PIN);
			break;
		case HALL_B_PIN:
			hall_monitor(HALL_B, READ_H(HALL_B_PIN) && HALL_B_PIN);
			break;
		case HALL_C_PIN:
			hall_monitor(HALL_C, READ_H(HALL_C_PIN) && HALL_C_PIN);
			break;
		default:
			break;
	}
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void Hall_InputInit(void)
{
	GPIO_InitTypeDef gAPin;
	gAPin.Pin 		= HALL_A_PIN | HALL_B_PIN | HALL_C_PIN;
	gAPin.Mode 		= GPIO_MODE_IT_RISING_FALLING;
	gAPin.Pull 		= GPIO_NOPULL;
	gAPin.Speed 	= GPIO_SPEED_LOW;
	HAL_GPIO_Init(HALL_PORT, &gAPin);
	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void Hall_ComputeRPM(float timeStep)
{
	hall_currentRpmValue = hall_currentRpmValue*0.6f + ((((float)Hall_GetSteps() / 24.0f) / timeStep) * 60.0f)*0.4f;
	Hall_SetSteps(0);
}

float Hall_GetRPM(void)
{
	return hall_currentRpmValue;
}

uint32_t Hall_GetSteps(void)
{
	return hall_steps;
}

void Hall_SetSteps(uint32_t val)
{
	hall_steps = val;
}