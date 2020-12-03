#include "hall.h"
#include "system.h"
#include "Serial.h"


volatile uint32_t 	hall_steps 	= 0;

volatile int hall_a_timer		= 0;
volatile int hall_a_state 		= 0;
volatile int hall_a_stateLast 	= -1;

volatile int hall_b_timer		= 0;
volatile int hall_b_state 		= 0;
volatile int hall_b_stateLast 	= -1;

volatile int hall_c_timer		= 0;
volatile int hall_c_state 		= 0;
volatile int hall_c_stateLast 	= -1;

volatile int drive_direction	= DRIVE_FORWARD; // 0 = forward

//
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);

	static int goodToGo = 0;
	goodToGo = 0;

	switch (GPIO_Pin)
	{
		case HALL_A_PIN:
			__NOP();
			__NOP();
			hall_a_state = READ_H(HALL_A_PIN) && 2048;

			if(hall_a_state != hall_a_stateLast)
			{
				hall_a_stateLast = hall_a_state;
				goodToGo = 1;
			}
			break;
		case HALL_B_PIN:
			__NOP();
			__NOP();
			hall_b_state = READ_H(HALL_B_PIN) && 1024;

			if(hall_b_state != hall_b_stateLast)
			{
				hall_b_stateLast = hall_b_state;
				goodToGo = 1;
			}
			break;
		case HALL_C_PIN:
			__NOP();
			__NOP();
			hall_c_state = READ_H(HALL_C_PIN) && 4096;

			if(hall_c_state != hall_c_stateLast)
			{
				hall_c_stateLast = hall_c_state;
				goodToGo = 1;
			}
			break;
		default:
			break;
	}


	if(drive_direction == DRIVE_FORWARD && goodToGo)
	{
	if(STATE_1)
	{
#ifdef BI_POLAR
		setMuxState(MUX_A, MUX_STATE_P_PWM);
		setMuxState(MUX_B, MUX_STATE_Z);
		setMuxState(MUX_C, MUX_STATE_M_PWM);
#else
		setMuxState(MUX_A, MUX_STATE_P_PWM_T);
		setMuxState(MUX_B, MUX_STATE_Z);
		setMuxState(MUX_C, MUX_STATE_HI);
#endif
	}
	else if(STATE_2)
	{
#ifdef BI_POLAR
		setMuxState(MUX_A, MUX_STATE_Z);
		setMuxState(MUX_B, MUX_STATE_P_PWM);
		setMuxState(MUX_C, MUX_STATE_M_PWM);
#else
		setMuxState(MUX_A, MUX_STATE_Z);
		setMuxState(MUX_B, MUX_STATE_P_PWM_T);
		setMuxState(MUX_C, MUX_STATE_HI);
#endif
	}
	else if(STATE_3)
	{
#ifdef BI_POLAR
		setMuxState(MUX_A, MUX_STATE_M_PWM);
		setMuxState(MUX_B, MUX_STATE_P_PWM);
		setMuxState(MUX_C, MUX_STATE_Z);
#else
		setMuxState(MUX_A, MUX_STATE_HI);
		setMuxState(MUX_B, MUX_STATE_P_PWM_T);
		setMuxState(MUX_C, MUX_STATE_Z);
#endif
	}
	else if(STATE_4)
	{
#ifdef BI_POLAR
		setMuxState(MUX_A, MUX_STATE_M_PWM);
		setMuxState(MUX_B, MUX_STATE_Z);
		setMuxState(MUX_C, MUX_STATE_P_PWM);
#else
		setMuxState(MUX_A, MUX_STATE_HI);
		setMuxState(MUX_B, MUX_STATE_Z);
		setMuxState(MUX_C, MUX_STATE_P_PWM_T);
#endif
	}
	else if(STATE_5)
	{
#ifdef BI_POLAR
		setMuxState(MUX_A, MUX_STATE_Z);
		setMuxState(MUX_B, MUX_STATE_M_PWM);
		setMuxState(MUX_C, MUX_STATE_P_PWM);
#else
		setMuxState(MUX_A, MUX_STATE_Z);
		setMuxState(MUX_B, MUX_STATE_HI);
		setMuxState(MUX_C, MUX_STATE_P_PWM_T);
#endif
	}
	else if(STATE_6)
	{
#ifdef BI_POLAR
		setMuxState(MUX_A, MUX_STATE_P_PWM);
		setMuxState(MUX_B, MUX_STATE_M_PWM);
		setMuxState(MUX_C, MUX_STATE_Z);
#else
		setMuxState(MUX_A, MUX_STATE_P_PWM_T);
		setMuxState(MUX_B, MUX_STATE_HI);
		setMuxState(MUX_C, MUX_STATE_Z);
#endif
	}
	}
	else if (drive_direction == DRIVE_BAKWARD && goodToGo)
	{
	if(STATE_1)
	{
#ifdef BI_POLAR
		setMuxState(MUX_A, MUX_STATE_P_PWM);
		setMuxState(MUX_B, MUX_STATE_Z);
		setMuxState(MUX_C, MUX_STATE_M_PWM);
#else
		setMuxState(MUX_A, MUX_STATE_HI);
		setMuxState(MUX_B, MUX_STATE_Z);
		setMuxState(MUX_C, MUX_STATE_P_PWM_T);
#endif
	}
	else if(STATE_2)
	{
#ifdef BI_POLAR
		setMuxState(MUX_A, MUX_STATE_Z);
		setMuxState(MUX_B, MUX_STATE_P_PWM);
		setMuxState(MUX_C, MUX_STATE_M_PWM);
#else
		setMuxState(MUX_A, MUX_STATE_Z);
		setMuxState(MUX_B, MUX_STATE_HI);
		setMuxState(MUX_C, MUX_STATE_P_PWM_T);
#endif
	}
	else if(STATE_3)
	{
#ifdef BI_POLAR
		setMuxState(MUX_A, MUX_STATE_M_PWM);
		setMuxState(MUX_B, MUX_STATE_P_PWM);
		setMuxState(MUX_C, MUX_STATE_Z);
#else
		setMuxState(MUX_A, MUX_STATE_P_PWM_T);
		setMuxState(MUX_B, MUX_STATE_HI);
		setMuxState(MUX_C, MUX_STATE_Z);
#endif
	}
	else if(STATE_4)
	{
#ifdef BI_POLAR
		setMuxState(MUX_A, MUX_STATE_M_PWM);
		setMuxState(MUX_B, MUX_STATE_Z);
		setMuxState(MUX_C, MUX_STATE_P_PWM);
#else
		setMuxState(MUX_A, MUX_STATE_P_PWM_T);
		setMuxState(MUX_B, MUX_STATE_Z);
		setMuxState(MUX_C, MUX_STATE_HI);
#endif
	}
	else if(STATE_5)
	{
#ifdef BI_POLAR
		setMuxState(MUX_A, MUX_STATE_Z);
		setMuxState(MUX_B, MUX_STATE_M_PWM);
		setMuxState(MUX_C, MUX_STATE_P_PWM);
#else
		setMuxState(MUX_A, MUX_STATE_Z);
		setMuxState(MUX_B, MUX_STATE_P_PWM_T);
		setMuxState(MUX_C, MUX_STATE_HI);
#endif
	}
	else if(STATE_6)
	{
#ifdef BI_POLAR
		setMuxState(MUX_A, MUX_STATE_P_PWM);
		setMuxState(MUX_B, MUX_STATE_M_PWM);
		setMuxState(MUX_C, MUX_STATE_Z);
#else
		setMuxState(MUX_A, MUX_STATE_HI);
		setMuxState(MUX_B, MUX_STATE_P_PWM_T);
		setMuxState(MUX_C, MUX_STATE_Z);
#endif
	}
	}

	if(goodToGo)
		hall_steps++;

	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

static void setMux(uint8_t mux, uint8_t b2, uint8_t b1, uint8_t b0)
{
	switch (mux)
	{
		case MUX_A:
			GPIOC->BSRR = (GPIO_BLEN_PIN << (!b2 << 4)) | (GPIO_S2_PIN << (!b0 << 4));
			GPIOA->BSRR = (GPIO_S3_PIN << (!b1 << 4));
			break;
		case MUX_B:
			GPIOB->BSRR = (GPIO_CLEN_PIN << (!b2 << 4)) | (GPIO_S5_PIN << (!b1 << 4)) | (GPIO_S4_PIN << (!b0 << 4));
			break;
		case MUX_C:
			GPIOC->BSRR = (GPIO_ALEN_PIN << (!b2 << 4)) | (GPIO_S1_PIN << (!b1 << 4)) | (GPIO_S0_PIN << (!b0 << 4));
			break;
		default:
			break;
	}
}

void setMuxState(uint8_t mux, uint8_t state)
{
	switch (state)
	{
		case MUX_STATE_P_PWM: // BOTH
			setMux(mux, 0, 0, 0);
			break;
		case MUX_STATE_M_PWM: // BOTH
			setMux(mux, 0, 0, 1);
			break;
		case MUX_STATE_HI: // BOTH
			setMux(mux, 0, 1, 0);
			break;
		case MUX_STATE_LO: // BOTH
			setMux(mux, 0, 1, 1);
			break;
		case MUX_STATE_P_PWM_T: //+PWM_TOP, LO_BOT
			setMux(mux, 1, 0, 0);
			break;
		case MUX_STATE_M_PWM_T: //-PWM_TOP, LO_BOT
			setMux(mux, 1, 0, 1);
			break;
		case MUX_STATE_Z: //HI_TOP, LO_BOT
			setMux(mux, 1, 1, 0);
			break;
		case MUX_STATE_TBD4: //LO_TOP, LO_BOT
			setMux(mux, 1, 1, 1);
			break;
		default:
			break;
	}
}

void Hall_Input_Init(void)
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

uint32_t get_hall_steps(void)
{
	return hall_steps;
}

void set_hall_steps(uint32_t val)
{
	hall_steps = val;
}

void set_direction(int dir)
{
	drive_direction = dir;
}
