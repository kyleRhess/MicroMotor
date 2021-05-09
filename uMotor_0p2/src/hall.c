#include "hall.h"

#if 0
static void hall_monitor(uint8_t hall_num, int state);
static volatile int hall_state[3] 		= {0};
static volatile int hall_stateLast[3] 	= {-1};
#endif

static volatile uint32_t hall_steps 	= 0;
static float hall_currentRpmValue 		= 0.0f;

#if 0
static void hall_monitor(uint8_t hall_num, int state)
{
	// nop's give a slight delay
	__NOP();
	__NOP();
	hall_state[hall_num] = state;
	if(hall_state[hall_num] != hall_stateLast[hall_num])
	{
		hall_stateLast[hall_num] = hall_state[hall_num];

		// inc hall state steps
		hall_steps++;
	}
}
#endif

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
#if 1
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);

	// nop's give a slight delay
	__NOP();
	a_state = READ_H(HALL_A_PIN) && HALL_A_PIN;
	b_state = READ_H(HALL_B_PIN) && HALL_B_PIN;
	c_state = READ_H(HALL_C_PIN) && HALL_C_PIN;

	static int countt = 0;
	countt++;
	hall_steps++;

#ifdef TRAPZ
	float pwmval = m_fTrapzPwmVal;
	if(a_state == 1 && b_state == 0 && c_state == 0)
	{
		m_fRotorThetaInit = 0.0f;

		PWM_Set_Duty(&PWMtimer.timer, 	TIM_CHANNEL_1, 	pwmval);
		PWM_Set_Duty(&PWMtimer.timer, 	TIM_CHANNEL_2, 	0.0f);
		PWM_Set_Duty(&PWMtimer.timer, 	TIM_CHANNEL_3, 	0.0f);
		System_WritePin(GPIOA, 			PIN_LOW_A, 		GPIO_PIN_RESET);
		System_WritePin(GPIOA, 			PIN_LOW_B, 		GPIO_PIN_RESET);
		System_WritePin(GPIOA, 			PIN_LOW_C, 		GPIO_PIN_SET);
	}
	else if(a_state == 1 && b_state == 1 && c_state == 0)
	{
		m_fRotorThetaInit = 60.0f;

		PWM_Set_Duty(&PWMtimer.timer, 	TIM_CHANNEL_1, 	0.0f);
		PWM_Set_Duty(&PWMtimer.timer, 	TIM_CHANNEL_2, 	pwmval);
		PWM_Set_Duty(&PWMtimer.timer, 	TIM_CHANNEL_3, 	0.0f);
		System_WritePin(GPIOA, 			PIN_LOW_A, 		GPIO_PIN_RESET);
		System_WritePin(GPIOA, 			PIN_LOW_B, 		GPIO_PIN_RESET);
		System_WritePin(GPIOA, 			PIN_LOW_C, 		GPIO_PIN_SET);
	}
	else if(a_state == 0 && b_state == 1 && c_state == 0)
	{
		m_fRotorThetaInit = 120.0f;

		PWM_Set_Duty(&PWMtimer.timer, 	TIM_CHANNEL_1, 	0.0f);
		PWM_Set_Duty(&PWMtimer.timer, 	TIM_CHANNEL_2, 	pwmval);
		PWM_Set_Duty(&PWMtimer.timer, 	TIM_CHANNEL_3, 	0.0f);
		System_WritePin(GPIOA, 			PIN_LOW_A, 		GPIO_PIN_SET);
		System_WritePin(GPIOA, 			PIN_LOW_B, 		GPIO_PIN_RESET);
		System_WritePin(GPIOA, 			PIN_LOW_C, 		GPIO_PIN_RESET);
	}
	else if(a_state == 0 && b_state == 1 && c_state == 1)
	{
		m_fRotorThetaInit = 180.0f;

		PWM_Set_Duty(&PWMtimer.timer, 	TIM_CHANNEL_1, 	0.0f);
		PWM_Set_Duty(&PWMtimer.timer, 	TIM_CHANNEL_2, 	0.0f);
		PWM_Set_Duty(&PWMtimer.timer, 	TIM_CHANNEL_3, 	pwmval);
		System_WritePin(GPIOA, 			PIN_LOW_A, 		GPIO_PIN_SET);
		System_WritePin(GPIOA, 			PIN_LOW_B, 		GPIO_PIN_RESET);
		System_WritePin(GPIOA, 			PIN_LOW_C, 		GPIO_PIN_RESET);
	}
	else if(a_state == 0 && b_state == 0 && c_state == 1)
	{
		m_fRotorThetaInit = 240.0f;

		PWM_Set_Duty(&PWMtimer.timer, 	TIM_CHANNEL_1, 	0.0f);
		PWM_Set_Duty(&PWMtimer.timer, 	TIM_CHANNEL_2, 	0.0f);
		PWM_Set_Duty(&PWMtimer.timer, 	TIM_CHANNEL_3, 	pwmval);
		System_WritePin(GPIOA, 			PIN_LOW_A, 		GPIO_PIN_RESET);
		System_WritePin(GPIOA, 			PIN_LOW_B, 		GPIO_PIN_SET);
		System_WritePin(GPIOA, 			PIN_LOW_C, 		GPIO_PIN_RESET);
	}
	else if(a_state == 1 && b_state == 0 && c_state == 1)
	{
		m_fRotorThetaInit = 300.0f;

		PWM_Set_Duty(&PWMtimer.timer, 	TIM_CHANNEL_1, 	pwmval);
		PWM_Set_Duty(&PWMtimer.timer, 	TIM_CHANNEL_2, 	0.0f);
		PWM_Set_Duty(&PWMtimer.timer, 	TIM_CHANNEL_3, 	0.0f);
		System_WritePin(GPIOA, 			PIN_LOW_A, 		GPIO_PIN_RESET);
		System_WritePin(GPIOA, 			PIN_LOW_B, 		GPIO_PIN_SET);
		System_WritePin(GPIOA, 			PIN_LOW_C, 		GPIO_PIN_RESET);
	}
#else
	if(a_state == 1 && b_state == 0 && c_state == 0)
	{
		m_fRotorThetaInit = 0.0f;
	}
	else if(a_state == 1 && b_state == 1 && c_state == 0)
	{
		m_fRotorThetaInit = 60.0f;
	}
	else if(a_state == 0 && b_state == 1 && c_state == 0)
	{
		m_fRotorThetaInit = 120.0f;
	}
	else if(a_state == 0 && b_state == 1 && c_state == 1)
	{
		m_fRotorThetaInit = 180.0f;
	}
	else if(a_state == 0 && b_state == 0 && c_state == 1)
	{
		m_fRotorThetaInit = 240.0f;
	}
	else if(a_state == 1 && b_state == 0 && c_state == 1)
	{
		m_fRotorThetaInit = 300.0f;
	}
#endif

	if((rotor_theta_init_L - m_fRotorThetaInit) == 60 || (rotor_theta_init_L - m_fRotorThetaInit) == -300)
	{
		reversing = 1;
	}
	else
	{
		reversing = 0;
	}

	rotor_theta_init_L 	= m_fRotorThetaInit;
	m_fMechAngle 		= 0;
	mechAngleoffset 	= -((float)Encoder_GetAngle() * 4.0f);

	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
#endif
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
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 1);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void Hall_ComputeRPM(float timeStep)
{
#define rpmK	0.9f
	if(reversing)
		hall_currentRpmValue = hall_currentRpmValue*rpmK + ((((float)Hall_GetSteps() / 144.0f) / timeStep) * -60.0f)*(1.0f-rpmK);
	else
		hall_currentRpmValue = hall_currentRpmValue*rpmK + ((((float)Hall_GetSteps() / 144.0f) / timeStep) * 60.0f)*(1.0f-rpmK);
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
