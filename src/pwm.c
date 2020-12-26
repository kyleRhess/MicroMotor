
#include "pwm.h"

static void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm)

static TIM_HandleTypeDef timer_PWM;
static PWM_Out PWMtimer;

static void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	if(htim_pwm->Instance==TIM1)
	{
		/* Peripheral clock enable */
		__TIM1_CLK_ENABLE();
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

		GPIO_InitStruct.Pin 		= PWM_POS_PIN | PWM_NEG_PIN;
		GPIO_InitStruct.Mode 		= GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull 		= GPIO_PULLDOWN;
		GPIO_InitStruct.Speed 		= GPIO_SPEED_LOW;
		GPIO_InitStruct.Alternate 	= GPIO_AF1_TIM1;
		HAL_GPIO_Init(PWM_PORT, &GPIO_InitStruct);
	}
}

int PWM_Init_Output()
{
	PWMtimer.frequency 				= PWM_FREQ;
	PWMtimer.TIM 					= TIM1;
	PWMtimer.timer 					= PWM_Init();
	return HAL_OK;
}

TIM_HandleTypeDef PWM_Init(void)
{
	TIM_MasterConfigTypeDef master;
	TIM_OC_InitTypeDef configOC;
	memset(&master, 0, sizeof(master));
	memset(&configOC, 0, sizeof(configOC));
	memset(&timer_PWM, 0, sizeof(timer_PWM));

	TIM_TypeDef * TIM 				= PWMtimer.TIM;
	timer_PWM.Instance 				= TIM;
	timer_PWM.Init.Prescaler 		= PWM_PRESCALE;
	timer_PWM.Init.CounterMode 		= TIM_COUNTERMODE_UP;
	timer_PWM.Init.Period 			= PWM_PERIOD;
	timer_PWM.Init.ClockDivision 	= TIM_CLOCKDIVISION_DIV1;

	timer_PWM.Lock = HAL_UNLOCKED;
	HAL_TIM_PWM_MspInit(&timer_PWM);
	timer_PWM.State= HAL_TIM_STATE_BUSY;
	TIM_Base_SetConfig(timer_PWM.Instance, &timer_PWM.Init);
	timer_PWM.State= HAL_TIM_STATE_READY;

	master.MasterOutputTrigger 		= TIM_TRGO_RESET;
	master.MasterSlaveMode 			= TIM_MASTERSLAVEMODE_DISABLE;

	configOC.OCMode 				= TIM_OCMODE_PWM1;
	configOC.Pulse 					= 0;
	configOC.OCPolarity 			= TIM_OCPOLARITY_HIGH;

	// Start channel 1 with positive polarity
	HAL_TIM_PWM_ConfigChannel(&timer_PWM, &configOC, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&timer_PWM, TIM_CHANNEL_1);

	// Start channel 2 with negative polarity
	configOC.OCPolarity = TIM_OCPOLARITY_LOW;
	HAL_TIM_PWM_ConfigChannel(&timer_PWM, &configOC, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&timer_PWM, TIM_CHANNEL_2);

	return timer_PWM;
}

void PWM_adjust_DutyCycle(float dutyCycle)
{
	if(dutyCycle > DUTY_LIM_H)
		dutyCycle = DUTY_LIM_H;
	if(dutyCycle < DUTY_LIM_L)
		dutyCycle = DUTY_LIM_L;

	uint32_t counts_Ccr = CNTS_FROM_US((1000000/PWM_FREQ)*(dutyCycle/100));
	timer_PWM.Instance->CCR1 = counts_Ccr;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
	timer_PWM.Instance->CCR2 = counts_Ccr;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
	timer_PWM.Instance->CCR3 = counts_Ccr;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
	timer_PWM.Instance->CCR4 = counts_Ccr;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
}

void PWM_adjust_Frequency(uint32_t newFreq)
{
	uint32_t period_cycles = CLOCK_CYCLES_PER_SECOND / newFreq;
	uint16_t prescaler = (uint16_t)(period_cycles / MAX_RELOAD + 1);
	uint16_t overflow = (uint16_t)((period_cycles + (prescaler / 2)) / prescaler);
	uint16_t duty = (uint16_t)(overflow / 2);

	timer_PWM.Instance->ARR = (uint32_t)overflow;
	timer_PWM.Instance->PSC = (uint32_t)prescaler;

	timer_PWM.Instance->CCR1 = (uint32_t)duty;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
	timer_PWM.Instance->CCR2 = (uint32_t)duty;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
	timer_PWM.Instance->CCR3 = (uint32_t)duty;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
	timer_PWM.Instance->CCR4 = (uint32_t)duty;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
}
