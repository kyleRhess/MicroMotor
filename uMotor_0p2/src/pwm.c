
#include "PWM.h"
#include "main.h"


void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

TIM_HandleTypeDef timer_PWM;

TIM_HandleTypeDef Initialize_PWM(PWM_Out * PWMType)
{
	TIM_TypeDef * TIM 	= PWMType->TIM;
	TIM_MasterConfigTypeDef master;
	TIM_OC_InitTypeDef configOC;
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};

	memset(&master, 0, sizeof(master));
	memset(&configOC, 0, sizeof(configOC));
	memset(&timer_PWM, 0, sizeof(timer_PWM));

	timer_PWM.Instance 					= TIM;
	timer_PWM.Init.Prescaler 			= PWM_PRESCALE;
	timer_PWM.Init.CounterMode 			= TIM_COUNTERMODE_CENTERALIGNED1;
	timer_PWM.Init.Period 				= PWM_PERIOD;//PWM_STEPS - 1; // ARR -> counter max
	timer_PWM.Init.ClockDivision 		= TIM_CLOCKDIVISION_DIV1;
	timer_PWM.Init.RepetitionCounter 	= 0;

	HAL_TIM_Base_Init(&timer_PWM);

	sClockSourceConfig.ClockSource 		= TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&timer_PWM, &sClockSourceConfig);

	HAL_TIM_PWM_Init(&timer_PWM);
	HAL_TIM_OC_Init(&timer_PWM);

	master.MasterOutputTrigger 			= TIM_TRGO_RESET;
	master.MasterSlaveMode 				= TIM_MASTERSLAVEMODE_DISABLE;

	HAL_TIMEx_MasterConfigSynchronization(&timer_PWM, &master);

	configOC.OCMode 					= TIM_OCMODE_PWM1;
	configOC.Pulse 						= 0;
	configOC.OCPolarity 				= TIM_OCPOLARITY_LOW;
	configOC.OCNPolarity 				= TIM_OCPOLARITY_HIGH;
	configOC.OCFastMode 				= TIM_OCFAST_DISABLE;
	configOC.OCIdleState 				= TIM_OCIDLESTATE_RESET;
	configOC.OCNIdleState 				= TIM_OCIDLESTATE_RESET;

	HAL_TIM_PWM_ConfigChannel(&timer_PWM, &configOC, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(&timer_PWM, &configOC, TIM_CHANNEL_2);
	HAL_TIM_PWM_ConfigChannel(&timer_PWM, &configOC, TIM_CHANNEL_3);

	sBreakDeadTimeConfig.OffStateRunMode 	= TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode 	= TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel 			= TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime 			= DEAD_TIME / 10;
	sBreakDeadTimeConfig.BreakState 		= TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity		= TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput 	= TIM_AUTOMATICOUTPUT_DISABLE;

	HAL_TIMEx_ConfigBreakDeadTime(&timer_PWM, &sBreakDeadTimeConfig);
	HAL_TIM_MspPostInit(&timer_PWM);

	return timer_PWM;
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
	if(htim_base->Instance==TIM1)
	{
		__HAL_RCC_TIM1_CLK_ENABLE();
		/* TIM1 interrupt Init */
		HAL_NVIC_SetPriority(TIM1_UP_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
	}
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	if(htim->Instance==TIM1)
	{
		__HAL_RCC_GPIOA_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();
		/**TIM1 GPIO Configuration
		PA7     ------> TIM1_CH1N
		PB0     ------> TIM1_CH2N
		PB1     ------> TIM1_CH3N
		PA8     ------> TIM1_CH1
		PA9     ------> TIM1_CH2
		PA10    ------> TIM1_CH3
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	}
}

void PWM_adjust_DutyCycle(TIM_HandleTypeDef * pwmHandle, uint32_t Channel, float dutyCycle)
{
	dutyCycle = 1.0f - dutyCycle;
#define FREQ_SCALE	1000000.0/PWM_FREQ
#define DUTY_LIM_H	0.98
#define DUTY_LIM_L	1.0 - DUTY_LIM_H

	if(dutyCycle > DUTY_LIM_H)
		dutyCycle = DUTY_LIM_H;
	if(dutyCycle < DUTY_LIM_L)
		dutyCycle = DUTY_LIM_L;

	PWM_adjust_PulseWidth(pwmHandle, Channel, (((float)FREQ_SCALE)*dutyCycle));
}

void PWM_adjust_PulseWidth(TIM_HandleTypeDef * pwmHandle, uint32_t Channel, float pulseWidth_us)
{
	uint32_t counts_Ccr = CNTS_FROM_US(pulseWidth_us);

	if(counts_Ccr > pwmHandle->Instance->ARR)
		counts_Ccr = pwmHandle->Instance->ARR;

    /*Assign the new DC count to the capture compare register.*/
    switch(Channel)
    {
		case TIM_CHANNEL_1:
			pwmHandle->Instance->CCR1 = counts_Ccr;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
			break;

		case TIM_CHANNEL_2:
			pwmHandle->Instance->CCR2 = counts_Ccr;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
			break;

		case TIM_CHANNEL_3:
			pwmHandle->Instance->CCR3 = counts_Ccr;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
			break;

		case TIM_CHANNEL_4:
			pwmHandle->Instance->CCR4 = counts_Ccr;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
			break;

		case TIM_CHANNEL_ALL:
			pwmHandle->Instance->CCR1 = counts_Ccr;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
			pwmHandle->Instance->CCR2 = counts_Ccr;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
			pwmHandle->Instance->CCR3 = counts_Ccr;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
			pwmHandle->Instance->CCR4 = counts_Ccr;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
			break;

    }
}

#define DEAD_TIME_BUF	(((DEAD_TIME / PULSE_NS_PER_CNT) / 2) + 1)
void PWM_Set_Duty(TIM_HandleTypeDef * pwmHandle, uint32_t Channel, float dutyCycle)
{
	static uint32_t counts_Ccr = 0;
	counts_Ccr = dutyCycle * pwmHandle->Instance->ARR;

	if(counts_Ccr > pwmHandle->Instance->ARR - DEAD_TIME_BUF)
		counts_Ccr = pwmHandle->Instance->ARR - DEAD_TIME_BUF;
	else if(counts_Ccr < DEAD_TIME_BUF)
		counts_Ccr = DEAD_TIME_BUF;

    /*Assign the new DC count to the capture compare register.*/
    switch(Channel)
    {
		case TIM_CHANNEL_1:
			pwmHandle->Instance->CCR1 = counts_Ccr;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
			break;

		case TIM_CHANNEL_2:
			pwmHandle->Instance->CCR2 = counts_Ccr;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
			break;

		case TIM_CHANNEL_3:
			pwmHandle->Instance->CCR3 = counts_Ccr;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
			break;

		case TIM_CHANNEL_4:
			pwmHandle->Instance->CCR4 = counts_Ccr;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
			break;

		case TIM_CHANNEL_ALL:
			pwmHandle->Instance->CCR1 = counts_Ccr;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
			pwmHandle->Instance->CCR2 = counts_Ccr;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
			pwmHandle->Instance->CCR3 = counts_Ccr;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
			pwmHandle->Instance->CCR4 = counts_Ccr;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
			break;

    }
}

void PWM_adjust_Frequency(TIM_HandleTypeDef * pwmHandle, uint32_t Channel, uint32_t newFreq)
{
	newFreq *= 4;

	uint32_t period_cycles = CLOCK_CYCLES_PER_SECOND / newFreq;
	uint16_t prescaler = (uint16_t)(period_cycles / MAX_RELOAD + 1);
	uint16_t overflow = (uint16_t)((period_cycles + (prescaler / 2)) / prescaler);
	uint16_t duty = (uint16_t)(overflow / 2);

	pwmHandle->Instance->ARR = (uint32_t)overflow;
	pwmHandle->Instance->PSC = (uint32_t)prescaler;

	switch(Channel)
	{
		case TIM_CHANNEL_1:
			pwmHandle->Instance->CCR1 = (uint32_t)duty;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
			break;

		case TIM_CHANNEL_2:
			pwmHandle->Instance->CCR2 = (uint32_t)duty;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
			break;

		case TIM_CHANNEL_3:
			pwmHandle->Instance->CCR3 = (uint32_t)duty;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
			break;

		case TIM_CHANNEL_4:
			pwmHandle->Instance->CCR4 = (uint32_t)duty;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
			break;

		case TIM_CHANNEL_ALL:
			pwmHandle->Instance->CCR1 = (uint32_t)duty;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
			pwmHandle->Instance->CCR2 = (uint32_t)duty;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
			pwmHandle->Instance->CCR3 = (uint32_t)duty;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
			pwmHandle->Instance->CCR4 = (uint32_t)duty;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
			break;

	}
}

