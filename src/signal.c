
#include "signal.h"

static void setMux(uint8_t mux, uint8_t b2, uint8_t b1, uint8_t b0);
static void Signal_SetMuxState(uint8_t mux, uint8_t state);

static int signal_currentEnableState 	= MOTOR_MODE_DISABLE;
static int signal_driveDirection		= MOTOR_MODE_FORWARD;
static float signal_currentPwmValue		= 0.0f;

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

static void Signal_SetMuxState(uint8_t mux, uint8_t state)
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

void Signal_Init(void)
{
	Signal_SetMuxState(MUX_C, MUX_STATE_Z);
	Signal_SetMuxState(MUX_B, MUX_STATE_Z);
	Signal_SetMuxState(MUX_A, MUX_STATE_Z);
	GPIO_InitTypeDef gMuxPins_0_2;
	gMuxPins_0_2.Pin 	= GPIO_S0_PIN | GPIO_S1_PIN | GPIO_S2_PIN | GPIO_ALEN_PIN | GPIO_BLEN_PIN;
	gMuxPins_0_2.Mode 	= GPIO_MODE_OUTPUT_PP;
	gMuxPins_0_2.Pull 	= GPIO_PULLDOWN;
	gMuxPins_0_2.Speed 	= GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOC, &gMuxPins_0_2);
	GPIO_InitTypeDef gMuxPins_3;
	gMuxPins_3.Pin 	= GPIO_S3_PIN;
	gMuxPins_3.Mode 	= GPIO_MODE_OUTPUT_PP;
	gMuxPins_3.Pull 	= GPIO_PULLDOWN;
	gMuxPins_3.Speed 	= GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOA, &gMuxPins_3);
	GPIO_InitTypeDef gMuxPins_4_5;
	gMuxPins_4_5.Pin 	= GPIO_S4_PIN | GPIO_S5_PIN | GPIO_CLEN_PIN;
	gMuxPins_4_5.Mode 	= GPIO_MODE_OUTPUT_PP;
	gMuxPins_4_5.Pull 	= GPIO_PULLDOWN;
	gMuxPins_4_5.Speed 	= GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOB, &gMuxPins_4_5);
	Signal_SetMuxState(MUX_C, MUX_STATE_Z);
	Signal_SetMuxState(MUX_B, MUX_STATE_Z);
	Signal_SetMuxState(MUX_A, MUX_STATE_Z);

	// DIS pin setup
	// Must set pin initially so that init routine does not invoke a brief LOW state.
	System_WritePin(GPIOB, GPIO_DIS_PIN, GPIO_PIN_SET);
	GPIO_InitTypeDef gDisPin;
	gDisPin.Pin 	= GPIO_DIS_PIN;
	gDisPin.Mode 	= GPIO_MODE_OUTPUT_PP;
	gDisPin.Pull 	= GPIO_NOPULL;
	gDisPin.Speed 	= GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOB, &gDisPin);
	System_WritePin(GPIOB, GPIO_DIS_PIN, GPIO_PIN_SET);

	// Kick the ISR once to get started
	Signal_MuxUpdate();
}

void Signal_SetMotorState(int state)
{
	if(state == MOTOR_MODE_ENABLE)
		System_WritePin(GPIOB, GPIO_DIS_PIN, 0);
	else
		System_WritePin(GPIOB, GPIO_DIS_PIN, 1);

	signal_currentEnableState = state;
}

void Signal_SetMotorPWM(float speed)
{
	signal_currentPwmValue = speed;

	if(signal_currentPwmValue < -100.0f)
		signal_currentPwmValue = -100.0f;
	if(signal_currentPwmValue >  100.0f)
		signal_currentPwmValue =  100.0f;
#ifdef BI_POLAR
	PWM_adjust_DutyCycle(((signal_currentPwmValue*0.5f) + 50.0f));
#else
	if(signal_currentPwmValue < 0.0f)
		Signal_SetDirection(MOTOR_MODE_BAKWARD);
	else
		Signal_SetDirection(MOTOR_MODE_FORWARD);

	PWM_adjust_DutyCycle(fabsf(signal_currentPwmValue));
#endif
}

float Signal_GetMotorPWM(void)
{
	return signal_currentPwmValue;
}

uint32_t Signal_GetMotorMode()
{
	uint32_t rc = 0;

	rc |= signal_currentEnableState;
	rc |= signal_driveDirection;
#ifdef BI_POLAR
	rc |= MOTOR_MODE_BIPOLAR;
#else
	rc |= MOTOR_MODE_UNIPOLAR;
#endif

	return rc;
}

void Signal_MuxUpdate(void)
{
	if(signal_driveDirection == MOTOR_MODE_FORWARD)
	{
	if(STATE_1)
	{
#ifdef BI_POLAR
		signal_SetMuxState(MUX_A, MUX_STATE_P_PWM);
		signal_SetMuxState(MUX_B, MUX_STATE_Z);
		signal_SetMuxState(MUX_C, MUX_STATE_M_PWM);
#else
		Signal_SetMuxState(MUX_A, MUX_STATE_P_PWM_T);
		Signal_SetMuxState(MUX_B, MUX_STATE_Z);
		Signal_SetMuxState(MUX_C, MUX_STATE_HI);
#endif
	}
	else if(STATE_2)
	{
#ifdef BI_POLAR
		Signal_SetMuxState(MUX_A, MUX_STATE_Z);
		Signal_SetMuxState(MUX_B, MUX_STATE_P_PWM);
		Signal_SetMuxState(MUX_C, MUX_STATE_M_PWM);
#else
		Signal_SetMuxState(MUX_A, MUX_STATE_Z);
		Signal_SetMuxState(MUX_B, MUX_STATE_P_PWM_T);
		Signal_SetMuxState(MUX_C, MUX_STATE_HI);
#endif
	}
	else if(STATE_3)
	{
#ifdef BI_POLAR
		Signal_SetMuxState(MUX_A, MUX_STATE_M_PWM);
		Signal_SetMuxState(MUX_B, MUX_STATE_P_PWM);
		Signal_SetMuxState(MUX_C, MUX_STATE_Z);
#else
		Signal_SetMuxState(MUX_A, MUX_STATE_HI);
		Signal_SetMuxState(MUX_B, MUX_STATE_P_PWM_T);
		Signal_SetMuxState(MUX_C, MUX_STATE_Z);
#endif
	}
	else if(STATE_4)
	{
#ifdef BI_POLAR
		Signal_SetMuxState(MUX_A, MUX_STATE_M_PWM);
		Signal_SetMuxState(MUX_B, MUX_STATE_Z);
		Signal_SetMuxState(MUX_C, MUX_STATE_P_PWM);
#else
		Signal_SetMuxState(MUX_A, MUX_STATE_HI);
		Signal_SetMuxState(MUX_B, MUX_STATE_Z);
		Signal_SetMuxState(MUX_C, MUX_STATE_P_PWM_T);
#endif
	}
	else if(STATE_5)
	{
#ifdef BI_POLAR
		Signal_SetMuxState(MUX_A, MUX_STATE_Z);
		Signal_SetMuxState(MUX_B, MUX_STATE_M_PWM);
		Signal_SetMuxState(MUX_C, MUX_STATE_P_PWM);
#else
		Signal_SetMuxState(MUX_A, MUX_STATE_Z);
		Signal_SetMuxState(MUX_B, MUX_STATE_HI);
		Signal_SetMuxState(MUX_C, MUX_STATE_P_PWM_T);
#endif
	}
	else if(STATE_6)
	{
#ifdef BI_POLAR
		Signal_SetMuxState(MUX_A, MUX_STATE_P_PWM);
		Signal_SetMuxState(MUX_B, MUX_STATE_M_PWM);
		Signal_SetMuxState(MUX_C, MUX_STATE_Z);
#else
		Signal_SetMuxState(MUX_A, MUX_STATE_P_PWM_T);
		Signal_SetMuxState(MUX_B, MUX_STATE_HI);
		Signal_SetMuxState(MUX_C, MUX_STATE_Z);
#endif
	}
	}
	else if (signal_driveDirection == MOTOR_MODE_BAKWARD)
	{
	if(STATE_1)
	{
#ifdef BI_POLAR
		Signal_SetMuxState(MUX_A, MUX_STATE_P_PWM);
		Signal_SetMuxState(MUX_B, MUX_STATE_Z);
		Signal_SetMuxState(MUX_C, MUX_STATE_M_PWM);
#else
		Signal_SetMuxState(MUX_A, MUX_STATE_HI);
		Signal_SetMuxState(MUX_B, MUX_STATE_Z);
		Signal_SetMuxState(MUX_C, MUX_STATE_P_PWM_T);
#endif
	}
	else if(STATE_2)
	{
#ifdef BI_POLAR
		Signal_SetMuxState(MUX_A, MUX_STATE_Z);
		Signal_SetMuxState(MUX_B, MUX_STATE_P_PWM);
		Signal_SetMuxState(MUX_C, MUX_STATE_M_PWM);
#else
		Signal_SetMuxState(MUX_A, MUX_STATE_Z);
		Signal_SetMuxState(MUX_B, MUX_STATE_HI);
		Signal_SetMuxState(MUX_C, MUX_STATE_P_PWM_T);
#endif
	}
	else if(STATE_3)
	{
#ifdef BI_POLAR
		Signal_SetMuxState(MUX_A, MUX_STATE_M_PWM);
		Signal_SetMuxState(MUX_B, MUX_STATE_P_PWM);
		Signal_SetMuxState(MUX_C, MUX_STATE_Z);
#else
		Signal_SetMuxState(MUX_A, MUX_STATE_P_PWM_T);
		Signal_SetMuxState(MUX_B, MUX_STATE_HI);
		Signal_SetMuxState(MUX_C, MUX_STATE_Z);
#endif
	}
	else if(STATE_4)
	{
#ifdef BI_POLAR
		Signal_SetMuxState(MUX_A, MUX_STATE_M_PWM);
		Signal_SetMuxState(MUX_B, MUX_STATE_Z);
		Signal_SetMuxState(MUX_C, MUX_STATE_P_PWM);
#else
		Signal_SetMuxState(MUX_A, MUX_STATE_P_PWM_T);
		Signal_SetMuxState(MUX_B, MUX_STATE_Z);
		Signal_SetMuxState(MUX_C, MUX_STATE_HI);
#endif
	}
	else if(STATE_5)
	{
#ifdef BI_POLAR
		Signal_SetMuxState(MUX_A, MUX_STATE_Z);
		Signal_SetMuxState(MUX_B, MUX_STATE_M_PWM);
		Signal_SetMuxState(MUX_C, MUX_STATE_P_PWM);
#else
		Signal_SetMuxState(MUX_A, MUX_STATE_Z);
		Signal_SetMuxState(MUX_B, MUX_STATE_P_PWM_T);
		Signal_SetMuxState(MUX_C, MUX_STATE_HI);
#endif
	}
	else if(STATE_6)
	{
#ifdef BI_POLAR
		Signal_SetMuxState(MUX_A, MUX_STATE_P_PWM);
		Signal_SetMuxState(MUX_B, MUX_STATE_M_PWM);
		Signal_SetMuxState(MUX_C, MUX_STATE_Z);
#else
		Signal_SetMuxState(MUX_A, MUX_STATE_HI);
		Signal_SetMuxState(MUX_B, MUX_STATE_P_PWM_T);
		Signal_SetMuxState(MUX_C, MUX_STATE_Z);
#endif
	}
	}
}

void Signal_SetDirection(int dir)
{
	signal_driveDirection = dir;
}
