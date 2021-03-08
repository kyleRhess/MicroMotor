
#include "signal.h"

static int signal_currentEnableState 	= MOTOR_MODE_DISABLE;
static int signal_driveDirection		= MOTOR_MODE_FORWARD;
static float signal_currentPwmValue		= 0.0f;
static float signal_currentPosition		= 0.0f;
static float signal_PositionKp			= 0.0f;
static float signal_PositionKi			= 0.0f;


void Signal_Init(void)
{
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
}

void Signal_SetMotorState(int state)
{
	if(state == MOTOR_MODE_ENABLE)
		System_WritePin(GPIOB, GPIO_DIS_PIN, 0);
	else
		System_WritePin(GPIOB, GPIO_DIS_PIN, 1);

	signal_currentEnableState = state;
}

int Signal_GetMotorState()
{
	return signal_currentEnableState;
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

	//PWM_adjust_DutyCycle(fabsf(signal_currentPwmValue));
#endif
}

void Signal_SetMotorPos(float position)
{
	signal_currentPosition = position;
}

float Signal_GetMotorPWM(void)
{
	return signal_currentPwmValue;
}

float Signal_GetMotorPos(void)
{
	return signal_currentPosition;
}

void Signal_SetMotorPosKp(float kp)
{
	signal_PositionKp = kp;
}

void Signal_SetMotorPosKi(float ki)
{
	signal_PositionKi = ki;
}

float Signal_GetMotorPosKp()
{
	return signal_PositionKp;
}

float Signal_GetMotorPosKi()
{
	return signal_PositionKi;
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

void Signal_SetDirection(int dir)
{
	signal_driveDirection = dir;
}
