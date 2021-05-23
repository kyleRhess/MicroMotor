
#include "signal.h"

//static int signal_currentEnableState 	= MOTOR_MODE_DISABLE;
static float signal_currentPwmValue		= 0.0f;
static float signal_currentTorque		= 0.0f;
static float signal_currentPosition		= 0.0f;
static float signal_PositionKp			= 0.0f;
static float signal_PositionKi			= 0.0f;
static uint32_t signal_heartBeatMs		= 0;

static uint32_t signal_DriveState		= MOTOR_MODE_DEFAULT;


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

void Signal_ResetMotor(void)
{
	Encoder_Reset();
	Signal_SetMotorTorque(0.0f);
	Signal_SetMotorSpeed(0.0f);
	Signal_SetMotorPos(0.0f);
	Signal_SetMotorState(MOTOR_MODE_DEFAULT);
	FOC_Init();
}

void Signal_SetMotorState(uint32_t state)
{
	signal_DriveState = state;
}

void Signal_ClearMotorState(uint32_t state)
{
	signal_DriveState &= ~state;

	if(!(signal_DriveState & MOTOR_MODE_ENABLE))
		System_WritePin(GPIOB, GPIO_DIS_PIN, 1);
}

uint32_t Signal_GetMotorState()
{
	return signal_DriveState;
}

void Signal_SetMotorSpeed(float speed)
{
	Signal_SetMotorState(Signal_GetMotorState() | MOTOR_MODE_SPEED);
	Signal_ClearMotorState(MOTOR_MODE_POSITION);
	Signal_ClearMotorState(MOTOR_MODE_TORQUE);
	signal_currentPwmValue = speed;
}

void Signal_SetMotorTorque(float torque)
{
	Signal_SetMotorState(Signal_GetMotorState() | MOTOR_MODE_TORQUE);
	Signal_ClearMotorState(MOTOR_MODE_POSITION);
	Signal_ClearMotorState(MOTOR_MODE_SPEED);

	signal_currentTorque = torque;
	if(signal_currentTorque < -0.0f)
		signal_currentTorque = 0.0f;
	if(signal_currentTorque >  100.0f)
		signal_currentTorque =  100.0f;
}

void Signal_SetMotorPos(float position)
{
	Signal_SetMotorState(Signal_GetMotorState() | MOTOR_MODE_POSITION);
	Signal_ClearMotorState(MOTOR_MODE_SPEED);
	Signal_ClearMotorState(MOTOR_MODE_TORQUE);

	signal_currentPosition = position;
}

void Signal_SetParam(uint8_t paramID, float paramValue)
{
	systemParams[paramID] = paramValue;
}

float Signal_GetMotorSpeed(void)
{
	return signal_currentPwmValue;
}

float Signal_GetMotorPos(void)
{
	return signal_currentPosition;
}

float Signal_GetMotorTorque(void)
{
	return signal_currentTorque;
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

void Signal_SetHeartBeatMs(uint32_t millis)
{
	signal_heartBeatMs = millis;
}

uint32_t Signal_GetHeartBeatMs(void)
{
	return signal_heartBeatMs;
}
