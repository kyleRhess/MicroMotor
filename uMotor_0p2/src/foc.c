
#include "foc.h"
#include "signal.h"

#define Q_WIND		1.0f
#define D_WIND		1.0f
#define SPEED_WIND	2.8f	// amps
#define POS_WIND	8000.0f // deg/s
#define Kp			0.055f
#define Ki			8.74399996f

// =====================================
// Space Vector Modulation variables
// =====================================
static uint8_t sector_S 			= 255;
static uint8_t N 					= 255;
static float T0						= 0;
static float T1						= 0;
static float T2						= 0;
static float Ta						= 0;
static float Tb						= 0;
static float Tc						= 0;
static float m_i 					= 0;
static float m_j 					= 0;
static float m_k 					= 0;
#ifdef SINE_MODULATION
static float v_a 					= 0;
static float v_b 					= 0;
static float v_c 					= 0;
#endif

// =====================================
// Estimator variables
// =====================================
static float m_fLastTimeMs 			= 0;
static float m_fDeltaT 				= 0;
static float m_fMechAngleLast		= 0.0f;

// =====================================
// Clarke & Park variables
// =====================================
static float i_a					= 0;
static float i_b					= 0;
static float i_c					= 0;
static float i_alpha 				= 0;
static float i_beta 				= 0;
static float v_d 					= 0;
static float v_q 					= 0;
static float v_alpha 				= 0;
static float v_beta 				= 0;
static int a_state 					= 0;
static int b_state 					= 0;
static int c_state 					= 0;

// =====================================
// EXTERNS
// =====================================

float m_fRotorThetaInit 			= 0;
float m_fMechAngle					= 0.0f;
float m_fCurrentA 					= 0.0f;
float m_fCurrentB 					= 0.0f;
float m_fCurrentC 					= 0.0f;
float m_fId 						= 0;
float m_fIq 						= 0;
float m_fSpeed						= 0;
float m_fRotorTheta					= 0.0f;

PID_Controller pi_axis_d;
PID_Controller pi_axis_q;
PID_Controller pi_speed;
PID_Controller pi_pos;

typedef struct FloatField
{
	float		torqueValue; 	// 1
	float		torqueReq;		// 2
	float		speedValue; 	// 3
	float		speedReq;		// 4
	float		posValue; 		// 5
	float		posReq;			// 6
	float		currentA;		// 7
	float		currentB;		// 8
	float		currentC;		// 8
	float		angle;			// 9
} __attribute__((__packed__)) FloatField;
static uint8_t uartTxBuff[64];
static FloatField txCmdData;

#ifdef FAST_TRIG
static float sin_fast(float x);
static float cos_fast(float x);
#endif

void FOC_Init(void)
{
	sector_S 				= 255;
	N 						= 255;
	T0						= 0;
	T1						= 0;
	T2						= 0;
	Ta						= 0;
	Tb						= 0;
	Tc						= 0;
	m_i 					= 0;
	m_j 					= 0;
	m_k 					= 0;
	#ifdef SINE_MODULATION
	v_a 					= 0;
	v_b 					= 0;
	v_c 					= 0;
	#endif
	m_fLastTimeMs 			= 0;
	m_fDeltaT 				= 0;
	m_fMechAngleLast		= 0.0f;
	i_a						= 0;
	i_b						= 0;
	i_c						= 0;
	i_alpha 				= 0;
	i_beta 					= 0;
	v_d 					= 0;
	v_q 					= 0;
	v_alpha 				= 0;
	v_beta 					= 0;
	a_state 				= 0;
	b_state 				= 0;
	c_state 				= 0;
	m_fRotorThetaInit 		= 0;
	m_fMechAngle			= 0.0f;
	m_fCurrentA 			= 0.0f;
	m_fCurrentB 			= 0.0f;
	m_fCurrentC 			= 0.0f;
	m_fId 					= 0;
	m_fIq 					= 0;
	m_fSpeed				= 0;
	m_fRotorTheta			= 0.0f;

	pi_axis_d.kP			= Kp;
	pi_axis_d.kI 			= Ki;
	pi_axis_d.kD 			= 0.0f;
	pi_axis_d.setPoint 		= 0.0f;
	pi_axis_d.deltaTime 	= (1.0f / 2000.0f);

	pi_axis_q.kP 			= Kp;
	pi_axis_q.kI 			= Ki;
	pi_axis_q.kD 			= 0.0f;
	pi_axis_q.setPoint 		= 0.0f;
	pi_axis_q.deltaTime 	= (1.0f / 2000.0f);

	pi_speed.kP 			= 0.008512000013f;
	pi_speed.kI 			= 0.04739300018f;
	pi_speed.kD 			= 0.0f;
	pi_speed.setPoint 		= 0.0f;
	pi_speed.deltaTime 		= (1.0f / 2000.0f);

	pi_pos.kP 				= 21.0f;
	pi_pos.kI 				= 0.0f;
	pi_pos.kD 				= 0.0f;
	pi_pos.setPoint 		= 0.0f;
	pi_pos.deltaTime 		= (1.0f / 2000.0f);

	PID_Initialize(&pi_axis_d);
	PID_Initialize(&pi_axis_q);
	PID_Initialize(&pi_speed);
	PID_Initialize(&pi_pos);

	pi_axis_d.windupMax 	= D_WIND;
	pi_axis_d.windupMin 	= -D_WIND;
	pi_axis_q.windupMax 	= Q_WIND;
	pi_axis_q.windupMin 	= -Q_WIND;
	pi_speed.windupMax 		= SPEED_WIND;
	pi_speed.windupMin 		= -SPEED_WIND;
	pi_pos.windupMax 		= POS_WIND;
	pi_pos.windupMin 		= -POS_WIND;

	if(sector_S == 255)
	{
		a_state = READ_H(HALL_A_PIN) && HALL_A_PIN;
		b_state = READ_H(HALL_B_PIN) && HALL_B_PIN;
		c_state = READ_H(HALL_C_PIN) && HALL_C_PIN;

		if(a_state == 1 && b_state == 0 && c_state == 0)
		{
			sector_S = 1;
			m_fRotorThetaInit = 0.0f;
		}
		else if(a_state == 1 && b_state == 1 && c_state == 0)
		{
			sector_S = 2;
			m_fRotorThetaInit = 60.0f;
		}
		else if(a_state == 0 && b_state == 1 && c_state == 0)
		{
			sector_S = 3;
			m_fRotorThetaInit = 120.0f;
		}
		else if(a_state == 0 && b_state == 1 && c_state == 1)
		{
			sector_S = 4;
			m_fRotorThetaInit = 180.0f;
		}
		else if(a_state == 0 && b_state == 0 && c_state == 1)
		{
			sector_S = 5;
			m_fRotorThetaInit = 240.0f;
		}
		else if(a_state == 1 && b_state == 0 && c_state == 1)
		{
			sector_S = 6;
			m_fRotorThetaInit = 300.0f;
		}
	}
}

void Run_SVM(void)
{
#if 0
	static int div = 0;
	div++;
	if(div == 2)
	{
		div = 0;

		txCmdData.angle 		= m_fRotorTheta;
		txCmdData.m_fCurrentA 	= m_fCurrentA;
		txCmdData.m_fCurrentB 	= m_fCurrentB;
		txCmdData.m_fCurrentC 	= m_fCurrentC;
		txCmdData.posReq 		= pi_pos.setPoint;
		txCmdData.posValue		= pi_pos.lastInput;
		txCmdData.speedReq 		= pi_speed.setPoint;
		txCmdData.speedValue 	= pi_speed.lastInput;
		txCmdData.torqueReq 	= pi_axis_q.setPoint;
		txCmdData.torqueValue 	= pi_axis_q.lastInput;

		memcpy(&uartTxBuff[0], &txCmdData, sizeof(txCmdData));
		Serial_TxData2(uartTxBuff, sizeof(txCmdData));
	}
#endif

	// Get time delta between computations
	m_fDeltaT = (float)Clock_GetUs() - m_fLastTimeMs;
	m_fDeltaT /= 1000000.0f;

	// Compute speed at a decimated rate
	if(m_fDeltaT > 0.001f)
	{
		m_fSpeed 			= (((m_fMechAngle - m_fMechAngleLast) / m_fDeltaT))/4.0f;
		m_fMechAngleLast 	= m_fMechAngle;
		m_fLastTimeMs 		= (float)Clock_GetUs();
	}

#if 0
	// Check for motor over-speed
	if(DPS_TO_RPM(fabsf(m_fSpeed)) > MAX_SPEED)
	{
		while(1)
		{
			Signal_SetMotorState(MOTOR_MODE_DISABLE);
		}
	}
#endif

	// Get rotor angle
	m_fMechAngle 	 = -((float)Encoder_GetAngle() * 4.0f);

	// Convert rotor angle to electrical angle
	m_fRotorTheta  = m_fRotorThetaInit + (m_fMechAngle) - 90.0f;


	static uint32_t elaps = 0;
	// Don't run controllers if motor is disabled
	if(!(Signal_GetMotorState() & MOTOR_MODE_ENABLE))
	{
		PWM_Set_Duty(&PWMtimer.timer, TIM_CHANNEL_1, 0.5f);
		PWM_Set_Duty(&PWMtimer.timer, TIM_CHANNEL_2, 0.5f);
		PWM_Set_Duty(&PWMtimer.timer, TIM_CHANNEL_3, 0.5f);
		elaps = Clock_GetMs();
		return;
	}
	else
	{
		static uint32_t lllsl = 0;
		static int fwd = 0;
		if(Clock_GetMsLast() < 10000 && Clock_GetMsLast() - lllsl >= 500)
		{
			if(fwd)
			{
				fwd = 0;
				pi_pos.setPoint -= 360.0f;
			}
			else
			{
				fwd = 1;
				pi_pos.setPoint += 360.0f;
			}

			lllsl = Clock_GetMsLast();
		}
		else
		{
			// TESTING
			if(Signal_GetMotorState() & MOTOR_MODE_HOMING)
				pi_pos.setPoint	+= (((float)Clock_GetMs() - elaps)/1000.0f) * 20.0f;
			else
				pi_pos.setPoint += (((float)Clock_GetMs() - elaps)/1000.0f) * (Signal_GetMotorPWM()*Signal_GetMotorPWM()*Signal_GetMotorPWM())/200.0f;
		}
	}
	elaps = Clock_GetMs();

#ifdef POS_CONTROL
	pi_speed.setPoint 	= PID_Update(&pi_pos, m_fMechAngle/4.0f);
	pi_axis_q.setPoint 	= PID_Update(&pi_speed, m_fSpeed);
#endif

#ifdef SPD_CONTROL
	pi_axis_q.setPoint 	= PID_Update(&pi_speed, m_fSpeed);
#endif

#ifdef TRQ_CONTROL
	pi_axis_q.setPoint 	= Signal_GetMotorPos()/10000.0f;
#endif

	// Phase currents
	i_a 			= m_fCurrentA; // measured value
	i_b 			= m_fCurrentB; // measured value
	i_c 			= m_fCurrentC;

	// Forward Clarke
	i_alpha 		= i_a;
	i_beta 			= ONE_SQR_THREE*i_b - ONE_SQR_THREE*i_c;

#ifdef FAST_TRIG // cos & sin approx
	// Forward Park
	m_fId 			= i_alpha*cos_fast(m_fRotorTheta * DEG_RAD) + i_beta*sin_fast(m_fRotorTheta * DEG_RAD);
	m_fIq 			= i_beta*cos_fast(m_fRotorTheta * DEG_RAD)  - i_alpha*sin_fast(m_fRotorTheta * DEG_RAD);

	// PI Control of Id and Iq
	v_d 			= PID_Update(&pi_axis_d, m_fId);
	v_q 			= PID_Update(&pi_axis_q, m_fIq);

	// Inverse Park transformation
	v_alpha 		= v_d*cos_fast(m_fRotorTheta * DEG_RAD) - v_q*sin_fast(m_fRotorTheta * DEG_RAD);
	v_beta 			= v_d*sin_fast(m_fRotorTheta * DEG_RAD) + v_q*cos_fast(m_fRotorTheta * DEG_RAD);
#else
	// Forward Park
	m_fId 			= i_alpha*cosf(m_fRotorTheta * DEG_RAD) + i_beta*sinf(m_fRotorTheta * DEG_RAD);
	m_fIq 			= i_beta*cosf(m_fRotorTheta * DEG_RAD)  - i_alpha*sinf(m_fRotorTheta * DEG_RAD);

	// PI Control of Id and Iq
	v_d 			= PID_Update(&pi_axis_d, m_fId);
	v_q 			= PID_Update(&pi_axis_q, m_fIq);

	// Inverse Park transformation
	v_alpha 		= v_d*cosf(m_fRotorTheta * DEG_RAD) - v_q*sinf(m_fRotorTheta * DEG_RAD);
	v_beta 			= v_d*sinf(m_fRotorTheta * DEG_RAD) + v_q*cosf(m_fRotorTheta * DEG_RAD);
#endif

#ifdef SINE_MODULATION
	// Sine modulation method
	v_a 			= v_alpha;
	v_b 			= -0.5f * v_alpha + SQR_THREE_TWO * v_beta;
	v_c 			= -0.5f * v_alpha - SQR_THREE_TWO * v_beta;
	PWM_Monitor(v_a, v_b, v_c);
#else
	// SVM modulation method
	m_i 			= SQR_THREE_TWO * v_alpha - v_beta * 0.5f;
	m_j 			= v_beta;
	m_k 			= -SQR_THREE_TWO * v_alpha - v_beta * 0.5f;
	N = ((m_i >= 0) ? 1 : 0) + ((m_j >= 0) ? 2 : 0) + ((m_k >= 0) ? 4 : 0);
	switch (N)
	{
		case 1:
			sector_S = 6;
			break;
		case 2:
			sector_S = 2;
			break;
		case 3:
			sector_S = 1;
			break;
		case 4:
			sector_S = 4;
			break;
		case 5:
			sector_S = 5;
			break;
		case 6:
			sector_S = 3;
			break;
		default:
			break;
	}

	switch(sector_S)
	{
		case 1:
			T1 = m_i;
			T2 = m_j;
			break;
		case 2:
			T1 = -m_k;
			T2 = -m_i;
			break;
		case 3:
			T1 = m_j;
			T2 = m_k;
			break;
		case 4:
			T1 = -m_i;
			T2 = -m_j;
			break;
		case 5:
			T1 = m_k;
			T2 = m_i;
			break;
		case 6:
			T1 = -m_j;
			T2 = -m_k;
			break;
		default:
			break;
	}
	T0 = 1 - T1 - T2;

	switch(sector_S)
	{
		case 1:
			Ta = T1 + T2 + T0/2;
			Tb = T2 + T0/2;
			Tc = T0/2;
			break;
		case 2:
			Ta = T1 +  T0/2;
			Tb = T1 + T2 + T0/2;
			Tc = T0/2;
			break;
		case 3:
			Ta = T0/2;
			Tb = T1 + T2 + T0/2;
			Tc = T2 + T0/2;
			break;
		case 4:
			Ta = T0/2;
			Tb = T1+ T0/2;
			Tc = T1 + T2 + T0/2;
			break;
		case 5:
			Ta = T2 + T0/2;
			Tb = T0/2;
			Tc = T1 + T2 + T0/2;
			break;
		case 6:
			Ta = T1 + T2 + T0/2;
			Tb = T0/2;
			Tc = T1 + T0/2;
			break;
		default:
			// possible error state
			Ta = 0;
			Tb = 0;
			Tc = 0;
	}

	// Drive PWM signals
	PWM_Set_Duty(&PWMtimer.timer, TIM_CHANNEL_1, Ta);
	PWM_Set_Duty(&PWMtimer.timer, TIM_CHANNEL_2, Tb);
	PWM_Set_Duty(&PWMtimer.timer, TIM_CHANNEL_3, Tc);
#endif
}

#ifdef FAST_TRIG
static float sin_fast(float x)
{
	float t = x * ONE_2PI;
	t = t - (int)t;

	if(t < 0.5f)
		return -16.0f*(t*t) + 8*t;
	else
		return 16.0f*(t*t) - 16*t - 8*t + 8;
}

static float cos_fast(float x)
{
	sin_fast(x + PI_2);
}
#endif
