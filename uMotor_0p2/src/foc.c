
#include "foc.h"
#include "signal.h"

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
static uint32_t m_fLastTimeUs 			= 0;
static float m_fLastTimeUsSpeed 	= 0;
static float m_fDeltaTSpeed 		= 0;
static float m_fDeltaT		 		= 0;

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

// =====================================
// EXTERNS
// =====================================

float m_fRotorThetaInit 			= 0;
double m_fMechAngle					= 0.0f;
double m_fMechAngleLast				= 0.0f;
float m_fCurrentA 					= 0.0f;
float m_fCurrentB 					= 0.0f;
float m_fCurrentC 					= 0.0f;
float m_fId 						= 0;
float m_fIq 						= 0;
float m_fSpeed						= 0;
float m_fRotorTheta					= 0.0f;


float m_fSpeedFilt					= 0;


float rotor_theta_init_L 			= 0;
int reversing 						= 0;
int a_state 						= 0;
int b_state 						= 0;
int c_state 						= 0;
double mechAngleoffset				= 0.0f;

PID_Controller pi_axis_d;
PID_Controller pi_axis_q;
PID_Controller pi_speed;
PID_Controller pi_pos;

typedef struct FloatField
{
	uint8_t		beg; 			// 1
	float		torqueValue; 	// 2
	float		torqueReq;		// 3
	float		speedValue; 	// 4
	float		speedReq;		// 5
	float		posValue; 		// 6
	float		posReq;			// 7
	float		currentA;		// 8
	float		currentB;		// 9
	float		currentC;		// 10
	float		angle;			// 11
	uint8_t		end; 			// 12
} __attribute__((__packed__)) FloatField;
static uint8_t uartTxBuff[128];
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
	m_fLastTimeUs 			= 0;
	m_fLastTimeUsSpeed 		= 0;
	m_fDeltaTSpeed 			= 0;
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
	pi_axis_d.deltaTime 	= (1.0f / PID_FREQ);

	pi_axis_q.kP 			= Kp;
	pi_axis_q.kI 			= Ki;
	pi_axis_q.kD 			= 0.0f;
	pi_axis_q.setPoint 		= 0.0f;
	pi_axis_q.deltaTime 	= (1.0f / PID_FREQ);

	pi_speed.kP 			= 0.008512000013f;
	pi_speed.kI 			= 0.04739300018f;
	pi_speed.kD 			= 0.0f;
	pi_speed.setPoint 		= 0.0f;
	pi_speed.deltaTime 		= (1.0f / PID_FREQ);

	pi_pos.kP 				= 35.0f;
	pi_pos.kI 				= 0.0f;
	pi_pos.kD 				= 0.0f;
	pi_pos.setPoint 		= 0.0f;
	pi_pos.deltaTime 		= (1.0f / PID_FREQ);

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

	rotor_theta_init_L = m_fRotorThetaInit;
}

static float float_rand( float min, float max )
{
    float scale = rand() / (float) RAND_MAX; /* [0, 1.0] */
    return min + scale * ( max - min );      /* [min, max] */
}

void Run_SVM(void)
{
#if 0
	static int div = 0;
	div++;
	if(div >= 2 && Clock_GetMsLast() > 4000)
	{
		div = 0;

		txCmdData.beg			= 0xAA;
		txCmdData.angle 		= m_fRotorTheta;
		txCmdData.currentA 		= m_fCurrentA;
		txCmdData.currentB 		= m_fCurrentB;
		txCmdData.currentC 		= m_fCurrentC;
		txCmdData.posReq 		= pi_pos.setPoint;
		txCmdData.posValue		= (m_fMechAngle / 4.0);
		txCmdData.speedReq 		= (Signal_GetMotorPWM()*Signal_GetMotorPWM()*Signal_GetMotorPWM())/125.0f;
		txCmdData.speedValue 	= m_fSpeedFilt;
		txCmdData.torqueReq 	= pi_axis_q.setPoint;
		txCmdData.torqueValue 	= m_fIq;
		txCmdData.end			= 0xAA;

		memcpy(&uartTxBuff[0], &txCmdData, sizeof(txCmdData));
		Serial_TxData2(uartTxBuff, sizeof(txCmdData));
	}
#endif

	// Get rotor angle
	m_fMechAngle		= -((double)Encoder_GetAngle() * 4.0);

	// Get time delta between computations (deal with wrap-around)
	if(m_fLastTimeUsSpeed > Clock_GetUs())
	{
		m_fDeltaTSpeed = (float)UINT32_MAX - m_fLastTimeUsSpeed;
		m_fLastTimeUsSpeed 	= (float)Clock_GetUs();
	}
	else
		m_fDeltaTSpeed = (float)Clock_GetUs() - m_fLastTimeUsSpeed;
	m_fDeltaTSpeed /= 1000000.0f;

	// Compute speed at a decimated rate
	if(m_fDeltaTSpeed > 0.001f)
	{
		m_fSpeed 			= ((float)(m_fMechAngle - m_fMechAngleLast) / 4.0f) / m_fDeltaTSpeed;
		m_fSpeedFilt		= m_fSpeedFilt * 0.99f + m_fSpeed * 0.01f;
		m_fMechAngleLast 	= m_fMechAngle;
		m_fLastTimeUsSpeed 	= (float)Clock_GetUs();
	}

	// Check for motor over-speed
	if(DPS_TO_RPM(fabsf(m_fSpeed)) > MAX_SPEED)
	{
		Signal_SetMotorState(MOTOR_MODE_OVERSPEED);
	}

	// Convert rotor angle to electrical angle
	m_fRotorTheta  	= m_fRotorThetaInit + (float)(m_fMechAngle - mechAngleoffset) - 90.0f;

	// Don't run controllers if motor is disabled
	if(!(Signal_GetMotorState() & MOTOR_MODE_ENABLE))
	{
		PWM_Set_Duty(&PWMtimer.timer, TIM_CHANNEL_1, 0.5f);
		PWM_Set_Duty(&PWMtimer.timer, TIM_CHANNEL_2, 0.5f);
		PWM_Set_Duty(&PWMtimer.timer, TIM_CHANNEL_3, 0.5f);
		return;
	}
	else
	{
		static uint32_t lllsl = 0;
		static int fwd = 0;
		if(Clock_GetMsLast() < 30 && Clock_GetMsLast() - lllsl >= 1000)
		{
			if(fwd)
			{
				fwd = 0;
				pi_pos.setPoint -= 180.0f;
			}
			else
			{
				fwd = 1;
				pi_pos.setPoint += 180.0f;
			}

//			pi_pos.setPoint = 1234567.8f;
			lllsl = Clock_GetMsLast();
		}
		else
		{
			if(m_fLastTimeUs > Clock_GetMs())
				m_fDeltaT = (float)UINT32_MAX - (float)m_fLastTimeUs;
			else
				m_fDeltaT = (float)Clock_GetMs() - (float)m_fLastTimeUs;
			m_fDeltaT /= 1000.0f;
			m_fLastTimeUs 	= Clock_GetMs();

//			pi_pos.setPoint += float_rand(-0.5f, 0.5f);
//			pi_pos.setPoint += m_fDeltaTSpeed * 12.0f;

//			// TESTING
			if(Signal_GetMotorState() & MOTOR_MODE_HOMING)
				pi_pos.setPoint	+= m_fDeltaT * 20.0f;
			else if(Signal_GetMotorState() & MOTOR_MODE_SPEED)
			{
				pi_pos.setPoint += m_fDeltaT * (Signal_GetMotorPWM()*Signal_GetMotorPWM()*Signal_GetMotorPWM())/125.0f;
			}
			else if(Signal_GetMotorState() & MOTOR_MODE_POSITION)
				pi_pos.setPoint = Signal_GetMotorPos()/1000.0f;
		}
	}

//	if(Signal_GetMotorState() & MOTOR_MODE_POSITION)
//	{
		pi_speed.setPoint 	= PID_Update(&pi_pos, (m_fMechAngle / 4.0));
		pi_axis_q.setPoint 	= PID_Update(&pi_speed, m_fSpeed);

//		pi_speed.setPoint	= Signal_GetMotorPWM()*80.0f;
//		pi_axis_q.setPoint 	= PID_Update(&pi_speed, m_fSpeed);

//	}
//	else if(Signal_GetMotorState() & MOTOR_MODE_SPEED)
//	{
//		pi_axis_q.setPoint 	= PID_Update(&pi_speed, m_fSpeed);
//	}

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
