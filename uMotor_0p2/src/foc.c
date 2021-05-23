
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

static ClockTimerus speedTimer;
static ClockTimerus speedTimerTrapz;

// =====================================
// EXTERNS
// =====================================

float m_fRotorThetaInit 			= 0;
float m_fRotorThetaNext 			= 0;
double m_fMechAngle					= 0.0f;
double m_fHomeOffset				= 0.0f;
double m_fMechAngleLast				= 0.0f;
float m_fCurrentA 					= 0.0f;
float m_fCurrentB 					= 0.0f;
float m_fCurrentC 					= 0.0f;
float m_fId 						= 0;
float m_fIq 						= 0;
float m_fSpeed						= 0;
float m_fRotorTheta					= 0.0f;
float m_fSpeedFilt					= 0;
float m_fSupplyVolt					= 0;
float m_fSupplyVoltInit				= 0;
float m_fThrotVolt					= 0;

float rotor_theta_init_L 			= 0;
int reversing 						= 0;
int a_state 						= 0;
int b_state 						= 0;
int c_state 						= 0;
double mechAngleoffset				= 0.0f;
float m_fTrapzPwmVal				= 0.0f;

float systemParams[256]				= {0};

PID_Controller pi_axis_d;
PID_Controller pi_axis_q;
PID_Controller pi_speed;
PID_Controller pi_pos;
PID_Controller pi_speed_trapz;

typedef struct FloatField
{
	uint8_t		beg; 			// 1
	uint32_t	timeUs; 		// 2
	float		torqueValue; 	// 3
	float		torqueReq;		// 4
	float		speedValue; 	// 5
	float		speedReq;		// 6
	float		posValue; 		// 7
	float		posReq;			// 8
	float		currentA;		// 9
	float		currentB;		// 10
	float		currentC;		// 11
	float		angle;			// 12
} __attribute__((__packed__)) FloatField;
static uint8_t uartTxBuff[128];
static FloatField txCmdData;


typedef struct lastvals
{
	float		angle;		// 11
	float		m_fMechAngle;			// 12
} lastvals;

#ifdef FAST_TRIG
static float sin_fast(float x);
static float cos_fast(float x);
#endif


static float rotorInit = 0.0f;
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
	m_fDeltaT 				= 0;
	m_fMechAngleLast		= 0.0f;
	m_fHomeOffset			= 0.0f;
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
	m_fRotorThetaNext 		= 0;
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

	pi_speed.kP 			= 0.05f;
	pi_speed.kI 			= 0.06f;
	pi_speed.kD 			= 0.0f;
	pi_speed.setPoint 		= 0.0f;
	pi_speed.deltaTime 		= (1.0f / 25.0f);

	pi_pos.kP 				= 30.0;
	pi_pos.kI 				= 0.0f;
	pi_pos.kD 				= 0.0f;
	pi_pos.setPoint 		= 0.0f;
	pi_pos.deltaTime 		= (1.0f / 50);

	pi_speed_trapz.kP 		= 0.01;
	pi_speed_trapz.kI 		= 0.005f;
	pi_speed_trapz.kD 		= 0.0f;
	pi_speed_trapz.setPoint = 0.0f;
	pi_speed_trapz.deltaTime= (1.0f / 25.0f);

	PID_Initialize(&pi_axis_d);
	PID_Initialize(&pi_axis_q);
	PID_Initialize(&pi_speed);
	PID_Initialize(&pi_pos);
	PID_Initialize(&pi_speed_trapz);

	pi_axis_d.windupMax 	= D_WIND;
	pi_axis_d.windupMin 	= -D_WIND;
	pi_axis_q.windupMax 	= Q_WIND;
	pi_axis_q.windupMin 	= -Q_WIND;
	pi_speed.windupMax 		= SPEED_WIND;
	pi_speed.windupMin 		= -SPEED_WIND;
	pi_pos.windupMax 		= POS_WIND;
	pi_pos.windupMin 		= -POS_WIND;
	pi_speed_trapz.windupMax= 0.5f;
	pi_speed_trapz.windupMin= 0;

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

	rotor_theta_init_L 	= m_fRotorThetaInit;
	m_fRotorThetaNext 	= m_fRotorThetaInit;

	rotorInit = m_fRotorThetaInit;

	Clock_StartTimerUs(&speedTimer, 4000);
	Clock_StartTimerUs(&speedTimerTrapz, 40000);
}

void Run_SVM(void)
{
#if 1
	static float setPoint = 0;
	static int div = 0;
	div++;
	if(div >= 5 && Clock_GetMs() > 100)
	{
		div = 0;

		txCmdData.beg			= 0xAA;
		txCmdData.timeUs		= Clock_GetUs();
		txCmdData.angle 		= m_fRotorTheta;
		txCmdData.currentA 		= m_fCurrentA;
		txCmdData.currentB 		= m_fCurrentB;
		txCmdData.currentC 		= m_fCurrentC;
		txCmdData.posReq 		= pi_pos.setPoint;
		txCmdData.posValue		= (m_fMechAngle / 4.0);
		txCmdData.speedReq 		= setPoint;
		txCmdData.speedValue 	= m_fSpeed;
		txCmdData.torqueReq 	= pi_axis_q.setPoint;
		txCmdData.torqueValue 	= m_fIq;


		char str[128] = {0};
		sprintf(str, "%.3f,%.2f,%.2f,%.2f,%.2f,%08X,%.1f\n",
				m_fRotorTheta,
				m_fIq,
				pi_axis_q.setPoint,
				m_fCurrentA,
				m_fSupplyVolt,
				(unsigned int)Signal_GetMotorState(),
				Hall_GetRPM());

		uint8_t chars = 0;
		for (chars = 0; chars < sizeof(str)/sizeof(str[0]); ++chars)
		{
			if(str[chars] == '\n')
				break;
		}

		memcpy(&uartTxBuff[0], &str, chars+1);
		Serial_TxData2(uartTxBuff, chars+1);


//		static uint32_t lllsl = 0;
//		static int fwd = -1;
//		if(Clock_GetMsLast() - lllsl >= 2000)
//		{
//			if(fwd == -1)
//			{
//				fwd = 0;
//				lllsl = Clock_GetMsLast();
//			}
//
//			if(fwd)
//			{
//				fwd = 0;
//				setPoint = 0.5f;
//			}
//			else
//			{
//				fwd = 1;
//				setPoint = -0.5f;
//			}
//			lllsl = Clock_GetMsLast();
//		}
	}
#endif

//	pi_speed_trapz.kP = Signal_GetMotorPosKp() / 5.0f;
//	pi_speed_trapz.kI = Signal_GetMotorPosKi() / 10.0f;

	// Get CPU cycles between FOC computations
	static unsigned long t1 = 0;
	static unsigned long t2 = 0;
	static unsigned long diff = 0;
	t2 = DWT->CYCCNT;
	diff = t2 - t1;
	t1 = DWT->CYCCNT;
	m_fDeltaT = ((float)diff / (float)100e+06);

	// Get rotor angle
	if(Hall_GetRPM() > 20.0f)
		m_fMechAngle += (Hall_GetRPM()*6.0f)*m_fDeltaT;
	else
		m_fMechAngle = 0.0f;
//	m_fMechAngle 		= 0;


//	if(m_fRotorThetaNext > m_fRotorThetaInit)
//	{
//		if (m_fRotorThetaNext - m_fRotorThetaInit > 61.0f)
//		{
//			m_fRotorThetaInit += 99.0f;
//		}
//		else
//		{
//			m_fRotorThetaInit += 10.0f;
//		}
//	}
//	else if (m_fRotorThetaNext < m_fRotorThetaInit)
//	{
//		if (m_fRotorThetaNext - m_fRotorThetaInit < -61.0f)
//		{
//			m_fRotorThetaInit -= 99.0f;
//		}
//		else
//		{
//			m_fRotorThetaInit -= 10.0f;
//		}
//	}

//	if((Hall_GetRPM()*6.0f) > 1000)
//	{
//		System_WritePin(GPIOB, GPIO_DIS_PIN, 1);
//		m_fRotorTheta = 0;
//	}

	if(m_fRotorThetaInit != m_fRotorThetaNext)
	{
		m_fMechAngle = 0.0f;
		m_fRotorThetaInit = m_fRotorThetaNext;
	}

	if(m_fMechAngle > 60.0f)
		m_fMechAngle = 60.0f;
	if(m_fMechAngle < -60.0f)
		m_fMechAngle = -60.0f;


	// Convert rotor angle to electrical angle
	m_fRotorTheta  	= m_fRotorThetaInit + (float)(m_fMechAngle*24.0f - 0) - 90.0f;



	if(m_fRotorTheta > 360.0f)
		m_fRotorTheta = 360.0f;
	if(m_fRotorTheta < -360.0f)
		m_fRotorTheta = -360.0f;

	// Don't run controllers if motor is disabled
	if(!(Signal_GetMotorState() & MOTOR_MODE_ENABLE))
	{
#ifdef TRAPZ
		PWM_Set_Duty(&PWMtimer.timer, TIM_CHANNEL_1, 0.0f);
		PWM_Set_Duty(&PWMtimer.timer, TIM_CHANNEL_2, 0.0f);
		PWM_Set_Duty(&PWMtimer.timer, TIM_CHANNEL_3, 0.0f);
#else
		PWM_Set_Duty(&PWMtimer.timer, TIM_CHANNEL_1, 0.5f);
		PWM_Set_Duty(&PWMtimer.timer, TIM_CHANNEL_2, 0.5f);
		PWM_Set_Duty(&PWMtimer.timer, TIM_CHANNEL_3, 0.5f);
#endif
	}
	else
	{
#ifdef TRAPZ
//		if(Clock_UpdateTimerUs(&speedTimerTrapz))
//		{
//			Hall_ComputeRPM(0.04f);
//			pi_speed_trapz.setPoint = Signal_GetMotorPWM();
//			m_fTrapzPwmVal = PID_Update(&pi_speed_trapz, Hall_GetRPM());
////			m_fTrapzPwmVal = fabsf(Signal_GetMotorPWM()/100.0f);
//		}
//		return;
#endif
	}

	// Compute speed at a decimated rate
	if(Clock_UpdateTimerUs(&speedTimer))
	{
		Hall_ComputeRPM(0.004f);

		if((Signal_GetMotorState() & MOTOR_MODE_ENABLE))
		{
			// Check for motor over-speed
			if(DPS_TO_RPM(fabsf(Hall_GetRPM())) > MAX_SPEED)
			{
				Signal_SetMotorState(Signal_GetMotorState() | MOTOR_MODE_OVERSPEED);
			}

			if(Signal_GetMotorState() & MOTOR_MODE_CRUISING)
			{
#ifdef TRAPZ
				m_fTrapzPwmVal = Signal_GetMotorTorque();
				return;
#else
				pi_speed.setPoint = Signal_GetMotorSpeed();
				pi_axis_q.setPoint 	= PID_Update(&pi_speed, Hall_GetRPM());
#endif
			}
		}
	}

	if(!(Signal_GetMotorState() & MOTOR_MODE_CRUISING))
	{
#ifdef TRAPZ
		m_fTrapzPwmVal = Signal_GetMotorTorque();
		return;
#else
		static float torqBias = 0;
		if(Signal_GetMotorTorque() < 1.0f && Hall_GetRPM() < 1.0f)
			torqBias = torqBias*0.99999f + Signal_GetMotorTorque()*0.00001f;

		pi_axis_q.setPoint 	= (Signal_GetMotorTorque() - torqBias)/2.0f;
#endif
	}


	if(!(Signal_GetMotorState() & MOTOR_MODE_ENABLE))
		return;

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
			Ta = 0.5f;
			Tb = 0.5f;
			Tc = 0.5f;
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
