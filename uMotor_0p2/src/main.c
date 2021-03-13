/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "system.h"
#include "PWM.h"
#include "Serial.h"
#include "diag/Trace.h"
#include "cmsis_device.h"
#include "PID.h"

#define PI					3.141592654f
#define TWOPI				6.283185307f
#define FOURPI				12.566370614f
#define SHIFT_120			2.094395102f
#define SHIFT_240 			4.188790205f
#define _PI_3				1.047197551f // pi / 3
#define	SQRT_3				1.732050808f // sqrt(3)
#define SQR_THREE_TWO 		0.866025404f // sqrt(3) / 2
#define ONE_THIRD			0.333333333f // 1 / 3
#define ONE_SQR_THREE		0.577350269f // 1 / sqrt(3)
#define	V_SUPPLY			48.0f		 //
#define	DEG_RAD				0.017453293f // degrees to radians
#define ONE_2PI				0.159154943f // 1 / (2pi)
#define PI_2				1.570796327f // pi / 2

#define ADC_SCALE 			0.000805664f
#define ADC_ZERO 			1.65f
#define ADC_RES 			13.33333333f

#define MAX_CURRENT			3.5f

#define POS_CONTROL
//#define SPD_CONTROL

//#define	FAST_TRIG
//#define	SINE_MODULATION

TIM_HandleTypeDef q_time;

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

#define FULL_RX 64
#define HALF_RX FULL_RX / 2

//static TIM_HandleTypeDef SamplingTimer 	= { .Instance = TIM9 };
static float deltaDegrees	 			= 0.0f;
static float degreesPsec	 			= 0.0f;
static uint32_t timeElapUs 				= 0;
float rotor_theta_init = 0;
float rotor_theta_init_L = 0;
int reversing = 0;
int a_state = 0;
int b_state = 0;
int c_state = 0;
float mechAngle	= 0.0f;
float mechAngleoffset	= 0.0f;
float currentA = 0.0f;
float currentB = 0.0f;
float i_d 					= 0;
float i_q 					= 0;
float speed					= 0;
float velintegrator 		= 0;
float velest 				= 0;
float rotor_theta			= 0.0f;

int thisEncCounts 			= 0;
int lastEncCounts 			= 0;
int ticksBetween 			= 0;
int ticksBetweenLast 		= 0;

#define ADC_BUF_LEN 3
uint32_t g_ADCValue1 = 0;
float g_ADCValue2 = 0;
float g_ADCValue3 = 0;
uint32_t g_ADCBuffer[ADC_BUF_LEN];
PWM_Out PWMtimer;

PID_Controller pi_axis_d;
PID_Controller pi_axis_q;
PID_Controller pi_speed;
PID_Controller pi_pos;

//static int InitSamplingTimer(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
//static void MX_TIM5_Init(void);
static void MX_ADC1_Init(void);
static void PWM_Monitor(float a, float b, float c);
static float sin_fast(float x);
static float cos_fast(float x);
static float max(float a, float b, float c);
static float min(float a, float b, float c);


// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

static void setHighSystemClk(void);

int main(int argc, char* argv[])
{
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	setHighSystemClk();
	HAL_Init();

	RCC->AHB1ENR 	= RCC_AHB1ENR_GPIOCEN;
	RCC->AHB1ENR 	|= RCC_AHB1ENR_GPIOAEN;
	RCC->AHB1ENR 	|= RCC_AHB1ENR_GPIOBEN;

	Signal_Init();

	MX_DMA_Init();
	MX_GPIO_Init();
	InitPWMOutput();

	// Start timer to count rotary encoder signals
	Encoder_Init();

	// Setup hall sensor inputs from BLDC motor
	Hall_InputInit();

	// Setup serial interface
	Serial_InitPort(921600, UART_STOPBITS_1, UART_WORDLENGTH_8B, UART_PARITY_NONE);

	Serial_RxData(RX_BUFF_SZ);

#define Kp	0.055f
#define Ki	8.74399996f//6.50f

	pi_axis_d.kP = Kp;
	pi_axis_d.kI = Ki;
	pi_axis_d.kD = 0.0f;
	pi_axis_d.setPoint = 0.0f;
	pi_axis_d.deltaTime = (1.0f / 2000.0f);
	PID_Initialize(&pi_axis_d);

	pi_axis_q.kP = Kp;
	pi_axis_q.kI = Ki;
	pi_axis_q.kD = 0.0f;
	pi_axis_q.setPoint = 0.0f;
	pi_axis_q.deltaTime = (1.0f / 2000.0f);
	PID_Initialize(&pi_axis_q);

#if 0
	pi_speed.kP = 0.0059799999f;//0.00555999996f;
	pi_speed.kI = 0.0884599984;//0.0888900012f;
#else
	pi_speed.kP = 0.002155999996f;
	pi_speed.kI = 0.0288900012f;
#endif
	pi_speed.kD = 0.000001f;
	pi_speed.setPoint = 0.0f;
	pi_speed.deltaTime = (1.0f / 2000.0f);
	PID_Initialize(&pi_speed);

	pi_pos.kP = 25.4f;//35.0;
	pi_pos.kI = 0.0;
	pi_pos.kD = 0.45f;//0.852f;
	pi_pos.setPoint = 0.0f;
	pi_pos.deltaTime = (1.0f / 2000.0f);
	PID_Initialize(&pi_pos);

#define Q_WIND		1.0f
#define D_WIND		1.0f
#define SPEED_WIND	2.8f	// amps
#define POS_WIND	6000.0f // deg/s

	pi_axis_d.windupMax 	= D_WIND;
	pi_axis_d.windupMin 	= -D_WIND;
	pi_axis_q.windupMax 	= Q_WIND;
	pi_axis_q.windupMin 	= -Q_WIND;
	pi_speed.windupMax 		= SPEED_WIND;
	pi_speed.windupMin 		= -SPEED_WIND;
	pi_pos.windupMax 		= POS_WIND;
	pi_pos.windupMin 		= -POS_WIND;

	// Start ADC + DMA before base PWM timer
	MX_ADC1_Init();
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&g_ADCBuffer, ADC_BUF_LEN);

	HAL_TIM_Base_Start_IT(&PWMtimer.timer);
	HAL_TIM_PWM_Start(&PWMtimer.timer, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&PWMtimer.timer, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&PWMtimer.timer, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&PWMtimer.timer, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&PWMtimer.timer, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&PWMtimer.timer, TIM_CHANNEL_3);

	while (1)
	{
//		if(Clock_GetMsLast() > 40000)
//			pi_speed.setPoint = 7000.0f;
//		else if(Clock_GetMsLast() > 35000)
//			pi_speed.setPoint = 100.0f;
//		else if(Clock_GetMsLast() > 30000)
//			pi_speed.setPoint = 7000.0f;
//		else if(Clock_GetMsLast() > 25000)
//			pi_speed.setPoint = 100.0f;
//		else if(Clock_GetMsLast() > 20000)
//			pi_speed.setPoint = 7000.0f;
//		else if(Clock_GetMsLast() > 15000)
//			pi_speed.setPoint = 100.0f;
//		else if(Clock_GetMsLast() > 10000)
//			pi_speed.setPoint = 7000.0f;

//		if(Clock_GetMsLast() > 30000)
//			pi_pos.setPoint = 0.0f;
//		else if(Clock_GetMsLast() > 20000)
//			pi_pos.setPoint = 0.0001f;
//		else if(Clock_GetMsLast() > 18000)
//			pi_pos.setPoint = -400.0f;
//		else if(Clock_GetMsLast() > 16000)
//			pi_pos.setPoint = 200.0f;
//		else if(Clock_GetMsLast() > 14000)
//			pi_pos.setPoint = -100.0f;
//		else if(Clock_GetMsLast() > 12000)
//			pi_pos.setPoint = 50.0f;
//		else if(Clock_GetMsLast() > 10000)
//			pi_pos.setPoint = 0.0f;

//		pi_speed.setPoint = Signal_GetMotorPos()*20.0f;

//		static uint32_t lllsl = 0;
//		static int fwd = 0;
//		if(Clock_GetMsLast() > 10000 && Clock_GetMsLast() - lllsl >= 1000)
//		{
//			if(fwd)
//			{
//				fwd = 0;
//				pi_pos.setPoint += 180.0f;
//			}
//			else
//			{
//				fwd = 1;
//				pi_pos.setPoint -= 180.0f;
//			}
//
//			lllsl = Clock_GetMsLast();
//		}
//
//		if(Clock_GetMsLast() > 30000)
//			pi_pos.setPoint = 0.0f;
	}
}

void Run_SVM(void)
{
	// Misc vars
//	static float rotor_theta			= 0.0f;
	static float mechAngleLast			= 0.0f;
	//static float speed					= 0.0f;
	static int sector_S 				= -1;
	static float T1						= 0;
	static float T2						= 0;
	static float T0						= 0;
	static float Ta						= 0;
	static float Tb						= 0;
	static float Tc						= 0;
	static float m_i 					= 0;
	static float m_j 					= 0;
	static float m_k 					= 0;
	static int N 						= -1;

	// Estimator vars
	static float lastTime 				= 0;
	static float deltat 				= 0;
	static float ki 					= 90.0f;
	static float kp 					= 40.0f;
//	static float velest 				= 0;
	static float posest 				= 0;
//	static float velintegrator 			= 0;
	static float poserr 				= 0;

	// Clarke & Park vars
	static float i_a					= 0;
	static float i_b					= 0;
	static float i_c					= 0;
	static float i_alpha 				= 0;
	static float i_beta 				= 0;
//	static float i_d 					= 0;
//	static float i_q 					= 0;
	static float v_d 					= 0;
	static float v_q 					= 0;
	static float v_alpha 				= 0;
	static float v_beta 				= 0;
#ifdef SINE_MODULATION
	static float v_a 					= 0;
	static float v_b 					= 0;
	static float v_c 					= 0;
#endif


	if(Signal_GetMotorState() & MOTOR_MODE_DISABLE)
	{
		PWM_Monitor(0.5f, 0.5f, 0.5f);
		return;
	}

	if(sector_S == -1)
	{
		a_state = READ_H(HALL_A_PIN) && HALL_A_PIN;
		b_state = READ_H(HALL_B_PIN) && HALL_B_PIN;
		c_state = READ_H(HALL_C_PIN) && HALL_C_PIN;

		if(a_state == 1 && b_state == 0 && c_state == 0)
		{
			sector_S = 1;
			rotor_theta_init = 0.0f;
		}
		else if(a_state == 1 && b_state == 1 && c_state == 0)
		{
			sector_S = 2;
			rotor_theta_init = 60.0f;
		}
		else if(a_state == 0 && b_state == 1 && c_state == 0)
		{
			sector_S = 3;
			rotor_theta_init = 120.0f;
		}
		else if(a_state == 0 && b_state == 1 && c_state == 1)
		{
			sector_S = 4;
			rotor_theta_init = 180.0f;
		}
		else if(a_state == 0 && b_state == 0 && c_state == 1)
		{
			sector_S = 5;
			rotor_theta_init = 240.0f;
		}
		else if(a_state == 1 && b_state == 0 && c_state == 1)
		{
			sector_S = 6;
			rotor_theta_init = 300.0f;
		}
		rotor_theta_init_L = rotor_theta_init;
	}

	mechAngle 	 = -((float)(int32_t)Encoder_GetCounts() * 0.06f);

	if(reversing)
		rotor_theta  = rotor_theta_init + (mechAngle - mechAngleoffset) - 30.0f;
	else
		rotor_theta  = rotor_theta_init + (mechAngle - mechAngleoffset) - 90.0f;


	// Get time delta between computations
	deltat = (float)Clock_GetUs() - lastTime;
	deltat /= 1000000.0f;


//		pi_pos.setPoint 		= 360.0f;
//		pi_pos.windupMax 		= Signal_GetMotorPosKp()*12000.0f;
//		pi_pos.windupMin 		= -Signal_GetMotorPosKp()*12000.0f;


	if(deltat >= 0.00001f)
	{
		//	Tracking filter velocity estimate
//		posest += velest*deltat;
//		poserr = (mechAngle / 4.0f) - posest;
//		velintegrator += poserr * ki * deltat;
//		velest = poserr * kp + velintegrator;

#ifdef POS_CONTROL
		// Basic speed calc
		speed = (((mechAngle - mechAngleLast) / deltat))/4.0f;

		if(ticksBetweenLast != 0)
			velintegrator = (1.0f/(((float)ticksBetweenLast * 25.0f)/1000000.0f))*(360.0f / 24000.0f);

		pi_speed.setPoint = PID_Update(&pi_pos, mechAngle/4.0f);
		pi_axis_q.setPoint = PID_Update(&pi_speed, speed);
#endif

#ifdef SPD_CONTROL
		// Basic speed calc
		speed = (((mechAngle - mechAngleLast) / deltat))/4.0f;

		if(ticksBetweenLast != 0)
			velintegrator = (1.0f/(((float)ticksBetweenLast * 25.0f)/1000000.0f))*(360.0f / 24000.0f);

		pi_axis_q.setPoint = PID_Update(&pi_speed, speed);
#endif

		mechAngleLast = mechAngle;

		lastTime = (float)Clock_GetUs();
	}

	// Phase currents
	i_a 			= currentA; // measured value
	i_b 			= currentB; // measured value
	i_c 			= -i_a - i_b;

	// Forward Clarke
	i_alpha 		= i_a;
	i_beta 			= ONE_SQR_THREE*i_b - ONE_SQR_THREE*i_c;

#ifdef FAST_TRIG // cos & sin approx
	// Forward Park
	i_d 			= i_alpha*cos_fast(rotor_theta * DEG_RAD) + i_beta*sin_fast(rotor_theta * DEG_RAD);
	i_q 			= i_beta*cos_fast(rotor_theta * DEG_RAD)  - i_alpha*sin_fast(rotor_theta * DEG_RAD);

	// PI Control of Id and Iq
	v_d 			= PID_Update(&pi_axis_d, i_d);
	v_q 			= PID_Update(&pi_axis_q, i_q);

	// Inverse Park transformation
	v_alpha 		= v_d*cos_fast(rotor_theta * DEG_RAD) - v_q*sin_fast(rotor_theta * DEG_RAD);
	v_beta 			= v_d*sin_fast(rotor_theta * DEG_RAD) + v_q*cos_fast(rotor_theta * DEG_RAD);
#else
	// Forward Park
	i_d 			= i_alpha*cosf(rotor_theta * DEG_RAD) + i_beta*sinf(rotor_theta * DEG_RAD);
	i_q 			= i_beta*cosf(rotor_theta * DEG_RAD)  - i_alpha*sinf(rotor_theta * DEG_RAD);

	// PI Control of Id and Iq
	v_d 			= PID_Update(&pi_axis_d, i_d);
	v_q 			= PID_Update(&pi_axis_q, i_q);

	// Inverse Park transformation
	v_alpha 		= v_d*cosf(rotor_theta * DEG_RAD) - v_q*sinf(rotor_theta * DEG_RAD);
	v_beta 			= v_d*sinf(rotor_theta * DEG_RAD) + v_q*cosf(rotor_theta * DEG_RAD);
#endif

#ifdef SINE_MODULATION
	// Sine modulation method
	v_a 			= v_alpha;
	v_b 			= -0.5f * v_alpha + SQR_THREE_TWO * v_beta;
	v_c 			= -0.5f * v_alpha - SQR_THREE_TWO * v_beta;
	PWM_Monitor(v_a, v_b, v_c);

	//	static float multi = 0.0f;
	//	multi += 0.00001f;
	//	if(multi >= 0.6f) multi = 0.6f;
	//	v_alpha = cosf(rotor_theta * DEG_RAD) * multi;
	//	v_beta = sinf(rotor_theta * DEG_RAD) * multi;
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
	PWM_Monitor(Ta, Tb, Tc);
#endif
}

static void PWM_Monitor(float a, float b, float c)
{
	PWM_Set_Duty(&PWMtimer.timer, TIM_CHANNEL_1, a);
	PWM_Set_Duty(&PWMtimer.timer, TIM_CHANNEL_2, b);
	PWM_Set_Duty(&PWMtimer.timer, TIM_CHANNEL_3, c);
}

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

void EXTI9_5_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(ENCODER_Z_PIN);
}

/*
 * Timer used for motor ESC control
 */
int InitPWMOutput()
{
	PWMtimer.frequency 		= PWM_FREQ;
	PWMtimer.TIM 			= TIM1;
	PWMtimer.timer 			= Initialize_PWM(&PWMtimer);

	return HAL_OK;
}

static void MX_GPIO_Init(void)
{
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
}

static void MX_DMA_Init(void)
{
	__HAL_RCC_DMA2_CLK_ENABLE();

	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

	HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

void TIM1_UP_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim1);
}

static void MX_ADC1_Init(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	ADC_InjectionConfTypeDef sConfigInjected = {0};

	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 3;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = 2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	sConfig.Channel = ADC_CHANNEL_15;
	sConfig.Rank = 3;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{

	currentB = ((((float)g_ADCValue2 * ADC_SCALE) - ADC_ZERO) * ADC_RES);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{

	currentB = ((((float)g_ADCValue2 * ADC_SCALE) - ADC_ZERO) * ADC_RES);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
#define FILT_K		0.90f

#ifdef FILT_K
#define FILT_K_1	(1 - FILT_K)
	g_ADCValue1 = g_ADCBuffer[2];
	g_ADCValue2 = g_ADCValue2*FILT_K_1 + ((float)g_ADCBuffer[0])*FILT_K;
	g_ADCValue3 = g_ADCValue3*FILT_K_1 + ((float)g_ADCBuffer[1])*FILT_K;
#else
	g_ADCValue2 = (float)g_ADCBuffer[1];
	g_ADCValue3 = (float)g_ADCBuffer[2];
#endif

	// 3.3V / 4096 = 0.000805664
	currentA = ((((float)g_ADCValue3 * ADC_SCALE) - ADC_ZERO) * ADC_RES);
	currentB = ((((float)g_ADCValue2 * ADC_SCALE) - ADC_ZERO) * ADC_RES);

	// EMRG stop if current gets too high
	if(currentA > MAX_CURRENT || currentB > MAX_CURRENT || currentA < -MAX_CURRENT || currentB < -MAX_CURRENT )
	{
		while(1)
		{
			Signal_SetMotorState(MOTOR_MODE_DISABLE);
		}
	}

	Run_SVM();
}

/*
 * Need to re-initialize clocks to use HSI for PLL and
 * use PLL as the system clock with proper configs.
 *
 * This does not get set properly initially since after reset
 * needs to use HSI as system clock
 */
void setHighSystemClk(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct;

	// Use HSI and activate PLL with HSI as source.
	// This is tuned for NUCLEO-F411; update it for your board.
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;

	// 16 is the average calibration value, adjust for your own board.
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;

	// This assumes the HSI_VALUE is a multiple of 1 MHz. If this is not
	// your case, you have to recompute these PLL constants.
	RCC_OscInitStruct.PLL.PLLM = (HSI_VALUE/1000000u);
	RCC_OscInitStruct.PLL.PLLN = 400; // for 100 MHz
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4; /* 84 MHz, conservative */
	RCC_OscInitStruct.PLL.PLLQ = 7; /* To make USB work. */
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	// Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
	// clocks dividers
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK
	  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;

	// This is expected to work for most large cores.
	// Check and update it for your own configuration.
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
