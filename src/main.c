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
#include "SPI.h"
#include "Serial.h"
#include "encoder.h"
#include "hall.h"
#include "PID.h"
#include "diag/Trace.h"
#include "cmsis_device.h"

TIM_HandleTypeDef q_time;


// Background timer for keeping time (25kHz)
static TIM_HandleTypeDef SamplingTimer 	= { .Instance = TIM9 };
static uint32_t timeElapUs 				= 0;
static uint32_t timeElapMs 				= 0;
static float 	timeElapMin 			= 0.0f;

// Degrees from encoder counts
static float deltaDegrees	 			= 0.0f;
static float degreesPsec	 			= 0.0f;

float pwm_duty = 0.0f;
float pwm_inc = PWM_INC_F;

// PWM timer stuff
PWM_Out PWMtimer;

PID_Controller speedControl;

// Function declarations
static void setHighSystemClk(void);
static int InitSamplingTimer(void);
static void MX_GPIO_Init(void);

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


static void set_motor_speed(float speed)
{

#ifdef BI_POLAR
	if(speed < -100.0f)
		speed = -100.0f;
	if(speed >  100.0f)
		speed =  100.0f;

	PWM_adjust_DutyCycle(&PWMtimer.timer, ((speed*0.5f) + 50.0f));
#else
	if(speed < 0.0f)
		set_direction(DRIVE_BAKWARD);
	else
		set_direction(DRIVE_FORWARD);

	PWM_adjust_DutyCycle(&PWMtimer.timer, fabsf(speed));
#endif
}

int main(int argc, char* argv[])
{
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	setHighSystemClk();
	HAL_Init();

	// Start PWM output (positive & negative)
	InitPWMOutput(&PWMtimer);

	// Start background timer (25kHz)
	InitSamplingTimer();

	// Start timer to count encoder pulses
	quadrature_timer_init(&q_time);

	// Set background timer interrupt priority
	HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);

	RCC->AHB1ENR 	= RCC_AHB1ENR_GPIOCEN;
	RCC->AHB1ENR 	|= RCC_AHB1ENR_GPIOAEN;
	RCC->AHB1ENR 	|= RCC_AHB1ENR_GPIOBEN;

	// MUX pins setup
	setMuxState(MUX_C, MUX_STATE_Z);
	setMuxState(MUX_B, MUX_STATE_Z);
	setMuxState(MUX_A, MUX_STATE_Z);
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
	setMuxState(MUX_C, MUX_STATE_Z);
	setMuxState(MUX_B, MUX_STATE_Z);
	setMuxState(MUX_A, MUX_STATE_Z);

	// DIS pin setup
	// Must set pin initially so that init routine does not invoke a brief LOW state.
	WritePin(GPIOB, GPIO_DIS_PIN, GPIO_PIN_SET);
	GPIO_InitTypeDef gDisPin;
	gDisPin.Pin 	= GPIO_DIS_PIN;
	gDisPin.Mode 	= GPIO_MODE_OUTPUT_PP;
	gDisPin.Pull 	= GPIO_NOPULL;
	gDisPin.Speed 	= GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOB, &gDisPin);
	WritePin(GPIOB, GPIO_DIS_PIN, GPIO_PIN_SET);

	MX_GPIO_Init();

	// Setup rotary encoder inputs
	Encoder_Z_Init();

	// Setup hall sensor inputs from BLDC motor
	Hall_Input_Init();

	// Setup serial interface
	InitSerial(115200, UART_STOPBITS_1, UART_WORDLENGTH_8B, UART_PARITY_NONE);

	rx_serial_data(dma_buffer_rx, FULL_RX);


	PID_Initialize(&speedControl);
	PID_SetKp(&speedControl, 0.02f);
	PID_SetKi(&speedControl, 0.025f);
	PID_SetKd(&speedControl, 0.0f);
	speedControl.deltaTime = 0.01f;
	PID_SetSetpoint(&speedControl, 240.0f);

	HAL_GPIO_EXTI_Callback(HALL_A_PIN);

	pwm_duty = -1.0f;
	pwm_inc = PWM_INC_F;

	while (1)
	{
		if(pwm_duty < -99.0f)
		{
			pwm_inc = 0.0f;
		}
		if(pwm_duty > 1.0f)
		{
			pwm_inc = 0.0f;
		}
		pwm_duty += pwm_inc;

		set_motor_speed(pwm_duty);
	}
}


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
}

static int InitSamplingTimer()
{
	__HAL_RCC_TIM9_CLK_ENABLE();

	// 25 kHz = 100E6/((49+1)*(79+1)*(1))
    SamplingTimer.Init.Prescaler 		= 49;
    SamplingTimer.Init.CounterMode 		= TIM_COUNTERMODE_UP;
    SamplingTimer.Init.Period 			= 79;
    SamplingTimer.Init.ClockDivision 	= TIM_CLOCKDIVISION_DIV1;

    if(HAL_TIM_Base_Init(&SamplingTimer) != HAL_OK)
    	return HAL_ERROR;

    if(HAL_TIM_Base_Start_IT(&SamplingTimer) != HAL_OK)
    	return HAL_ERROR;

    return HAL_OK;
}

void TIM1_BRK_TIM9_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&SamplingTimer);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static int timer9Divisor = 0;
	static float deltaDegreesLast = 0.0f;


//	hall_a_state
//
//	hall_a_timer -= 40;





	timeElapUs += 40;
	timer9Divisor++;
	if(timer9Divisor >= 25)
	{
		// 1000 ms tick
		timeElapMs 			+= 1;
		timeElapMin			+= (0.001f/60.0f);
		timer9Divisor 		= 0;

		deltaDegrees 		= ((float)(int32_t)q_time.Instance->CNT * (360.0f / (6000.0f * 4.0f)));
		degreesPsec 		= (deltaDegrees - deltaDegreesLast)/0.1f;
		deltaDegreesLast 	= deltaDegrees;

		static int thingy = 0;
		static int state_to_set = 1;
		static float rpmnow = 0.0f;
		if(timeElapUs % 10000 == 0)
		{
			static char dataToSend[64];
			int tosendsize = 0;
			tosendsize = sprintf(dataToSend, "%.2f, %.2f\n", pwm_duty, (float)rpmnow);
			tx_serial_data(&dataToSend, tosendsize);
		}

//			static int dir = 0;
//			if(dir == 1)
//				dir = 0;
//			else
//				dir = 1;
//			set_direction(dir);

		if(timeElapMs % 10 == 0)
		{
			PID_Update(&speedControl, rpmnow);

			rpmnow = rpmnow*0.95 + ((((float)get_hall_steps() / 24.0f) / 0.01f) * 60.0f)*0.05;
			set_hall_steps(0);


//			set_motor_speed(20.0f);
//			pwm_duty = 20.1f;
		}
	}
}




/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/


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
