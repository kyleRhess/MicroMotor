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
#include "adc.h"
#include "Serial.h"
#include "diag/Trace.h"
#include "cmsis_device.h"
#include "PID.h"


TIM_HandleTypeDef q_time;
TIM_HandleTypeDef htim1;
UART_HandleTypeDef huart1 = { .Instance = USART1 };
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);

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

	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	Signal_Init();

	MX_DMA_Init();
	MX_GPIO_Init();
	InitPWMOutput();

	// Start timer to count rotary encoder signals
	//Encoder_Init();
	//Encoder_ZInit();

#ifdef TRAPZ
	GPIO_InitTypeDef gLowPin;
	gLowPin.Pin = PIN_LOW_A|PIN_LOW_B|PIN_LOW_C;
	gLowPin.Mode = GPIO_MODE_OUTPUT_PP;
	gLowPin.Pull = GPIO_PULLDOWN;
	gLowPin.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOA, &gLowPin);
	System_WritePin(GPIOA, PIN_LOW_A, GPIO_PIN_RESET);
	System_WritePin(GPIOA, PIN_LOW_B, GPIO_PIN_RESET);
	System_WritePin(GPIOA, PIN_LOW_C, GPIO_PIN_RESET);
#endif

#ifdef DEBUG_PIN
	// Testing GPIO pin only
	GPIO_InitTypeDef gPins;
	gPins.Pin 		= GPIO_PIN_12;
	gPins.Mode 		= GPIO_MODE_OUTPUT_PP;
	gPins.Pull 		= GPIO_PULLUP;
	gPins.Speed 	= GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOB, &gPins);
#endif

	// Setup hall sensor inputs from BLDC motor
	Hall_InputInit();

	// Setup serial interface
	Serial_InitPort(921600, UART_STOPBITS_1, UART_WORDLENGTH_8B, UART_PARITY_NONE);

	//
	Serial_RxData(RX_BUFF_SZ);

	// Prepare FOC
	FOC_Init();

	// Start ADC + DMA before base PWM timer
	ADC_Init();

	// Start PWM signals and base-timer (40khz)
	HAL_TIM_Base_Start_IT(&PWMtimer.timer);
	HAL_TIM_PWM_Start(&PWMtimer.timer, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&PWMtimer.timer, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&PWMtimer.timer, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&PWMtimer.timer, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&PWMtimer.timer, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&PWMtimer.timer, TIM_CHANNEL_3);

	while (1)
	{
		// TODO: Move Signal monitor elsewhere
		///////////////////////////////////////////////////////////
		if((Signal_GetMotorState() & MOTOR_MODE_OVERCURRENT) ||
		   (Signal_GetMotorState() & MOTOR_MODE_OVERSPEED))
		{
			Signal_ClearMotorState(MOTOR_MODE_ENABLE);
			System_WritePin(GPIOB, GPIO_DIS_PIN, 1);
		}
		else
		{
			if(Signal_GetMotorState() & MOTOR_MODE_ENABLE)
				System_WritePin(GPIOB, GPIO_DIS_PIN, 0);
			else
				System_WritePin(GPIOB, GPIO_DIS_PIN, 1);
		}

		if(m_fSpeed < 0)
			Signal_SetMotorState(MOTOR_MODE_REVERSING);
		else
			Signal_ClearMotorState(MOTOR_MODE_REVERSING);

		if(Clock_GetMs() - Signal_GetHeartBeatMs() >= 500 && Clock_GetMs() != Signal_GetHeartBeatMs())
			Signal_SetMotorState(MOTOR_MODE_NOHEART);



		///////////////////////////////////////////////////////////


//		pi_speed.kP = Signal_GetMotorPosKp()/10.0f;
//		pi_speed.kI = Signal_GetMotorPosKi()/10.0f;

//		static float lastSP = 0;
//		static float lasttime = 0;
//		pi_pos.windupMax 		= Signal_GetMotorPosKp()*12000.0f;
//		pi_pos.windupMin 		= -Signal_GetMotorPosKp()*12000.0f;
//		pi_pos.setPoint = lastSP + (Clock_GetTimeS() - lasttime) * Signal_GetMotorPos()*10.0f;
//		lasttime = Clock_GetTimeS();
//		lastSP = pi_pos.setPoint;
	}
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
	RCC_OscInitStruct.PLL.PLLM = 16;//(HSI_VALUE/1000000u);
	RCC_OscInitStruct.PLL.PLLN = 200; // for 100 MHz
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; /* 84 MHz, conservative */
	RCC_OscInitStruct.PLL.PLLQ = 4; /* To make USB work. */
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	// Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
	// clocks dividers
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;

	// This is expected to work for most large cores.
	// Check and update it for your own configuration.
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3);

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
