#include "system.h"


void WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
	if(PinState != GPIO_PIN_RESET)
		GPIOx->BSRR = GPIO_Pin;
	else
		GPIOx->BSRR = (uint32_t)GPIO_Pin << 16U;
}

uint8_t ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	return (GPIOx->IDR & GPIO_Pin) != (uint32_t)0;
}


/*
 * Maps input values 'x' from 'in_min' to 'out_min', and from 'in_max' to 'out_max.'
 */
float mapVal(float x, float in_min, float in_max, float out_min, float out_max)
{
	if(x > in_max) x = in_max;
	if(x < in_min) x = in_min;

	return ((x - in_min) * ((out_max - out_min) / (in_max - in_min))) + out_min;
}
