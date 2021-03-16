#ifndef ADC_H_ /* include guard */
#define ADC_H_

#include "stm32f4xx.h"
#include "system.h"

#define ADC_BUF_LEN 		3

#define ADC_SCALE 			0.000805664f 	// 3.3V / 4096 = 0.000805664
#define ADC_ZERO 			1.65f			// 3.3V / 2
#define ADC_RES 			13.33333333f	// (1 / 750uOhm) / 100
#define MAX_CURRENT			3.5f			// Maximum current per phase

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

void ADC_Init(void);

#endif /* ADC_H_ */
