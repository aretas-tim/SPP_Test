/*
 * adc.h
 *
 *  Created on: May 10, 2016
 *      Author: me
 */

#ifndef ADC_H_
#define ADC_H_

#include "stm32l4xx_hal_adc.h"
#include "stm32l4xx_hal_gpio.h"

#define ADC_SAMPLE_CYCLES_VBAT ADC_SAMPLETIME_640CYCLES_5
#define ADC_SAMPLE_CYCLES_PCI_12V ADC_SAMPLETIME_24CYCLES_5

#define ADC_CHANNEL_PCIE_12V_SENSE ADC_CHANNEL_9

#define ADC_VREFINT_RESULT_TO_VCC_MV_DIVIDEND 4956160
#define ADC_VBAT_PCI_MULTIPLIER 1000 //scale it up
#define ADC_VBAT_RESULT_TO_MV_DIVISOR 414 //approx
#define ADC_PCIE_RESULT_TO_MV_DIVISOR 295 //approx

#define ADC_VBAT_LOW_COUNTS 950 //approximately 2.3V
#define ADC_VBAT_HIGH_COUNTS 1241 //approximately 3V
#define ADC_VBAT_CHARGE_BELOW_COUNTS 1200 //approximately 2.9V


uint16_t ADC_GetVBat(ADC_HandleTypeDef* hadc);
uint16_t ADC_GetPCIE12VSense(ADC_HandleTypeDef* hadc);
uint16_t ADC_GetVRefInt(ADC_HandleTypeDef* hadc);
uint16_t ADC_GetVBus(ADC_HandleTypeDef* hadc);
uint16_t ADC_GetVMain(ADC_HandleTypeDef* hadc);

void ADC_EnableSense(void);
void ADC_DisableSense(void);

#define ADC_TIMEOUT 10 /* milliseconds*/

#endif /* ADC_H_ */
