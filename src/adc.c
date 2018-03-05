/*
 * adc.c
 *
 *  Created on: May 10, 2016
 *      Author: me
 */

#include "stm32l4xx_hal.h"
#include "adc.h"
#include "uart_debug.h"


/* this is the BACKUP battery sense! */
uint16_t Adc_getVBat(ADC_HandleTypeDef* hadc) {

    ADC123_COMMON->CCR |= ADC_CCR_VBATEN;

    ADC_ChannelConfTypeDef sConfig;

    sConfig.Channel = ADC_CHANNEL_18; //VBat Sense on CH18
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    HAL_ADC_ConfigChannel(hadc, &sConfig);

    HAL_ADC_Start(hadc);
    HAL_ADC_PollForConversion(hadc, 1000);
    uint32_t result = HAL_ADC_GetValue(hadc);
    //UartDebug_printuint32(result);

    HAL_ADC_Stop(hadc);
    ADC123_COMMON->CCR &= ~ADC_CCR_VBATEN; //disable channel to prevent draining the backup battery after making the measurement

    return ((uint16_t) result);
}
uint16_t Adc_getPCIE12VSense(ADC_HandleTypeDef* hadc) {

    ADC_ChannelConfTypeDef sConfig;

    sConfig.Channel = ADC_CHANNEL_9; //PCI-E Sense on PA4
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    HAL_ADC_ConfigChannel(hadc, &sConfig);

    HAL_ADC_Start(hadc);
    HAL_ADC_PollForConversion(hadc, 1000);
    uint32_t result = HAL_ADC_GetValue(hadc);
    //UartDebug_printuint32(result);

    HAL_ADC_Stop(hadc);


    return ((uint16_t) result);

}

uint16_t Adc_getVRefInt(ADC_HandleTypeDef* hadc) {

    ADC_ChannelConfTypeDef sConfig;

    sConfig.Channel = ADC_CHANNEL_0; //VRefInt Sense on CH0
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    HAL_ADC_ConfigChannel(hadc, &sConfig);

    HAL_ADC_Start(hadc);
    HAL_ADC_PollForConversion(hadc, 1000);
    uint32_t result = HAL_ADC_GetValue(hadc);
    //UartDebug_printuint32(result);

    HAL_ADC_Stop(hadc);


    return ((uint16_t) result);
}

uint16_t Adc_getVCCmV(ADC_HandleTypeDef* hadc) {
    uint16_t vrefint = Adc_getVRefInt(hadc);
    if(vrefint) {
        return (uint16_t) ((uint32_t) ADC_VREFINT_RESULT_TO_VCC_MV_DIVIDEND / vrefint);
    } else {
#ifdef DEBUG
        UartDebug_sendline("VRefInt was 0 when attempting to get VCC voltage.\n");
#endif /* DEBUG */
        return 0;
    }
}

uint16_t Adc_getVBus(ADC_HandleTypeDef* hadc) {


    ADC_ChannelConfTypeDef sConfig;

    sConfig.Channel = ADC_CHANNEL_6; //VBus Sense on CH6, PA1
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    HAL_ADC_ConfigChannel(hadc, &sConfig);

    HAL_ADC_Start(hadc);
    HAL_ADC_PollForConversion(hadc, 1000);
    uint32_t result = HAL_ADC_GetValue(hadc);
    //UartDebug_printuint32(result);

    HAL_ADC_Stop(hadc);


    return ((uint16_t) result);
}

uint16_t Adc_getVMain(ADC_HandleTypeDef* hadc) {


    ADC_ChannelConfTypeDef sConfig;

    sConfig.Channel = ADC_CHANNEL_8; //VMain Sense on CH8, PA3
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    HAL_ADC_ConfigChannel(hadc, &sConfig);

    HAL_ADC_Start(hadc);
    HAL_ADC_PollForConversion(hadc, 1000);
    uint32_t result = HAL_ADC_GetValue(hadc);
    //UartDebug_printuint32(result);

    HAL_ADC_Stop(hadc);


    return ((uint16_t) result);
}

void Adc_enableSense(void) {
    GPIOB->BSRR = GPIO_PIN_2; //enable sense
    HAL_Delay(1); //wait for it to stabilize
}

void Adc_disableSense(void) {
    GPIOB->BRR = GPIO_PIN_2;
}
