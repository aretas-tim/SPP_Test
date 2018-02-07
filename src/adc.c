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
uint16_t ADC_GetVBat(ADC_HandleTypeDef* hadc) {
    /*//ADC1->ISR &= 0xFFFFFFE3; //clear EOC, EOS, OVR
    if(!(ADC1->ISR & ADC_ISR_ADRDY)) { //not ready
        ADC1->ISR |= ADC_ISR_ADRDY;
    }
    while(!(ADC1->ISR & ADC_ISR_ADRDY)); //spin until ready
    ADC1->SQR1 = 0x000000480; //one conversion, on channel 18 (VBat)
    ADC1->CR |= ADC_CR_ADSTART; //start conversion
    //while(ADC1->CR & ADC_CR_ADSTART); //spin until complete
    while(!(ADC1->ISR & ADC_ISR_EOC)); //spin until end of conversion flag is set
    uint16_t result = ADC1->DR; //clears end of conversion flag automatically
    //ADC1->ISR |= ADC_ISR_EOS; //clear end of sequence flag
    //while(ADC1->CR & ADC_CR_ADSTART); //spin until complete*/

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
    //uart_debug_printuint32(result);

    HAL_ADC_Stop(hadc);

    ADC123_COMMON->CCR &= ~ADC_CCR_VBATEN; //disable channel to prevent draining the backup battery after making the measurement

    return ((uint16_t) result);
}
uint16_t ADC_GetPCIE12VSense(ADC_HandleTypeDef* hadc) {
    /*//ADC1->ISR &= 0xFFFFFFE3; //clear EOC, EOS, OVR
    if(!(ADC1->ISR & ADC_ISR_ADRDY)) { //not ready
        ADC1->ISR |= ADC_ISR_ADRDY;
    }
    while(!(ADC1->ISR & ADC_ISR_ADRDY)); //spin until ready
    ADC1->SQR1 = 0x000000240; //one conversion, on channel 9 (PCI-E Sense)
    ADC1->CR |= ADC_CR_ADSTART; //start conversion
    //
    while(!(ADC1->ISR & ADC_ISR_EOC)); //spin until end of conversion flag is set
    uint16_t result = ADC1->DR; //clears end of conversion flag automatically
    //ADC1->ISR |= ADC_ISR_EOS; //clear end of sequence flag
    //while(ADC1->CR & ADC_CR_ADSTART); //spin until complete*/
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
    //uart_debug_printuint32(result);

    HAL_ADC_Stop(hadc);


    return ((uint16_t) result);

}

uint16_t ADC_GetVRefInt(ADC_HandleTypeDef* hadc) {
    /*//ADC1->ISR &= 0xFFFFFFE3; //clear EOC, EOS, OVR
    if(!(ADC1->ISR & ADC_ISR_ADRDY)) { //not ready
        ADC1->ISR |= ADC_ISR_ADRDY;
    }
    while(!(ADC1->ISR & ADC_ISR_ADRDY)); //spin until ready
    ADC1->SQR1 = 0x000000480; //one conversion, on channel 18 (VBat)
    ADC1->CR |= ADC_CR_ADSTART; //start conversion
    //while(ADC1->CR & ADC_CR_ADSTART); //spin until complete
    while(!(ADC1->ISR & ADC_ISR_EOC)); //spin until end of conversion flag is set
    uint16_t result = ADC1->DR; //clears end of conversion flag automatically
    //ADC1->ISR |= ADC_ISR_EOS; //clear end of sequence flag
    //while(ADC1->CR & ADC_CR_ADSTART); //spin until complete*/

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
    //uart_debug_printuint32(result);

    HAL_ADC_Stop(hadc);


    return ((uint16_t) result);
}

uint16_t ADC_GetVCCmV(ADC_HandleTypeDef* hadc) {
    uint16_t vrefint = ADC_GetVRefInt(hadc);
    if(vrefint) {
        return (uint16_t) ((uint32_t) ADC_VREFINT_RESULT_TO_VCC_MV_DIVIDEND / vrefint);
    } else {
#ifdef DEBUG
        uart_debug_sendline("VRefInt was 0 when attempting to get VCC voltage.\n");
#endif /* DEBUG */
        return 0;
    }
}

uint16_t ADC_GetVBus(ADC_HandleTypeDef* hadc) {

    //GPIOB->BSRR = GPIO_PIN_2; //enable sense
    //HAL_Delay(1); //wait for it to stabilize

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
    //uart_debug_printuint32(result);

    HAL_ADC_Stop(hadc);

    //GPIOB->BRR = GPIO_PIN_2; //disable sense

    return ((uint16_t) result);
}

uint16_t ADC_GetVMain(ADC_HandleTypeDef* hadc) {
    //GPIOB->BSRR = GPIO_PIN_2; //enable sense
    //HAL_Delay(1); //wait for it to stabilize

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
    //uart_debug_printuint32(result);

    HAL_ADC_Stop(hadc);

    //GPIOB->BRR = GPIO_PIN_2; //disable sense

    return ((uint16_t) result);
}

void ADC_EnableSense(void) {
    GPIOB->BSRR = GPIO_PIN_2; //enable sense
    HAL_Delay(1); //wait for it to stabilize
}

void ADC_DisableSense(void) {
    GPIOB->BRR = GPIO_PIN_2;
}
