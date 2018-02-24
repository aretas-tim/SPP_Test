/*
 * init.h
 *
 *  Created on: Feb 23, 2018
 *      Author: BenRogerson
 */

#ifndef INIT_H_
#define INIT_H_

#include "stm32l4xx_hal.h"
#include "pat-config.h"

void Init_systemClockConfig(void);
void Init_timer2init(TIM_HandleTypeDef* htim2);
void Init_timer7init(void);
void Init_timer3init(void);
void Init_timer17init(void);
void Init_debugUartInit(UART_HandleTypeDef* huart_debug );
void Init_gpioInit(void);
void Init_spiMsInit(SPI_HandleTypeDef* hspi_ms);
void Init_spiIccInit(SPI_HandleTypeDef* hspi_icc, DMA_HandleTypeDef* hdma_icc_rx, DMA_HandleTypeDef* hdma_icc_tx);
void Init_dmaInit(void);


#endif /* INIT_H_ */
