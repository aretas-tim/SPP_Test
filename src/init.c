/*
 * init.c
 *
 *  Created on: Feb 23, 2018
 *      Author: BenRogerson
 */

#include "init.h"


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows :
  *            System Clock source            = PLL (MSI)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            PLL_M                          = 1
  *            PLL_N                          = 40
  *            PLL_R                          = 2
  *            PLL_P                          = 7
  *            PLL_Q                          = 4
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  // USB timing is correct
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;

  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);


  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_USB;
  PeriphClkInit.Usart3ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV4;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  __PWR_CLK_ENABLE();

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}


void TIMER2_Init(void) {
    __TIM2_CLK_ENABLE();
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.RepetitionCounter = 0;
    HAL_TIM_Base_Init(&htim2);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

    TIM2->CNT = 0x0;
    TIM2->PSC = 0x1;
    TIM2->ARR = 0xFFFFFFFF;


}

void TIMER7_Init(void) { //for led update callbacks
    __TIM7_CLK_ENABLE();
    TIM7->CNT = 0x0;
    TIM7->ARR = 0xF804;
    TIM7->PSC = 0x0015; //prescale by 21
    TIM7->CR1 = 0x0080; //use autoreload
    TIM7->DIER = 0x0001; //enable interrupt generation



    HAL_NVIC_SetPriority(TIM7_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);
    TIM7->CR1 |= 0x0001;
}

//timer 8 is used to run the USB LEDs in PWM mode
void TIMER3_Init(void) {

    __TIM3_CLK_ENABLE();
    //SO MANY OPTIONS
    TIM3->CR1 = 0x0080; //auto-reload enabled
    TIM3->CR2 = 0x0000; //nothing special
    TIM3->SMCR = 0x0; //timer3 used as master
    TIM3->DIER = 0x0; //Timer3 does not generate interrupts
    TIM3->SR = 0x0; //clear the status register
    TIM3->EGR = 0x0; //do not generate any events
    TIM3->CCMR1 = 0x00006868; //dog's breakfast of bits. OC1, OC2 as output, Preload-enabled, PWM Mode 1
    TIM3->CCMR2 = 0x00000068; //same on OC3
    TIM3->CCER = 0x0333; // outputs enabled, active low
    TIM3->CNT = 0x0; //clear counter
    TIM3->PSC = 0x4E2; //prescale by 1250
    TIM3->ARR = 0xFF; //make it an 8 bit timer
    TIM3->BDTR = 0x00008000; //Main output enabled. MOE.. Moe.. moe.. moe.. moe.. moe..
    TIM3->DCR = 0x0; //not using DMA (though it'd be nifty if it did..)
    TIM3->DMAR = 0x0;
    TIM3->OR1 = 0x0;

    TIM3->CCR1 = 0x20;
    TIM3->CCR2 = 0x40;
    TIM3->CCR3 = 0x80;

    TIM3->CR1 |= 0x0001; //enable timer
}

/* timer17 is used to update secondTicks
 */
void TIMER17_Init(void) {
    __TIM17_CLK_ENABLE();

    TIM17->CR1 = 0x0080; //ARR in use, timer not enabled
    TIM17->CR2 = 0x0;
    TIM17->DIER = 0x0001; //update interrupt
    TIM17->SR = 0x0000; //clear flags
    TIM17->CCMR1 = 0x0;
    TIM17->CCMR2 = 0x0;
    TIM17->CNT = 0x0;
    TIM17->PSC = 2000; //count rate of 40kHz (presuming the CPU is running at 80MHz
    TIM17->ARR = 40000; //max count of 40k, update rate of 1Hz
    TIM17->RCR = 0x0; //how many overflows until we generate an update event, less 1 (8 bits) (so a 1 to 256 range)
    TIM17->CCR1 = 0x0;
    //do not start timer yet!
    HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM17_IRQn, 0, 0); //high priority is fine, all the interrupt does is set a flag, actual update is handled in main
    HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM17_IRQn);
    TIM17->CR1 |= 0x0001;
}

/* init function for the debug UART (USART2 on devboard, USART1 on device)*/
void DEBUG_UART_PERIPH_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;

#ifdef DEVICE_0_A_1_1_U
    /** DEVBOARD ICC disabled*/
    __UART4_CLK_ENABLE();

    huart_debug.Instance = UART4;

    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(UART4_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(UART4_IRQn);


#endif
    huart_debug.Init.BaudRate = 230400;
    huart_debug.Init.WordLength = UART_WORDLENGTH_8B;
    huart_debug.Init.StopBits = UART_STOPBITS_1;
    huart_debug.Init.Parity = UART_PARITY_NONE;
    huart_debug.Init.Mode = UART_MODE_TX; //TX only
    huart_debug.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart_debug.Init.OverSampling = UART_OVERSAMPLING_16;
    huart_debug.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED;
    huart_debug.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    HAL_UART_Init(&huart_debug);
#ifdef DEVICE_0_A_0_1
    huart_debug.Instance->BRR = 1388; //@TODO figure out why this sucks and i have to hard-code the BRR.
#endif
}



/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through
        * the Code Generation settings)
*/
void GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */


    __GPIOA_CLK_ENABLE();
    __GPIOB_CLK_ENABLE();
    __GPIOC_CLK_ENABLE();
    __GPIOD_CLK_ENABLE();
    __GPIOH_CLK_ENABLE();


#ifdef DEVICE_0_A_1_1_U
    //SPI Mass Storage helper stuff (HOLD, WP, not actually used but has to be present for it to work properly)
    //@TODO hardware write-protect when USB disconnected?
    //Port B
    //Pin 4: Mass Storage Storage Slave Select (output PP)
    //Pin 5: Encrypted Storage Hold (output PP)
    //Pin 6: Encrypted Storage Write Protect (output PP)

    // Outputs
    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL; //has external pullup
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIOB->BSRR = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;


    //@TODO re-enable the above. this is so it doesn't interfere with the secure micro's USB for now
    GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    //PA9: VBUS Detect Input
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT; //detect VBus
    GPIO_InitStruct.Pull = GPIO_NOPULL; //no pull needed
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);



    //USB LEDs
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD; //do not fight with the LED driver chip in case its enabled
    GPIO_InitStruct.Pull = GPIO_NOPULL; //external pulldown
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    //PA3 Power Good Input
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP; //external pulldown
    GPIO_InitStruct.Alternate = 0; //no alternate
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    //PB0 Charge Enable (Active Low, pulldown on battery charge IC)
    //PB1 Boost Enable (Active High, external pulldown)
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL; //external pulls
    GPIO_InitStruct.Alternate = 0; //no alternate
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIOB->BRR = GPIO_PIN_0|GPIO_PIN_1; //charge enabled, boost disabled (these should always be complementary else we're just wasting battery or backfeeding USB)

    //PC4 & PC5, Charge Current Set, (pulldowns on battery charge IC)
    /**
     * Charge current table: (from RT9525 datasheet, p11)
     * PC4 | PC5 | Input Current Limit
     * ===============================
     *  L  |  L  | 90mA
     *  L  |  H  | 475mA <-- this is where we want to be if connected to a non-OTG host and can negotiate enough power
     *  H  |  L  | 1.5A
     *  H  |  H  | Charge Suspend
     */
    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN; //external pulls
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = 0; //no alternate
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    GPIOC->BRR = GPIO_PIN_4; //always safe to pull 100mA unless we're connected to an OTG host, but that's handled in the USB section
    GPIOC->BSRR = GPIO_PIN_5;

    //IPC pins
    //COMMENTED OUT FOR DEVBOARD
    //PC13 (wakeup / transfer ready)
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    GPIOC->BSRR = GPIO_PIN_13;

    //PD2 IPC IRQ line (active low output)
    GPIOD->BSRR = GPIO_PIN_2; //set this first to prevent a false request
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP; //OD output, prevents contention
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);


#endif /* DEVICE_0_A_1_1_U */
}

void SPI_MS_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;

#ifdef DEVICE_0_A_1_1_U
    __SPI1_CLK_ENABLE();

    hspi_ms.Instance = SPI1;

    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP; //ensure a valid level at all times
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN; //ensure a valid level at all times, low on the clock to maintain polarity
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


#endif /* DEVICE_0_A_1_1_U */
    hspi_ms.Init.Mode = SPI_MODE_MASTER;
    hspi_ms.Init.Direction = SPI_DIRECTION_2LINES;
    hspi_ms.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi_ms.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi_ms.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi_ms.Init.NSS = SPI_NSS_SOFT;
    hspi_ms.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8; /* ~10MHz.*/
    hspi_ms.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi_ms.Init.TIMode = SPI_TIMODE_DISABLED;
    hspi_ms.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
    hspi_ms.Init.NSSPMode = SPI_NSS_PULSE_DISABLED;

    HAL_SPI_Init(&hspi_ms);
}

void SPI_ICC_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;

#ifdef DEVICE_0_A_1_1_U
    __SPI3_CLK_ENABLE();

    hspi_icc.Instance = SPI3;

    GPIO_InitStruct.Pin = GPIO_PIN_15; //select, change back to PA15 for non-devboard stuff @TODO
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    SYSCFG->EXTICR[4] = (SYSCFG->EXTICR[4] & ~SYSCFG_EXTICR4_EXTI15) | SYSCFG_EXTICR4_EXTI15_PA; //clear the EXTI15 bits, set to PA15
    EXTI->RTSR1 |= EXTI_RTSR1_RT15; //enable rising edge interrupt on external pins 15
    EXTI->IMR1 |= EXTI_IMR1_IM15; //unmask the interrupt request

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0); //lower priority than the DMA interrupt
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_15);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn); //enable interrupt


    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP; // pullups on data lines
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN; //ensure a valid level at all times, low on the clock to maintain polarity
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(SPI3_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(SPI3_IRQn);

#endif /* DEVICE_0_A_1_1_U */

    hspi_icc.Init.Mode = SPI_MODE_SLAVE;
    hspi_icc.Init.Direction = SPI_DIRECTION_2LINES;
    hspi_icc.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi_icc.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi_icc.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi_icc.Init.NSS = SPI_NSS_HARD_INPUT;
    hspi_icc.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi_icc.Init.TIMode = SPI_TIMODE_DISABLED;
    hspi_icc.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
    hspi_icc.Init.NSSPMode = SPI_NSS_PULSE_DISABLED;

    HAL_SPI_Init(&hspi_icc); //init SPI

    hdma_icc_rx.Instance = DMA2_Channel1;
    hdma_icc_rx.Init.Request = DMA_REQUEST_3;
    hdma_icc_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_icc_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_icc_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_icc_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_icc_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_icc_rx.Init.Mode = DMA_NORMAL;
    hdma_icc_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    HAL_DMA_Init(&hdma_icc_rx); //init DMA

    hspi_icc.hdmarx = &hdma_icc_rx; //link DMA to SPI
    hdma_icc_rx.Parent = &hspi_icc; //link SPI to DMA

    hdma_icc_tx.Instance = DMA2_Channel2;
    hdma_icc_tx.Init.Request = DMA_REQUEST_3;
    hdma_icc_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_icc_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_icc_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_icc_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_icc_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_icc_tx.Init.Mode = DMA_NORMAL;
    hdma_icc_tx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    HAL_DMA_Init(&hdma_icc_tx); //init DMA

    hspi_icc.hdmatx = &hdma_icc_tx; //link DMA to SPI
    hdma_icc_tx.Parent = &hspi_icc; //link SPI to DMA
}

//handles basic DMA init, make sure this is called before the DMAs are further set up
void DMA_Init(void) {
    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE(); //clock both DMAs

    //enable the ICC DMA IRQs with highest priority
    HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 1, 0);
    HAL_NVIC_SetPriority(DMA2_Channel2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
    HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);
}

