/**
  ******************************************************************************
  * @file    GPIO/GPIO_IOToggle/Src/main.c
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    25-November-2015
  * @brief   This example describes how to configure and use GPIOs through
  *          the STM32L4xx HAL API.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "pat-config.h"
#include "stm32l476xx.h"
#include "stm32l4xx_nucleo.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_it.h"
#include "usb_device.h"
#include "uart_debug.h"
#include "spi_flash.h"
#include "led.h"
#include "tunnel.h"
#include "rtc.h"
#include "tunnel_lengths.h"
#include "sram2.h"
#include "backup_regs.h"
#include "adc.h"
#include <stdio.h>
#include <stdbool.h>
#include "usbd_tunnel_hid.h"
#include "usbd_tunnel_hid_if.h"
#include "usbd_hotkey_hid_if.h"
#include "icc.h"

typedef enum {
    PHASE_IDLE,
    PHASE_HEADER,
    PHASE_BODY
} CommandReceivePhase;

ADC_HandleTypeDef hadc;
I2C_HandleTypeDef hi2c;
RNG_HandleTypeDef hrng;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
SPI_HandleTypeDef hspi_ms; /* mass storage SPI */
SPI_HandleTypeDef hspi_icc; /* intercontroller communication SPI */
DMA_HandleTypeDef hdma_icc_rx; /* ICC DMA RX */
DMA_HandleTypeDef hdma_icc_tx; /* ICC DMA TX */
UART_HandleTypeDef huart_debug;
TIM_HandleTypeDef htim6;
RTC_HandleTypeDef hrtc;

volatile uint32_t ticks = 0;

AuthData_tdCombinedStore authDataStore;
TransportTunnel tunnel;
size_t commRxBufferNeeded = TUNNEL_HEADER_LEN;
uint8_t commRxInHeader = true;
TUNNEL_BUFFER_CTX commandInFlight;

WorkStatus statusWork = WORK_WORKING;
LockStatus statusLock = LOCK_LOCKED;

volatile uint8_t doUnlockFlag = false;
volatile uint8_t doEnterPinFlag = false;
volatile uint8_t doVerifyPinFlag = false;
volatile uint8_t doEnterOtcFlag = false;

uint32_t lockAtTick = 0;
volatile uint8_t secondTicksUpdated = false;
bool testAddEntryFlag = false;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void GPIO_Init(void);
static void DEBUG_UART_PERIPH_Init(void);
void SPI_MS_Init(void);
void SPI_ICC_Init(void);
void DMA_Init(void);
void TIMER2_Init(void); //timer2 is going to run at full CPU speed and can be used to time things with high precision
void TIMER7_Init(void);
void TIMER3_Init(void);
void TIMER17_Init(void); //secondTicks
void memDump(uint8_t* start, size_t end);
void testFunc(uint32_t ignored);
void testKBCallback(void);
uint8_t getCommand(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* commandInFlight);
//test stuff
void testCallback(void);

size_t (*receiveAvailableFunc) (void);
size_t (*receiveDataFunc) (uint8_t*, size_t);
uint8_t (*receiveByteFunc) (void);




int main(void)
{


  /* This sample code shows how to use GPIO HAL API to toggle LED2 IOs
    in an infinite loop. */

  /* STM32L4xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();


  /* Configure the system clock to 80 MHz */
  SystemClock_Config();
  //__HAL_RCC_RTC_ENABLE(); //no RTC, handled on secure micro. we don't care about this so-called 'real' time
  //RTC_Init();
  //WWDG->CR = 0x0;
  DMA_Init(); //do this first
  GPIO_Init();
  MX_USB_DEVICE_Init();
  DEBUG_UART_PERIPH_Init();
  TIMER3_Init();
  TIMER7_Init();
  TIMER17_Init();

  //ScanI2C();
  LED_Init(&hi2c, GPIOB, GPIO_PIN_4);
  LED_SetLEDs(true);
  //SPI_MS_Init(); //@TODO reenable this
  SPI_ICC_Init();

  UART_Debug_Init(&huart_debug);



  /* enable interrupts */
  __enable_irq();

  /*actual stuff starts here*/

  SPI_FLASH_SelectNone();
  SPI_FLASH_SetHold(false);
  SPI_FLASH_Reset();

  //Icc_init(&hspi_icc, GPIOA, GPIO_PIN_1, GPIOA, GPIO_PIN_0); //devboard
  Icc_init(&hspi_icc, GPIOC, GPIO_PIN_13, GPIOD, GPIO_PIN_2);
  Icc_start();

  HAL_Delay(10); //will force systick to fire


  uart_debug_sendline("USB Micro Started!\r\n");






  while (1)
  {

    uint8_t commandReceived = 0; //= getCommand(&tunnel, &commandInFlight);
    if(commandReceived) {
        //commandRunner(&commandInFlight);
        //HAL_Delay(100);
    }

    if(U2F_HID_IsMessageWaiting()) {
        //U2F_HandleHIDMessage(); //if there's a U2F message waiting over USB HID, handle it
    }

    if(secondTicksUpdated) {
        secondTicksUpdated = false;
        static bool ledOn = false;
        if(ledOn) {
            GPIOA->BRR = GPIO_PIN_5;
            ledOn = false;
        } else {
            GPIOA->BSRR = GPIO_PIN_5;
            ledOn = true;
        }

        U2F_HID_SecondTick(); //prevents the U2F HID system from locking up due to an unresponsive or ended host process
        uart_debug_sendline("Second Tick!\n");


    }



  }
}

void testCallback(void) {
    testAddEntryFlag = true;
}










/* attempts to get a command from the byte source that is the receive***Func pointers
 * returns true if a command was successfully read, false if it was not.
 * only modifies the commandInFlight if it returns true
 * uses the commandReceiveBuffer and related head count to hold partial commands until they're completely in-place
 * to boot this out to be thread-safe and support multiple invocations, need to pack the state (static variables, buffers, function pointers, etc)
 * in to a context struct
 */
uint8_t getCommand(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* commandInFlight) {
#ifdef ARGLE /*ifdeffed out so this compiles whlie we're working on it. probably going to do this differently anyway*/
#define SRAM2_TUNNEL_RX_BUFF_LEN 0
    static uint8_t commandReceiveBuffer[TUNNEL_HEADER_LEN]; //don't need to keep more than the header locally, everything else can be read out in to the command buffer context
    static CommandReceivePhase phase = PHASE_IDLE; //what phase we're at
    static size_t cmdLen = 0; //the total length we're trying to read
    static size_t bytesRead = 0; //how many bytes we've already read
    size_t bytesAvailable = receiveAvailableFunc();
    if(!bytesAvailable) {
        //no bytes to read, return false
        return false;
    } else {
        if(phase == PHASE_IDLE) {
            //here we read until we have the two header magic bytes
            if(bytesRead == 0) {
                commandReceiveBuffer[0] = receiveByteFunc();
                bytesRead = 1;
            }
            while(receiveAvailableFunc() > 0) {
                commandReceiveBuffer[1] = receiveByteFunc();
                if((commandReceiveBuffer[0] == TUNNEL_HEADER_BYTE) && (commandReceiveBuffer[1] == TUNNEL_HEADER_BYTE)) {
                    bytesRead = 2;
                    phase = PHASE_HEADER;
                    break;
                } else {
                    commandReceiveBuffer[0] = commandReceiveBuffer[1]; //shift up the byte
                }
            }

        }
        if(phase == PHASE_HEADER) {
            bytesAvailable = receiveAvailableFunc(); //update this
            if((bytesAvailable + bytesRead) >= TUNNEL_HEADER_LEN) {
                bytesRead += receiveDataFunc(commandReceiveBuffer + bytesRead, TUNNEL_HEADER_LEN - bytesRead); //read out our header
                if(bytesRead >= TUNNEL_HEADER_LEN) {
                    cmdLen = extract32(commandReceiveBuffer, TUNNEL_POS_LEN);
#ifdef DEBUG
                    uart_debug_sendline("Tunnel Incoming Header Dump:\n");
                    uart_debug_hexdump(commandReceiveBuffer, TUNNEL_HEADER_LEN);
#endif /*DEBUG*/
                    if(cmdLen <= SRAM2_TUNNEL_RX_BUFF_LEN) {
                        phase = PHASE_BODY;
                    } else {
                        //too long to process
                        phase = PHASE_IDLE;
                        bytesRead = 0;
                        cmdLen = 0;
                        memset(commandReceiveBuffer, 0, TUNNEL_HEADER_LEN); //clear this out
                        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_OUT_OF_MEMORY); //inform the host
                    }
                }
            }
        }
        if(phase == PHASE_BODY) {
            bytesAvailable = receiveAvailableFunc(); //update this
            if(bytesAvailable + bytesRead >= cmdLen) {
                //we can read in the whole command
                TUNNEL_BufferInit(commandInFlight); //initialize (or re-initialize) the buffer context to handle the new command

                commandInFlight->tag = extract16(commandReceiveBuffer, TUNNEL_POS_TAG);
                if(TUNNEL_TAG_CMD_CLEAR == commandInFlight->tag) {
                    commandInFlight->hasAuth = false;
                } else if (TUNNEL_TAG_CMD_ENC == commandInFlight->tag) {
                    commandInFlight->hasAuth = true;
                } else {
                    commandInFlight->hasAuth = false;
                    commandInFlight->command = TUNNEL_ORD_BAD_TAG;
                }
                uint32_t parameterLen = receiveDataFunc(commandInFlight->params, (cmdLen - TUNNEL_HEADER_LEN)); //read out the rest of the command in to the parameter buffer
#ifdef DEBUG
                uart_debug_sendline("Tunnel Incoming Command Dump:\n");
                uart_debug_hexdump(commandInFlight->params, parameterLen);
#endif /*DEBUG*/
                if(TUNNEL_TAG_CMD_ENC == commandInFlight->tag) {
                    //encrypted command
                    TUNNEL_AES_CTR_CryptInPlace(tunnel, commandInFlight->params, parameterLen);
#ifdef DEBUG
                    uart_debug_sendline("Tunnel Decrypted Command Dump:\n");
                    uart_debug_hexdump(commandInFlight->params, parameterLen);
#endif /* DEBUG */
                    /* have a decrypted buffer at this point */
                    uint8_t hmacBuff[TUNNEL_HASH_LENGTH + TUNNEL_NONCE_LENGTH * 2]; //temp buffer for our HMAC paramaters
                    //digest the actual paramaters (including the command code), put in the hmacBuff
                    mbedtls_md(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), commandInFlight->params, (parameterLen - TUNNEL_NONCE_LENGTH - TUNNEL_HMAC_LENGTH), hmacBuff);
                    //copy even nonce over to hmac buff
                    memcpy(hmacBuff + TUNNEL_HASH_LENGTH, tunnel->nonceEven, TUNNEL_NONCE_LENGTH);
                    //copy the incoming odd nonce
                    memcpy(hmacBuff + (TUNNEL_HASH_LENGTH + TUNNEL_NONCE_LENGTH), commandInFlight->params + (parameterLen - (TUNNEL_NONCE_LENGTH + TUNNEL_HMAC_LENGTH)), TUNNEL_NONCE_LENGTH);
                    uint8_t hmacResultBuff[TUNNEL_HMAC_LENGTH]; //the result of the hmac goes here
                    //run the HMAC
                    mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), tunnel->sessionKey, TUNNEL_SESSION_KEY_LENGTH, hmacBuff, (TUNNEL_HASH_LENGTH + TUNNEL_NONCE_LENGTH * 2), hmacResultBuff);
                    //check local hmac vs incoming hmac
                    uint8_t authPassed = compareDigests(hmacResultBuff, commandInFlight->params + (parameterLen- TUNNEL_HMAC_LENGTH), TUNNEL_HMAC_LENGTH);
                    if(authPassed) {
                        uart_debug_sendline("Tunnel Command Authorization Passed.\n");
                        memcpy(tunnel->nonceOdd, commandInFlight->params + (parameterLen - TUNNEL_NONCE_LENGTH - TUNNEL_HMAC_LENGTH), TUNNEL_NONCE_LENGTH); /* copy out the nonceOdd from the host for future use*/
                        commandInFlight->paramHead = parameterLen - (TUNNEL_HASH_LENGTH + TUNNEL_NONCE_LENGTH + TUNNEL_FIELD_ORD_LEN);
                        commandInFlight->authOkay = true;
                    } else {
                        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_AUTHFAIL);
                        uart_debug_sendline("Tunnel Command Authorization Failed.\n");
                        phase = PHASE_IDLE;
                        bytesRead = 0;
                        cmdLen = 0;
                        memset(commandReceiveBuffer, 0, TUNNEL_HEADER_LEN); //clear this out
                        return false;
                    }

                } else {
                    //clear command
                    commandInFlight->paramHead = parameterLen - TUNNEL_FIELD_ORD_LEN;
                    commandInFlight->authOkay = false;
                }
                commandInFlight->command = extract32(commandInFlight->params, 0);
                commandInFlight->params = commandInFlight->params + TUNNEL_FIELD_ORD_LEN; //set this pointer 4 in to pass over the command ordinal
                commandInFlight->extractHead = 0;
                phase = PHASE_IDLE;
                bytesRead = 0;
                cmdLen = 0;
                memset(commandReceiveBuffer, 0, TUNNEL_HEADER_LEN); //clear this out
                return true;
            }
        }
    }
#endif /* ARGLE*/
    return false;
}















void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart == &huart_debug) {
        uart_debug_callback();
    }

}


void HAL_SYSTICK_Callback(void) {
    ticks++;
    //KEYPAD_Scan();
    ledCallback();
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    //handles detection of VBus
    if (GPIO_Pin == GPIO_PIN_9) {
        USBD_Conf_CallbackReceived();
    }

}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspicb) {
    if(hspicb == &hspi_ms) {
        SPI_FLASH_IT_RxComplete(&hspi_ms);
    } else if (hspicb == &hspi_icc) {
        Icc_dmaRxCompleteCallback();
    }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspicb) {
    if (hspicb == &hspi_icc) {
        Icc_dmaTxCompleteCallback();
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspicb) {
    if(hspicb == &hspi_icc) {
        uart_debug_sendstring("ICC SPI Error: ");
        uart_debug_hexprint32(hspicb->ErrorCode);
        uart_debug_newline();
        uart_debug_hexprint32(hspicb->Instance->SR);
        uart_debug_newline();
    }
}

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

  //initialize HSE and PLL

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
  /*
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSEState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 40;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;*/


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
  //PeriphClkInit.PLLSAI1.PLLSAI1M = 3;
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/*void ADC_Init(void) {
    __HAL_RCC_ADC_CLK_ENABLE();

    ADC123_COMMON->CCR |= ADC_CCR_VBATEN | ADC_CCR_TSEN | ADC_CCR_VREFEN; //everything else is okay

    ADC_ChannelConfTypeDef sConfig;

    hadc.Instance = ADC1;

    hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
    hadc.Init.Resolution = ADC_RESOLUTION_12B;
    hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc.Init.LowPowerAutoWait = DISABLE;
    hadc.Init.ContinuousConvMode = DISABLE;
    hadc.Init.NbrOfConversion = 1;
    hadc.Init.DiscontinuousConvMode = DISABLE;
    hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc.Init.DMAContinuousRequests = ENABLE;
    hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc.Init.OversamplingMode = DISABLE;
    HAL_ADC_Init(&hadc);

    sConfig.Channel = ADC_CHANNEL_0; //VREF internal
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    HAL_ADC_ConfigChannel(&hadc, &sConfig);

    HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED); //calibrate ADC


}*/




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
    //TIM2->CCR1 |= 0x0001; //enable the timer
    /* basically just clears the timer*/

}

void TIMER7_Init(void) { //for led update callbacks
    __TIM7_CLK_ENABLE();
    //TIM_MasterConfigTypeDef sMasterConfig;

    /*htim6.Instance = TIM7;
    htim6.Init.Prescaler = 0;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 0xFFFF;
    HAL_TIM_Base_Init(&htim6);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig);*/

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

    /*__USART3_CLK_ENABLE();

    huart_debug.Instance = USART3;

    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(USART3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);*/
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
    //__GPIOF_CLK_ENABLE();

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

    //USB pins (not including USB D+/D-)

    //PB14: USB switch /OE Output
    //@TODO re-enable this
    /*
    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD; //wired OR with the secure Micro
    GPIO_InitStruct.Pull = GPIO_NOPULL; //external pullup
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIOB->BRR = GPIO_PIN_14; //enable USB
    */

    //PB13: USB switch select Output
    //@TODO re-enable this
    /*
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL; //external pulldown
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIOB->BRR = GPIO_PIN_13; //select us as the USB device
    */
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
    //GPIOC->BSRR = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8; //high-impedance for now


    //Power/Charge Control Pins

    //PA2 Charge Active Input
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
    /*
    //PA1 (wakeup / transfer ready) for devboard
    GPIOA->BSRR = GPIO_PIN_1;
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = 0; //no alternate
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    //PA0 IPC IRQ line (active low output)
    GPIOA->BSRR = GPIO_PIN_0; //set this first to prevent a false request
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP; //OD output, prevents contention
    GPIO_InitStruct.Alternate = 0; //no alternate
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    //setup board LED on PA5 @TODO remove this
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Alternate = 0; //no alternate
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); */



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

    //HAL_NVIC_SetPriority(SPI1_IRQn, 1, 0);
    //HAL_NVIC_EnableIRQ(SPI1_IRQn);

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
     //re-enable for non-devboard
    /*
    SYSCFG->EXTICR[2] = (SYSCFG->EXTICR[2] & ~SYSCFG_EXTICR2_EXTI4) | SYSCFG_EXTICR2_EXTI4_PA; //clear the EXTI4 bits, set to PA4
    EXTI->RTSR1 |= EXTI_RTSR1_RT4; //enable rising edge interrupt on external pins 15
    EXTI->IMR1 |= EXTI_IMR1_IM4; //unmask the interrupt request


    HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0); //lower priority than the DMA interrupt
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn); //enable interrupt
    */

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




/************************ Portions (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
