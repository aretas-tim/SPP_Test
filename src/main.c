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
#include "init.h"


typedef enum {
    PHASE_IDLE,
    PHASE_HEADER,
    PHASE_BODY
} Main_CommandReceivePhase;

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

AuthData_CombinedStore authDataStore;
TunnelStructures_TransportTunnel tunnel;
size_t Main_commRxBufferNeeded = TUNNEL_HEADER_LEN;
uint8_t Main_commRxInHeader = true;
TunnelStructures_TunnelBufferCtx commandInFlight;

Status_WorkStatus statusWork = WORK_WORKING;
Status_LockStatus statusLock = LOCK_LOCKED;

volatile uint8_t Main_doUnlockFlag = false;
volatile uint8_t Main_doEnterPinFlag = false;
volatile uint8_t Main_doVerifyPinFlag = false;
volatile uint8_t Main_doEnterOtcFlag = false;

uint32_t Main_lockAtTick = 0;
volatile uint8_t secondTicksUpdated = false;
bool Main_testAddEntryFlag = false;


static uint8_t Main_getCommand(TunnelStructures_TransportTunnel* tunnel, TunnelStructures_TunnelBufferCtx* commandInFlight);
//test stuff
void Main_testCallback(void);

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
  Init_systemClockConfig();
  Init_dmaInit(); //do this first
  Init_gpioInit();
  MX_USB_DEVICE_Init();
  Init_debugUartInit(&huart_debug);
  Init_timer3init();
  Init_timer7init();
  Init_timer17init();


  LED_Init(&hi2c, GPIOB, GPIO_PIN_4);
  LED_SetLEDs(true);
  //Init_spiMsInit(&hspi_ms); //@TODO reenable this
  Init_spiIccInit(&hspi_icc, &hdma_icc_rx, &hdma_icc_tx);

  UartDebug_init(&huart_debug);



  /* enable interrupts */
  __enable_irq();

  /*actual stuff starts here*/

  SpiFlash_selectNone();
  SpiFlash_setHold(false);
  SpiFlash_reset();

  Icc_init(&hspi_icc, GPIOC, GPIO_PIN_13, GPIOD, GPIO_PIN_2);
  Icc_start();
  HAL_Delay(10); //will force systick to fire
  UartDebug_sendline("USB Micro Started!\r\n");


while (1)
  {

    uint8_t commandReceived = 0;
    if(commandReceived) {

    }

    if(U2fHid_isMessageWaiting()) {

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

        U2fHid_secondTick(); //prevents the U2F HID system from locking up due to an unresponsive or ended host process
        UartDebug_sendline("BSPS-Firmware-USB Second Tick!\n");


    }



  }
}

void Main_testCallback(void) {
    Main_testAddEntryFlag = true;
}


/* attempts to get a command from the byte source that is the receive***Func pointers
 * returns true if a command was successfully read, false if it was not.
 * only modifies the commandInFlight if it returns true
 * uses the commandReceiveBuffer and related head count to hold partial commands until they're completely in-place
 * to boot this out to be thread-safe and support multiple invocations, need to pack the state (static variables, buffers, function pointers, etc)
 * in to a context struct
 */
uint8_t Main_getCommand(TunnelStructures_TransportTunnel* tunnel, TunnelStructures_TunnelBufferCtx* commandInFlight) {
#ifdef ARGLE /*ifdeffed out so this compiles whlie we're working on it. probably going to do this differently anyway*/
#define SRAM2_TUNNEL_RX_BUFF_LEN 0
    static uint8_t commandReceiveBuffer[TUNNEL_HEADER_LEN]; //don't need to keep more than the header locally, everything else can be read out in to the command buffer context
    static Main_CommandReceivePhase phase = PHASE_IDLE; //what phase we're at
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
                    cmdLen = Utilities_extract32(commandReceiveBuffer, TUNNEL_POS_LEN);
#ifdef DEBUG
                    UartDebug_sendline("Tunnel Incoming Header Dump:\n");
                    UartDebug_hexdump(commandReceiveBuffer, TUNNEL_HEADER_LEN);
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

                commandInFlight->tag = Utilities_extract16(commandReceiveBuffer, TUNNEL_POS_TAG);
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
                UartDebug_sendline("Tunnel Incoming Command Dump:\n");
                UartDebug_hexdump(commandInFlight->params, parameterLen);
#endif /*DEBUG*/
                if(TUNNEL_TAG_CMD_ENC == commandInFlight->tag) {
                    //encrypted command
                    TUNNEL_AES_CTR_CryptInPlace(tunnel, commandInFlight->params, parameterLen);
#ifdef DEBUG
                    UartDebug_sendline("Tunnel Decrypted Command Dump:\n");
                    UartDebug_hexdump(commandInFlight->params, parameterLen);
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
                    uint8_t authPassed = Utilities_compareDigests(hmacResultBuff, commandInFlight->params + (parameterLen- TUNNEL_HMAC_LENGTH), TUNNEL_HMAC_LENGTH);
                    if(authPassed) {
                        UartDebug_sendline("Tunnel Command Authorization Passed.\n");
                        memcpy(tunnel->nonceOdd, commandInFlight->params + (parameterLen - TUNNEL_NONCE_LENGTH - TUNNEL_HMAC_LENGTH), TUNNEL_NONCE_LENGTH); /* copy out the nonceOdd from the host for future use*/
                        commandInFlight->paramHead = parameterLen - (TUNNEL_HASH_LENGTH + TUNNEL_NONCE_LENGTH + TUNNEL_FIELD_ORD_LEN);
                        commandInFlight->authOkay = true;
                    } else {
                        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_AUTHFAIL);
                        UartDebug_sendline("Tunnel Command Authorization Failed.\n");
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
                commandInFlight->command = Utilities_extract32(commandInFlight->params, 0);
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
        UartDebug_callback();
    }

}


void HAL_SYSTICK_Callback(void) {
    ticks++;
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
        SpiFlash_interruptRxComplete(&hspi_ms);
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
        UartDebug_sendString("ICC SPI Error: ");
        UartDebug_hexprint32(hspicb->ErrorCode);
        UartDebug_newline();
        UartDebug_hexprint32(hspicb->Instance->SR);
        UartDebug_newline();
    }
}




/************************ Portions (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
