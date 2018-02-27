/** 
  ******************************************************************************
  * @file    GPIO/GPIO_IOToggle/Src/stm32l4xx_it.c
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    25-November-2015
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
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
/*#include "main.h"*/
#include "stm32l4xx_it.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx.h"
#include "led.h"
#include "status.h"
//#include "tpm_pp.h"
#include "rtc.h"
#include "pat-config.h"
#include <stdbool.h>
#include "icc.h"

/** @addtogroup STM32L4xx_HAL_Examples
  * @{
  */
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

extern UART_HandleTypeDef huart_debug;
extern Status_WorkStatus statusWork;
extern Status_LockStatus statusLock;
//extern uint32_t secondTicks;
extern uint8_t secondTicksUpdated;
extern I2C_HandleTypeDef hi2c;
extern SPI_HandleTypeDef hspi_icc;
extern DMA_HandleTypeDef hdma_icc_tx;
extern DMA_HandleTypeDef hdma_icc_rx;
extern SPI_HandleTypeDef hspi_ms;


/** @addtogroup GPIO_IOToggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

/******************************************************************************/
/*                 STM32L4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32l4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

void USART2_IRQHandler(void) {
#ifdef DEVBOARD
    HAL_UART_IRQHandler(&huart_debug);
    LED_DebugSet();
#elif defined DEVICE_0_A_1_1_S
    HAL_UART_IRQHandler(&huart_debug);
    //no LED
#endif
    HAL_UART_IRQHandler(&huart_debug);
}

void USART1_IRQHandler(void) {
#ifdef DEVICE_0_A_0_1
    HAL_UART_IRQHandler(&huart_debug);
    LED_DebugSet();
#endif
}

void LPUART1_IRQHandler(void) {
#ifdef DEVBOARD
    if(huart_comm.Instance->ISR & UART_FLAG_RXNE) {
        uart_comm_receive();
    } else if (huart_comm.Instance->ISR & UART_FLAG_ORE) { //overrun, ignore the byte
        huart_comm.Instance->ICR = 0x00000008; //can't find a mnemonic for this one.
    }
    HAL_UART_IRQHandler(&huart_comm);
    LED_CommSet();
#endif
}
void USART3_IRQHandler(void) {
#ifdef DEVICE_0_A_0_1
    if(huart_comm.Instance->ISR & UART_FLAG_RXNE) {
        uart_comm_receive();
    } else if (huart_comm.Instance->ISR & UART_FLAG_ORE) { //overrun, ignore the byte
        huart_comm.Instance->ICR = 0x00000008; //can't find a mnemonic for this one.
    }
    HAL_UART_IRQHandler(&huart_comm);
    LED_CommSet();
#elif defined DEVBOARD
    HAL_UART_IRQHandler(&huart_debug);
    LED_DebugSet();
#endif
    HAL_UART_IRQHandler(&huart_debug);
}

void UART4_IRQHandler(void) {
#ifdef DEVICE_0_A_1_1_U
    HAL_UART_IRQHandler(&huart_debug);
#endif /* DEVICE_0_A_1_1_U */
}

void TIM6_DAC_IRQHandler(void) {
    //HAL_TIM_IRQHandler(&htim6);
    TIM6->SR = 0x0; //clear interrupt flag?
    //KEYPAD_Scan();
}


void TIM7_IRQHandler(void) {
    //GPIOC->BSRR = GPIO_PIN_3;
    /* note, must clear the interrupt or it'll just keep firing*/
    TIM7->SR = 0x0; //clear the interrupt flag
    LED_HeartbeatUpdate(statusLock, statusWork);
    //GPIOC->BRR = GPIO_PIN_3;
    //UartDebug_putchar('.');

}

void TIM1_TRG_COM_TIM17_IRQHandler(void) {
    //UartDebug_printuint32(TIM17->SR);
    //UartDebug_newline();
    //UartDebug_putchar('.');
    if(TIM17->SR) {
        TIM17->SR = 0x0; //clear the interrupt flag
        //secondTicks++;
        secondTicksUpdated = true;
        //PP_AssertDisable();

    }
}

void RTC_Alarm_IRQHandler(void) {
    //no alarms?
}
void OTG_FS_IRQHandler(void) {
    HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}

void I2C1_EV_IRQHandler(void) {
    HAL_I2C_EV_IRQHandler(&hi2c);
}

void SPI1_IRQHandler(void) {
    HAL_SPI_IRQHandler(&hspi_ms);
}

void SPI2_IRQHandler(void) {
    HAL_SPI_IRQHandler(&hspi_icc);
}

void SPI3_IRQHandler(void) {
    HAL_SPI_IRQHandler(&hspi_icc);
}

/*void WWDG_IRQHandler(void) {
    //do nothing?
}

void UART5_IRQHandler(void) {
    //something fucky is going on
    //this is getting called, same with the WWDG handler
    //neither of which is used
    UART5->ICR = 0x00121B5F;//nuke all interrupts
}*/

void DMA2_Channel2_IRQHandler(void) {

    HAL_DMA_IRQHandler(&hdma_icc_tx);
    //UartDebug_sendline("DMA 2 Ch 2 Interrupt!\n");
}

void DMA2_Channel1_IRQHandler(void) {
    /*if(__HAL_DMA_GET_FLAG(&hdma_icc_rx, __HAL_DMA_GET_HT_FLAG_INDEX(&hdma_icc_rx)) != RESET) {
        UartDebug_sendline("DMA 2 Ch 1 Half-Complete Interrupt!\n");
    } else {
        UartDebug_sendline("DMA 2 Ch 1 Interrupt!\n");
    }*/

    HAL_DMA_IRQHandler(&hdma_icc_rx);

}

void EXTI4_IRQHandler(void) {
    if(EXTI->PR1 & GPIO_PIN_4) {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);
        Icc_deselectedCallbackHandler();
        UartDebug_sendline("NSS rising edge interrupt.\n");
    }
}

void EXTI15_10_IRQHandler(void) {
    if(EXTI->PR1 & GPIO_PIN_15) {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_15);
        Icc_deselectedCallbackHandler();
        UartDebug_sendline("NSS rising edge interrupt.\n");
    }
}


/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
