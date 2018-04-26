/*****< HAL.c >***************************************************************/
/*      Copyright 2012 - 2014 Stonestreet One.                               */
/*      All Rights Reserved.                                                 */
/*                                                                           */
/*      Copyright 2015 Texas Instruments Incorporated.                       */
/*      All Rights Reserved.                                                 */
/*                                                                           */
/*  HAL - Hardware Abstraction function for ST STM3240G-EVAL Board           */
/*                                                                           */
/*  Author:  Marcus Funk                                                     */
/*                                                                           */
/*** MODIFICATION HISTORY ****************************************************/
/*                                                                           */
/*   mm/dd/yy  F. Lastname    Description of Modification                    */
/*   --------  -----------    -----------------------------------------------*/
/*   07/05/12  M. Funk        Initial creation.                              */
/*   11/24/14  R. Malovany    Update.                                        */
/*****************************************************************************/

/* Library includes. */
#include "HAL.h"                 /* Function for Hardware Abstraction.       */
#include "HALCFG.h"
#include "BTPSKRNL.h"            /* BTPS Kernel Header.                      */
#include "stm32l4xx.h"


   /* The following defines the Buffer sizes that will be used for the  */
   /* console UART.                                                     */
   /* * NOTE * As the HAL_ConsoleWrite operation does not block, the    */
   /*          output buffer MUST be large enough to store the longest  */
   /*          printed message.                                         */
#define HAL_OUTPUT_BUFFER_SIZE            3000
#define HAL_INPUT_BUFFER_SIZE             64

/*
These below defines are bootstraped in by Tim here to get this sample prgram running on stm32l4xx
*/
#define USART_WordLength_8b                  ((uint32_t)0x0000)
#define USART_StopBits_1                     ((uint32_t)0x0000)
#define USART_Parity_No                      ((uint32_t)0x0000)
#define USART_Mode_Rx                        ((uint32_t)0x0004)
#define USART_Mode_Tx                        ((uint32_t)0x0008)
#define USART_HardwareFlowControl_None       ((uint32_t)0x0000)



/***********/

#define EnableConsoleUartPeriphClock()    CONSOLE_UART_RCC_PERIPH_CLK_CMD(CONSOLE_UART_RCC_PERIPH_CLK_BIT, ENABLE)
#define DisableConsoleUartPeriphClock()   CONSOLE_UART_RCC_PERIPH_CLK_CMD(CONSOLE_UART_RCC_PERIPH_CLK_BIT, DISABLE)

#define DisableInterrupts()               __set_PRIMASK(1)
#define EnableInterrupts()                __set_PRIMASK(0)

/* The following structure contains the buffers for the Console UART.   */
typedef struct _tagHAL_UartContext_t
{
   USART_TypeDef *Base;

   unsigned char  RxBuffer[HAL_INPUT_BUFFER_SIZE];
   unsigned int          RxBufferSize;
   volatile unsigned int RxBytesFree;
   unsigned int          RxInIndex;
   unsigned int          RxOutIndex;

   unsigned char  TxBuffer[HAL_OUTPUT_BUFFER_SIZE];
   unsigned int          TxBufferSize;
   volatile unsigned int TxBytesFree;
   unsigned int          TxInIndex;
   unsigned int          TxOutIndex;
} HAL_UartContext_t;

   /* Default UART config.                                              */
//static USART_InitTypeDef ConsoleUartConfig = {115200, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No, USART_Mode_Rx | USART_Mode_Tx, USART_HardwareFlowControl_None};

//changed the below line from USART_InitTypeDef to UART_InitTypeDef
static UART_InitTypeDef ConsoleUartConfig = {115200, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No, USART_Mode_Rx | USART_Mode_Tx, USART_HardwareFlowControl_None};


//Tim comment out below 3 lines because not using these objects to initializae the GPIO or UART
//static BTPSCONST GPIO_InitTypeDef HAL_TXDGpioConfiguration  =  {(1 << CONSOLE_TXD_PIN), GPIO_Mode_AF,  GPIO_Speed_25MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL};
//static BTPSCONST GPIO_InitTypeDef HAL_RXDGpioConfiguration  =  {(1 << CONSOLE_RXD_PIN), GPIO_Mode_AF,  GPIO_Speed_25MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL};
//static BTPSCONST GPIO_InitTypeDef HAL_LEDGpioConfiguration  =  {(1 << HAL_LED_PIN),     GPIO_Mode_OUT, GPIO_Speed_25MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL};

HAL_UartContext_t        HAL_UartContext;

static void HAL_RxInterrupt(void);
static void HAL_TxInterrupt(void);

   /* The following function is the Interrupt Service Routine for the   */
   /* UART RX interrupt.  The function is passed the Context of the UART*/
   /* that is to be serviced.                                           */
static void HAL_RxInterrupt(void)
{
   /* read the first byte from the port.                                */
   while((HAL_UartContext.RxBytesFree) && (USART_GetFlagStatus(HAL_UartContext.Base, USART_FLAG_RXNE) == SET))
   {
      /* Read a character from the port into the receive buffer         */
      HAL_UartContext.RxBuffer[HAL_UartContext.RxInIndex] = (unsigned char)USART_ReceiveData(HAL_UartContext.Base);

      HAL_UartContext.RxBytesFree --;
      HAL_UartContext.RxInIndex ++;

      /* See if we need to roll the RxInIndex back to 0.                */
      if(HAL_UartContext.RxInIndex == HAL_UartContext.RxBufferSize)
         HAL_UartContext.RxInIndex = 0;
   }

   /* if the buffer is full, read in the remaining data from the port.  */
   if(!HAL_UartContext.RxBytesFree)
   {
      while(USART_GetFlagStatus(HAL_UartContext.Base, USART_FLAG_RXNE) == SET)
         USART_ReceiveData(HAL_UartContext.Base);
   }
}

   /* The following function is the FIFO Primer and Interrupt Service   */
   /* Routine for the UART TX interrupt.  The function is passed the    */
   /* Context of the UART that is to be serviced.                       */
static void HAL_TxInterrupt(void)
{
   /* The interrupt was caused by the THR becoming empty.  Are there any*/
   /* more characters to transmit?                                      */
   //while((HAL_UartContext.TxBytesFree != HAL_UartContext.TxBufferSize) && (HAL_UartContext.Base->SR & USART_FLAG_TXE))

	//NOTE!!!!! There is no uart status register on the stm32l4. The above line checks the status register for the TXE (Transmit data register empty) bit which is
	//in the UART_ISR data register on the STM32L4
	while((HAL_UartContext.TxBytesFree != HAL_UartContext.TxBufferSize) && (HAL_UartContext.Base->ISR & USART_FLAG_TXE))
   {
      /* Place the next character into the output buffer.               */
      USART_SendData(HAL_UartContext.Base, HAL_UartContext.TxBuffer[HAL_UartContext.TxOutIndex]);

      /* Adjust the character counts and check to see if the index needs*/
      /* to be wrapped.                                                 */
      HAL_UartContext.TxBytesFree ++;
      HAL_UartContext.TxOutIndex ++;
      if(HAL_UartContext.TxOutIndex == HAL_UartContext.TxBufferSize)
         HAL_UartContext.TxOutIndex = 0;
   }

   if(HAL_UartContext.TxBytesFree == HAL_UartContext.TxBufferSize)
   {  //tim changed
      /* No more data to send, inhibit transmit interrupts.             */
      //USART_ITConfig(HAL_UartContext.Base, USART_IT_TXE, DISABLE);
	   NVIC_DisableIRQ(USART2_IRQn);
   }
}

   /* The following function handles the UART interrupts for the        */
   /* console.                                                          */
void CONSOLE_UART_IRQ_HANDLER(void)
{
   unsigned int Flags;
   unsigned int Control;

   Flags   = HAL_UartContext.Base->ISR;
   Control = HAL_UartContext.Base->CR1;
   /* Check to see if data is available in the Receive Buffer.          */
   if((Flags & (USART_FLAG_RXNE | USART_FLAG_ORE)) && (Control & (1 << (USART_IT_RXNE & 0x1F))))
      HAL_RxInterrupt();
   else
   {
      /* Check to see if the transmit buffers are ready for more data.  */
      if((Flags & USART_FLAG_TXE) && (Control & (1 << (USART_IT_TXE & 0x1F))))
         HAL_TxInterrupt();
   }
}


/* The following function configures the hardware as required for the   */
/* sample applications.                                                 */
//void HAL_ConfigureHardware(void)
void HAL_ConfigureHardware(UART_HandleTypeDef* huart_console)
{


   BTPS_MemInitialize(&HAL_UartContext, 0, sizeof(HAL_UartContext_t));

   HAL_UartContext.Base         = CONSOLE_UART_BASE;
   HAL_UartContext.RxBufferSize = HAL_INPUT_BUFFER_SIZE;
   HAL_UartContext.RxBytesFree  = HAL_INPUT_BUFFER_SIZE;
   HAL_UartContext.TxBufferSize = HAL_OUTPUT_BUFFER_SIZE;
   HAL_UartContext.TxBytesFree  = HAL_OUTPUT_BUFFER_SIZE;

   NVIC_SetPriorityGrouping(3);

   /* Enable the peripheral clocks for the UART.                        */
   //EnableConsoleUartPeriphClock();


   //changed by tim
   /* Configure used GPIO                                               */
  /* RCC_AHB1PeriphClockCmd(CONSOLE_TXD_GPIO_AHB_BIT, ENABLE);
   GPIO_Init(CONSOLE_TXD_GPIO_PORT, (GPIO_InitTypeDef *)&HAL_TXDGpioConfiguration);
   GPIO_PinAFConfig(CONSOLE_TXD_GPIO_PORT, CONSOLE_TXD_PIN, CONSOLE_UART_GPIO_AF);*/

   /*----Below Added by Tim-------*/
   GPIO_InitTypeDef gpioInitStruct;

   __USART3_CLK_ENABLE(); //enable uart3 clock for console uart
   __GPIOC_CLK_ENABLE(); //enable the clock for port C used by Console uart tx pin

   /*TX Pin initialization for PC10 on UART3*/
   gpioInitStruct.Pin = GPIO_PIN_10;
   gpioInitStruct.Mode = GPIO_MODE_AF_PP;
   gpioInitStruct.Pull = GPIO_NOPULL;// changed from GPIO_PULLUP
   gpioInitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM; //5-25 Mhz
   gpioInitStruct.Alternate = GPIO_AF7_USART3;
   HAL_GPIO_Init(GPIOC, &gpioInitStruct);
   /*-----------------------------*/

   /*RCC_AHB1PeriphClockCmd(CONSOLE_RXD_GPIO_AHB_BIT, ENABLE);
   GPIO_Init(CONSOLE_RXD_GPIO_PORT, (GPIO_InitTypeDef *)&HAL_RXDGpioConfiguration);
   GPIO_PinAFConfig(CONSOLE_RXD_GPIO_PORT, CONSOLE_RXD_PIN, CONSOLE_UART_GPIO_AF);*/

   /*RX Pin initialization for PC11 on UART3*/
   gpioInitStruct.Pin = GPIO_PIN_11;
   gpioInitStruct.Mode = GPIO_MODE_AF_PP;
   gpioInitStruct.Pull = GPIO_NOPULL;// changed from GPIO_PULLUP
   gpioInitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM; //5-25 Mhz
   gpioInitStruct.Alternate = GPIO_AF7_USART3;
   HAL_GPIO_Init(GPIOC, &gpioInitStruct);

   //configure led for PA5 on nucleo board
   //RCC_AHB1PeriphClockCmd(HAL_LED_GPIO_AHB_BIT, ENABLE);
   __GPIOA_CLK_ENABLE(); //enable the clock for GPIOA for led on PA5
   //GPIO_Init(HAL_LED_GPIO_PORT, (GPIO_InitTypeDef *)&HAL_LEDGpioConfiguration);
   gpioInitStruct.Pin = GPIO_PIN_5;
   gpioInitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   gpioInitStruct.Pull = GPIO_NOPULL;// changed from GPIO_PULLUP
   gpioInitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM; //5-25 Mhz
   HAL_GPIO_Init(GPIOA, &gpioInitStruct);

   /* Initialize the UART.                                              */
   /*USART_Init(HAL_UartContext.Base, &ConsoleUartConfig);
   USART_ITConfig(HAL_UartContext.Base, USART_IT_RXNE, ENABLE);
   USART_Cmd(HAL_UartContext.Base, ENABLE);*/

   huart_console->Instance = USART3;
   huart_console->Init.BaudRate = 115200;
   huart_console->Init.WordLength = UART_WORDLENGTH_8B;
   huart_console->Init.StopBits = UART_STOPBITS_1;
   huart_console->Init.Parity = UART_PARITY_NONE;
   huart_console->Init.Mode = UART_MODE_TX_RX;
   huart_console->Init.HwFlowCtl = UART_HWCONTROL_NONE;
   huart_console->Init.OverSampling = UART_OVERSAMPLING_16;
   huart_console->Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED;
   huart_console->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
   HAL_UART_Init(huart_console);

   //NVIC_SetPriority(CONSOLE_UART_IRQ, 0xF);
   //NVIC_EnableIRQ(CONSOLE_UART_IRQ);

   NVIC_SetPriority(USART3_IRQn, 0xF);
   NVIC_EnableIRQ(USART3_IRQn);
}

   /* The following function is used to illuminate an LED.  The number  */
   /* of LEDs on a board is board specific.  If the LED_ID provided does*/
   /* not exist on the hardware platform then nothing is done.          */
void HAL_LedOn(int LED_ID)
{
   if(LED_ID == 0)
      GPIO_SetBits(HAL_LED_GPIO_PORT, (1 << HAL_LED_PIN));
}

   /* The following function is used to extinguish an LED.  The number  */
   /* of LEDs on a board is board specific.  If the LED_ID provided does*/
   /* not exist on the hardware platform then nothing is done.          */
void HAL_LedOff(int LED_ID)
{
   if(LED_ID == 0)
      GPIO_ResetBits(HAL_LED_GPIO_PORT, (1 << HAL_LED_PIN));
}

   /* The following function is used to toggle the state of an LED.  The*/
   /* number of LEDs on a board is board specific.  If the LED_ID       */
   /* provided does not exist on the hardware platform then nothing is  */
   /* done.                                                             */
void HAL_LedToggle(int LED_ID)
{
   if(LED_ID == 0)
   {
      if(GPIO_ReadOutputDataBit(HAL_LED_GPIO_PORT, (1 << HAL_LED_PIN)) == Bit_SET)
         HAL_LedOff(LED_ID);
      else
         HAL_LedOn(LED_ID);
   }
}

   /* The following function is used to retrieve data from the UART     */
   /* input queue.  the function receives a pointer to a buffer that    */
   /* will receive the UART characters a the length of the buffer.  The */
   /* function will return the number of characters that were returned  */
   /* in Buffer.                                                        */
int HAL_ConsoleRead(int Length, char *Buffer)
{
   int ret_val;

   if((Length) && (Buffer))
   {
      /* Set the size to be copied equal to the smaller of the length   */
      /* and the bytes in the receive buffer.                           */
      ret_val = HAL_UartContext.RxBufferSize - HAL_UartContext.RxBytesFree;
      ret_val = (ret_val < Length) ? ret_val : Length;

      if(ret_val > (HAL_UartContext.RxBufferSize - HAL_UartContext.RxOutIndex))
      {
         /* The data wraps around the end of the buffer, so copy it in  */
         /* two steps.                                                  */
         Length = (HAL_UartContext.RxBufferSize - HAL_UartContext.RxOutIndex);
         BTPS_MemCopy(Buffer, &HAL_UartContext.RxBuffer[HAL_UartContext.RxOutIndex], Length);
         BTPS_MemCopy((Buffer + Length), HAL_UartContext.RxBuffer, (ret_val - Length));

         HAL_UartContext.RxOutIndex = ret_val - Length;
      }
      else
      {
         BTPS_MemCopy(Buffer, &HAL_UartContext.RxBuffer[HAL_UartContext.RxOutIndex], ret_val);

         HAL_UartContext.RxOutIndex += ret_val;

         if(HAL_UartContext.RxOutIndex == HAL_UartContext.RxBufferSize)
            HAL_UartContext.RxOutIndex = 0;
      }

      HAL_UartContext.RxBytesFree += ret_val;
   }
   else
      ret_val = 0;

   return(ret_val);
}


   /* The following function is used to send data to the UART output    */
   /* queue.  the function receives a pointer to a buffer that will     */
   /* contains the data to send and the length of the data.  The        */
   /* function will return the number of characters that were           */
   /* successfully saved in the output buffer.                          */
int HAL_ConsoleWrite(int Length, char *Buffer)
{
   int ret_val;
   int Count;
   int BytesFree;

   if((Length) && (Buffer))
   {
      ret_val = 0;

      while(Length)
      {
         /* Wait for space to be availale in the buffer.                */
         while(!HAL_UartContext.TxBytesFree)
            BTPS_Delay(1);

         /* The data may have to be copied in 2 phases.  Calculate the  */
         /* number of character that can be placed in the buffer before */
         /* the buffer must be wrapped.                                 */
         BytesFree = HAL_UartContext.TxBytesFree;
         Count = Length;
         Count = (BytesFree < Count) ? BytesFree : Count;
         Count = ((HAL_UartContext.TxBufferSize - HAL_UartContext.TxInIndex) < Count) ? (HAL_UartContext.TxBufferSize - HAL_UartContext.TxInIndex) : Count;

         BTPS_MemCopy(&(HAL_UartContext.TxBuffer[HAL_UartContext.TxInIndex]), Buffer, Count);

         /* Adjust the counts and index.                                */
         Buffer                      += Count;
         Length                      -= Count;
         ret_val                     += Count;
         HAL_UartContext.TxInIndex   += Count;
         if(HAL_UartContext.TxInIndex == HAL_UartContext.TxBufferSize)
            HAL_UartContext.TxInIndex = 0;

         DisableInterrupts();
         HAL_UartContext.TxBytesFree -= Count;
         //changed by tim
         //USART_ITConfig(HAL_UartContext.Base, USART_IT_TXE, ENABLE);
         NVIC_EnableIRQ(USART3_IRQn);
         EnableInterrupts();
      }
   }
   else
      ret_val = 0;

   return(ret_val);
}

//pasted here by tim so LED stuff will work
void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin)
{
  /* Check the parameters */
  //assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  //assert_param(IS_GPIO_PIN(GPIO_Pin));

  //GPIOx->BSRRL = GPIO_Pin;
  GPIOx->BSRR = GPIO_Pin;
}

//pasted here by tim so LED stuff will work
void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin)
{
  /* Check the parameters */
  //assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  //assert_param(IS_GPIO_PIN(GPIO_Pin));

  GPIOx->BRR = GPIO_Pin;
}

uint8_t GPIO_ReadOutputDataBit(GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin)
{
  uint8_t bitstatus = 0x00;

  /* Check the parameters */
  //assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  //assert_param(IS_GET_GPIO_PIN(GPIO_Pin));

  if (((GPIOx->ODR) & GPIO_Pin) != (uint32_t)Bit_RESET)
  {
    bitstatus = (uint8_t)Bit_SET;
  }
  else
  {
    bitstatus = (uint8_t)Bit_RESET;
  }
  return bitstatus;
}

FlagStatus USART_GetFlagStatus(USART_TypeDef* USARTx, uint32_t USART_FLAG)
{
  FlagStatus bitstatus = RESET;
  /* Check the parameters */
  //assert_param(IS_USART_ALL_PERIPH(USARTx));
  //assert_param(IS_USART_FLAG(USART_FLAG));

  /* The CTS flag is not available for UART4 and UART5 */
  if (USART_FLAG == USART_FLAG_CTS)
  {
    assert_param(IS_USART_1236_PERIPH(USARTx));
  }

  //if ((USARTx->SR & USART_FLAG) != (uint16_t)RESET)
  if ((USARTx->ISR & USART_FLAG) != (uint16_t)RESET)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}

/**
  * @brief  Returns the most recent received data by the USARTx peripheral.
  * @param  USARTx: where x can be 1, 2, 3, 4, 5, 6, 7 or 8 to select the USART or
  *         UART peripheral.
  * @retval The received data.
  */
uint16_t USART_ReceiveData(USART_TypeDef* USARTx)
{
  /* Check the parameters */
  //assert_param(IS_USART_ALL_PERIPH(USARTx));

  /* Receive Data */
  //changed by tim
  //return (uint16_t)(USARTx->DR & (uint16_t)0x01FF);
  return (uint16_t)(USARTx->RDR & (uint16_t)0x01FF);
}

/**
  * @brief  Transmits single data through the USARTx peripheral.
  * @param  USARTx: where x can be 1, 2, 3, 4, 5, 6, 7 or 8 to select the USART or
  *         UART peripheral.
  * @param  Data: the data to transmit.
  * @retval None
  */
void USART_SendData(USART_TypeDef* USARTx, uint16_t Data)
{
  /* Check the parameters */
  //assert_param(IS_USART_ALL_PERIPH(USARTx));
  //assert_param(IS_USART_DATA(Data));

  /* Transmit Data */
  //changed by tim
  //USARTx->DR = (Data & (uint16_t)0x01FF);
  USARTx->TDR = (Data & (uint16_t)0x01FF);
}

/* init function for the debug UART (USART2 on devboard, USART1 on device)*/
void Init_debugUartInit(UART_HandleTypeDef* huart_debug ) {
	GPIO_InitTypeDef Init_gpioInitStruct;

	__GPIOA_CLK_ENABLE();
	__USART2_CLK_ENABLE();

    huart_debug->Instance = USART2;

    Init_gpioInitStruct.Pin = GPIO_PIN_2;
    Init_gpioInitStruct.Mode = GPIO_MODE_AF_PP;
    Init_gpioInitStruct.Pull = GPIO_PULLUP;
    Init_gpioInitStruct.Speed = GPIO_SPEED_LOW;
    Init_gpioInitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &Init_gpioInitStruct);

    HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    huart_debug->Init.BaudRate = 230400;
    huart_debug->Init.WordLength = UART_WORDLENGTH_8B;
    huart_debug->Init.StopBits = UART_STOPBITS_1;
    huart_debug->Init.Parity = UART_PARITY_NONE;
    huart_debug->Init.Mode = UART_MODE_TX; //TX only
    huart_debug->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart_debug->Init.OverSampling = UART_OVERSAMPLING_16;
    huart_debug->Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED;
    huart_debug->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    HAL_UART_Init(huart_debug);

}

