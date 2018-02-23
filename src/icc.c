/*
 * icc.c
 *
 *  Created on: Mar 17, 2017
 *      Author: me
 */

/* IMPORTANT: Directionality
 * Directionality is copied from USB where possible to keep it simple (as simple as this can get)
 * IN: Towards the host, away from the secure micro
 * OUT: towards the secure micro, away from the host
 *
 * DMA RX is an IN transaction
 * DMA TX is an OUT transaction
 * */

#include "icc.h"
#include "hotkey.h"
#include "sram2.h"


Icc_tdTargetInfo* Icc_currentTarget = NULL; /* pointer to the buffer that the DMA is currently using, direction depends on context */
uint16_t Icc_currentTransferLen = 0; /* holds how many bytes we're expecting to transfer */
Icc_tdDmaStateEnum Icc_dmaState = ICC_DMA_ERROR;
Icc_tdTransferInfo Icc_lastTransferInfo = {.targetValid = false, .target = 0, .length = 0};
uint8_t Icc_targetInfoBuffer[ICC_TARGET_INFO_BUFFER_LENGTH]; //because its shared between two functions now, saves memory

SPI_HandleTypeDef* Icc_spiHandle = NULL;
bool Icc_isInitialized = false;
/* these are multiplexed with the wakeup pin, as it has no function outside of deep sleep */
GPIO_TypeDef* Icc_readyPort = NULL;
uint16_t Icc_readyPin = 0; //no pin
/* outgoing IRQ to secure micro, to let it know we have data waiting */
GPIO_TypeDef* Icc_irqPort = NULL;
uint16_t Icc_irqPin = 0; //no pin

volatile bool Icc_waitForTransferCompleteCallback = false; //used to pause the deselected callback until the rx/tx complete callback has finished
uint8_t Icc_headerInBuffer[ICC_HEADER_LENGTH]; /* header is always "in", though it proceeds no further than us */
uint8_t Icc_commandInBuffer[ICC_COMMAND_BUFFER_LENGTH];






static bool Icc_iccCommandHandler(uint8_t command, uint16_t len); /* short commands */
static bool Icc_iccExtendedCommandHandler(uint8_t* data, uint16_t len);
static void Icc_headerHandler(uint8_t header[ICC_HEADER_LENGTH]);
static void Icc_iccTransmitCompleteCallback(void);
static void Icc_assertDataWaiting(void);
static bool Icc_checkForDataWaiting(void);
//there is no clear, check for data waiting will clear as-needed
static Icc_tdTargetInfo* Icc_lookupTargetInfo(uint8_t target);
static void Icc_preparePowerMode(void);
static void Icc_prepareLastTransferInfo(uint8_t target);
static void Icc_prepareTargetList(uint16_t len);
static void Icc_prepareTargetMaximumsList(uint16_t len);
static void Icc_setPowerMode(uint8_t mode);
static void Icc_setUSBEnabled(bool isUSBEnabled);
static HAL_StatusTypeDef Icc_setupHeaderReceive(void);

Icc_tdTargetInfo Icc_targetLEDs = {
        .number = ICC_TARGET_LEDS,
        .lastTransferDirection = 0,
        .lastTransferLen = 0,
        .inBuffer = NULL,
        .inBufferLen = 0,
        .inBufferLenMax = 0,
        .inBufferState = ICC_BUFFER_STATE_IDLE,
        .outBuffer = NULL,
        .outBufferLen = 0,
        .outBufferLenMax = 0,
        .outBufferState = ICC_BUFFER_STATE_IDLE,
        .rxCallback = NULL,
        .txCompleteCallback = NULL
};

Icc_tdTargetInfo Icc_targetTunnel = {
        .number = ICC_TARGET_TUNNEL,
        .lastTransferDirection = 0,
        .lastTransferLen = 0,
        .inBuffer = NULL,
        .inBufferLen = 0,
        .inBufferLenMax = 0,
        .inBufferState = ICC_BUFFER_STATE_IDLE,
        .outBuffer = NULL,
        .outBufferLen = 0,
        .outBufferLenMax = 0,
        .outBufferState = ICC_BUFFER_STATE_IDLE,
        .rxCallback = NULL,
        .txCompleteCallback = NULL
};

Icc_tdTargetInfo Icc_targetU2f = {
        .number = ICC_TARGET_U2F,
        .lastTransferDirection = 0,
        .lastTransferLen = 0,
        .inBuffer = NULL,
        .inBufferLen = 0,
        .inBufferLenMax = 0,
        .inBufferState = ICC_BUFFER_STATE_IDLE,
        .outBuffer = NULL,
        .outBufferLen = 0,
        .outBufferLenMax = 0,
        .outBufferState = ICC_BUFFER_STATE_IDLE,
        .rxCallback = NULL,
        .txCompleteCallback = NULL
};

Icc_tdTargetInfo Icc_targetHotkey = {
        .number = ICC_TARGET_HOTKEY,
        .lastTransferDirection = 0,
        .lastTransferLen = 0,
        .inBuffer = NULL,
        .inBufferLen = 0,
        .inBufferLenMax = 0,
        .inBufferState = ICC_BUFFER_STATE_IDLE,
        .outBuffer = NULL,
        .outBufferLen = 0,
        .outBufferLenMax = 0,
        .outBufferState = ICC_BUFFER_STATE_IDLE,
        .rxCallback = NULL,
        .txCompleteCallback = NULL
};
Icc_tdTargetInfo Icc_targetIcc = {
        .number = ICC_TARGET_ICC,
        .lastTransferDirection = 0,
        .lastTransferLen = 0,
        .inBuffer = Icc_commandInBuffer,
        .inBufferLen = 0,
        .inBufferLenMax = ICC_COMMAND_BUFFER_LENGTH,
        .inBufferState = ICC_BUFFER_STATE_IDLE,
        .outBuffer = NULL,
        .outBufferLen = 0,
        .outBufferState = ICC_BUFFER_STATE_IDLE,
        .rxCallback = Icc_iccExtendedCommandHandler, /* short commands are handled by a separate code path, not this callback */
        .txCompleteCallback = Icc_iccTransmitCompleteCallback
};

Icc_tdTargetInfo* Icc_targetInfoList[ICC_NUM_TARGETS] = {
        &Icc_targetIcc,
        &Icc_targetLEDs,
        &Icc_targetTunnel,
        &Icc_targetU2f,
        &Icc_targetHotkey,
};



void Icc_init(SPI_HandleTypeDef* hspi, GPIO_TypeDef* readyPort, uint16_t readyPin, GPIO_TypeDef* IRQPort, uint16_t IRQPin) {
    //do init things where once we know what needs to be initialized
    if(hspi != NULL) {
        Icc_spiHandle = hspi;
    } else {
        Icc_spiHandle = NULL;
        Icc_readyPort = NULL;
        Icc_readyPin = 0;
        Icc_isInitialized = false;
        uart_debug_sendline("ICC unable to start. SPI Handle was NULL.\n");
        return;
    }
    if(NULL != readyPort && 0 != readyPin) { //0 on a pin means no pin
        Icc_readyPort = readyPort;
        Icc_readyPin = readyPin;
        Icc_irqPort = IRQPort;
        Icc_irqPin = IRQPin;
    } else {
        Icc_spiHandle = NULL;
        Icc_readyPort = NULL;
        Icc_readyPin = 0;
        Icc_isInitialized = false;
        uart_debug_sendline("ICC unable to start. Transfer ready port/pin was invalid.\n");
        return;
    }
    //set up local targets
    bool targetSetupOkay = Icc_setupTarget(ICC_TARGET_ICC, Icc_commandInBuffer, ICC_COMMAND_BUFFER_LENGTH, Icc_iccExtendedCommandHandler, ICC_COMMAND_BUFFER_LENGTH, Icc_iccTransmitCompleteCallback);
    if(!targetSetupOkay) {
        uart_debug_sendline("ICC unable to start, ICC target setup failed.");
        Icc_isInitialized = false;
        return;
    }

    Icc_isInitialized = true;
}

/* ICC start
 * starts up the ICC by setting up to receive the first packet and pulling the interrupt line low to
 * tells the secure micro we have data waiting (we don't, but it will get it to pull the target info from us)
 *
 * call this after Init and after all targets have been set up
 */
void Icc_start(void) {
    if(!Icc_isInitialized) {
        return;
    }
    //setup DMA to receive first packet
    HAL_StatusTypeDef rc =  Icc_setupHeaderReceive();
    if(rc != HAL_OK) {
        uart_debug_sendstring("Unable to enable DMA receive during ICC Startup (");
        uart_debug_printuint8(rc);
        uart_debug_sendline(")\n");
        uart_debug_sendline("ICC unable to start.\n");
        return;
    } else {
        uart_debug_sendline("ICC Started.\n");
    }
    if(Icc_irqPort != NULL) { //if this is non-null, which it can be to operate in polling-only
        Icc_irqPort->BRR = Icc_irqPin; //bring this low to let the secure micro know we're ready and it can query us
    }
}

/**
 * sets up a target
 * param target the target number (use the macros)
 * param rxBuffer a data buffer to put received data, can be NULL if no data is expected
 * param rxBufferLen the length of the buffer, can be 0
 * param rxCallback a pointer to the function to call when data has been received from the secure micro. can be NULL if no data is expected
 * param txCompleteCallback a pointer to the function to call when a transmission to the secure micro has been completed, can be NULL if notification is required.
 */
bool Icc_setupTarget(uint8_t target, uint8_t* rxBuffer, uint16_t rxBufferLen, bool (*rxCallback)(uint8_t*, uint16_t), uint16_t maxTxLen, void (*txCompleteCallback)(void)) {
    Icc_tdTargetInfo* targetInfo = Icc_lookupTargetInfo(target);
    if(targetInfo == NULL) {
        return false;
    } else {
        targetInfo->rxCallback = rxCallback;
        targetInfo->txCompleteCallback = txCompleteCallback;
        targetInfo->inBuffer = rxBuffer;
        targetInfo->inBufferLenMax = rxBufferLen;
        targetInfo->outBufferLenMax = maxTxLen;
        if((targetInfo->rxCallback != NULL) && (targetInfo->inBuffer != NULL) && (targetInfo->inBufferLenMax > 0)) {
            targetInfo->inBufferState = ICC_BUFFER_STATE_READY;
        }
        return true;
    }
}

/* param force true to force it to run (used in the NSS rising interrupt to ensure the rx fifo is flushed)
 *
 */
HAL_StatusTypeDef Icc_setupHeaderReceive() {
    //if(Icc_dmaState != ICC_DMA_HEADER) { //prevent it from running when already setup
        HAL_SPIEx_FlushRxFifo(Icc_spiHandle);
        Icc_dmaState = ICC_DMA_HEADER;
        uart_debug_sendline("ICC Waiting for Header.\n");
        HAL_StatusTypeDef rc = HAL_SPI_Receive_DMA(Icc_spiHandle, Icc_headerInBuffer, ICC_HEADER_LENGTH);
        Icc_checkForDataWaiting(); //only ever run this now, to prevent the secure micro from being too aggressive.
    //}
    return rc;
}


void Icc_dmaRxCompleteCallback(void) {
    //grab a copy of the data transfer register from the RX DMA channel,
    //this *should* be 0 (all data transferred)
    //if its not, we're missing bytes, probably caused by /SS going high before all the data was transferred
    uint32_t transferRemainingLen = Icc_spiHandle->hdmarx->Instance->CNDTR;
    uart_debug_sendline("ICC DMA Rx Complete Callback\n");
    switch(Icc_dmaState) {
        case ICC_DMA_HEADER:
            if(transferRemainingLen) {
                Icc_setupHeaderReceive(); //try to get another header, something went bad with this one
#ifdef DEBUG_ICC
                uart_debug_sendline("ICC Received short header.\n");
#endif /* DEBUG_ICC */
            } else {
                Icc_dmaState = ICC_DMA_IDLE;
                Icc_headerHandler(Icc_headerInBuffer); //that was easy
                Icc_readyPort->BRR = Icc_readyPin; //take this low to say we're ready.
            }
            break;
        case ICC_DMA_RECEIVE:
            Icc_readyPort->BSRR = Icc_readyPin; //take this back high, no longer ready
            Icc_dmaState = ICC_DMA_IDLE;
            if(Icc_currentTarget != NULL) {
                //call the rx callback, if it exists
                if(Icc_currentTarget->rxCallback != NULL && Icc_currentTarget->inBufferLenMax > 0) {
                    Icc_currentTarget->inBufferLen = Icc_currentTransferLen - transferRemainingLen;
                    Icc_currentTarget->inBufferState = ICC_BUFFER_STATE_PROCESSING; //busy being processed elsewhere
                    uart_debug_sendstring("ICC received ");
                    uart_debug_printuint32(Icc_currentTarget->inBufferLen);
                    uart_debug_sendline(" bytes. Processing.\n");
                    bool rc = Icc_currentTarget->rxCallback(Icc_currentTarget->inBuffer, Icc_currentTarget->inBufferLen);
                    if(!rc) { //buffer released by callback
                        Icc_currentTarget->inBufferState = ICC_BUFFER_STATE_READY;
                    } // else buffer kept by callback for later processing or further transmission back to the host
                    Icc_lastTransferInfo.targetValid = true;
                } else {
                    uart_debug_sendline("ICC received data to valid target, but there was no rx callback.\n");
                    Icc_lastTransferInfo.targetValid = false; //transferred to an invalid target, i.e. nothing was done with the data
                }
                Icc_lastTransferInfo.target = Icc_currentTarget->number | ICC_HEADER_TARGET_IN;

            } else {
                uart_debug_sendline("ICC received data to invalid target.\n");
                Icc_lastTransferInfo.target = ICC_TARGET_NULL | ICC_HEADER_TARGET_IN;
                Icc_lastTransferInfo.targetValid = false;
            }
            Icc_lastTransferInfo.length = Icc_currentTransferLen - transferRemainingLen;

            Icc_setupHeaderReceive(); //set up to receive the next header
            break;
        default:
            uart_debug_sendstring("ICC DMA RX Complete Callback called while in invalid Icc_dmaState: ");
            uart_debug_printuint32(Icc_dmaState);
            uart_debug_newline();
            break;
    }
    Icc_waitForTransferCompleteCallback = false; //finished, anything that was waiting no longer needs to wait
}

/* we've finished sending some data to the secure micro */
void Icc_dmaTxCompleteCallback(void) {
    if(Icc_dmaState != ICC_DMA_TRANSMIT) {
        uart_debug_sendstring("ICC DMA TX Complete Callback called while in invalid Icc_dmaState:");
        uart_debug_printuint32(Icc_dmaState);
        uart_debug_newline();
        return;
    }
    Icc_readyPort->BSRR = Icc_readyPin; //take this back high, no longer ready
    uint32_t remainingTransferLen = Icc_spiHandle->hdmatx->Instance->CNDTR; //get any remaining transfer length
    uart_debug_sendline("ICC DMA Tx Complete Callback\n");
    uint32_t actualTransferLen;
    if(remainingTransferLen >= Icc_currentTransferLen) {
        //there were no bytes transferred before /SS went high, or someone was mucking with Icc_currentTransferLen
        actualTransferLen = 0;
    } else {
        actualTransferLen = Icc_currentTransferLen - remainingTransferLen;
    }

    if(Icc_currentTarget != NULL) {

        //check for underflow
        if(Icc_currentTarget->outBufferLen < actualTransferLen) {
            //how did we get here anyway?
            Icc_currentTarget->outBufferLen = 0; //no more data available
#ifdef DEBUG_ICC
            uart_debug_sendline("ICC Transmit Underflowed.\n");
#endif /* DEBUG_ICC */

        } else {
            Icc_currentTarget->outBufferLen -= actualTransferLen;
        }
        if(0 == Icc_currentTarget->outBufferLen) {
            Icc_currentTarget->outBufferState = ICC_BUFFER_STATE_IDLE;
            if(Icc_currentTarget->txCompleteCallback != NULL) {
                Icc_currentTarget->txCompleteCallback(); //run the callback if its not NULL
            }
        } else {
            Icc_currentTarget->outBuffer += Icc_currentTransferLen; //move pointer forward
        }
        Icc_dmaState = ICC_DMA_IDLE; //no matter what we're back to idling
        Icc_lastTransferInfo.length = actualTransferLen;
        Icc_lastTransferInfo.targetValid = true;
        Icc_lastTransferInfo.target = Icc_currentTarget->number;
        Icc_currentTarget = NULL; //null out our current target as we're finished for now, secure micro will need to request more data
    } else {
        Icc_lastTransferInfo.length = actualTransferLen; //subtract off any remaining length in the DMA
        Icc_lastTransferInfo.targetValid = false;
        Icc_lastTransferInfo.target = ICC_TARGET_NULL;
    }
    Icc_waitForTransferCompleteCallback = false;
    Icc_setupHeaderReceive(); //prep to receive the next header
}

/*if the in buffer of a target was kept around by the rxCallback
 * this will release it
 */
void ICC_ReleaseInBuffer(uint8_t target) {
    Icc_tdTargetInfo* targetInfo = Icc_lookupTargetInfo(target); /* look up the target info structure, get the pointer */
    if(targetInfo->inBufferState == ICC_BUFFER_STATE_PROCESSING) { //only release if its in the processing state
        targetInfo->inBufferState = ICC_BUFFER_STATE_READY;
#ifdef DEBUG_ICC
    } else {
            uart_debug_sendstring("ICC_ReleaseInBuffer called with buffer not in correct state (");
            uart_debug_printuint8(targetInfo->inBufferState);
            uart_debug_sendstring(").\n");
#endif /* DEBUG_ICC */
    }
}

Icc_tdTargetInfo* Icc_lookupTargetInfo(uint8_t target) {
    for(uint32_t i = 0; i < ICC_NUM_TARGETS; ++i) {
        if(Icc_targetInfoList[i]->number == (target & ICC_TARGET_NUMBER_MASK)) { //mask off direction bit
            return Icc_targetInfoList[i];
        }
    }
    return NULL;
}

/**
 * checks the known targets for any out data waiting and updates the IRQ line
 */
bool Icc_checkForDataWaiting(void) {
    bool isDataWaiting = false;
    for(uint32_t i = 0; i < ICC_NUM_TARGETS; ++i) {
        if(Icc_targetInfoList[i]->outBufferState == ICC_BUFFER_STATE_READY) {
            isDataWaiting = true;
            uart_debug_sendline("ICC Has Out Data Waiting.\n");
            break;
        }
    }
    if(Icc_irqPort != NULL) { //this can be null, in theory.
        if(isDataWaiting) {
            Icc_irqPort->BRR = Icc_irqPin;
        } else {
            Icc_irqPort->BSRR = Icc_irqPin;
        }
    }
    return isDataWaiting;
}

void Icc_assertDataWaiting(void) {
    if(Icc_irqPort != NULL) { //this can be null, in theory.
        Icc_irqPort->BRR = Icc_irqPin;
    }
}

/**
 * handles the header upon reception. Sets up the DMA to transmit/receive the data portion
 *
 * param header a 4-length uint8_t array
 */
void Icc_headerHandler(uint8_t header[ICC_HEADER_LENGTH]) {
    uint8_t target = header[ICC_HEADER_TARGET_POS];
    uint8_t command = header[ICC_HEADER_COMMAND_POS];
    uint16_t length = ((uint16_t) header[ICC_HEADER_LENGTH_MSB_POS] << 8) + header[ICC_HEADER_LENGTH_LSB_POS];
    Icc_tdTargetInfo* targetInfo = Icc_lookupTargetInfo(target); /* look up the target info structure, get the pointer */
    uart_debug_sendstring("ICC Header Received: ");
    uart_debug_hexdump(header, ICC_HEADER_LENGTH);
    if(targetInfo == NULL) {
        //no known target
        //@TODO handle error
        Icc_lastTransferInfo.length = 0;
        Icc_lastTransferInfo.target = target;
        Icc_lastTransferInfo.targetValid = false;
#ifdef DEBUG_ICC
        uart_debug_sendstring("ICC Header Error: Invalid Target (");
        uart_debug_printuint8(target);
        uart_debug_sendstring(")\n");
#endif /* DEBUG_ICC */
    } else {
        Icc_currentTarget = targetInfo; //setting this is important
        if(ICC_HEADER_TARGET_IN == (target & ICC_TARGET_DIR_MASK)) {
            //in transaction, so for us, a reception
            if((targetInfo->inBufferState != ICC_BUFFER_STATE_READY)) {
#ifdef DEBUG_ICC
                uart_debug_sendstring("ICC Header Error. In Transaction to non-ready buffer (");
                uart_debug_printuint8(targetInfo->number);
                uart_debug_sendstring(" ).\n");
#endif /* DEBUG_ICC */
                Icc_currentTransferLen = 0;
                Icc_dmaState = ICC_DMA_ERROR;
            } else if ((targetInfo->inBufferLenMax == 0) ||
                       (targetInfo->rxCallback == NULL)) {
#ifdef DEBUG_ICC
                uart_debug_sendstring("ICC Header Error. In Transaction to invalid buffer (");
                uart_debug_printuint8(targetInfo->number);
                uart_debug_sendstring(" ).\n");
#endif /* DEBUG_ICC */
                Icc_currentTransferLen = 0;
                Icc_dmaState = ICC_DMA_ERROR;
            } else {
                if(targetInfo->inBufferLenMax < length) { //check for potential overflow
                    //@TODO make note of this so we can better handle the error later
#ifdef DEBUG_ICC
                    uart_debug_sendstring("ICC Header Error. In Transaction larger than available buffer space ( ");
                    uart_debug_printuint32(length);
                    uart_debug_sendstring(" / ");
                    uart_debug_printuint32(targetInfo->inBufferLenMax);
                    uart_debug_sendstring(" ). Truncating.\n");
#endif /* DEBUG_ICC */
                    length = targetInfo->inBufferLenMax; //cap overflow
                }
                Icc_currentTransferLen = length;
                Icc_dmaState = ICC_DMA_RECEIVE;
                Icc_waitForTransferCompleteCallback = true; //let the deselected callback handler know it needs to wait
                HAL_SPI_Receive_DMA(Icc_spiHandle, targetInfo->inBuffer, length); //set up receive
                //wait for rxComplete callback
            }
        } else {
            //out transaction, transmit
            if((&Icc_targetIcc == targetInfo) && (command != ICC_COMMAND_EXTENDED)) {
                if(targetInfo->outBufferState != ICC_BUFFER_STATE_IDLE) {
#ifdef DEBUG_ICC
                    uart_debug_sendline("ICC Internal Error. ICC information requested while response waiting.\n");
#endif /* DEBUG_ICC */
                    Icc_currentTransferLen = 0;
                    Icc_dmaState = ICC_DMA_ERROR;
                } else {
                    Icc_iccCommandHandler(command, length);
                }

            }
            if(targetInfo->outBufferState != ICC_BUFFER_STATE_READY) {
#ifdef DEBUG_ICC
                uart_debug_sendstring("ICC Header Error. Out Transaction from non-ready buffer (");
                uart_debug_printuint8(targetInfo->number);
                uart_debug_sendstring(" ).\n");
#endif /* DEBUG_ICC */
                Icc_currentTransferLen = 0;
                Icc_dmaState = ICC_DMA_ERROR;
            } else { //less checks here. no txCallback is not an error (might not need or want to be informed) and length of 0 is odd but gets caught anyway below

                if(targetInfo->outBufferLen < length) { //check for potential overflow
                    //@TODO make note of this so we can better handle the error later
#ifdef DEBUG_ICC
                    uart_debug_sendstring("ICC Header Error. Out Transaction larger than available data ( ");
                    uart_debug_printuint32(length);
                    uart_debug_sendstring(" / ");
                    uart_debug_printuint32(targetInfo->outBufferLen);
                    uart_debug_sendstring(" ).\n");
                    uart_debug_sendline("Garbage will be present.");
#endif /* DEBUG_ICC */
                    length = targetInfo->outBufferLen; //cap overflow
                }
                Icc_currentTransferLen = length;
                Icc_dmaState = ICC_DMA_TRANSMIT;
                Icc_waitForTransferCompleteCallback = true; //let the deselected callback handler know it needs to wait
                HAL_SPI_Transmit_DMA(Icc_spiHandle, targetInfo->outBuffer, length); //set up transmit
            }
        }
    }
}

/* handles the short ICC commands we can receive, which is most of them
 * returns true if data was added to the ICC out buffer
 * false if not (generally because the command was unrecognized) */
bool Icc_iccCommandHandler(uint8_t command, uint16_t len) {
    bool ret = false;
    switch(command) {
        case ICC_COMMAND_LIST_TARGETS:
            Icc_prepareTargetList(len);
            break;
        case ICC_COMMAND_GET_MAXIMUMS:
            Icc_prepareTargetMaximumsList(len);
            break;
        case ICC_COMMAND_LAST_XFER_STATUS:
            Icc_prepareLastTransferInfo(command);
            break;
        case ICC_COMMAND_SET_SHUTDOWN:
            Icc_setPowerMode(ICC_POWER_MODE_SHUTDOWN);
            break;
        case ICC_COMMAND_SET_LOW_POWER:
            Icc_setPowerMode(ICC_POWER_MODE_LOW);
            break;
        case ICC_COMMAND_SET_FULL_POWER:
            Icc_setPowerMode(ICC_POWER_MODE_FULL);
            break;
        case ICC_COMMAND_GET_POWER_MODE:
            Icc_preparePowerMode();
            break;
        case ICC_COMMAND_USB_DISABLE:
            Icc_setUSBEnabled(false);
            break;
        case ICC_COMMAND_USB_ENABLE:
            Icc_setUSBEnabled(true);
            break;
        default:
#ifdef DEBUG_ICC
            uart_debug_sendstring("ICC Short Command Handler called with unknown command (");
            uart_debug_printuint8(command);
            uart_debug_sendline(").\n");
#endif /* DEBUG_ICC */
            ret = false;
            break;

    }
    return ret;
}

bool Icc_iccExtendedCommandHandler(uint8_t* data, uint16_t len) {
    if((len == 0) || (data == NULL)) {
        uart_debug_sendline("ICC Extended Command Handler called with zero length or null data.\n");
        return false;
    }
    uart_debug_sendstring("ICC Extended Command Handler called with unknown command (");
    uart_debug_printuint8(data[0]);
    uart_debug_sendline(").\n");
    return false;
}

/* should be called when /CS goes high
 * handles cleanup.
 * Note this CAN be different from the DMA RX and TX callbacks, if the length fields are mismatched from the actual amount of data transferred
 */
void Icc_deselectedCallbackHandler(void) {
    if(NULL != Icc_readyPort) {
        Icc_readyPort->BSRR = Icc_readyPin; //set this high again as we're not ready
    }
    /* list of issues that might happen:
     * not enough bytes transferred
     * too many bytes transferred (though this will get picked up earlier as the DMA completes)
     * bad target causing a need to clean up the SPI (main issue)
     */
    HAL_SPI_StateTypeDef state = HAL_SPI_GetState(Icc_spiHandle);
    if(Icc_spiHandle == NULL || state == HAL_SPI_STATE_RESET) {
        //spurious callback? handle is null or hasn't been initialized
        return;
    }
    if(state != HAL_SPI_STATE_READY) {
        //deselected but the DMA transfer is still in progress
        HAL_SPI_DMAStop(Icc_spiHandle); //call this. this will call the rx or tx callback as appropriate which will then call our callback.. callback
    }
    if(Icc_dmaState == ICC_DMA_RECEIVE || Icc_dmaState == ICC_DMA_TRANSMIT) {
        while(Icc_waitForTransferCompleteCallback); //spin  until the callbacks have run so we don't trip over ourselves
    }
    Icc_currentTarget = NULL;
    Icc_currentTransferLen = 0;
    Icc_setupHeaderReceive();
}

void Icc_setPowerMode(uint8_t mode) {
    switch(mode) {
        case ICC_POWER_MODE_FULL:
        case ICC_POWER_MODE_LOW:
        case ICC_POWER_MODE_SHUTDOWN:
            uart_debug_sendstring("ICC Power Mode Changed: ");
            uart_debug_printuint8(mode);
            uart_debug_newline();
            break;
        default:
            uart_debug_sendstring("ICC Power Mode Changed with invalid mode: ");
            uart_debug_printuint8(mode);
            uart_debug_newline();
            break;
    }
}

void Icc_preparePowerMode(void) {
    //@TODO actually do power modes
    static uint8_t powerMode = ICC_POWER_MODE_FULL;
    Icc_targetIcc.outBuffer = &powerMode;
    Icc_targetIcc.outBufferLen = 1;
    Icc_targetIcc.outBufferState = ICC_BUFFER_STATE_READY;
}

void Icc_prepareLastTransferInfo(uint8_t targetNum) {
    static uint8_t lastTransferInfoBuff[ICC_LAST_TRANSFER_INFO_LEN];
    Icc_tdTargetInfo* target = Icc_lookupTargetInfo(targetNum);
    if(NULL != target) {
        lastTransferInfoBuff[ICC_LAST_TRANSFER_INFO_TARGET_POS] = target->lastTransferDirection | target->number;
        lastTransferInfoBuff[ICC_LAST_TRANSFER_INFO_VALID_POS] = true;
        lastTransferInfoBuff[ICC_LAST_TRANSFER_INFO_LENGTH_LSB_POS] = target->lastTransferLen & 0x00FF;
        lastTransferInfoBuff[ICC_LAST_TRANSFER_INFO_LENGTH_MSB_POS] = (target->lastTransferLen & 0xFF00) >> 8;
    } else {
        lastTransferInfoBuff[ICC_LAST_TRANSFER_INFO_TARGET_POS] = targetNum;
        lastTransferInfoBuff[ICC_LAST_TRANSFER_INFO_VALID_POS] = false;
        lastTransferInfoBuff[ICC_LAST_TRANSFER_INFO_LENGTH_LSB_POS] = 0;
        lastTransferInfoBuff[ICC_LAST_TRANSFER_INFO_LENGTH_MSB_POS] = 0;
    }

    Icc_targetIcc.outBuffer = lastTransferInfoBuff;
    Icc_targetIcc.outBufferLen = ICC_LAST_TRANSFER_INFO_LEN;
    Icc_targetIcc.outBufferState = ICC_BUFFER_STATE_READY;
}

/*
 * length controls how many bytes are made available in the output
 */
void Icc_prepareTargetList(uint16_t len) {
    uint8_t* targetInfoBuffer = Icc_targetInfoBuffer;
    if(len > ICC_TARGET_INFO_BUFFER_LENGTH) {
        len = ICC_TARGET_INFO_BUFFER_LENGTH; //limit length to prevent overruns
    }
    targetInfoBuffer[0] = ICC_NUM_TARGETS;
    if(len > 1) { //short-circuited for len == 1 since that's how the secure micro can discover how many targets we have
        for(uint32_t i = 0; i < ICC_NUM_TARGETS; ++i) {
            uint32_t baseOffset = (i * ICC_TARGET_LIST_ITEM_LENGTH) + 1;
            targetInfoBuffer[baseOffset + ICC_TARGET_LIST_TARGET_POS] = Icc_targetInfoList[i]->number;
            if(Icc_targetInfoList[i]->inBufferState == ICC_BUFFER_STATE_READY) {
                targetInfoBuffer[baseOffset + ICC_TARGET_LIST_IN_LSB_POS] = Icc_targetInfoList[i]->inBufferLenMax & 0xFF;
                targetInfoBuffer[baseOffset + ICC_TARGET_LIST_IN_MSB_POS] = (Icc_targetInfoList[i]->inBufferLenMax & 0xFF00) >> 8;
            }  else {
                //not ready, return 0 for potential length
                targetInfoBuffer[baseOffset + ICC_TARGET_LIST_IN_LSB_POS] = 0;
                targetInfoBuffer[baseOffset + ICC_TARGET_LIST_IN_MSB_POS] = 0;
            }
            if(Icc_targetInfoList[i]->outBufferState == ICC_BUFFER_STATE_READY) {
                targetInfoBuffer[baseOffset + ICC_TARGET_LIST_OUT_LSB_POS] = Icc_targetInfoList[i]->outBufferLen & 0xFF;
                targetInfoBuffer[baseOffset + ICC_TARGET_LIST_OUT_MSB_POS] = (Icc_targetInfoList[i]->outBufferLen & 0xFF00) >> 8;
            }  else {
                //not ready, return 0 for potential length
                targetInfoBuffer[baseOffset + ICC_TARGET_LIST_OUT_LSB_POS] = 0;
                targetInfoBuffer[baseOffset + ICC_TARGET_LIST_OUT_MSB_POS] = 0;
            }
        }
    }
    Icc_targetIcc.outBuffer = targetInfoBuffer;
    Icc_targetIcc.outBufferLen = ICC_NUM_TARGETS * ICC_TARGET_LIST_ITEM_LENGTH;
    Icc_targetIcc.outBufferState = ICC_BUFFER_STATE_READY;
}


/*
 * length controls how many bytes are made available in the output
 */
void Icc_prepareTargetMaximumsList(uint16_t len) {
    uint8_t* targetInfoBuffer = Icc_targetInfoBuffer;
    if(len > ICC_TARGET_INFO_BUFFER_LENGTH) {
        len = ICC_TARGET_INFO_BUFFER_LENGTH; //limit length to prevent overruns
    }
    targetInfoBuffer[0] = ICC_NUM_TARGETS;
    for(uint32_t i = 0; i < ICC_NUM_TARGETS; ++i) {
        uint32_t baseOffset = (i * ICC_TARGET_LIST_ITEM_LENGTH) + 1;
        targetInfoBuffer[baseOffset + ICC_TARGET_LIST_TARGET_POS] = Icc_targetInfoList[i]->number;

        targetInfoBuffer[baseOffset + ICC_TARGET_LIST_IN_LSB_POS] = Icc_targetInfoList[i]->inBufferLenMax & 0xFF;
        targetInfoBuffer[baseOffset + ICC_TARGET_LIST_IN_MSB_POS] = (Icc_targetInfoList[i]->inBufferLenMax & 0xFF00) >> 8;

        targetInfoBuffer[baseOffset + ICC_TARGET_LIST_OUT_LSB_POS] = Icc_targetInfoList[i]->outBufferLenMax & 0xFF;
        targetInfoBuffer[baseOffset + ICC_TARGET_LIST_OUT_MSB_POS] = (Icc_targetInfoList[i]->outBufferLenMax & 0xFF00) >> 8;

    }

    Icc_targetIcc.outBuffer = targetInfoBuffer;
    Icc_targetIcc.outBufferLen = ICC_NUM_TARGETS * ICC_TARGET_LIST_ITEM_LENGTH;
    Icc_targetIcc.outBufferState = ICC_BUFFER_STATE_READY;
}

void Icc_setUSBEnabled(bool usbEnabled) {
    if(usbEnabled) {
#ifdef DEBUG_ICC
    uart_debug_sendline("USB Enabled via ICC.\n");
#endif /* DEBUG_ICC */
        //@TODO  provide a callback or callback struct so this doesn't need a pointer to the USB device to start the USB
        // should also release the USB enable line
    } else {
        //must be disabling it
#ifdef DEBUG_ICC
        uart_debug_sendline("USB Disabled via ICC.\n");
#endif /* DEBUG_ICC */
    }

}

void Icc_iccTransmitCompleteCallback(void) {
    Icc_targetIcc.outBufferState = ICC_BUFFER_STATE_IDLE;
    Icc_targetIcc.outBufferLen = 0;
    Icc_targetIcc.outBuffer = NULL;

}

/* 2018-02-01 old echo stuff commented out.
bool ICC_EchoHandler(uint8_t* buff, uint16_t len) {
    if(ICC_Target_Echo.outBufferState == ICC_BUFFER_STATE_READY) {
        // some silly person hasn't retrieved their last echo data
        uart_debug_sendline("Echo Data received while echo data still in buffer. Overwritten.");
    } else if (ICC_Target_Echo.outBufferState == ICC_BUFFER_STATE_DMA) {//should never be in DMA state
        uart_debug_sendline("Echo Data received while echo data being sent out. How?");
        return false; //do nothing in this case
    }
    uart_debug_sendstring("Echoing ");
    uart_debug_printuint32(len);
    uart_debug_sendline(" bytes of data.\n");
    ICC_Target_Echo.outBuffer = buff;
    ICC_Target_Echo.outBufferLen = len;
    ICC_Target_Echo.outBufferState = ICC_BUFFER_STATE_READY; //flag it as ready-to-send

    return true; //true to keep buffer around, as its a shared buffer
}


void ICC_EchoTransmitCompleteCallback(void) {
    uart_debug_sendline("Echo Complete.\n");
    ICC_Target_Echo.outBuffer = NULL;
    ICC_Target_Echo.outBufferLen = 0;
    ICC_Target_Echo.outBufferState = ICC_BUFFER_STATE_IDLE;
    ICC_ReleaseInBuffer(ICC_TARGET_ECHO); //release the in buffer
}
*/
