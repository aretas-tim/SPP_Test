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


ICC_TargetInfo* ICC_CurrentTarget = NULL; /* pointer to the buffer that the DMA is currently using, direction depends on context */
uint16_t ICC_CurrentTransferLen = 0; /* holds how many bytes we're expecting to transfer */
ICC_DMAStateEnum ICC_DMAState = ICC_DMA_ERROR;

ICC_TransferInfo ICC_LastTransferInfo = {.targetValid = false, .target = 0, .length = 0};

volatile bool ICC_WaitForTransferCompleteCallback = false; //used to pause the deselected callback until the rx/tx complete callback has finished

uint8_t ICC_HeaderInBuffer[ICC_HEADER_LENGTH]; /* header is always "in", though it proceeds no further than us */
uint8_t ICC_CommandInBuffer[ICC_COMMAND_BUFFER_LENGTH];

bool ICC_ICCCommandHandler(uint8_t command, uint16_t len); /* short commands */
bool ICC_ICCExtendedCommandHandler(uint8_t* data, uint16_t len);
void ICC_HeaderHandler(uint8_t header[ICC_HEADER_LENGTH]);
void ICC_ICCTransmitCompleteCallback(void);

void ICC_AssertDataWaiting(void);
bool ICC_CheckForDataWaiting(void);
//there is no clear, check for data waiting will clear as-needed

ICC_TargetInfo* ICC_LookupTargetInfo(uint8_t target);

void ICC_PreparePowerMode(void);
void ICC_PrepareLastTransferInfo(uint8_t target);
void ICC_PrepareTargetList(uint16_t len);
void ICC_PrepareTargetMaximumsList(uint16_t len);

void ICC_SetPowerMode(uint8_t mode);

void ICC_SetUSBEnabled(bool isUSBEnabled);




uint8_t ICC_TargetInfoBuffer[ICC_TARGET_INFO_BUFFER_LENGTH]; //because its shared between two functions now, saves memory


//bool ICC_EchoHandler(uint8_t* buff, uint16_t len);
//void ICC_EchoTransmitCompleteCallback(void);

ICC_TargetInfo ICC_Target_ICC = {
        .number = ICC_TARGET_ICC,
        .lastTransferDirection = 0,
        .lastTransferLen = 0,
        .inBuffer = ICC_CommandInBuffer,
        .inBufferLen = 0,
        .inBufferLenMax = ICC_COMMAND_BUFFER_LENGTH,
        .inBufferState = ICC_BUFFER_STATE_IDLE,
        .outBuffer = NULL,
        .outBufferLen = 0,
        .outBufferState = ICC_BUFFER_STATE_IDLE,
        .rxCallback = ICC_ICCExtendedCommandHandler, /* short commands are handled by a separate code path, not this callback */
        .txCompleteCallback = ICC_ICCTransmitCompleteCallback
};

ICC_TargetInfo ICC_Target_LEDs = {
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

ICC_TargetInfo ICC_Target_Tunnel = {
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

ICC_TargetInfo ICC_Target_U2F = {
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

ICC_TargetInfo ICC_Target_Hotkey = {
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

ICC_TargetInfo* ICC_TargetInfoList[ICC_NUM_TARGETS] = {
        &ICC_Target_ICC,
        &ICC_Target_LEDs,
        &ICC_Target_Tunnel,
        &ICC_Target_U2F,
        &ICC_Target_Hotkey,
};


SPI_HandleTypeDef* ICC_SpiHandle = NULL;

bool ICC_IsInitialized = false;

/* these are multiplexed with the wakeup pin, as it has no function outside of deep sleep */
GPIO_TypeDef* ICC_ReadyPort = NULL;
uint16_t ICC_ReadyPin = 0; //no pin

/* outgoing IRQ to secure micro, to let it know we have data waiting */
GPIO_TypeDef* ICC_IRQPort = NULL;
uint16_t ICC_IRQPin = 0; //no pin


HAL_StatusTypeDef ICC_SetupHeaderReceive(void);

void ICC_Init(SPI_HandleTypeDef* hspi, GPIO_TypeDef* readyPort, uint16_t readyPin, GPIO_TypeDef* IRQPort, uint16_t IRQPin) {
    //do init things where once we know what needs to be initialized
    if(hspi != NULL) {
        ICC_SpiHandle = hspi;
    } else {
        ICC_SpiHandle = NULL;
        ICC_ReadyPort = NULL;
        ICC_ReadyPin = 0;
        ICC_IsInitialized = false;
        uart_debug_sendline("ICC unable to start. SPI Handle was NULL.\n");
        return;
    }
    if(NULL != readyPort && 0 != readyPin) { //0 on a pin means no pin
        ICC_ReadyPort = readyPort;
        ICC_ReadyPin = readyPin;
        ICC_IRQPort = IRQPort;
        ICC_IRQPin = IRQPin;
    } else {
        ICC_SpiHandle = NULL;
        ICC_ReadyPort = NULL;
        ICC_ReadyPin = 0;
        ICC_IsInitialized = false;
        uart_debug_sendline("ICC unable to start. Transfer ready port/pin was invalid.\n");
        return;
    }
    //set up local targets
    bool targetSetupOkay = ICC_SetupTarget(ICC_TARGET_ICC, ICC_CommandInBuffer, ICC_COMMAND_BUFFER_LENGTH, ICC_ICCExtendedCommandHandler, ICC_COMMAND_BUFFER_LENGTH, ICC_ICCTransmitCompleteCallback);
    if(!targetSetupOkay) {
        uart_debug_sendline("ICC unable to start, ICC target setup failed.");
        ICC_IsInitialized = false;
        return;
    }

    ICC_IsInitialized = true;
}

/* ICC start
 * starts up the ICC by setting up to receive the first packet and pulling the interrupt line low to
 * tells the secure micro we have data waiting (we don't, but it will get it to pull the target info from us)
 *
 * call this after Init and after all targets have been set up
 */
void ICC_Start(void) {
    if(!ICC_IsInitialized) {
        return;
    }
    //setup DMA to receive first packet
    HAL_StatusTypeDef rc =  ICC_SetupHeaderReceive();
    if(rc != HAL_OK) {
        uart_debug_sendstring("Unable to enable DMA receive during ICC Startup (");
        uart_debug_printuint8(rc);
        uart_debug_sendline(")\n");
        uart_debug_sendline("ICC unable to start.\n");
        return;
    } else {
        uart_debug_sendline("ICC Started.\n");
    }
    if(ICC_IRQPort != NULL) { //if this is non-null, which it can be to operate in polling-only
        ICC_IRQPort->BRR = ICC_IRQPin; //bring this low to let the secure micro know we're ready and it can query us
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
bool ICC_SetupTarget(uint8_t target, uint8_t* rxBuffer, uint16_t rxBufferLen, bool (*rxCallback)(uint8_t*, uint16_t), uint16_t maxTxLen, void (*txCompleteCallback)(void)) {
    ICC_TargetInfo* targetInfo = ICC_LookupTargetInfo(target);
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
HAL_StatusTypeDef ICC_SetupHeaderReceive() {
    //if(ICC_DMAState != ICC_DMA_HEADER) { //prevent it from running when already setup
        HAL_SPIEx_FlushRxFifo(ICC_SpiHandle);
        ICC_DMAState = ICC_DMA_HEADER;
        uart_debug_sendline("ICC Waiting for Header.\n");
        HAL_StatusTypeDef rc = HAL_SPI_Receive_DMA(ICC_SpiHandle, ICC_HeaderInBuffer, ICC_HEADER_LENGTH);
        ICC_CheckForDataWaiting(); //only ever run this now, to prevent the secure micro from being too aggressive.
    //}
    return rc;
}


void ICC_DMARxCompleteCallback(void) {
    //grab a copy of the data transfer register from the RX DMA channel,
    //this *should* be 0 (all data transferred)
    //if its not, we're missing bytes, probably caused by /SS going high before all the data was transferred
    uint32_t transferRemainingLen = ICC_SpiHandle->hdmarx->Instance->CNDTR;
    uart_debug_sendline("ICC DMA Rx Complete Callback\n");
    switch(ICC_DMAState) {
        case ICC_DMA_HEADER:
            if(transferRemainingLen) {
                ICC_SetupHeaderReceive(); //try to get another header, something went bad with this one
#ifdef DEBUG_ICC
                uart_debug_sendline("ICC Received short header.\n");
#endif /* DEBUG_ICC */
            } else {
                ICC_DMAState = ICC_DMA_IDLE;
                ICC_HeaderHandler(ICC_HeaderInBuffer); //that was easy
                ICC_ReadyPort->BRR = ICC_ReadyPin; //take this low to say we're ready.
            }
            break;
        case ICC_DMA_RECEIVE:
            ICC_ReadyPort->BSRR = ICC_ReadyPin; //take this back high, no longer ready
            ICC_DMAState = ICC_DMA_IDLE;
            if(ICC_CurrentTarget != NULL) {
                //call the rx callback, if it exists
                if(ICC_CurrentTarget->rxCallback != NULL && ICC_CurrentTarget->inBufferLenMax > 0) {
                    ICC_CurrentTarget->inBufferLen = ICC_CurrentTransferLen - transferRemainingLen;
                    ICC_CurrentTarget->inBufferState = ICC_BUFFER_STATE_PROCESSING; //busy being processed elsewhere
                    uart_debug_sendstring("ICC received ");
                    uart_debug_printuint32(ICC_CurrentTarget->inBufferLen);
                    uart_debug_sendline(" bytes. Processing.\n");
                    bool rc = ICC_CurrentTarget->rxCallback(ICC_CurrentTarget->inBuffer, ICC_CurrentTarget->inBufferLen);
                    if(!rc) { //buffer released by callback
                        ICC_CurrentTarget->inBufferState = ICC_BUFFER_STATE_READY;
                    } // else buffer kept by callback for later processing or further transmission back to the host
                    ICC_LastTransferInfo.targetValid = true;
                } else {
                    uart_debug_sendline("ICC received data to valid target, but there was no rx callback.\n");
                    ICC_LastTransferInfo.targetValid = false; //transferred to an invalid target, i.e. nothing was done with the data
                }
                ICC_LastTransferInfo.target = ICC_CurrentTarget->number | ICC_HEADER_TARGET_IN;

            } else {
                uart_debug_sendline("ICC received data to invalid target.\n");
                ICC_LastTransferInfo.target = ICC_TARGET_NULL | ICC_HEADER_TARGET_IN;
                ICC_LastTransferInfo.targetValid = false;
            }
            ICC_LastTransferInfo.length = ICC_CurrentTransferLen - transferRemainingLen;

            ICC_SetupHeaderReceive(); //set up to receive the next header
            break;
        default:
            uart_debug_sendstring("ICC DMA RX Complete Callback called while in invalid ICC_DMAState: ");
            uart_debug_printuint32(ICC_DMAState);
            uart_debug_newline();
            break;
    }
    ICC_WaitForTransferCompleteCallback = false; //finished, anything that was waiting no longer needs to wait
}

/* we've finished sending some data to the secure micro */
void ICC_DMATxCompleteCallback(void) {
    if(ICC_DMAState != ICC_DMA_TRANSMIT) {
        uart_debug_sendstring("ICC DMA TX Complete Callback called while in invalid ICC_DMAState:");
        uart_debug_printuint32(ICC_DMAState);
        uart_debug_newline();
        return;
    }
    ICC_ReadyPort->BSRR = ICC_ReadyPin; //take this back high, no longer ready
    uint32_t remainingTransferLen = ICC_SpiHandle->hdmatx->Instance->CNDTR; //get any remaining transfer length
    uart_debug_sendline("ICC DMA Tx Complete Callback\n");
    uint32_t actualTransferLen;
    if(remainingTransferLen >= ICC_CurrentTransferLen) {
        //there were no bytes transferred before /SS went high, or someone was mucking with ICC_CurrentTransferLen
        actualTransferLen = 0;
    } else {
        actualTransferLen = ICC_CurrentTransferLen - remainingTransferLen;
    }

    if(ICC_CurrentTarget != NULL) {

        //check for underflow
        if(ICC_CurrentTarget->outBufferLen < actualTransferLen) {
            //how did we get here anyway?
            ICC_CurrentTarget->outBufferLen = 0; //no more data available
#ifdef DEBUG_ICC
            uart_debug_sendline("ICC Transmit Underflowed.\n");
#endif /* DEBUG_ICC */

        } else {
            ICC_CurrentTarget->outBufferLen -= actualTransferLen;
        }
        if(0 == ICC_CurrentTarget->outBufferLen) {
            ICC_CurrentTarget->outBufferState = ICC_BUFFER_STATE_IDLE;
            if(ICC_CurrentTarget->txCompleteCallback != NULL) {
                ICC_CurrentTarget->txCompleteCallback(); //run the callback if its not NULL
            }
        } else {
            ICC_CurrentTarget->outBuffer += ICC_CurrentTransferLen; //move pointer forward
        }
        ICC_DMAState = ICC_DMA_IDLE; //no matter what we're back to idling
        ICC_LastTransferInfo.length = actualTransferLen;
        ICC_LastTransferInfo.targetValid = true;
        ICC_LastTransferInfo.target = ICC_CurrentTarget->number;
        ICC_CurrentTarget = NULL; //null out our current target as we're finished for now, secure micro will need to request more data
    } else {
        ICC_LastTransferInfo.length = actualTransferLen; //subtract off any remaining length in the DMA
        ICC_LastTransferInfo.targetValid = false;
        ICC_LastTransferInfo.target = ICC_TARGET_NULL;
    }
    ICC_WaitForTransferCompleteCallback = false;
    ICC_SetupHeaderReceive(); //prep to receive the next header
}

/*if the in buffer of a target was kept around by the rxCallback
 * this will release it
 */
void ICC_ReleaseInBuffer(uint8_t target) {
    ICC_TargetInfo* targetInfo = ICC_LookupTargetInfo(target); /* look up the target info structure, get the pointer */
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

ICC_TargetInfo* ICC_LookupTargetInfo(uint8_t target) {
    for(uint32_t i = 0; i < ICC_NUM_TARGETS; ++i) {
        if(ICC_TargetInfoList[i]->number == (target & ICC_TARGET_NUMBER_MASK)) { //mask off direction bit
            return ICC_TargetInfoList[i];
        }
    }
    return NULL;
}

/**
 * checks the known targets for any out data waiting and updates the IRQ line
 */
bool ICC_CheckForDataWaiting(void) {
    bool isDataWaiting = false;
    for(uint32_t i = 0; i < ICC_NUM_TARGETS; ++i) {
        if(ICC_TargetInfoList[i]->outBufferState == ICC_BUFFER_STATE_READY) {
            isDataWaiting = true;
            uart_debug_sendline("ICC Has Out Data Waiting.\n");
            break;
        }
    }
    if(ICC_IRQPort != NULL) { //this can be null, in theory.
        if(isDataWaiting) {
            ICC_IRQPort->BRR = ICC_IRQPin;
        } else {
            ICC_IRQPort->BSRR = ICC_IRQPin;
        }
    }
    return isDataWaiting;
}

void ICC_AssertDataWaiting(void) {
    if(ICC_IRQPort != NULL) { //this can be null, in theory.
        ICC_IRQPort->BRR = ICC_IRQPin;
    }
}

/**
 * handles the header upon reception. Sets up the DMA to transmit/receive the data portion
 *
 * param header a 4-length uint8_t array
 */
void ICC_HeaderHandler(uint8_t header[ICC_HEADER_LENGTH]) {
    uint8_t target = header[ICC_HEADER_TARGET_POS];
    uint8_t command = header[ICC_HEADER_COMMAND_POS];
    uint16_t length = ((uint16_t) header[ICC_HEADER_LENGTH_MSB_POS] << 8) + header[ICC_HEADER_LENGTH_LSB_POS];
    ICC_TargetInfo* targetInfo = ICC_LookupTargetInfo(target); /* look up the target info structure, get the pointer */
    uart_debug_sendstring("ICC Header Received: ");
    uart_debug_hexdump(header, ICC_HEADER_LENGTH);
    if(targetInfo == NULL) {
        //no known target
        //@TODO handle error
        ICC_LastTransferInfo.length = 0;
        ICC_LastTransferInfo.target = target;
        ICC_LastTransferInfo.targetValid = false;
#ifdef DEBUG_ICC
        uart_debug_sendstring("ICC Header Error: Invalid Target (");
        uart_debug_printuint8(target);
        uart_debug_sendstring(")\n");
#endif /* DEBUG_ICC */
    } else {
        ICC_CurrentTarget = targetInfo; //setting this is important
        if(ICC_HEADER_TARGET_IN == (target & ICC_TARGET_DIR_MASK)) {
            //in transaction, so for us, a reception
            if((targetInfo->inBufferState != ICC_BUFFER_STATE_READY)) {
#ifdef DEBUG_ICC
                uart_debug_sendstring("ICC Header Error. In Transaction to non-ready buffer (");
                uart_debug_printuint8(targetInfo->number);
                uart_debug_sendstring(" ).\n");
#endif /* DEBUG_ICC */
                ICC_CurrentTransferLen = 0;
                ICC_DMAState = ICC_DMA_ERROR;
            } else if ((targetInfo->inBufferLenMax == 0) ||
                       (targetInfo->rxCallback == NULL)) {
#ifdef DEBUG_ICC
                uart_debug_sendstring("ICC Header Error. In Transaction to invalid buffer (");
                uart_debug_printuint8(targetInfo->number);
                uart_debug_sendstring(" ).\n");
#endif /* DEBUG_ICC */
                ICC_CurrentTransferLen = 0;
                ICC_DMAState = ICC_DMA_ERROR;
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
                ICC_CurrentTransferLen = length;
                ICC_DMAState = ICC_DMA_RECEIVE;
                ICC_WaitForTransferCompleteCallback = true; //let the deselected callback handler know it needs to wait
                HAL_SPI_Receive_DMA(ICC_SpiHandle, targetInfo->inBuffer, length); //set up receive
                //wait for rxComplete callback
            }
        } else {
            //out transaction, transmit
            if((&ICC_Target_ICC == targetInfo) && (command != ICC_COMMAND_EXTENDED)) {
                if(targetInfo->outBufferState != ICC_BUFFER_STATE_IDLE) {
#ifdef DEBUG_ICC
                    uart_debug_sendline("ICC Internal Error. ICC information requested while response waiting.\n");
#endif /* DEBUG_ICC */
                    ICC_CurrentTransferLen = 0;
                    ICC_DMAState = ICC_DMA_ERROR;
                } else {
                    ICC_ICCCommandHandler(command, length);
                }

            }
            if(targetInfo->outBufferState != ICC_BUFFER_STATE_READY) {
#ifdef DEBUG_ICC
                uart_debug_sendstring("ICC Header Error. Out Transaction from non-ready buffer (");
                uart_debug_printuint8(targetInfo->number);
                uart_debug_sendstring(" ).\n");
#endif /* DEBUG_ICC */
                ICC_CurrentTransferLen = 0;
                ICC_DMAState = ICC_DMA_ERROR;
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
                ICC_CurrentTransferLen = length;
                ICC_DMAState = ICC_DMA_TRANSMIT;
                ICC_WaitForTransferCompleteCallback = true; //let the deselected callback handler know it needs to wait
                HAL_SPI_Transmit_DMA(ICC_SpiHandle, targetInfo->outBuffer, length); //set up transmit
            }
        }
    }
}

/* handles the short ICC commands we can receive, which is most of them
 * returns true if data was added to the ICC out buffer
 * false if not (generally because the command was unrecognized) */
bool ICC_ICCCommandHandler(uint8_t command, uint16_t len) {
    bool ret = false;
    switch(command) {
        case ICC_COMMAND_LIST_TARGETS:
            ICC_PrepareTargetList(len);
            break;
        case ICC_COMMAND_GET_MAXIMUMS:
            ICC_PrepareTargetMaximumsList(len);
            break;
        case ICC_COMMAND_LAST_XFER_STATUS:
            ICC_PrepareLastTransferInfo(command);
            break;
        case ICC_COMMAND_SET_SHUTDOWN:
            ICC_SetPowerMode(ICC_POWER_MODE_SHUTDOWN);
            break;
        case ICC_COMMAND_SET_LOW_POWER:
            ICC_SetPowerMode(ICC_POWER_MODE_LOW);
            break;
        case ICC_COMMAND_SET_FULL_POWER:
            ICC_SetPowerMode(ICC_POWER_MODE_FULL);
            break;
        case ICC_COMMAND_GET_POWER_MODE:
            ICC_PreparePowerMode();
            break;
        case ICC_COMMAND_USB_DISABLE:
            ICC_SetUSBEnabled(false);
            break;
        case ICC_COMMAND_USB_ENABLE:
            ICC_SetUSBEnabled(true);
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

bool ICC_ICCExtendedCommandHandler(uint8_t* data, uint16_t len) {
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
void ICC_DeselectedCallbackHandler(void) {
    if(NULL != ICC_ReadyPort) {
        ICC_ReadyPort->BSRR = ICC_ReadyPin; //set this high again as we're not ready
    }
    /* list of issues that might happen:
     * not enough bytes transferred
     * too many bytes transferred (though this will get picked up earlier as the DMA completes)
     * bad target causing a need to clean up the SPI (main issue)
     */
    HAL_SPI_StateTypeDef state = HAL_SPI_GetState(ICC_SpiHandle);
    if(ICC_SpiHandle == NULL || state == HAL_SPI_STATE_RESET) {
        //spurious callback? handle is null or hasn't been initialized
        return;
    }
    if(state != HAL_SPI_STATE_READY) {
        //deselected but the DMA transfer is still in progress
        HAL_SPI_DMAStop(ICC_SpiHandle); //call this. this will call the rx or tx callback as appropriate which will then call our callback.. callback
    }
    if(ICC_DMAState == ICC_DMA_RECEIVE || ICC_DMAState == ICC_DMA_TRANSMIT) {
        while(ICC_WaitForTransferCompleteCallback); //spin  until the callbacks have run so we don't trip over ourselves
    }
    ICC_CurrentTarget = NULL;
    ICC_CurrentTransferLen = 0;
    ICC_SetupHeaderReceive();
}

void ICC_SetPowerMode(uint8_t mode) {
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

void ICC_PreparePowerMode(void) {
    //@TODO actually do power modes
    static uint8_t powerMode = ICC_POWER_MODE_FULL;
    ICC_Target_ICC.outBuffer = &powerMode;
    ICC_Target_ICC.outBufferLen = 1;
    ICC_Target_ICC.outBufferState = ICC_BUFFER_STATE_READY;
}

void ICC_PrepareLastTransferInfo(uint8_t targetNum) {
    static uint8_t lastTransferInfoBuff[ICC_LAST_TRANSFER_INFO_LEN];
    ICC_TargetInfo* target = ICC_LookupTargetInfo(targetNum);
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

    ICC_Target_ICC.outBuffer = lastTransferInfoBuff;
    ICC_Target_ICC.outBufferLen = ICC_LAST_TRANSFER_INFO_LEN;
    ICC_Target_ICC.outBufferState = ICC_BUFFER_STATE_READY;
}

/*
 * length controls how many bytes are made available in the output
 */
void ICC_PrepareTargetList(uint16_t len) {
    uint8_t* targetInfoBuffer = ICC_TargetInfoBuffer;
    if(len > ICC_TARGET_INFO_BUFFER_LENGTH) {
        len = ICC_TARGET_INFO_BUFFER_LENGTH; //limit length to prevent overruns
    }
    targetInfoBuffer[0] = ICC_NUM_TARGETS;
    if(len > 1) { //short-circuited for len == 1 since that's how the secure micro can discover how many targets we have
        for(uint32_t i = 0; i < ICC_NUM_TARGETS; ++i) {
            uint32_t baseOffset = (i * ICC_TARGET_LIST_ITEM_LENGTH) + 1;
            targetInfoBuffer[baseOffset + ICC_TARGET_LIST_TARGET_POS] = ICC_TargetInfoList[i]->number;
            if(ICC_TargetInfoList[i]->inBufferState == ICC_BUFFER_STATE_READY) {
                targetInfoBuffer[baseOffset + ICC_TARGET_LIST_IN_LSB_POS] = ICC_TargetInfoList[i]->inBufferLenMax & 0xFF;
                targetInfoBuffer[baseOffset + ICC_TARGET_LIST_IN_MSB_POS] = (ICC_TargetInfoList[i]->inBufferLenMax & 0xFF00) >> 8;
            }  else {
                //not ready, return 0 for potential length
                targetInfoBuffer[baseOffset + ICC_TARGET_LIST_IN_LSB_POS] = 0;
                targetInfoBuffer[baseOffset + ICC_TARGET_LIST_IN_MSB_POS] = 0;
            }
            if(ICC_TargetInfoList[i]->outBufferState == ICC_BUFFER_STATE_READY) {
                targetInfoBuffer[baseOffset + ICC_TARGET_LIST_OUT_LSB_POS] = ICC_TargetInfoList[i]->outBufferLen & 0xFF;
                targetInfoBuffer[baseOffset + ICC_TARGET_LIST_OUT_MSB_POS] = (ICC_TargetInfoList[i]->outBufferLen & 0xFF00) >> 8;
            }  else {
                //not ready, return 0 for potential length
                targetInfoBuffer[baseOffset + ICC_TARGET_LIST_OUT_LSB_POS] = 0;
                targetInfoBuffer[baseOffset + ICC_TARGET_LIST_OUT_MSB_POS] = 0;
            }
        }
    }
    ICC_Target_ICC.outBuffer = targetInfoBuffer;
    ICC_Target_ICC.outBufferLen = ICC_NUM_TARGETS * ICC_TARGET_LIST_ITEM_LENGTH;
    ICC_Target_ICC.outBufferState = ICC_BUFFER_STATE_READY;
}


/*
 * length controls how many bytes are made available in the output
 */
void ICC_PrepareTargetMaximumsList(uint16_t len) {
    uint8_t* targetInfoBuffer = ICC_TargetInfoBuffer;
    if(len > ICC_TARGET_INFO_BUFFER_LENGTH) {
        len = ICC_TARGET_INFO_BUFFER_LENGTH; //limit length to prevent overruns
    }
    targetInfoBuffer[0] = ICC_NUM_TARGETS;
    for(uint32_t i = 0; i < ICC_NUM_TARGETS; ++i) {
        uint32_t baseOffset = (i * ICC_TARGET_LIST_ITEM_LENGTH) + 1;
        targetInfoBuffer[baseOffset + ICC_TARGET_LIST_TARGET_POS] = ICC_TargetInfoList[i]->number;

        targetInfoBuffer[baseOffset + ICC_TARGET_LIST_IN_LSB_POS] = ICC_TargetInfoList[i]->inBufferLenMax & 0xFF;
        targetInfoBuffer[baseOffset + ICC_TARGET_LIST_IN_MSB_POS] = (ICC_TargetInfoList[i]->inBufferLenMax & 0xFF00) >> 8;

        targetInfoBuffer[baseOffset + ICC_TARGET_LIST_OUT_LSB_POS] = ICC_TargetInfoList[i]->outBufferLenMax & 0xFF;
        targetInfoBuffer[baseOffset + ICC_TARGET_LIST_OUT_MSB_POS] = (ICC_TargetInfoList[i]->outBufferLenMax & 0xFF00) >> 8;

    }

    ICC_Target_ICC.outBuffer = targetInfoBuffer;
    ICC_Target_ICC.outBufferLen = ICC_NUM_TARGETS * ICC_TARGET_LIST_ITEM_LENGTH;
    ICC_Target_ICC.outBufferState = ICC_BUFFER_STATE_READY;
}

void ICC_SetUSBEnabled(bool usbEnabled) {
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

void ICC_ICCTransmitCompleteCallback(void) {
    ICC_Target_ICC.outBufferState = ICC_BUFFER_STATE_IDLE;
    ICC_Target_ICC.outBufferLen = 0;
    ICC_Target_ICC.outBuffer = NULL;

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