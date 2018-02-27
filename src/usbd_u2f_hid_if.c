/**
  ******************************************************************************
  * @file    usbd_customhid_if_template.c
  * @author  MCD Application Team
  * @version V2.4.1
  * @date    19-June-2015
  * @brief   USB Device Custom HID interface file.
  *             This template should be copied to the user folder, renamed and customized
  *          following user needs.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <usbd_u2f_hid_if.h>
#include "usbd_pat_comp.h"
#include "u2f.h"
#include "u2f_hid.h"
#include "led.h" /* wink */
#include <stdbool.h>
#include "utilities.h" /* Utilities_packToBuffer32 */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

static int8_t U2fHid_init     (void);
static int8_t U2fHid_deinit   (void);
static int8_t U2fHid_outEvent (uint8_t* report);
static void U2fHid_responseTransmitComplete(void);
uint16_t U2fHid_sendData(uint32_t channel, uint8_t command, uint8_t* data, uint16_t len);

/* Private variables ---------------------------------------------------------*/

extern USBD_HandleTypeDef hUsbDeviceFS; //so we can send from this file


uint8_t U2fHid_payloadBuffer[U2F_HID_MAX_PAYLOAD_LEN]; /* cause i guess we have to go all out? there's no "size exceeded" option so this'll just suck up 8k of RAM */
/* there seems to be no downside to allocating channels starting at 1 and just incrementing it
 * given there's no way to deallocate a channel or any channel-specific information that needs to be kept
 */
uint32_t U2fHid_lastAllocatedChannel = 0;
volatile U2F_HID_StateTypeDef U2fHid_internalState = U2F_HID_IDLE;
uint8_t U2fHid_commandInFlight = 0; //no command
uint8_t U2fHid_receiveTimeoutSecondsRemaining = 0;
uint16_t U2fHid_expectedPayloadLength = 0;
uint16_t U2fHid_payloadReceived = 0;
uint32_t U2fHid_activeChannelID = 0; /* 0 is inactive */
uint8_t U2fHid_nextSequenceNumberExpected = 0; //0 to 127


__ALIGN_BEGIN static uint8_t U2fHid_reportDesc[USBD_U2F_HID_REPORT_DESC_SIZE] __ALIGN_END =
{

  0x06, LOBYTE(FIDO_USAGE_PAGE), HIBYTE(FIDO_USAGE_PAGE), /* usage page 0xFF00 3*/
  0x09, FIDO_USAGE_U2FHID, /* usage 5*/
  0xA1, 0x01, /* collection 1 7*/
  0x75, 0x08, /* 8 bit report size 9*/
  0x15, 0x00, /* minimum 0 11*/
  0x26, 0xFF, 0x00, /* maximum 255 14*/
  0x95, 0x40, /* report count 16*/ /* change this to change the amount of data sent */
  0x09, FIDO_USAGE_DATA_IN, /* usage 18*/
  0x81, 0x02, /* input (array) 20*/
  0x95, 0x40, /* report count 22*/ /* change this to change the amount of data sent */
  0x09, FIDO_USAGE_DATA_OUT, /* usage 24*/
  0x91, 0x02, /* output (array) 26*/
  0xC0    /*     END_COLLECTION                 27*/

};

USBD_U2F_HID_ItfTypeDef USBD_U2F_HID_Callbacks =
{
        U2fHid_reportDesc,
        U2fHid_init,
        U2fHid_deinit,
        U2fHid_outEvent,
        U2fHid_responseTransmitComplete
};
/* Private functions ---------------------------------------------------------*/

void U2fHid_resetReceive(void);
void U2fHid_sendErrorResponse(uint32_t cid, uint8_t errorValue);

/**
  * @brief  TEMPLATE_CUSTOM_HID_Init
  *         Initializes the CUSTOM HID media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t U2fHid_init(void)
{
    U2fHid_lastAllocatedChannel = 0;
    U2fHid_internalState = U2F_HID_IDLE;
    U2fHid_commandInFlight = 0; //no command
    U2fHid_receiveTimeoutSecondsRemaining = 0;
    U2fHid_expectedPayloadLength = 0;
    U2fHid_payloadReceived = 0;
    U2fHid_activeChannelID = 0; /* 0 is inactive */
    U2fHid_nextSequenceNumberExpected = 0; //0 to 127
  return (0);
}

/**
  * @brief  TEMPLATE_CUSTOM_HID_DeInit
  *         DeInitializes the CUSTOM HID media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t U2fHid_deinit(void)
{
  /*
     Add your deinitialization code here 
  */  
    //@TODO if we do dynamic mem for the registration response, remember to free it here just in case
  return (0);
}

/* call this once a second to keep the U2F HID system from locking up
 *
 */
void U2fHid_secondTick(void) {
    if((U2fHid_internalState == U2F_HID_RECEIVE) && U2fHid_receiveTimeoutSecondsRemaining) {
        U2fHid_receiveTimeoutSecondsRemaining--;
        if(0 == U2fHid_receiveTimeoutSecondsRemaining) {
            //timed out
            U2fHid_resetReceive();
        }
    }
}

/*
 * resets the multi-packet receive functionality and puts the state machine back in to the idle state
 */
void U2fHid_resetReceive(void) {
    U2fHid_payloadReceived = 0;
    U2fHid_nextSequenceNumberExpected = 0;
    U2fHid_expectedPayloadLength = 0;
    U2fHid_activeChannelID = 0;
    U2fHid_internalState = U2F_HID_IDLE;
}

/* helper as you can't send a reference to a #define */
void U2fHid_sendErrorResponse(uint32_t cid, uint8_t errorValue) {
    U2fHid_sendData(cid, U2FHID_ERROR, &errorValue, 1);
}


uint8_t U2fHid_handleContinuationPacket(U2FHID_FRAME* frame) {
    if(frame->cont.seq == U2fHid_nextSequenceNumberExpected) {
        memcpy(U2fHid_payloadBuffer + U2fHid_payloadReceived, frame->cont.data, U2F_HID_CONT_FRAME_DATA_LEN); //copy data
        U2fHid_payloadReceived += U2F_HID_CONT_FRAME_DATA_LEN; //increase this
        U2fHid_nextSequenceNumberExpected++;
        if(U2fHid_payloadReceived >= U2fHid_expectedPayloadLength) {
            switch(U2fHid_commandInFlight) {
                case U2FHID_PING:
                    //have the entirety of a PING command, send it back immediately.
                    U2fHid_sendResponse(U2FHID_PING, U2fHid_payloadBuffer, U2fHid_expectedPayloadLength);
                    break;
                case U2FHID_MSG:
                    //this needs to be handled outside the USB interrupt to keep things moving
                    U2fHid_internalState = U2F_HID_MESSAGE_READY;
                    break;
                default:
                    //this should have been caught at the init packet
#ifdef DEBUG
                    UartDebug_sendline("U2F HID received multi-packet message with unknown command. Aborting.\n");
#endif /* DEBUG */
                    U2fHid_sendErrorResponse(U2fHid_activeChannelID, ERR_INVALID_CMD);
                    U2fHid_resetReceive();
                    break;
            }
        }
    } else {
        //bad sequence

        U2fHid_sendErrorResponse(frame->cid, ERR_INVALID_SEQ);
        U2fHid_resetReceive();
    }
    return 0;
}

uint8_t U2fHid_handleInitiationPacket(U2FHID_FRAME* frame) {
    U2fHid_activeChannelID = frame->cid;
    uint16_t bcnt = (frame->init.bcnth << 8) + frame->init.bcntl;
    switch(frame->init.cmd) {
        case U2FHID_PING:
            //ping, just send the data back as-is
            if(bcnt <= U2F_HID_INIT_FRAME_DATA_LEN) {
                //whole thing fits in the init frame, just return it immediately
                U2fHid_sendResponse(U2FHID_PING, frame->init.data, bcnt);
            } else {
                //too long to send back immediately, have to start up the receive engine
                U2fHid_internalState = U2F_HID_RECEIVE;
                memcpy(U2fHid_payloadBuffer, frame->init.data, U2F_HID_INIT_FRAME_DATA_LEN);
                U2fHid_expectedPayloadLength = bcnt;
                U2fHid_payloadReceived = U2F_HID_INIT_FRAME_DATA_LEN;
                U2fHid_nextSequenceNumberExpected = 0; //starts at 0 for the first continuation packet
                U2fHid_receiveTimeoutSecondsRemaining = U2F_HID_RECEIVE_TIMEOUT_SECONDS + 1; // +1 to ensure at least the specified timeout
            }
            U2fHid_commandInFlight = frame->init.cmd;
            break;
        case U2FHID_MSG:
            //message, needs to be sent for processing to the U2F subsystem
            memcpy(U2fHid_payloadBuffer, frame->init.data, U2F_HID_INIT_FRAME_DATA_LEN);
            U2fHid_expectedPayloadLength = bcnt;
            U2fHid_payloadReceived = U2F_HID_INIT_FRAME_DATA_LEN;
            if(U2fHid_expectedPayloadLength <= U2F_HID_INIT_FRAME_DATA_LEN) {
                //whole thing fits in the init frame, just return it immediately
                U2fHid_internalState = U2F_HID_MESSAGE_READY;
            } else {
                //too long to send back immediately, have to start up the receive engine
                U2fHid_internalState = U2F_HID_RECEIVE;
                U2fHid_nextSequenceNumberExpected = 0;
                memcpy(U2fHid_payloadBuffer, frame->init.data, U2F_HID_INIT_FRAME_DATA_LEN);
                U2fHid_expectedPayloadLength = bcnt;
                U2fHid_payloadReceived = U2F_HID_INIT_FRAME_DATA_LEN;
                U2fHid_receiveTimeoutSecondsRemaining = U2F_HID_RECEIVE_TIMEOUT_SECONDS + 1; // +1 to ensure at least the specified timeout
            }
            U2fHid_commandInFlight = frame->init.cmd;
            break;
        case U2FHID_INIT:
            if(frame->cid != CID_BROADCAST) {
                //not the broadcast channel, this should not happen.
                U2fHid_sendErrorResponse(frame->cid, ERR_INVALID_CMD);
            } else if(bcnt == INIT_NONCE_SIZE) { //right payload size for an init
                memcpy(U2fHid_payloadBuffer, frame->init.data, INIT_NONCE_SIZE);
                //@TODO clean this up. its horrible
                U2fHid_lastAllocatedChannel++; //increment the channel allocation number first
                *((uint32_t*) (U2fHid_payloadBuffer + INIT_NONCE_SIZE)) = U2fHid_lastAllocatedChannel; //pack the given CID in to the buffer
                U2fHid_payloadBuffer[INIT_NONCE_SIZE + 4] = U2FHID_IF_VERSION;
                U2fHid_payloadBuffer[INIT_NONCE_SIZE + 5] = U2F_HID_DEVICE_VERSION_MAJOR;
                U2fHid_payloadBuffer[INIT_NONCE_SIZE + 6] = U2F_HID_DEVICE_VERSION_MINOR;
                U2fHid_payloadBuffer[INIT_NONCE_SIZE + 7] = U2F_HID_DEVICE_VERSION_BUILD;
                U2fHid_payloadBuffer[INIT_NONCE_SIZE + 8] = CAPFLAG_WINK;
                //send the response
                U2fHid_sendResponse(U2FHID_INIT, U2fHid_payloadBuffer, 17);
            } else {
                //receive should not have left reset
                U2fHid_sendErrorResponse(frame->cid, ERR_INVALID_LEN);
            }
            break;
        case U2FHID_WINK:
            //no parameters, just do it
            LED_DoWink();
            U2fHid_sendResponse(U2FHID_WINK, NULL, 0);
            break;
        default:
            U2fHid_internalState = U2F_HID_TRANSMIT;
            U2fHid_sendErrorResponse(frame->cid, ERR_INVALID_CMD);
    }
    return 0;
}


/**
  * @brief  TEMPLATE_CUSTOM_HID_Control
  *         Manage the CUSTOM HID class events       
  * @param  event_idx: event index
  * @param  state: event state
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t U2fHid_outEvent  (uint8_t* report ) {

#ifdef DEBUG_U2F
    UartDebug_addToBuffer("U2F USB HID Received:\n", 22);
    UartDebug_hexdump(report, USBD_U2F_HID_OUTREPORT_BUF_SIZE);
    UartDebug_newline();
#endif /* DEBUG_U2F */


    U2FHID_FRAME* frame = (U2FHID_FRAME*) report; //cast it over


    if((frame->cid != CID_BROADCAST) && (frame->cid > U2fHid_lastAllocatedChannel)) {
        //there's no specific error for "not a recognized cid" so.. other
        U2fHid_sendErrorResponse(frame->cid, ERR_OTHER);
#ifdef DEBUG
        UartDebug_sendline("U2F HID Packed received on unallocated channel.\n");
#endif /* DEBUG */
    }

    if(U2fHid_internalState == U2F_HID_RECEIVE) {
        //if we're in the middle of a multi-packet receive
        if(frame->cid == U2fHid_activeChannelID) {
            if((frame->type & TYPE_MASK) == TYPE_CONT) {
                U2fHid_handleContinuationPacket(frame);
            } else if(frame->init.cmd == U2FHID_SYNC) { //if we're not a continuation packet must be an initialization, which is odd
                //sync request from the host, i.e. reset transaction
                U2fHid_internalState = U2F_HID_TRANSMIT;
                U2fHid_resetReceive();
                U2fHid_sendData(frame->cid, U2FHID_SYNC, NULL, 0);
            } else { //not a continuation, not a sync, must be an sequencing error
#ifdef DEBUG
                UartDebug_sendline("U2F HID Non-Sync Init packet received when cont packet expected.\n");
#endif /* DEBUG */
                //the client on the host should re-attempt the entire transmission
                U2fHid_internalState = U2F_HID_TRANSMIT;
                U2fHid_resetReceive();
                U2fHid_sendErrorResponse(frame->cid, ERR_INVALID_SEQ);
            }
        } else {
            //not the active channel. immediately return that someone else has the channel
            U2fHid_sendErrorResponse(frame->cid, ERR_CHANNEL_BUSY);

        }
    } else if(U2fHid_internalState == U2F_HID_IDLE) {
        if((frame->type & TYPE_MASK) == TYPE_INIT) {
            U2fHid_handleInitiationPacket(frame);
        } /*else {
            //must be a continuation packet with no init, drop it silently
        }*/
    } else if (U2fHid_internalState == U2F_HID_ERROR) {
        //in an internal error state, possibly because we have no storage available
        U2fHid_sendErrorResponse(frame->cid, ERR_OTHER);
    } else {
        //any other internal state, including transmit
        if(frame->cid == U2fHid_activeChannelID) {
            if(((frame->type & TYPE_MASK) == TYPE_INIT) && (frame->init.cmd == U2FHID_SYNC)) {
                //sync request from the host, i.e. reset transaction
                U2fHid_resetReceive();
                U2fHid_sendResponse(U2FHID_SYNC, NULL, 0);
            } else {
#ifdef DEBUG
                UartDebug_sendline("U2F HID Non-Sync Init packet received when no packet expected.\n");
#endif /* DEBUG */
                //the client on the host should re-attempt the entire transmission
                U2fHid_internalState = U2F_HID_TRANSMIT;
                U2fHid_resetReceive();
                U2fHid_sendErrorResponse(frame->cid, ERR_INVALID_SEQ);
            }

        } else {
            //not the active channel. immediately return that someone else has the channel
            U2fHid_sendErrorResponse(frame->cid, ERR_CHANNEL_BUSY);
        }
    }
    return (0);
}

/* sends a response by setting the long-running response information
 * this can be temporarily paused on a packet-by-packet basis by immediate responses
 * in theory the response can be delayed indefinitely by badly-behaving software on the host
 * returns the number of bytes queued for transmission, or 0 in case of an error
 */
uint16_t U2fHid_sendResponse(uint8_t response, uint8_t* data, uint16_t dataLen) {
#ifdef USBD_COMPOSITE
    USBD_U2F_HID_HandleTypeDef *hhid = &(((PAT_COMP_Data*) hUsbDeviceFS.pClassData)->U2FHIDData);
#elif USB_STANDALONE_U2F
    USBD_U2F_HID_HandleTypeDef *hhid = ((USBD_U2F_HID_HandleTypeDef*) hUsbDeviceFS.pClassData;
#endif

    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED ) {
#ifdef DEBUG
        UartDebug_sendline("Attempted to send U2F HID response while USB not configured.\n");
#endif /* DEBUG */
        return 0;
    }
    if(hhid->transmitState != HID_IDLE) {
#ifdef DEBUG
        UartDebug_sendline("Attempted to send U2F HID response while response already being transmitted.\n");
#endif /* DEBUG */
        return 0;
    }
    if((data == NULL) && (dataLen != 0)) {
        //NULL data and a dataLen of 0 is valid
#ifdef DEBUG
        UartDebug_sendline("U2F HID Response attempted with null data and non-zero dataLen.\n");
#endif /* DEBUG */
        return 0;
    }
    //prep the first packet, use the local transmit buffer
    //cast it to a U2FHID_FRAME* to work with it in a more logical fashion
    U2FHID_FRAME* frame = (U2FHID_FRAME*) hhid->transmitReportBuffer;
    frame->cid = U2fHid_activeChannelID;
    frame->type = TYPE_INIT;
    frame->init.cmd |= response;
    frame->init.bcnth = dataLen >> 8;
    frame->init.bcntl = dataLen & 0xFF;

    if(dataLen <= U2F_HID_INIT_FRAME_DATA_LEN) {
        memcpy(frame->init.data, data, dataLen);
        memset(frame->init.data + dataLen, 0, U2F_HID_INIT_FRAME_DATA_LEN - dataLen); //zero fill rest of the report
        hhid->transmitBuffer = NULL;
        hhid->transmitBufferLen = 0;
    } else {
        memcpy(frame->init.data, data, dataLen);
        hhid->transmitBuffer = data + U2F_HID_INIT_FRAME_DATA_LEN;
        hhid->transmitBufferLen = dataLen - U2F_HID_INIT_FRAME_DATA_LEN;
        hhid->nextSequenceNum = 0; //make sure to reset this
    }


    U2fHid_internalState = U2F_HID_TRANSMIT;
    //UartDebug_sendline("USB HID Sent:\n");
    //UartDebug_hexdump(report, TUNNEL_HID_EPIN_SIZE);
    //if the immediate system is not transmitting, send. the dataIn function will take care of the rest.
    if(hhid->transmitImmediateState == HID_IDLE) {
        hhid->transmitState = HID_BUSY;
        USBD_LL_Transmit (&hUsbDeviceFS, U2F_HID_EPIN_ADDR, hhid->transmitReportBuffer, U2F_HID_EPIN_SIZE);
    } else {
        hhid->transmitState = HID_FULL; //flag to dataIn that the report buffer already has data in it
    }
    return dataLen;
}



/* sends bytes to the host
 * this is to be used internally by this file
 * external callers should use U2fHid_sendResponse, which limits responses to being sent to the active channel and can handle more than an init packet.
 * this should only be used for preempting responses over SendResponse to prevent excessive queueing of immediate responses
 */
uint16_t U2fHid_sendData(uint32_t channel, uint8_t response, uint8_t* data, uint16_t dataLen) {

#ifdef USBD_COMPOSITE
    USBD_U2F_HID_HandleTypeDef *hhid = &(((PAT_COMP_Data*) hUsbDeviceFS.pClassData)->U2FHIDData);
#elif USB_STANDALONE_U2F
    USBD_U2F_HID_HandleTypeDef *hhid = ((USBD_U2F_HID_HandleTypeDef*) hUsbDeviceFS.pClassData;
#endif
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED ) {
#ifdef DEBUG
        UartDebug_sendline("Attempted to send U2F HID data while USB not configured.\n");
#endif /* DEBUG */
        return 0;
    }
    if((data == NULL) && (dataLen != 0)) {
        //NULL data and a dataLen of 0 is valid
#ifdef DEBUG
        UartDebug_sendline("U2F HID Data Send attempted with null data and non-zero dataLen.\n");
#endif /* DEBUG */
        return 0;
    }
    if(dataLen > U2F_HID_INIT_FRAME_DATA_LEN) {
        //as this is designed to send back quick channel busy and error responses, it doesn't handle multi-packet responses
#ifdef DEBUG
        UartDebug_sendline("U2F HID Data Send attempted with more than one packet of data.\n");
#endif /* DEBUG */
        return 0;
    }
    if(hhid->transmitImmediateState != HID_IDLE) {
#ifdef DEBUG
        UartDebug_sendline("U2F HID Data Send attempted while busy.\n");
#endif /* DEBUG */
        return 0;
    }
    //prep the first packet, use the local immediate transmit buffer
    U2FHID_FRAME* frame = (U2FHID_FRAME*) hhid->transmitImmediateBuffer;
    frame->cid = channel;
    frame->type = TYPE_INIT;
    frame->init.cmd |= response;
    frame->init.bcnth = dataLen >> 8;
    frame->init.bcntl = dataLen & 0xFF;

    memcpy(frame->init.data, data, dataLen);
    memset(frame->init.data + dataLen, 0, U2F_HID_INIT_FRAME_DATA_LEN - dataLen); //zero fill rest of the report

    if(hhid->transmitState == HID_IDLE) {
        //if we're not sending anything else at the moment
        //send data immediately
        hhid->transmitImmediateState = HID_BUSY;
        USBD_LL_Transmit (&hUsbDeviceFS, U2F_HID_EPIN_ADDR, hhid->transmitImmediateBuffer, U2F_HID_EPIN_SIZE);
    } else {
        hhid->transmitImmediateState = HID_FULL; //flag that we need to be sent on the next dataIn call
    }
    return dataLen;

    //further packets if required will be send out by the dataIn function
}

/* call this from the low-level USB interface when the transmit of a processed command is complete
 * to transition back to an idle state
 */
static void U2fHid_responseTransmitComplete(void) {
#ifdef DEBUG
    if(U2fHid_internalState != U2F_HID_TRANSMIT) {
        UartDebug_sendline("U2F HID Transmit Complete called but not in transmit state.\n");
    } else {
        UartDebug_sendline("U2F HID Transmit Complete called.\n");
    }
#endif /* DEBUG */
    U2fHid_resetReceive(); //call this for now
}

bool U2fHid_isMessageWaiting(void) {
    return (U2fHid_internalState == U2F_HID_MESSAGE_READY);
}
uint16_t U2fHid_getMessageLength(void) {
    return U2fHid_payloadReceived;
}
uint16_t U2fHid_readMessage(uint8_t* msg, uint16_t maxMsgLen) {
    uint16_t lenToCopy = U2fHid_payloadReceived;
    if(lenToCopy > maxMsgLen) {
        lenToCopy = maxMsgLen;
    }
    memcpy(msg, U2fHid_payloadBuffer, lenToCopy);
    return lenToCopy;
}

/************************ portions (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
