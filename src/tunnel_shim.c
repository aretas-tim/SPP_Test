/*
 * tunnel_shim.c
 *
 *  Created on: Sep 13, 2017
 *      Author: me
 */

#include "tunnel_shim.h"
#include <stdint.h>
#include <stdbool.h>
#include "uart_debug.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

//static int8_t TUNNEL_HID_Init     (void);
//static int8_t TUNNEL_HID_DeInit   (void);
//static int8_t TUNNEL_HID_OutEvent (uint8_t* report);
void TUNNEL_Shim_TransmitCompleteCallback(TunnelShimContext* ctx);


void TUNNEL_Shim_ReceiveInitiationPacket(TunnelShimContext* ctx, TunnelShimPacket* packet, uint16_t packetLen);
void TUNNEL_Shim_ReceiveContinuationPacket(TunnelShimContext* ctx, TunnelShimPacket* packet, uint16_t packetLen);
void TUNNEL_Shim_HandleBroadcastPacket(TunnelShimContext* ctx, TunnelShimPacket* packet, uint16_t packetLen);
void TUNNEL_Shim_InitHandler(TunnelShimContext* ctx, TunnelShimPacket* packet, uint16_t packetLen);
void TUNNEL_Shim_ReleaseChannel(TunnelShimContext* ctx, TunnelShimPacket* packet, uint16_t packetLen);
void TUNNEL_Shim_ResynchHandler(TunnelShimContext* ctx, TunnelShimPacket* packet, uint16_t packetLen);
void TUNNEL_Shim_HandleRetransmitRequest(TunnelShimContext* ctx, TunnelShimPacket* packet, uint16_t packetLen);
void TUNNEL_Shim_TransmitAcknowledgeHandler(TunnelShimContext* ctx, TunnelShimPacket* packet, uint16_t packetLen);
void TUNNEL_Shim_TransmitCompleteHandler(TunnelShimContext* ctx, TunnelShimPacket* packet, uint16_t packetLen);

uint16_t TUNNEL_Shim_DoSend(TunnelShimContext* ctx, uint8_t* response, uint16_t len, uint8_t command);
uint16_t TUNNEL_Shim_SendImmediate(TunnelShimContext* ctx, uint8_t* immediateData, uint16_t len, uint8_t command, uint8_t channel);
uint16_t TUNNEL_Shim_SendImmediateError(TunnelShimContext* ctx, uint8_t errorCode, uint8_t channel);
uint16_t TUNNEL_Shim_SendPing(TunnelShimContext* ctx, uint8_t* response, uint16_t len);

bool TUNNEL_Shim_GetNextContinuationPacket(TunnelShimContext* ctx, TunnelShimPacket* outgoingPacket);

void TUNNEL_Shim_ResetTransaction(TunnelShimContext *ctx);


/**
 * gets the number of packets required for a given number of bytes using the provided context
 * returns 255 if numBytes is too large to be handled by the provided context
 */
uint8_t TUNNEL_Shim_GetNumPacketsRequired(TunnelShimContext* ctx, uint16_t numBytes) {
    uint32_t maxLen = ctx->packetByteLen - INITIATION_PACKET_OVERHEAD;
    maxLen += TUNNEL_SHIM_MAX_SEQUENCE_NUM * (ctx->packetByteLen - CONTINUATION_PACKET_OVERHEAD);
    if(maxLen < numBytes) {
        return 255; //error, can't handle this much data
    }
    uint8_t numPacketsRequired;
    if(numBytes <= (ctx->packetByteLen - INITIATION_PACKET_OVERHEAD)) {
        numPacketsRequired = 1;
    } else {
        numPacketsRequired = 1 + ((numBytes + 1) / (ctx->packetByteLen - CONTINUATION_PACKET_OVERHEAD)); //+1 to fix division offset
    }


    return numPacketsRequired;
}

/**
 * gets the number of bytes from an Initiation packet
 * returns 0 if this is not an initiation packet
 */
uint16_t TUNNEL_Shim_GetNumBytesFromPacket(TunnelShimPacket* packet) {
    if((packet->id & TUNNEL_SHIM_ID_MASK) == TUNNEL_SHIM_ID_INITIATION) {
        uint16_t dataLength = packet->initiation.byteCountHigh;//(((uint16_t) packet->initiation.byteCountHigh) << 8) + packet->initiation.byteCountLow;
        dataLength <<= 8; //shift up
        dataLength += packet->initiation.byteCountLow;
        return dataLength;
    } else { //continuation packet
        return 0;
    }
}

void setBitInMap(uint16_t bitNum, uint8_t* map, uint16_t mapByteLen) {
    uint16_t byteNum = bitNum >> 3;
    if(byteNum >= mapByteLen) {
        return;
    } else {
        map[byteNum] |= (uint8_t) 1 << (bitNum & 0x7);
    }
}

void clearBitInMap(uint16_t bitNum, uint8_t* map, uint16_t mapByteLen) {
    uint16_t byteNum = bitNum >> 3;
    if(byteNum >= mapByteLen) {
        return;
    } else {
        map[byteNum] &= ~((uint8_t) 1 << (bitNum & 0x7));
    }
}

bool checkBitInMap(uint16_t bitNum, uint8_t* map, uint16_t mapByteLen) {
    uint16_t byteNum = bitNum >> 3;
    if(byteNum >= mapByteLen) {
        return false;
    } else {
        return (map[byteNum] & ((uint8_t) 1 << (bitNum & 0x7))) ? true : false; //abuse conditional so we're not returning "128" for true or something
    }
}

/**
 * gets the bit number of the lowest unset bit in the map
 * just a linear search.
 * this is not designed for use on large maps
 * returns BITMAP_NO_BIT_FOUND (0xFFFFFFFF) if there are no unset bits in the map
 * otherwise returns from 0 to 524,287 (2^19 less one) depending on the length of the map
 */
uint32_t getLowestUnsetBitInMap(uint8_t* map, uint16_t mapByteLen) {
    uint16_t byteNum = 0;
    while((map[byteNum] == 0xFF) && (byteNum < mapByteLen)) {
        byteNum++;
    }
    if(byteNum < mapByteLen) { //at least one bit in this byte is 0
        uint32_t bitNum = 0;
        uint8_t byte = map[byteNum];
        //search linearly through the byte, LSBit to MSBit
        for(uint8_t b = 1; b; b <<= 1) {
            if(byte & b) {
                bitNum++; //increment this
            } else {
                //not set, found it!
                break;
            }
        }
        bitNum += byteNum << 3; //add our byte number
        return bitNum;

    } else {
        return BITMAP_NO_BIT_FOUND; //error, no bit found
    }
}


//bool isInitiationPacket


/**
 * initializes a TunnelShimContext, with the provided buffer as backing for the packets and the given parameters
 */
uint8_t TUNNEL_Shim_InitContext(TunnelShimContext* ctx, uint8_t* buffer, uint16_t bufferLen, uint16_t packetLen, TunnelShimPacketFunctions* funcs) {
    if((buffer == NULL) || (ctx == NULL)) {
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 0)
        uart_debug_sendline("Cannot Init TunnelShimContext with null context or data buffer.\n");
#endif
        return TUNNEL_SHIM_CONTEXT_ERROR_NO_CONTEXT;
    }
    if(packetLen < INITIATION_PACKET_OVERHEAD) {
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 0)
        uart_debug_sendline("Cannot Init TunnelShimContext with short packet length.\n");
#endif
        return TUNNEL_SHIM_CONTEXT_ERROR_PACKETS_TOO_SHORT;
    }
    if(funcs == NULL || funcs->transmit == NULL) {
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 0)
        uart_debug_sendline("Cannot Init TunnelShimContext with no function pointers.\n");
#endif
        return TUNNEL_SHIM_CONTEXT_ERROR_NO_TX_FUNC;
    }

    uint16_t immediatePacketLen = packetLen;
    if(immediatePacketLen > TUNNEL_SHIM_MAX_IMM_BUFFER_LEN) {
        immediatePacketLen = TUNNEL_SHIM_MAX_IMM_BUFFER_LEN;
    }
    //uint16_t initPacketDataLen = packetLen - INITIATION_PACKET_OVERHEAD;
    uint16_t minBufferLen = TUNNEL_SHIM_MIN_BUFFER_SPACE + packetLen      + (2 * immediatePacketLen);
    /*                      minimum payload                packet buffer     immediate packet buffers (may be shorter than a full packet) */
    if(bufferLen < minBufferLen) {
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 0)
        uart_debug_sendline("Cannot init context without sufficient buffer space.\n");
#endif
        return TUNNEL_SHIM_CONTEXT_ERROR_NOT_ENOUGH_BUFFER_SPACE;
    }
    ctx->immediateBufferByteLen = immediatePacketLen;

    uint8_t* buffPtr = buffer;
    for(uint8_t i = 0; i < TUNNEL_SHIM_NUM_IMM_BUFFERS; ++i) {
        ctx->immediatePacketBuffers[i] = (TunnelShimPacket*) buffPtr;
        buffPtr += ctx->immediateBufferByteLen;
    }
    ctx->packetBuffer = (TunnelShimPacket*) (buffPtr);
    buffPtr += packetLen;
    ctx->dataBuffer = buffPtr;
    ctx->dataBufferLen = bufferLen - (packetLen + ctx->immediateBufferByteLen * 2);
    ctx->dataBufferHead = 0;

    ctx->outgoingByteCount = 0;
    ctx->outgoingData = NULL;
    ctx->outgoingPacketCount = 0;

    ctx->immediatePacketBufferHead = 0;
    ctx->immediatePacketBufferTail = 0; /* send from the tail */
    ctx->immediatePacketBufferRemaining = TUNNEL_SHIM_NUM_IMM_BUFFERS; /* how many slots left in our buffer */


    memset(ctx->channelsAllocatedMap, 0, TUNNEL_SHIM_CHANNEL_BITMAP_LEN);
    setBitInMap(TUNNEL_SHIM_CHANNEL_RESERVED, ctx->channelsAllocatedMap, TUNNEL_SHIM_CHANNEL_BITMAP_LEN); //"allocate" the reserved and broadcast channels
    setBitInMap(TUNNEL_SHIM_CHANNEL_BROADCAST, ctx->channelsAllocatedMap, TUNNEL_SHIM_CHANNEL_BITMAP_LEN);

    ctx->commandInFlight = TUNNEL_SHIM_COMMAND_NONE;
    ctx->currentChannel = 0;
    ctx->previousChannel = 0;

    ctx->timeoutActive = false;
    ctx->timeoutTicksRemaining = 0;


    ctx->packetByteLen = packetLen;
    ctx->numPacketsExpected = 0;
    ctx->numBytesExpected = 0;
    memset(ctx->packetsMap, 0, TUNNEL_SHIM_PACKET_BITMAP_LEN);

    ctx->funcs = funcs; //set our fps

    ctx->state = TUNNEL_SHIM_READY;
    ctx->isTransmitting = false;

    return TUNNEL_SHIM_CONTEXT_SUCCESS;
}

/* de-initializes a context
 * do not use a de-initialized context without re-initializing it!
 */
uint8_t TUNNEL_Shim_DeInit(TunnelShimContext* ctx) {
    if(ctx == NULL) {
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 0)
        uart_debug_sendline("Cannot de-init TunnelShimContext with null context .\n");
#endif
        return TUNNEL_SHIM_CONTEXT_ERROR_NO_CONTEXT; //failed? @TODO fix this
    }
    ctx->state = TUNNEL_SHIM_UNCONFIGURED;
    ctx->isTransmitting = false;

    for(uint8_t i = 0; i < TUNNEL_SHIM_NUM_IMM_BUFFERS; ++i) {
        ctx->immediatePacketBuffers[i] = NULL;
    }
    ctx->immediateBufferByteLen = 0;
    ctx->immediatePacketBufferHead = 0;
    ctx->immediatePacketBufferTail = 0; /* send from the tail */
    ctx->immediatePacketBufferRemaining = 0; /* how many slots left in our buffer */

    ctx->packetBuffer = NULL;
    ctx->dataBuffer = NULL;
    ctx->dataBufferLen = 0;
    ctx->dataBufferHead = 0;

    memset(ctx->channelsAllocatedMap, 0, TUNNEL_SHIM_CHANNEL_BITMAP_LEN);

    ctx->commandInFlight = TUNNEL_SHIM_COMMAND_NONE;
    ctx->currentChannel = 0;
    ctx->previousChannel = 0;

    ctx->timeoutActive = false;
    ctx->timeoutTicksRemaining = 0;


    ctx->packetByteLen = 0;
    ctx->numPacketsExpected = 0;
    ctx->numBytesExpected = 0;


    memset(ctx->packetsMap, 0, TUNNEL_SHIM_PACKET_BITMAP_LEN);

    //run the disconnect callback if its set to do any other necessary cleanup
    if(ctx->funcs->disconnectCallback != NULL) {
        ctx->funcs->disconnectCallback();
    }
    //ctx->funcs->disconnectCallback = NULL; //settable elsewhere

    return 0;
}

/**
 * sets the callback to call when the USB has been disconnected to end any ongoing tunnel
 */
void TUNNEL_Shim_SetDisconnectedCallback(TunnelShimContext* ctx, void (*callback)(void)) {
    if(ctx == NULL) {
        return; // TUNNEL_SHIM_CONTEXT_ERROR_NO_CONTEXT;
    } else {
        ctx->funcs->disconnectCallback = callback;
    }

}

/**
  * receives a packet from the lower level
  */
uint8_t TUNNEL_Shim_RecievePacket(TunnelShimContext* ctx, uint8_t* packetBytes, uint16_t packetLen) {
    if(ctx == NULL) {
        return TUNNEL_SHIM_CONTEXT_ERROR_NO_CONTEXT;
    }
    if((packetBytes == NULL) || (packetLen < INITIATION_PACKET_OVERHEAD)) {
        return TUNNEL_SHIM_CONTEXT_ERROR_NO_DATA;
    }

    //uart_debug_sendline("Packet Received:\n");
    //uart_debug_hexdump(packetBytes, packetLen);

    TunnelShimPacket* packet = (TunnelShimPacket*) packetBytes; //cast our incoming packet

    if(ctx->state == TUNNEL_SHIM_UNCONFIGURED) {
        //can't do anything
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 1)
        uart_debug_sendline("Tunnel shim packet received in unconfigured state.\n");
#endif
        return TUNNEL_SHIM_CONTEXT_ERROR_NOT_CONFIGURED;
    }
    if((packet->channel & TUNNEL_SHIM_CHANNEL_MASK) == TUNNEL_SHIM_CHANNEL_BROADCAST) {
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 2)
        uart_debug_sendline("Packet on broadcast channel received.\n");
#endif /* DEBUG_PRINT_SHIM */
        TUNNEL_Shim_HandleBroadcastPacket(ctx, packet, packetLen);
    } else if((packet->id & TUNNEL_SHIM_ID_MASK) == TUNNEL_SHIM_ID_INITIATION) {
        //initiation packet
        switch(packet->initiation.command & (~TUNNEL_SHIM_ID_MASK)) {
            //not unconfigured
            case TUNNEL_SHIM_COMMAND_MESSAGE:
            case TUNNEL_SHIM_COMMAND_PING:
                TUNNEL_Shim_ReceiveInitiationPacket(ctx, packet, packetLen);
                break;
            case TUNNEL_SHIM_COMMAND_RELEASE:
                TUNNEL_Shim_ReleaseChannel(ctx, packet, packetLen);
                break;
            case TUNNEL_SHIM_COMMAND_INIT:
                TUNNEL_Shim_InitHandler(ctx, packet, packetLen);
                break;
            case TUNNEL_SHIM_COMMAND_COMPLETE:
                TUNNEL_Shim_TransmitCompleteHandler(ctx, packet, packetLen);
                break;
            case TUNNEL_SHIM_COMMAND_ACKNOWLEDGE:
                //uart_debug_hexprint32(hhid->transmitState);
                //uart_debug_putchar(' ');
                TUNNEL_Shim_TransmitAcknowledgeHandler(ctx, packet, packetLen);
                //uart_debug_hexprint32(hhid->transmitState);
                //uart_debug_newline();
                break;
            case TUNNEL_SHIM_COMMAND_RESYNCH:
                TUNNEL_Shim_ResynchHandler(ctx, packet, packetLen);
                break;
            case TUNNEL_SHIM_COMMAND_RETRANSMIT:
                TUNNEL_Shim_HandleRetransmitRequest(ctx, packet, packetLen);
                break;
            case TUNNEL_SHIM_COMMAND_ERROR:
                //this is odd, we should be the ones sending error messages
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 1)
                uart_debug_sendline("Tunnel Shim Error packet received.\n");
#endif
                break;
            case TUNNEL_SHIM_COMMAND_NONE:
            default:
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 1)
                uart_debug_sendline("Unexpected initiation packet command received.\n");
#endif
                break;
        }
    } else {
        //must be continuation packet
        if(ctx->state == TUNNEL_SHIM_RECEIVE) {
            //receive packet
            TUNNEL_Shim_ReceiveContinuationPacket(ctx, packet, packetLen);
        } else {
            //not expecting a continuation packet
            //request a resynch as we likely lost the initiation packet
            TUNNEL_Shim_SendImmediate(ctx, NULL, 0, TUNNEL_SHIM_COMMAND_RESYNCH, (packet->channel & TUNNEL_SHIM_CHANNEL_MASK));
        }
    }
    return (0);
}

/* basic callback to let us know when all packets have been sent out
 *
 */
void TUNNEL_Shim_TransmitCompleteCallback(TunnelShimContext* ctx) {
    ctx->state = TUNNEL_SHIM_TRANSMIT_COMPLETE;
}

/**
 * returns the number of bytes available from the out buffer, if the out buffer has a complete message waiting
 * if nothing is available or the message is incomplete, returns 0
 */
size_t TUNNEL_Shim_MessageAvailable(TunnelShimContext* ctx) {
    if(ctx == NULL) {
        return 0;
    }
    if(ctx->funcs->disableInterrupt != NULL) {
        ctx->funcs->disableInterrupt();
    }
    size_t messageLen = 0;
    if(ctx->state == TUNNEL_SHIM_MESSAGE_WAITING) {
        messageLen = ctx->dataBufferHead;
    }
    if(ctx->funcs->enableInterrupt != NULL) {
        //2017-10-24 do not enable this block. it'll overrun memcpy and ruin your day.
        //uart_debug_sendstring("Tunnel Shim Enable Interrupt In Message Available at: ");
        //uart_debug_hexprint32((uint32_t) ctx->funcs->enableInterrupt); //yes it'll complain about the cast
        //uart_debug_newline();

        ctx->funcs->enableInterrupt();
    }
    return messageLen;
}


/**
 * gets the waiting message by setting a pointer to a uint8_t* and returning the number of valid bytes at the location pointed to.
 * also returns the channel the message came in on (for future compatibility once we have multichannel tunnels
 *
 * @param ctx (in variable) the context to get the message from
 * @param message (out variable) a pointer to a uint8_t* that will be set to the location of the message
 * @param channel (out variable) a pointer to a uint8_t that will be set to the channel the message was received on
 * @returns the number of valid bytes in the message, if 0, no message was available or it had a length of 0
 */
size_t TUNNEL_Shim_GetMessage(TunnelShimContext* ctx, uint8_t** message, uint8_t* channel) {
    if(ctx == NULL) {
        return 0;
    }
    if(ctx->funcs->disableInterrupt != NULL) {
        ctx->funcs->disableInterrupt();
    }
    if((ctx->state != TUNNEL_SHIM_MESSAGE_WAITING) || (ctx->dataBufferHead == 0)) {
        return 0;
    }
    *message = ctx->dataBuffer;
    *channel = ctx->currentChannel;
    ctx->state = TUNNEL_SHIM_PROCESSING;
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 1)
    uart_debug_sendline("Shim Message Retrieved by Upper Layer.\n");
#endif
    if(ctx->funcs->enableInterrupt != NULL) {
        ctx->funcs->enableInterrupt();
    }
    return ctx->dataBufferHead;
}





/**
 * handles an incoming retransmit request from the host
 * internal use, ctx, packet and packetLen should all have been checked and be valid
 */
void TUNNEL_Shim_HandleRetransmitRequest(TunnelShimContext* ctx, TunnelShimPacket* packet, uint16_t packetLen) {
    if(ctx->currentChannel != (packet->channel & TUNNEL_SHIM_CHANNEL_MASK)) {
        TUNNEL_Shim_SendImmediateError(ctx, TUNNEL_SHIM_ERROR_CHANNEL_BUSY, (packet->channel & TUNNEL_SHIM_CHANNEL_MASK));
    } else if((ctx->state == TUNNEL_SHIM_TRANSMIT) || (ctx->state == TUNNEL_SHIM_TRANSMIT_COMPLETE)) {
        //USBD_TUNNEL_HID_HandleTypeDef *hhid = &(((PAT_COMP_Data*) hUsbDeviceFS.pClassData)->tunnelHIDData);
        //uint8_t lowestRetransmittedPacketNum = TUNNEL_SHIM_MAX_SEQUENCE_NUM;
        uint16_t byteCount = TUNNEL_Shim_GetNumBytesFromPacket(packet);
        if(packetLen < byteCount + INITIATION_PACKET_OVERHEAD) {
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 1)
            uart_debug_sendline("Short PacketLen in retransmit request.\n");
#endif
            byteCount = packetLen - INITIATION_PACKET_OVERHEAD;
        }

        for(uint16_t i = 0; i < byteCount; ++i) {
            uint8_t packetToRetransmit = packet->initiation.data[i];
            if(packetToRetransmit < ctx->outgoingPacketCount) {
                clearBitInMap(packetToRetransmit, ctx->packetsMap, TUNNEL_SHIM_PACKET_BITMAP_LEN);
                /*if(packetToRetransmit < lowestRetransmittedPacketNum) {
                    lowestRetransmittedPacketNum = packetToRetransmit;
                }*/
            }
        }
        if(ctx->state == TUNNEL_SHIM_TRANSMIT_COMPLETE) {
            ctx->state = TUNNEL_SHIM_TRANSMIT; //set this back to transmit
            if(!ctx->isTransmitting) { //if nothing is being sent
                //TunnelShimPacket* pkt = NULL;
                bool doSend = TUNNEL_Shim_GetNextContinuationPacket(ctx, ctx->packetBuffer);
                if(doSend) {
                    ctx->funcs->transmit((uint8_t*) ctx->packetBuffer, ctx->packetByteLen); //send something
                    ctx->isTransmitting = true;
                } else {
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 1)
                    uart_debug_sendline("No packet found to send from retransmit request.\n");
#endif
                }
            } //else an immediate is being sent, will get picked up
        } //else should get picked up in normal GetNextTransmitPacket call
    } else {
        TUNNEL_Shim_SendImmediateError(ctx, TUNNEL_SHIM_ERROR_FAILED, (packet->channel & TUNNEL_SHIM_CHANNEL_MASK)); //could not resynch
    }

}

/**
 * InitHandler, handles channel initiation packets, returning the nonce and either a channel or the reserved channel
 * indicating no channels available
 */
void TUNNEL_Shim_InitHandler(TunnelShimContext* ctx, TunnelShimPacket* packet, uint16_t packetLen) {
    uint16_t byteLen = TUNNEL_Shim_GetNumBytesFromPacket(packet);
    if(byteLen != TUNNEL_SHIM_INIT_NONCE_LEN || packetLen <= TUNNEL_SHIM_INIT_NONCE_LEN) {
        //we can't actually inform the requester because we have insufficient information to successfully contact them
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 1)
        uart_debug_sendline("Bad Shim Init Request Received.\n");
#endif
        return;
    }

    uint8_t initResponse[TUNNEL_SHIM_INIT_NONCE_LEN + 1];
    memcpy(initResponse, packet->initiation.data, TUNNEL_SHIM_INIT_NONCE_LEN);
    uint32_t unallocatedChannel = getLowestUnsetBitInMap(ctx->channelsAllocatedMap, TUNNEL_SHIM_CHANNEL_BITMAP_LEN);
    if(unallocatedChannel == BITMAP_NO_BIT_FOUND) {
        //can't allocate channel, send back the reserved channel ID which indicates failure
        initResponse[TUNNEL_SHIM_INIT_NONCE_LEN] = TUNNEL_SHIM_CHANNEL_RESERVED;
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 1)
        uart_debug_sendline("Channel Allocation Failed. No Channels left.");
#endif
    } else {
        //allocate the channel and send it back
        initResponse[TUNNEL_SHIM_INIT_NONCE_LEN] = (uint8_t) (unallocatedChannel & 0xFF);
        setBitInMap(initResponse[TUNNEL_SHIM_INIT_NONCE_LEN], ctx->channelsAllocatedMap, TUNNEL_SHIM_CHANNEL_BITMAP_LEN);
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 1)
        uart_debug_sendstring("Allocated channel: ");
        uart_debug_printuint8(initResponse[TUNNEL_SHIM_INIT_NONCE_LEN]);
        uart_debug_newline();
#endif
    }
    uint16_t sendResult = TUNNEL_Shim_SendImmediate(ctx, initResponse, TUNNEL_SHIM_INIT_NONCE_LEN + 1, TUNNEL_SHIM_COMMAND_INIT, TUNNEL_SHIM_CHANNEL_BROADCAST);
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 0)
    if(sendResult != (TUNNEL_SHIM_INIT_NONCE_LEN + 1)) {
        uart_debug_sendline("Error on send in Tunnel Shim Init Handler.\n");
    }
#endif
}

/**
 * handles any broadcast packets received
 * currently just init commands are sent over the broadcast channel
 */
void TUNNEL_Shim_HandleBroadcastPacket(TunnelShimContext* ctx, TunnelShimPacket* packet, uint16_t packetLen) {
    //if(ctx->currentChannel == TUNNEL_SHIM_CHANNEL_BROADCAST) {
        //@TODO replace this with a switch statement if we get more broadcast stuff
        if((packet->initiation.command & (~TUNNEL_SHIM_ID_MASK)) == TUNNEL_SHIM_COMMAND_INIT) {
            TUNNEL_Shim_InitHandler(ctx, packet, packetLen);
        } else {
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 1)
            uart_debug_sendline("Unknown Broadcast command received.\n");
#endif
            //don't send anything, as we don't know who sent it
        }
    /*} else {
        uart_debug_sendline("Non-Broadcast packet routed to broadcast packet handler.\n");
    }*/
}

/**
 * releases a channel
 * now with actual channel release action
 */
void TUNNEL_Shim_ReleaseChannel(TunnelShimContext* ctx, TunnelShimPacket* packet, uint16_t packetLen) {
    (void) packetLen; //unused
    uint8_t channel = (packet->channel & TUNNEL_SHIM_CHANNEL_MASK);
    //if the requesting channel is the active channel (currently receiving/processing/transmitting), can't end the channel
    if((channel == ctx->currentChannel) && ((ctx->state != TUNNEL_SHIM_READY) || (ctx->state != TUNNEL_SHIM_TRANSMIT_COMPLETE))) {
        TUNNEL_Shim_SendImmediateError(ctx, TUNNEL_SHIM_ERROR_RELEASE_IMPOSSIBLE, channel);

    } else {
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 1)
        uart_debug_sendstring("Tunnel Shim Channel Released: ");
        uart_debug_printuint8(channel);
        uart_debug_newline();
#endif
        //should not be coming in as an immediate
        TUNNEL_Shim_DoSend(NULL, 0, TUNNEL_SHIM_COMMAND_RELEASE, channel);
        clearBitInMap(channel, ctx->channelsAllocatedMap, TUNNEL_SHIM_CHANNEL_BITMAP_LEN);
    }
}

/**
 * resynch. resets the state if possible
 * on receive or if there's a message waiting (but hasn't made it to processing), it cancels the command
 * on transmit, resets the entire transmit and starts it sending from the first packet
 */
void TUNNEL_Shim_ResynchHandler(TunnelShimContext* ctx, TunnelShimPacket* packet, uint16_t packetLen) {
    (void) packetLen; //unusued
    if((packet->channel & TUNNEL_SHIM_CHANNEL_MASK) != ctx->currentChannel) {
        //drop packet, not for the active channel
        TUNNEL_Shim_SendImmediateError(ctx, TUNNEL_SHIM_ERROR_CHANNEL_BUSY, (packet->channel & TUNNEL_SHIM_CHANNEL_MASK));
    } else {
        //for current channel
        switch(ctx->state) {
            case TUNNEL_SHIM_READY:
                //should never be received in a ready state, or gets picked up by fact there should be no active channel
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 1)
                uart_debug_sendline("Resynch received when in ready state. Packet Dropped.\n");
#endif
                break;
            case TUNNEL_SHIM_RECEIVE:
            case TUNNEL_SHIM_MESSAGE_WAITING:
                //reset receive, cycle back to ready
                ctx->currentChannel = TUNNEL_SHIM_CHANNEL_RESERVED;
                ctx->state = TUNNEL_SHIM_READY;
                ctx->commandInFlight = TUNNEL_SHIM_COMMAND_NONE;
                ctx->dataBufferHead = 0;
                ctx->numPacketsExpected = 0;
                memset(ctx->packetsMap, 0, TUNNEL_SHIM_PACKET_BITMAP_LEN); //clear this
                ctx->timeoutActive = false;
                ctx->timeoutTicksRemaining = 0;
                break;
            case TUNNEL_SHIM_PROCESSING:
                //impossible
                TUNNEL_Shim_SendImmediateError(ctx, TUNNEL_SHIM_ERROR_RESYNCH_IMPOSSIBLE, (packet->channel & TUNNEL_SHIM_CHANNEL_MASK));
                break;
            case TUNNEL_SHIM_TRANSMIT:
            case TUNNEL_SHIM_TRANSMIT_COMPLETE:
                memset(ctx->packetsMap, 0, TUNNEL_SHIM_PACKET_BITMAP_LEN); //clear this
                ctx->state = TUNNEL_SHIM_TRANSMIT; //set this back to transmit if it was complete
                //send it all again
                TUNNEL_Shim_DoSend(ctx, ctx->outgoingData, ctx->outgoingByteCount, ctx->commandInFlight);
                break;
            default:
                break;
        }
    }
}

/**
 * stops our transmit timeout, as the host has acknowledge that we're sending in a response
 */
void TUNNEL_Shim_TransmitAcknowledgeHandler(TunnelShimContext* ctx, TunnelShimPacket* packet, uint16_t packetLen) {
    (void) packetLen;
    if(((packet->channel & TUNNEL_SHIM_CHANNEL_MASK) == ctx->currentChannel) && (TUNNEL_SHIM_TRANSMIT_ACK_WAIT == ctx->state)) {
        //host has acknowledge we're transmitting, do not have to re-transmit initiation packet after timeout
        //TUNNEL_Shim_StopTimer(ctx);
        ctx->timeoutActive = false;
        ctx->timeoutTicksRemaining = 0;
        ctx->ackWaitCountRemaining = 0;
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 2)
        uart_debug_sendline("Transmit Acknowledged by host.\n");
#endif

        //USBD_TUNNEL_HID_HandleTypeDef *hhid = &(((PAT_COMP_Data*) hUsbDeviceFS.pClassData)->tunnelHIDData);
        if(ctx->outgoingPacketCount > 1) { //send the rest of the packets
            ctx->state = TUNNEL_SHIM_TRANSMIT;
            //TUNNEL_Shim_SendRemainingPackets(ctx);
            if(!ctx->isTransmitting) {
                //TunnelShimPacket* pkt = NULL;
                bool doSend = TUNNEL_Shim_GetNextContinuationPacket(ctx, ctx->packetBuffer);
                if(doSend) {
                    ctx->funcs->transmit((uint8_t*) ctx->packetBuffer, ctx->packetByteLen);
                    ctx->isTransmitting = true;
                } else {
                    //no packet to send?
                    uart_debug_sendline("Tunnel Shim Transmit Complete in Ack. Handler by NULL next packet.\n");
                    ctx->state = TUNNEL_SHIM_TRANSMIT_COMPLETE;
                }
            } //else will get picked up in callback to GetNextTransmitPacket(...) by lower layer

        } else { //transmit complete
            uart_debug_sendline("Tunnel Shim Transmit Complete in Ack. Handler.\n");
            ctx->state = TUNNEL_SHIM_TRANSMIT_COMPLETE;
        }
    } else {
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 1)
        uart_debug_sendstring("Transmit Acknowledge received but not waiting for acknowledge (");
        uart_debug_printuint8(ctx->state);
        uart_debug_sendline(").\n");
#endif
    }
}

/**
 * handles receiving the reception of a transmit complete packet from the host
 * transitions back to ready and cancels any timeout to retransmit the whole thing
 */
void TUNNEL_Shim_TransmitCompleteHandler(TunnelShimContext* ctx, TunnelShimPacket* packet, uint16_t packetLen) {
    (void) packetLen;
    bool doAck = false;
    if((packet->channel & TUNNEL_SHIM_CHANNEL_MASK) == ctx->previousChannel) {
        doAck = true;
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 1)
        uart_debug_sendline("Transmit Complete by host but already complete.\n");
#endif
    } else if((packet->channel & TUNNEL_SHIM_CHANNEL_MASK) == ctx->currentChannel) {
        if((ctx->state == TUNNEL_SHIM_TRANSMIT_COMPLETE) || (ctx->state == TUNNEL_SHIM_TRANSMIT_ACK_WAIT)) {
            //zero our timeout in case an acknowledge was not sent
            ctx->previousChannel = ctx->currentChannel;
            ctx->currentChannel = TUNNEL_SHIM_CHANNEL_RESERVED; //transition back to ready
            ctx->state = TUNNEL_SHIM_READY;

            ctx->timeoutActive = false;
            ctx->timeoutTicksRemaining = 0; //clear our global timeout too
            ctx->ackWaitCountRemaining = 0;
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 2)
            uart_debug_sendline("Transmit Complete by host.\n");
#endif
            doAck = true;
        } else {
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 1)
            uart_debug_sendstring("Transmit Complete received but not in transmit complete or ack wait state (");
            uart_debug_printuint8(ctx->state);
            uart_debug_sendline(").\n");
#endif
            doAck = false;
        }

    } else {
        doAck = false;
    }
    if(doAck) {
        TUNNEL_Shim_SendImmediate(ctx, NULL, 0, TUNNEL_SHIM_COMMAND_COMPLETE_ACKNOWLEDGE, (packet->channel & TUNNEL_SHIM_CHANNEL_MASK));
    } else {

    }
}

/**
 * handles reception of an initiation packet containing a message or ping request
 * (i.e. anything that isn't able to be handled immediately)
 */
void TUNNEL_Shim_ReceiveInitiationPacket(TunnelShimContext* ctx, TunnelShimPacket* packet, uint16_t packetLen) {
    (void) packetLen;
    uint8_t channel = (packet->channel & TUNNEL_SHIM_CHANNEL_MASK);
    if((ctx->currentChannel != TUNNEL_SHIM_CHANNEL_RESERVED) && (channel != ctx->currentChannel)) {
        //drop packet, not for the active channel
        TUNNEL_Shim_SendImmediateError(ctx, TUNNEL_SHIM_ERROR_CHANNEL_BUSY, channel);
    } else if (!(checkBitInMap(channel, ctx->channelsAllocatedMap, TUNNEL_SHIM_CHANNEL_BITMAP_LEN))) {
        //not for us, drop
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 1)
        uart_debug_sendline("Packet Received on unallocated channel. Dropped.\n");
#endif
    } else {
        uint16_t dataLength = TUNNEL_Shim_GetNumBytesFromPacket(packet);
        uint8_t numPacketsRequired = TUNNEL_Shim_GetNumPacketsRequired(ctx, dataLength); //always at least 1
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 1)
        uart_debug_sendstring("Initiation Packet Received. Expecting ");
        uart_debug_printuint8(numPacketsRequired);
        uart_debug_sendstring(" packets for ");
        uart_debug_printuint32(dataLength);
        uart_debug_sendline(" bytes of payload.\n");
#endif

        if((numPacketsRequired > TUNNEL_SHIM_MAX_PACKET_COUNT) || (dataLength > ctx->dataBufferLen)) {
            //well crud. this should never happen
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 0)
            uart_debug_sendline("Tunnel Shim Receive Impossible. Not enough buffer space.\n");
#endif
            TUNNEL_Shim_SendImmediateError(ctx, TUNNEL_SHIM_ERROR_OUT_OF_MEMORY, channel);
            return;
        }

        //start our global timer
        ctx->timeoutTicksRemaining = TUNNEL_SHIM_TIMEOUT_TRANSMIT_GLOBAL;
        ctx->timeoutActive = true;

        ctx->commandInFlight = packet->initiation.command & (~TUNNEL_SHIM_ID_MASK); //mask off initiation flag bit

        ctx->currentChannel = channel;
        ctx->previousChannel = TUNNEL_SHIM_CHANNEL_RESERVED; //clear this back to the reserved channel
        if(numPacketsRequired == 1) {
            //i.e. everything is contained in the initiation packet
            memcpy(ctx->dataBuffer, packet->initiation.data, dataLength);

            TUNNEL_Shim_SendImmediate(ctx, NULL, 0,  TUNNEL_SHIM_COMMAND_COMPLETE, ctx->currentChannel);  /*2017-08-24 changed this from an ACK, 2017-09-15 moved this out of the switch statement */
            switch(packet->initiation.command & (~TUNNEL_SHIM_ID_MASK)) {
                case TUNNEL_SHIM_COMMAND_MESSAGE:
                    ctx->state = TUNNEL_SHIM_MESSAGE_WAITING;
                    ctx->dataBufferHead = dataLength;
                    //TUNNEL_Shim_SendImmediate(ctx, NULL, 0, TUNNEL_SHIM_COMMAND_COMPLETE, ctx->currentChannel);
                    break;
                case TUNNEL_SHIM_COMMAND_PING:
                    ctx->state = TUNNEL_SHIM_PROCESSING;
                    TUNNEL_Shim_DoSend(ctx, ctx->dataBuffer, dataLength, TUNNEL_SHIM_COMMAND_PING);
                    break;
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 0)
                default:
                    //unknown command, should have been caught earlier
                    uart_debug_sendline("Initiation handler received unexpected command.\n");
#endif
            }

        } else {
            ctx->state = TUNNEL_SHIM_RECEIVE;
            ctx->numBytesExpected = dataLength;
            ctx->numPacketsExpected = numPacketsRequired;


            //at least one continuation packet is expected to follow.

            memcpy(ctx->dataBuffer, packet->initiation.data, ctx->packetByteLen - INITIATION_PACKET_OVERHEAD);
            memset(ctx->packetsMap, 0, TUNNEL_SHIM_PACKET_BITMAP_LEN); //clear our packets map
            setBitInMap(0, ctx->packetsMap, TUNNEL_SHIM_PACKET_BITMAP_LEN); //set that we have the initiation packet

            TUNNEL_Shim_SendImmediate(ctx, &numPacketsRequired, 1, TUNNEL_SHIM_COMMAND_ACKNOWLEDGE, ctx->currentChannel);
        }
    }
}

/**
 * receives a continuation packet, must be message or ping (or something unforeseen when writing this comment)
 */
void TUNNEL_Shim_ReceiveContinuationPacket(TunnelShimContext* ctx, TunnelShimPacket* packet, uint16_t packetLen) {
    (void) packetLen;
    if((packet->channel & TUNNEL_SHIM_CHANNEL_MASK) != ctx->currentChannel) {
        //drop packet, not for the active channel
        TUNNEL_Shim_SendImmediateError(ctx, TUNNEL_SHIM_ERROR_CHANNEL_BUSY, (packet->channel & TUNNEL_SHIM_CHANNEL_MASK));
    } else {
        uint16_t dataPosition = ctx->packetByteLen - INITIATION_PACKET_OVERHEAD;
        dataPosition += (ctx->packetByteLen - CONTINUATION_PACKET_OVERHEAD) * (packet->continuation.sequence - 1);
        memcpy(ctx->dataBuffer + dataPosition, packet->continuation.data, ctx->packetByteLen - CONTINUATION_PACKET_OVERHEAD);
        setBitInMap(packet->continuation.sequence, ctx->packetsMap, TUNNEL_SHIM_PACKET_BITMAP_LEN); //set that we have the packet
        //check if we have all the data
        uint16_t highestReceivedPacketNum = getLowestUnsetBitInMap(ctx->packetsMap, TUNNEL_SHIM_PACKET_BITMAP_LEN);
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 2)
        uart_debug_sendstring("Received packet ");
        uart_debug_printuint32(highestReceivedPacketNum);
        uart_debug_sendstring(" of ");
        uart_debug_printuint32(ctx->numPacketsExpected);
        uart_debug_sendstring(" expected.\n");
#endif
        if(highestReceivedPacketNum >= ctx->numPacketsExpected) {
            //inform the host we have all the packets
            TUNNEL_Shim_SendImmediate(ctx, NULL, 0, TUNNEL_SHIM_COMMAND_COMPLETE, ctx->currentChannel);
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 2)
            uart_debug_sendline("All packets received.\n");
#endif
            //figure out what to do
            switch(ctx->commandInFlight) {
                case TUNNEL_SHIM_COMMAND_MESSAGE:
                    uart_debug_sendline("Message Waiting.\n");
                    ctx->state = TUNNEL_SHIM_MESSAGE_WAITING;
                    ctx->dataBufferHead = ctx->numBytesExpected;
                    break;
                case TUNNEL_SHIM_COMMAND_PING:
                    uart_debug_sendline("Ping processing.\n");
                    ctx->state = TUNNEL_SHIM_PROCESSING;
                    //TUNNEL_Shim_DoSend(ctx, ctx->dataBuffer, ctx->numBytesExpected, TUNNEL_SHIM_COMMAND_PING);
                    TUNNEL_Shim_SendPing(ctx, ctx->dataBuffer, ctx->numBytesExpected);
                    break;
                default:
                    //this should have been caught earlier. re-catch it
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 0)
                    uart_debug_sendline("Default case in ReceiveContinuationPacket. Unknown Command. Transaction Reset.\n");
#endif
                    TUNNEL_Shim_SendImmediateError(ctx, TUNNEL_SHIM_ERROR_UNKNOWN_COMMAND, (packet->channel & TUNNEL_SHIM_CHANNEL_MASK));
                    TUNNEL_Shim_ResetTransaction(ctx);
            }
        }
    }
}


/**
 * packetizes a bunch of data and puts it in the provided buffer
 *
 *
 * @param packets in/out pointer to where the packets should be written to
 * @param numPackets in/out pointer to a uint8_t, incoming this is the maximum number of packets that can be written, outgoing is the number of packets that were written
 * @param command in the command byte
 * @param data in the data to packetize
 * @param dataLen in the length of the data
 * @return true if successful, false if not
 */
/*bool TUNNEL_HID_Packetize(TunnelShimPacket* packets,  uint8_t* numPackets, uint8_t command, uint8_t* data, uint16_t dataLen) {
    //uart_debug_sendline("Pre-Packetize Data Dump:\n");
    //uart_debug_hexdump(data, dataLen);
    uint8_t numPacketsRequired = TUNNEL_HID_GetNumPacketsRequired(dataLen); //always at least 1
    if((numPacketsRequired > *numPackets) || (numPacketsRequired > (TUNNEL_SHIM_MAX_SEQUENCE_NUM + 1))) {
        //well crud
        uart_debug_sendline("Tunnel Shim Packetize Failed. Not enough packet space.\n");
        return false;
    }
    uart_debug_sendstring("Packetizing. Using ");
    uart_debug_printuint8(numPacketsRequired);
    uart_debug_sendstring(" packets for ");
    uart_debug_printuint32(dataLen);
    uart_debug_sendline(" bytes of payload.\n");
    uint8_t channel = TUNNEL_HID_CurrentChannel;
    packets[0].channel = channel;
    packets[0].initiation.command = TUNNEL_SHIM_ID_INITIATION | command;
    packets[0].initiation.byteCountHigh = (dataLen & 0xFF00) >> 8;
    packets[0].initiation.byteCountLow = dataLen & 0xFF;
    if(numPacketsRequired > 1) {
        //more than one packet, first packet is full
        memcpy(packets[0].initiation.data, data, TUNNEL_SHIM_USB_FS_INIT_DATA_SIZE);

    } else {
        //only one packet, first packet may not be full
        memcpy(packets[0].initiation.data, data, dataLen);
        uint16_t fillLength = (TUNNEL_SHIM_USB_FS_INIT_DATA_SIZE) - dataLen;
        if(fillLength) {
            memset(packets[0].initiation.data + INITIATION_PACKET_OVERHEAD + dataLen, 0, fillLength);
        }
    }

    uint16_t dataPosition = TUNNEL_SHIM_USB_FS_INIT_DATA_SIZE; //offset in to the data array

    for(uint8_t packetNum = 1; packetNum < numPacketsRequired; ++packetNum) {
        packets[packetNum].channel = channel;
        packets[packetNum].continuation.sequence = packetNum;
        if(packetNum == (numPacketsRequired - 1)) {
            //last packet
            uint16_t bytesRemaining = dataLen - dataPosition;
            if(bytesRemaining > dataLen) {
                uart_debug_sendline("bytes remaining greater than datalen. trapped.\n");
                while(true);//trap
            }
            memcpy(packets[packetNum].continuation.data, data + dataPosition, bytesRemaining);
            uint16_t fillLength = TUNNEL_SHIM_USB_FS_CONT_DATA_SIZE - bytesRemaining;
            if(fillLength) {
                memset(packets[packetNum].continuation.data + CONTINUATION_PACKET_OVERHEAD + bytesRemaining, 0, fillLength);
            }
            dataPosition += bytesRemaining;
        } else {
            //not the last packet, full data segment
            memcpy(packets[packetNum].continuation.data, data + dataPosition, TUNNEL_SHIM_USB_FS_CONT_DATA_SIZE);
            dataPosition += (TUNNEL_SHIM_USB_FS_CONT_DATA_SIZE);
        }
        //dataPosition += TUNNEL_SHIM_REPORT_SIZE - CONTINUATION_PACKET_OVERHEAD;
    }
    *numPackets = numPacketsRequired; //copy this over
    //uart_debug_sendline("Post-Packetize Packet Dump:\n");
    //uart_debug_hexdump((uint8_t*) packets, (*numPackets) * TUNNEL_SHIM_REPORT_SIZE);
    return true;
}*/

/**
 * sends a ping response
 */
uint16_t TUNNEL_Shim_SendPing(TunnelShimContext* ctx, uint8_t* response, uint16_t len) {
    if(ctx->state != TUNNEL_SHIM_PROCESSING) {
        return 0;
    }
    //should already be in an interrupt context
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 1)
    uart_debug_sendline("Tunnel Shim Ping Response Starting.\n");
#endif
    return TUNNEL_Shim_DoSend(ctx, response, len, TUNNEL_SHIM_COMMAND_PING);
}

/**
 * send response
 * called to send a message response back to the host
 */
uint16_t TUNNEL_Shim_SendResponse(TunnelShimContext* ctx, uint8_t* response, uint16_t len) {
    if(ctx == NULL) {
        return 0;
    }
    if(ctx->state != TUNNEL_SHIM_PROCESSING) {
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 0)
        uart_debug_sendline("Tunnel Shim Send Response called but context not in processing state.\n");
#endif
        return 0;
    }
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 2)
    uart_debug_sendline("Tunnel Shim Send Response called.\n");
#endif
    if(ctx->funcs->disableInterrupt != NULL) {
        ctx->funcs->disableInterrupt();
    }
    return TUNNEL_Shim_DoSend(ctx, response, len, TUNNEL_SHIM_COMMAND_MESSAGE);
    if(ctx->funcs->enableInterrupt != NULL) {
        ctx->funcs->enableInterrupt();
    }
}

/**
 * internal function to prep the init packet in to the outgoing packet buffer
 */
void TUNNEL_Shim_PrepInitPacket(TunnelShimContext* ctx, TunnelShimPacket* pkt) {
    pkt->channel = TUNNEL_SHIM_ID_INITIATION | ctx->currentChannel;
    pkt->initiation.command = ctx->commandInFlight;
    pkt->initiation.byteCountHigh = (ctx->outgoingByteCount & 0xFF00) >> 8;
    pkt->initiation.byteCountLow = ctx->outgoingByteCount & 0x00FF;

    if(ctx->outgoingByteCount < (ctx->packetByteLen - INITIATION_PACKET_OVERHEAD)) {
        memcpy(pkt->initiation.data, ctx->outgoingData, ctx->outgoingByteCount);
        uint16_t bytesRemaining = (ctx->packetByteLen - INITIATION_PACKET_OVERHEAD) - ctx->outgoingByteCount;
        memset(pkt->initiation.data + ctx->outgoingByteCount, 0, bytesRemaining); //zero out remaining buffer
    } else { //at least a full packet
        memcpy(pkt->initiation.data, ctx->outgoingData, (ctx->packetByteLen - INITIATION_PACKET_OVERHEAD));
    }
}

/**
 * internal function to actually send the data out the packet interface
 *
 *
 * @param response the response data
 * @param len the length of the response
 * @param command the command byte
 * @returns the number of bytes written and queued to the USB
 */
uint16_t TUNNEL_Shim_DoSend(TunnelShimContext* ctx, uint8_t* response, uint16_t len, uint8_t command) {
    //USBD_TUNNEL_HID_HandleTypeDef *hhid = &(((PAT_COMP_Data*) hUsbDeviceFS.pClassData)->tunnelHIDData);
    //USBD_TUNNEL_HID_HandleTypeDef     *hhid = (USBD_TUNNEL_HID_HandleTypeDef*)pdev->pClassData; //this is for standalone

    if(command != ctx->commandInFlight) {
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 0)
        uart_debug_sendline("Tunnel shim response mismatch with command in flight.\n");
#endif
    }

    memset(ctx->packetsMap, 0, TUNNEL_SHIM_PACKET_BITMAP_LEN); //wipe this out

    ctx->outgoingByteCount = len;
    ctx->outgoingPacketCount = TUNNEL_Shim_GetNumPacketsRequired(ctx, len);
    ctx->outgoingData = response;

    if(ctx->outgoingPacketCount > TUNNEL_SHIM_MAX_PACKET_COUNT) {
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 1)
        uart_debug_sendline("Response too long to send.\n");
#endif
        return 0;
    }





    if(ctx->funcs->transmit != NULL) {
        if(!(ctx->isTransmitting)) {
            ctx->isTransmitting = true;
            ctx->state = TUNNEL_SHIM_TRANSMIT_ACK_WAIT;
            TUNNEL_Shim_PrepInitPacket(ctx, ctx->packetBuffer);
            ctx->funcs->transmit((uint8_t*) ctx->packetBuffer, ctx->packetByteLen);
            setBitInMap(0, ctx->packetsMap, TUNNEL_SHIM_PACKET_BITMAP_LEN);
            ctx->timeoutActive = true;
            ctx->timeoutTicksRemaining = TUNNEL_SHIM_SHORT_TIMEOUT;
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 2)
            uart_debug_sendline("Tunnel Shim Send started from Do Send.\n");
#endif
        } else {//else an immediate transmit is in progress, will get picked up later
            ctx->state = TUNNEL_SHIM_TRANSMIT_INIT;
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 2)
            uart_debug_sendline("Tunnel Shim Send queued for later from Do Send.\n");
#endif
        }
        return len;
    } else {
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 1)
        uart_debug_sendline("Error in tunnel shim send. Send function NULL.\n");
#endif
        return 0;
    }
}

/** sends an error response as an immediate
 */
uint16_t TUNNEL_Shim_SendImmediateError(TunnelShimContext* ctx, uint8_t errorCode, uint8_t channel) {
    //static uint8_t code = 0;
    //code = errorCode;
    return TUNNEL_Shim_SendImmediate(ctx, &errorCode, 1, TUNNEL_SHIM_COMMAND_ERROR, channel);
}

/**
 * SendImmediate. sends an immediate response to the host, taking priority over the 'normal' send routine and able to inject a single packet
 * theoretically able to saturate the bus with requests
 * designed to be called from the packet receive routine as called by the USB Interrupt. might have concurrency issues if called from another context
 */
uint16_t TUNNEL_Shim_SendImmediate(TunnelShimContext* ctx, uint8_t* immediateData, uint16_t len, uint8_t command, uint8_t channel) {
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 2)
    uart_debug_sendstring("Send Immediate: Command ");
    uart_debug_hexprint8(command);
    uart_debug_sendstring(" on Channel ");
    uart_debug_hexprint8(channel);
    uart_debug_newline();
#endif
    /*if(len > 0) {
        uart_debug_hexdump(immediateData, len);
        uart_debug_newline();
    }*/
    if(len > (ctx->immediateBufferByteLen - INITIATION_PACKET_OVERHEAD)) {
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 0)
        uart_debug_sendline("Immediate packet too long to send. Must fit in one packet.\n");
#endif
        return 0;
    }



    if(!(ctx->isTransmitting)) {
        //if we're not transmitting, just send
        ctx->packetBuffer->channel = TUNNEL_SHIM_ID_INITIATION | channel;
        ctx->packetBuffer->initiation.command = command;
        ctx->packetBuffer->initiation.byteCountHigh = (len & 0xFF00) >> 8;
        ctx->packetBuffer->initiation.byteCountLow = len & 0x00FF;
        if((len == 0) || (immediateData == NULL)) {
            //no data, just fill with zeroes
            memset(ctx->packetBuffer->initiation.data, 0, ctx->packetByteLen - INITIATION_PACKET_OVERHEAD);
        } else {
            memcpy(ctx->packetBuffer->initiation.data, immediateData, len);
            uint16_t fillLength = (ctx->packetByteLen - INITIATION_PACKET_OVERHEAD) - len;
            if(fillLength) {
                memset(ctx->packetBuffer->initiation.data + len, 0, fillLength);
            }
        }
        ctx->isTransmitting = true;
        ctx->funcs->transmit((uint8_t*) ctx->packetBuffer, ctx->packetByteLen);
    } else {
        //is transmitting, add our immediate in to the immediate buffer if we can
        if(ctx->immediatePacketBufferRemaining == 0) {
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 1)
            uart_debug_sendline("Immediate packet cannot be sent. Buffers full.\n");
#endif
            return 0;
        }
        //pack our data in to it
        ctx->immediatePacketBuffers[ctx->immediatePacketBufferHead]->channel = TUNNEL_SHIM_ID_INITIATION | channel;
        ctx->immediatePacketBuffers[ctx->immediatePacketBufferHead]->initiation.command = command;
        ctx->immediatePacketBuffers[ctx->immediatePacketBufferHead]->initiation.byteCountHigh = (len & 0xFF00) >> 8;
        ctx->immediatePacketBuffers[ctx->immediatePacketBufferHead]->initiation.byteCountLow = len & 0x00FF;
        if((len == 0) || (immediateData == NULL)) {
            //no data, just fill with zeroes
            memset(ctx->immediatePacketBuffers[ctx->immediatePacketBufferHead]->initiation.data, 0, ctx->immediateBufferByteLen - INITIATION_PACKET_OVERHEAD);
        } else {
            memcpy(ctx->immediatePacketBuffers[ctx->immediatePacketBufferHead]->initiation.data, immediateData, len);
            uint16_t fillLength = (ctx->immediateBufferByteLen - INITIATION_PACKET_OVERHEAD) - len;
            if(fillLength) {
                memset(ctx->immediatePacketBuffers[ctx->immediatePacketBufferHead]->initiation.data + len, 0, fillLength);
            }
        }

        ctx->immediatePacketBufferRemaining--;
        if(++ctx->immediatePacketBufferHead >= TUNNEL_SHIM_NUM_IMM_BUFFERS) {
            ctx->immediatePacketBufferHead = 0;
        }
    } //else will get picked up in the transmit complete callback
    return len;
}

//call this from the sysTick to handle the receive timeout
//short and sweet. Call TUNNEL_HID_CheckTimeout() from the main task to actually handle the timeout
void TUNNEL_Shim_DoTick(TunnelShimContext* ctx) {
    if(ctx == NULL || ctx->state == TUNNEL_SHIM_UNCONFIGURED) {
        return;
    }
    if(ctx->funcs->disableInterrupt != NULL) {
        ctx->funcs->disableInterrupt();
    }

    if(ctx->timeoutActive && (ctx->timeoutTicksRemaining > 0)) {
        --ctx->timeoutTicksRemaining;
    }
    //finished critical section, re-enable interrupt
    if(ctx->funcs->enableInterrupt != NULL) {
        ctx->funcs->enableInterrupt();
    }
}

/*
 * resets the internals of a specific transaction but none of the long-running state
 * any interrupts that may modify the struct should be disabled before calling this
 */
void TUNNEL_Shim_ResetTransaction(TunnelShimContext *ctx) {
    ctx->commandInFlight = TUNNEL_SHIM_COMMAND_NONE;
    ctx->currentChannel = TUNNEL_SHIM_CHANNEL_RESERVED;
    ctx->previousChannel = TUNNEL_SHIM_CHANNEL_RESERVED;

    ctx->dataBufferHead = 0;

    ctx->timeoutActive = false;
    ctx->timeoutTicksRemaining = 0;

    ctx->numBytesExpected = 0;
    ctx->outgoingByteCount = 0;
    ctx->outgoingData = NULL;
    ctx->outgoingPacketCount = 0;

    ctx->state = TUNNEL_SHIM_READY;
}

/**
 * CheckTimeout
 * call from the main function to check if there's been a timeout and handle it appropriately
 * as the tunnel should be one of the main sources of long-running tasks and we don't have to worry about timeouts when running a long task from ourselves
 * this should be fine
 */
void TUNNEL_Shim_CheckTimeout(TunnelShimContext* ctx) {
    if(ctx == NULL || ctx->state == TUNNEL_SHIM_UNCONFIGURED) {
        return;
    }
    if(ctx->funcs->disableInterrupt != NULL) {
        ctx->funcs->disableInterrupt();
    }

    if(ctx->timeoutActive && (ctx->timeoutTicksRemaining == 0)) {
        if((ctx->state == TUNNEL_SHIM_TRANSMIT_ACK_WAIT) && (ctx->ackWaitCountRemaining != 0)) {
            //packet *should* still be in the output buffer, but don't trust it!
            ctx->ackWaitCountRemaining--; //decrement this




            if(ctx->funcs->transmit != NULL) {
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 1)
                uart_debug_sendstring("Tunnel Shim Ack Wait Timeout retransmitting ");
#endif
                if(!(ctx->isTransmitting)) {
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 1)
                    uart_debug_sendline("now.\n");
#endif
                    ctx->isTransmitting = true;
                    ctx->state = TUNNEL_SHIM_TRANSMIT_ACK_WAIT;
                    TUNNEL_Shim_PrepInitPacket(ctx, ctx->packetBuffer);
                    ctx->funcs->transmit((uint8_t*) ctx->packetBuffer, ctx->packetByteLen);
                    setBitInMap(0, ctx->packetsMap, TUNNEL_SHIM_PACKET_BITMAP_LEN);
                    ctx->timeoutActive = true;
                    ctx->timeoutTicksRemaining = TUNNEL_SHIM_SHORT_TIMEOUT;
                    ctx->ackWaitCountRemaining = TUNNEL_SHIM_ACK_WAIT_MAX_COUNT;
                } else { //else an immediate transmit is in progress, will get picked up later
                    ctx->state = TUNNEL_SHIM_TRANSMIT_INIT;
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 1)
                    uart_debug_sendline("delayed due to immediate.\n");
#endif
                }
            } else {
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 0)
                uart_debug_sendline("Error in tunnel shim init timeout callback. Send function NULL.\n");
#endif
            }
        } else {
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 1)
            uart_debug_sendstring("Tunnel Shim Global Time Out. Resetting.\n");
#endif
            TUNNEL_Shim_SendImmediateError(ctx, TUNNEL_SHIM_ERROR_TIMED_OUT, ctx->currentChannel);
            TUNNEL_Shim_ResetTransaction(ctx);
        }
    }
    if(ctx->funcs->enableInterrupt != NULL) {
        ctx->funcs->enableInterrupt();
    }
}
/**
 * resets the internal state
 * do not use while something is going on and expect it to work or have any consistency
 */
void TUNNEL_Shim_Reset(TunnelShimContext* ctx) {
    if(ctx == NULL) {
        //can't do anything with a null context
        return;
    }
    if(ctx->funcs->disableInterrupt != NULL) {
        ctx->funcs->disableInterrupt();
    }
    TUNNEL_Shim_ResetTransaction(ctx);
    memset(ctx->channelsAllocatedMap, 0, TUNNEL_SHIM_CHANNEL_BITMAP_LEN);
    setBitInMap(TUNNEL_SHIM_CHANNEL_RESERVED, ctx->channelsAllocatedMap, TUNNEL_SHIM_CHANNEL_BITMAP_LEN); //"allocate" the reserved and broadcast channels
    setBitInMap(TUNNEL_SHIM_CHANNEL_BROADCAST, ctx->channelsAllocatedMap, TUNNEL_SHIM_CHANNEL_BITMAP_LEN);

    //run the disconnect callback if its set to do any other necessary cleanup
    if(ctx->funcs->disconnectCallback != NULL) {
        ctx->funcs->disconnectCallback();
    }

    ctx->state = TUNNEL_SHIM_READY;
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 1)
    uart_debug_sendline("Tunnel Shim Layer Reset.\n");
#endif
    if(ctx->funcs->enableInterrupt != NULL) {
        ctx->funcs->enableInterrupt();
    }

}

/**
 * interrupts that affect the state of the context should be disabled before calling this function
 *
 */
/* 2017-09-27: moved in to the single TUNNEL_Shim_CheckTimeout function
void TUNNEL_Shim_TransmitInitTimeoutCallback(TunnelShimContext* ctx) {
    //start critical section, disable interrupt

    if(ctx != NULL && ctx->state != TUNNEL_SHIM_TRANSMIT_ACK_WAIT) {
        //context is null OR we're not in the ack wait state
        return;
    }
    if(ctx->funcs->disableInterrupt != NULL) {
        ctx->funcs->disableInterrupt();
    }

    //packet *should* still be in the output buffer, but don't trust it!
    TUNNEL_Shim_PrepInitPacket(ctx);

    ctx->state = TUNNEL_SHIM_TRANSMIT_INIT;

    if(ctx->funcs->transmit != NULL) {
        if(!(ctx->isTransmitting)) {
            ctx->isTransmitting = true;
            ctx->funcs->transmit((uint8_t*) ctx->packetBuffer, ctx->packetByteLen);
            setBitInMap(0, ctx->packetsMap, TUNNEL_SHIM_PACKET_BITMAP_LEN);
            ctx->timeoutActive = true;
            ctx->timeoutTicksRemaining = TUNNEL_SHIM_SHORT_TIMEOUT;
        } //else an immediate transmit is in progress, will get picked up later
    } else {
        uart_debug_sendline("Error in tunnel shim init timeout callback. Send function NULL.\n");
    }

    if(ctx->funcs->enableInterrupt != NULL) {
        ctx->funcs->enableInterrupt();
    }

}
*/


/*
 * internal helper to grab the next normal packet for transmit, if it exists
 */
bool TUNNEL_Shim_GetNextContinuationPacket(TunnelShimContext* ctx, TunnelShimPacket* outgoingPacket) {
    if((ctx == NULL) || (ctx->state != TUNNEL_SHIM_TRANSMIT)) {
        //*outgoingPacket = NULL;
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 1)
        uart_debug_sendline("GetNextContinuationPacket: Not in transmit state or null context.\n");
#endif
        return false;
    }
    uint32_t nextPacketNum32 = getLowestUnsetBitInMap(ctx->packetsMap, TUNNEL_SHIM_PACKET_BITMAP_LEN);
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 2)
    uart_debug_sendstring("GetNextContinuationPacket looking at packet: ");
    uart_debug_printuint32(nextPacketNum32);
    uart_debug_newline();
#endif
    if(nextPacketNum32 < ctx->outgoingPacketCount) {
        uint8_t nextPacketNum = nextPacketNum32 & 0xFF;
        outgoingPacket->channel = TUNNEL_SHIM_ID_CONTINUATION | ctx->currentChannel;
        outgoingPacket->continuation.sequence = nextPacketNum;
        uint16_t dataPosition = ctx->packetByteLen - INITIATION_PACKET_OVERHEAD;
        dataPosition += (ctx->packetByteLen - CONTINUATION_PACKET_OVERHEAD) * (outgoingPacket->continuation.sequence - 1);
        uint16_t packetValidLen = ctx->outgoingByteCount - dataPosition;
        if(packetValidLen >= ctx->packetByteLen) {
            //full packet
            memcpy(outgoingPacket->continuation.data, ctx->outgoingData + dataPosition, ctx->packetByteLen - CONTINUATION_PACKET_OVERHEAD);
        } else if (packetValidLen > 0) {
            //non-full packet
            memcpy(outgoingPacket->continuation.data, ctx->outgoingData + dataPosition, packetValidLen);
            uint16_t bytesRemaining = ctx->packetByteLen - packetValidLen - CONTINUATION_PACKET_OVERHEAD;
            memset(outgoingPacket->continuation.data + packetValidLen, 0, bytesRemaining);
        } else {
            //empty packet. should not be here as this should be picked up first.
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 1)
            uart_debug_sendline("Attempted to send empty packet in tunnel shim layer.\n");
#endif
            ctx->state = TUNNEL_SHIM_TRANSMIT_COMPLETE;
            //*outgoingPacket = NULL;
            return false;
        }
        setBitInMap(nextPacketNum, ctx->packetsMap, TUNNEL_SHIM_PACKET_BITMAP_LEN);
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 2)
        uart_debug_sendstring("GetNextContinuationPacket: Sending Packet #");
        uart_debug_printuint32(nextPacketNum32);
        uart_debug_newline();
#endif
        return true;
    } else {
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 2)
        uart_debug_sendline("GetNextContinuationPacket: Transmit Complete.\n");
#endif
        ctx->state = TUNNEL_SHIM_TRANSMIT_COMPLETE;
        //*outgoingPacket = NULL;
        return false;
    }

}

/*
 * get the next packet for transmit. call this from the transmit callback to grab the next packet
 *
 * @param ctx the context to get the packet from, must not be null
 * @param outgoingPacket (out parameter) pointer to a TunnelShimPacket, contents will be updated to the next packet to send if true is returned
 * @return true if outgoingPacket was updated, false otherwise
 */
bool TUNNEL_Shim_GetNextPacket(TunnelShimContext* ctx, TunnelShimPacket* outgoingPacket) {
    if(ctx == NULL) {
        //*outgoingPacket = NULL;
        return false;
    }
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 2)
    uart_debug_sendline("Tunnel Shim GetNextPacket Called.\n");
#endif
    bool packetReady = false;
    //if(ctx->immState != TUNNEL_SHIM_IMM_IDLE) {
    if(ctx->immediatePacketBufferRemaining != TUNNEL_SHIM_NUM_IMM_BUFFERS) {
        //something in the immediate buffers, send it out
        outgoingPacket->channel = ctx->immediatePacketBuffers[ctx->immediatePacketBufferTail]->channel;
        outgoingPacket->initiation.command = ctx->immediatePacketBuffers[ctx->immediatePacketBufferTail]->initiation.command;
        outgoingPacket->initiation.byteCountHigh = ctx->immediatePacketBuffers[ctx->immediatePacketBufferTail]->initiation.byteCountHigh;
        outgoingPacket->initiation.byteCountLow = ctx->immediatePacketBuffers[ctx->immediatePacketBufferTail]->initiation.byteCountLow;
        memcpy(outgoingPacket->initiation.data, ctx->immediatePacketBuffers[ctx->immediatePacketBufferTail]->initiation.data, ctx->immediateBufferByteLen - INITIATION_PACKET_OVERHEAD);
        if(ctx->packetByteLen > ctx->immediateBufferByteLen) {
            //if our packets are longer than an immediate (which can be size-limited)
            //zero out the rest of the packetBuffer before sending
            //uint16_t bytesRemaining = ctx->packetByteLen - ctx->immediatePacketByteLen;
            memset(outgoingPacket->initiation.data + (ctx->immediateBufferByteLen - INITIATION_PACKET_OVERHEAD), 0, ctx->packetByteLen - ctx->immediateBufferByteLen);
        }
        ctx->immediatePacketBufferTail++;
        if(ctx->immediatePacketBufferTail > TUNNEL_SHIM_NUM_IMM_BUFFERS) {
            ctx->immediatePacketBufferTail = 0;
        }
        ctx->immediatePacketBufferRemaining++;
        //*outgoingPacket = ctx->packetBuffer;
        packetReady = true;
    } else if (ctx->state == TUNNEL_SHIM_TRANSMIT_INIT) {
        TUNNEL_Shim_PrepInitPacket(ctx, outgoingPacket);
        setBitInMap(0, ctx->packetsMap, TUNNEL_SHIM_PACKET_BITMAP_LEN);
        //*outgoingPacket = ctx->packetBuffer;
        ctx->state = TUNNEL_SHIM_TRANSMIT_ACK_WAIT;
        ctx->timeoutActive = true;
        ctx->timeoutTicksRemaining = TUNNEL_SHIM_SHORT_TIMEOUT;
        ctx->ackWaitCountRemaining = TUNNEL_SHIM_ACK_WAIT_MAX_COUNT;
        packetReady = true;
    } else if (ctx->state == TUNNEL_SHIM_TRANSMIT ) {
        packetReady = TUNNEL_Shim_GetNextContinuationPacket(ctx, outgoingPacket);
    } else {
        //*outgoingPacket = NULL;
        packetReady = false;
    }
    ctx->isTransmitting = packetReady;
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 2)
    uart_debug_sendstring("Tunnel Shim GetNextPacket Finished. Sending: ");
    uart_debug_printBool(packetReady);
    uart_debug_newline();
#endif
    return packetReady;

}
