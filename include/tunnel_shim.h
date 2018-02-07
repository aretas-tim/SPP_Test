/*
 * tunnel_shim.h
 *
 *  Created on: Jun 20, 2017
 *      Author: me
 */

#ifndef TUNNEL_SHIM_H_
#define TUNNEL_SHIM_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h> /* size_t not defined?*/

#define TUNNEL_SHIM_VERBOSE 0 /* verbosity level of debug stuff */

#define TUNNEL_SHIM_MAX_SEQUENCE_NUM 0xFF
#define TUNNEL_SHIM_MAX_PACKET_COUNT (TUNNEL_SHIM_MAX_SEQUENCE_NUM + 1) /* for the initiation packet */
#define TUNNEL_SHIM_PACKET_BITMAP_LEN (TUNNEL_SHIM_MAX_PACKET_COUNT >> 3)

#define INITIATION_PACKET_OVERHEAD 4
#define CONTINUATION_PACKET_OVERHEAD 2

/* this is exactly where some polymorphism would come in really, really handy */

#define TUNNEL_SHIM_USB_FS_REPORT_SIZE 64
#define TUNNEL_SHIM_USB_FS_INIT_DATA_SIZE (TUNNEL_SHIM_USB_FS_REPORT_SIZE - INITIATION_PACKET_OVERHEAD)
#define TUNNEL_SHIM_USB_FS_CONT_DATA_SIZE (TUNNEL_SHIM_USB_FS_REPORT_SIZE - CONTINUATION_PACKET_OVERHEAD)

#define TUNNEL_SHIM_USB_HS_REPORT_SIZE 1024
#define TUNNEL_SHIM_USB_HS_INIT_DATA_SIZE (TUNNEL_SHIM_USB_HS_REPORT_SIZE - INITIATION_PACKET_OVERHEAD)
#define TUNNEL_SHIM_USB_HS_CONT_DATA_SIZE (TUNNEL_SHIM_USB_HS_REPORT_SIZE - CONTINUATION_PACKET_OVERHEAD)

#define TUNNEL_SHIM_BT_REPORT_SIZE 20
#define TUNNEL_SHIM_BT_INIT_DATA_SIZE (TUNNEL_SHIM_BT_REPORT_SIZE - INITIATION_PACKET_OVERHEAD)
#define TUNNEL_SHIM_BT_CONT_DATA_SIZE (TUNNEL_SHIM_BT_REPORT_SIZE - CONTINUATION_PACKET_OVERHEAD)

#define TUNNEL_SHIM_CHANNEL_COUNT 128
#define TUNNEL_SHIM_CHANNEL_BITMAP_LEN (TUNNEL_SHIM_CHANNEL_COUNT >> 3)

#define TUNNEL_SHIM_NUM_IMM_BUFFERS 2 /* two immediate packets, just in case */
#define TUNNEL_SHIM_MAX_IMM_BUFFER_LEN (INITIATION_PACKET_OVERHEAD + TUNNEL_SHIM_MAX_PACKET_COUNT)

#define TUNNEL_SHIM_MIN_BUFFER_SPACE 4606 /* based on the BT maximum buffer data length */

typedef enum tdTunnelShimPacketState {
    TUNNEL_SHIM_PACKET_ERROR,
    TUNNEL_SHIM_PACKET_IDLE,
    TUNNEL_SHIM_PACKET_TRANSMITTED,
    TUNNEL_SHIM_PACKET_RECEIVED
} TunnelShimPacketState;

typedef struct tdTunnelShimPacket {
    union {
        uint8_t channel;
        uint8_t id;
    };
    union {
        struct {
            uint8_t command;
            uint8_t byteCountLow;
            uint8_t byteCountHigh;
            uint8_t data[0]; /* data buffer, 0 means flexible length */
        } initiation;
        struct {
            uint8_t sequence;
            uint8_t data[0]; /* data buffer, 0 for flexible length */
        } continuation;
    };
} TunnelShimPacket;




typedef enum tdTunnelShimState {
    TUNNEL_SHIM_UNCONFIGURED,
    TUNNEL_SHIM_READY,
    TUNNEL_SHIM_RECEIVE,
    TUNNEL_SHIM_MESSAGE_WAITING,
    TUNNEL_SHIM_PROCESSING,
    TUNNEL_SHIM_TRANSMIT_INIT, /* send first packet, transition to ACK_WAIT */
    TUNNEL_SHIM_TRANSMIT_ACK_WAIT, /*waiting for a transmit acknowledge from the host */
    TUNNEL_SHIM_TRANSMIT,
    TUNNEL_SHIM_TRANSMIT_COMPLETE
} TunnelShimHandlerState;

typedef enum tdTunnelShimImmediateState {
    TUNNEL_SHIM_IMM_IDLE,  /*no immediate */
    TUNNEL_SHIM_IMM_PENDING, /* send from the immediate buffer */
    TUNNEL_SHIM_IMM_SENDING, /* something is being sent */
    TUNNEL_SHIM_IMM_SENDING_WITH_PENDING, /* something is being sent and we have something waiting */
    TUNNEL_SHIM_IMM_FULL /* two pending sends */
} TunnelShimImmediateState;

typedef struct tdTunnelShimPacketFunctions {
    uint16_t (*transmit)(uint8_t*, uint16_t); //function called from the shim to initiate data transfer, further packets are sent via callback to TUNNEL_Shim_GetNextTransmitPacket(...)
    void (*disconnectCallback)(void); //will be called when the shim layer is disconnected, can be NULL
    //bool (*transmitInProgress)(void); //return true there's an ongoing transmit
    void (*disableInterrupt)(void); //called to disable any interrupts to perform internal functions that should not be interrupted
    void (*enableInterrupt)(void); //called to enable the interrupts that were disabled by disableInterrupts
} TunnelShimPacketFunctions;

typedef struct tdTunnelShimContext {
    TunnelShimHandlerState state;
    //TunnelShimImmediateState immState;
    bool isTransmitting;

    //volatile size_t TUNNEL_HID_DataOutExpected; //how much data we expect to receive
    uint16_t numBytesExpected; //how much data we expect to receive
    uint8_t numPacketsExpected; //how many packets that data should be in


    uint8_t outgoingPacketCount; //how many packets we're transmitting in this sequence
    uint16_t outgoingByteCount; //how many bytes of valid data to send
    uint8_t* outgoingData;
    uint8_t packetsMap[TUNNEL_SHIM_PACKET_BITMAP_LEN]; //bitmap, if we've received or transmitted a given packet (depending on state)

    uint8_t channelsAllocatedMap[TUNNEL_SHIM_CHANNEL_BITMAP_LEN]; //how many channels we've a
    uint8_t commandInFlight; //what we're receiving at the moment

    uint32_t timeoutTicksRemaining; //downcounter, how many ticks accumulated before we timeout, used for global only
    bool timeoutActive; //if we're even worried about the timeout at all
    uint8_t ackWaitCountRemaining; //how many resends until we give up

    uint8_t currentChannel; //the current channel we're operating on
    uint8_t previousChannel; //last channel, used for retransmit requests and to do the transmit complete

    uint16_t packetByteLen; /* total count of byte length of packet. FS USB is 64, HS USB is 1024, BT is currently 20, etc */
    uint16_t dataBufferLen; /* some implementations may not use the full sequence number space, HS USB for instance */
    uint16_t dataBufferHead; //write to the head
    uint8_t* dataBuffer;

    TunnelShimPacket* packetBuffer; /* a single packet */
    TunnelShimPacket* immediatePacketBuffers[TUNNEL_SHIM_NUM_IMM_BUFFERS]; /* spot to hold two packets of "immediate" data that has priority over normal data */

    //uint8_t numImmediatePacketBuffers; /* how many immediate packets we have in the buffer (now fixed at 2)*/
    uint8_t immediatePacketBufferHead; /*write to the head */
    uint8_t immediatePacketBufferTail; /* send from the tail */
    uint8_t immediatePacketBufferRemaining; /* how many slots left in our buffer */
    uint16_t immediateBufferByteLen; /* how many bytes the immediate buffers hold, if not packetByteLen */

    TunnelShimPacketFunctions* funcs; /* function pointer struannot convert to a pointer typect */
} TunnelShimContext;

#define TUNNEL_SHIM_ID_MASK 0x80
#define TUNNEL_SHIM_ID_INITIATION 0x80
#define TUNNEL_SHIM_ID_CONTINUATION 0x00

#define TUNNEL_SHIM_COMMAND_NONE 0x00
#define TUNNEL_SHIM_COMMAND_MESSAGE 0x01 //payload is a tunnel command or response (depending on direction)
#define TUNNEL_SHIM_COMMAND_PING 0x02 //payload should be received then immediately turned around
#define TUNNEL_SHIM_COMMAND_INIT 0x03 //initiaties a channel
#define TUNNEL_SHIM_COMMAND_RELEASE 0x04 //releases a channel
#define TUNNEL_SHIM_COMMAND_COMPLETE_ACKNOWLEDGE 0x7A //tells the host we've acknowledged the transmit complete
#define TUNNEL_SHIM_COMMAND_COMPLETE 0x7B //signals that the host has received the response correctly
#define TUNNEL_SHIM_COMMAND_ACKNOWLEDGE 0x7C //acknowledges the initiation packet of a message packet OR a multi-packet reception
#define TUNNEL_SHIM_COMMAND_RESYNCH 0x7D //resynchs the channel
#define TUNNEL_SHIM_COMMAND_RETRANSMIT 0x7E //requests retransmission
#define TUNNEL_SHIM_COMMAND_ERROR 0x7F //error code in data segment



#define TUNNEL_SHIM_CHANNEL_RESERVED 0x00
#define TUNNEL_SHIM_CHANNEL_BROADCAST (TUNNEL_SHIM_CHANNEL_COUNT - 1)
#define TUNNEL_SHIM_MAX_CHANNEL_NUMBER (TUNNEL_SHIM_CHANNEL_BROADCAST - 1)
#define TUNNEL_SHIM_CHANNEL_MASK 0x7F

#define TUNNEL_SHIM_TIMEOUT_TRANSMIT_GLOBAL 20000 /* milliseconds for a complete transaction before giving up (global timeout, based on systick */
#define TUNNEL_SHIM_TIMEOUT_TRANSMIT_INIT 50 /* 100us increments to wait for acknowledgement of initiation packet reception before retrying, based on internal timer */

#define TUNNEL_SHIM_INIT_NONCE_LEN 8

#define TUNNEL_SHIM_ERROR_NONE 0x00
#define TUNNEL_SHIM_ERROR_CHANNEL_BUSY 0x01
#define TUNNEL_SHIM_ERROR_RELEASE_IMPOSSIBLE 0x02
#define TUNNEL_SHIM_ERROR_OUT_OF_MEMORY 0x03
#define TUNNEL_SHIM_ERROR_UNKNOWN_COMMAND 0x04
#define TUNNEL_SHIM_ERROR_FAILED 0x05
#define TUNNEL_SHIM_ERROR_RESYNCH_IMPOSSIBLE 0x06
#define TUNNEL_SHIM_ERROR_TIMED_OUT 0x07

/* internal error codes */
#define TUNNEL_SHIM_CONTEXT_SUCCESS 0
#define TUNNEL_SHIM_CONTEXT_ERROR_PACKETS_TOO_SHORT 0x01
#define TUNNEL_SHIM_CONTEXT_ERROR_NOT_ENOUGH_BUFFER_SPACE 0x02
#define TUNNEL_SHIM_CONTEXT_ERROR_NO_CONTEXT 0x03
#define TUNNEL_SHIM_CONTEXT_ERROR_NO_DATA 0x04
#define TUNNEL_SHIM_CONTEXT_ERROR_NOT_CONFIGURED 0x05
#define TUNNEL_SHIM_CONTEXT_ERROR_NO_TX_FUNC 0x06

#define BITMAP_NO_BIT_FOUND 0xFFFFFFFF

#define TUNNEL_SHIM_SHORT_TIMEOUT 50
#define TUNNEL_SHIM_LONG_TIMEOUT 1000

#define TUNNEL_SHIM_ACK_WAIT_MAX_COUNT 10 /* how many times we'll resend an initiation packet */

uint8_t TUNNEL_Shim_InitContext(TunnelShimContext* ctx, uint8_t* buffer, uint16_t bufferLen, uint16_t packetLen, TunnelShimPacketFunctions* funcs);
uint8_t TUNNEL_Shim_DeInit(TunnelShimContext* ctx);

//functions used by the lower layer (should be called from an interrupt context if operating via interrupts
bool TUNNEL_Shim_GetNextPacket(TunnelShimContext* ctx, TunnelShimPacket* outgoingPacket);
uint8_t TUNNEL_Shim_RecievePacket(TunnelShimContext* ctx, uint8_t* packetBytes, uint16_t packetLen);

//functions used by the upper layer (interrupt(s) used will be disabled for the duration of the call)
uint16_t TUNNEL_Shim_SendResponse(TunnelShimContext* ctx, uint8_t* response, uint16_t len);
void TUNNEL_Shim_CheckTimeout(TunnelShimContext* ctx);
void TUNNEL_Shim_DoTick(TunnelShimContext* ctx);
size_t TUNNEL_Shim_GetMessage(TunnelShimContext* ctx, uint8_t** message, uint8_t* channel);
size_t TUNNEL_Shim_MessageAvailable(TunnelShimContext* ctx);
void TUNNEL_Shim_Reset(TunnelShimContext* ctx);

#endif /* TUNNEL_SHIM_H_ */
