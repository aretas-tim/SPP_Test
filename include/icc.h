/*
 * icc.h
 *
 *  Created on: Mar 17, 2017
 *      Author: me
 */

#ifndef ICC_H_
#define ICC_H_

#include <stdbool.h>
#include <stdint.h>
#include "hotkey.h"

/* intercontroller communication header */

#ifdef DEBUG
#define DEBUG_ICC
#endif /* DEBUG */

#define ICC_HEADER_LENGTH 4
#define ICC_HEADER_TARGET_POS 0
#define ICC_HEADER_COMMAND_POS 1 /* to squeeze in a command, only used for ICC internals at the moment but hopefully reduces SPI traffic overall*/
#define ICC_HEADER_LENGTH_LSB_POS 2 /* little-endian*/
#define ICC_HEADER_LENGTH_MSB_POS 3
#define ICC_TARGET_DIR_MASK 0x80
#define ICC_TARGET_NUMBER_MASK 0x7F
#define ICC_HEADER_TARGET_IN 0x00 /* inwards data, secure -> ICC */
#define ICC_HEADER_TARGET_OUT 0x80 /* outwards data,  ICC -> secure */
#define ICC_TARGET_ICC 0x00
#define ICC_TARGET_LEDS 0x01
#define ICC_TARGET_TUNNEL 0x10
#define ICC_TARGET_U2F 0x11
#define ICC_TARGET_HOTKEY 0x12
#define ICC_TARGET_NULL 0x7F /* null target */
#define ICC_NUM_TARGETS 5 /* ICC in/out, LEDs in, Tunnel in/out, U2F in/out, Hotkey in, echo in/out */
#define ICC_TARGET_LIST_ITEM_LENGTH 5 /* bytes per target in the list, currently one for the target number, two for available/potential transfer length in the in/out direction */
#define ICC_TARGET_INFO_BUFFER_LENGTH (ICC_NUM_TARGETS * ICC_TARGET_LIST_ITEM_LENGTH + 1) /* +1 for our number of targets */
#define ICC_TARGET_LIST_TARGET_POS 0
#define ICC_TARGET_LIST_IN_LSB_POS 1
#define ICC_TARGET_LIST_IN_MSB_POS 2
#define ICC_TARGET_LIST_OUT_LSB_POS 3
#define ICC_TARGET_LIST_OUT_MSB_POS 4
#define ICC_COMMAND_BUFFER_LENGTH 1024 /* should be as long as the longest valid command and parameters. 2018-02-01 upped this to 1k since echos now happen via command interface */
#define ICC_COMMAND_EXTENDED 0x00 /* command and parameters in data section, not immediate*/
#define ICC_COMMAND_LIST_TARGETS 0x01 /* gets a list of valid targets and data waiting or space available */
#define ICC_COMMAND_GET_MAXIMUMS 0x02 /* gets the maximums from the USB micro, should only need to be called once */
#define ICC_COMMAND_SET_SHUTDOWN 0x11 /* puts the USB Micro in to deep sleep, must set wakeup pin to wake */
#define ICC_COMMAND_SET_LOW_POWER 0x12 /* set low power mode (no USB, no mass storage) */
#define ICC_COMMAND_SET_FULL_POWER 0x13 /* sets to full power mode (USB active, mass storage active) */
#define ICC_COMMAND_GET_POWER_MODE 0x10 /* gets the power mode */
#define ICC_COMMAND_USB_DISABLE 0x20 /* tell the USB micro to disconnect itself from the USB */
#define ICC_COMMAND_USB_ENABLE 0x21 /* tells the USB micro it can reconnect to USB, if otherwise able */
#define ICC_COMMAND_LAST_XFER_STATUS 0x80 /* gets the status of the most recent transfer, not including calls to this command, bitwise OR in the requested endpoint to the lower 7 bits */
#define ICC_LAST_TRANSFER_INFO_LEN 4
#define ICC_LAST_TRANSFER_INFO_TARGET_POS 0
#define ICC_LAST_TRANSFER_INFO_VALID_POS 1
#define ICC_LAST_TRANSFER_INFO_LENGTH_LSB_POS 2
#define ICC_LAST_TRANSFER_INFO_LENGTH_MSB_POS 3
#define ICC_POWER_MODE_RESERVED 0 /* so a bad transfer can be detected. 0xFF is also reserved but if we have 254 power modes we've done it wrong */
#define ICC_POWER_MODE_FULL 1
#define ICC_POWER_MODE_LOW 2
#define ICC_POWER_MODE_SHUTDOWN 3 /* must wake via wakeup pin */


typedef enum tdICC_BufferState {
    ICC_BUFFER_STATE_IDLE, /* buffer is idle, no data available */
    ICC_BUFFER_STATE_READY, /* ready to send/receive */
    ICC_BUFFER_STATE_DMA, /* owned by the DMA controller, being sent/received from the secure micro as appropriate to the buffer */
    ICC_BUFFER_STATE_PROCESSING /* in processing by something local */
} Icc_BufferState;

typedef enum tdICC_DMAStateEnum {
    ICC_DMA_ERROR,
    ICC_DMA_HEADER,
    ICC_DMA_IDLE,
    ICC_DMA_RECEIVE,
    ICC_DMA_TRANSMIT
}Icc_DmaStateEnum;

typedef struct tdICC_TransferInfo {
    bool targetValid; /* if the target is valid */
    uint8_t target; /* target, even if invalid */
    uint16_t length; /* transfer length */
} Icc_TransferInfo;

/* remember, for us, tx = out
 * rx = in
 */
typedef struct tdICC_TargetInfo {
    uint8_t number; /* target number without direction bit */
    uint8_t lastTransferDirection; /* direction of the last transfer,  either ICC_HEADER_TARGET_IN or ICC_HEADER_TARGET_OUT */
    uint16_t lastTransferLen; /* number of valid bytes in the last transfer */
    uint8_t* inBuffer; /* in direction buffer, this is local to us, but size may vary */
    uint16_t inBufferLen; /* bytes available in the in buffer */
    uint16_t inBufferLenMax; /* maximum length of the in buffer */
    Icc_BufferState inBufferState; /* state of the in buffer */
    uint8_t* outBuffer; /* out direction buffer, pointer to buffer provided by other subsystem*/
    uint16_t outBufferLen; /* bytes available in the out buffer */
    uint16_t outBufferLenMax; /* maximum length of an outgoing transfer, can be zero */
    Icc_BufferState outBufferState; /* state of the out buffer */
    void (*txCompleteCallback) (void); /* transmit function callback */
    bool (*rxCallback) (uint8_t*, uint16_t); /* receive function callback. return true to keep buffer ownership, false to release buffer on return */
} Icc_TargetInfo;


void Icc_init(SPI_HandleTypeDef* hspi, GPIO_TypeDef* readyPort, uint16_t readyPin, GPIO_TypeDef* IRQPort, uint16_t IRQPin);
bool Icc_setupTarget(uint8_t target, uint8_t* rxBuffer, uint16_t rxBufferLen, bool (*rxCallback)(uint8_t*, uint16_t), uint16_t maxTxLen, void (*txCompleteCallback)(void));
void Icc_start(void);
void Icc_dmaRxCompleteCallback(void);
void Icc_dmaTxCompleteCallback(void);
void Icc_deselectedCallbackHandler(void);


#endif /* ICC_H_ */
