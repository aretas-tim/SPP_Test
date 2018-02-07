/*
 * uart_debug.c
 *
 *  Created on: Feb 2, 2016
 *      Author: me
 *
 *  make sure to call UART_Debug_Init with a valid (and initialized) UART_HandleTypeDef* before calling any other function
 *  call UART_Debug_Deinit to null the reference and disable things.
 *  UART_Debug assumes it has exclusive control over the UART
 *
 */

#ifndef UART_DEBUG_C
#define UART_DEBUG_C

#include "uart_debug.h"
#include "sram2.h"

#define DEBUG_UART_TRANSMIT_BUFFER_LEN SRAM2_UART_DEBUG_TX_BUFF_LEN
#define DEBUG_UART_TEMP_BUFFER_LEN 64 //SRAM2_UART_DEBUG_TEMP_BUFF_LEN /* 16 bytes at 2 chars + one space per byte except the last, which is a newline*/



UART_HandleTypeDef* debugUART = NULL;

/* local functions*/
void uart_debug_transmit(void);

uint8_t* debugUARTBuffer = (uint8_t*) SRAM2_UART_DEBUG_TX_BUFF;
uint8_t debugUARTBufferTemp[DEBUG_UART_TEMP_BUFFER_LEN]; //= (uint8_t*) SRAM2_UART_DEBUG_TEMP_BUFF;
volatile uint32_t debugUARTBufferHead = 0;
volatile uint32_t debugUARTBufferTail = 0;
volatile uint32_t debugUARTBufferTransmitHead = 0;
volatile uint32_t debugUARTBufferRemaining = DEBUG_UART_TRANSMIT_BUFFER_LEN;
volatile int8_t debugUARTBusy = -1;

int8_t UART_Debug_Init(UART_HandleTypeDef* uart) {
    if(NULL == uart) {
        /* why are we being called with an null pointer? */
        return -1;
    }
    debugUART = uart;
    debugUARTBusy = 0;
    return 0;
}

void UART_Debug_Deinit(void) {
    debugUART = NULL;
    if(NULL != debugUARTBuffer) {
        free(debugUARTBuffer);
    }
    if(NULL != debugUARTBufferTemp) {
        free(debugUARTBufferTemp);
    }
    debugUARTBusy = -1;
    debugUARTBufferHead = 0;
    debugUARTBufferTail = 0;
    debugUARTBufferTransmitHead = 0;
    debugUARTBufferRemaining = DEBUG_UART_TRANSMIT_BUFFER_LEN;
}

void uart_debug_printuint8(uint8_t val) {
    debugUARTBufferTemp[0] = (val / 100) + '0';
    val %= 100;
    debugUARTBufferTemp[1] = (val / 10) + '0';
    debugUARTBufferTemp[2] = (val % 10) + '0';
    uart_debug_addToBuffer(debugUARTBufferTemp, 3);
}

void uart_debug_printuint32(uint32_t val) {
    uint32_t pos = DEBUG_UART_TEMP_BUFFER_LEN;
    do {
        debugUARTBufferTemp[--pos] = (val % 10) + '0';
        val /= 10;
    } while(val);
    uart_debug_addToBuffer(debugUARTBufferTemp + pos, DEBUG_UART_TEMP_BUFFER_LEN - pos);
}

void uart_debug_printuint64(uint64_t val) {
    uint64_t pos = DEBUG_UART_TEMP_BUFFER_LEN;
    do {
        debugUARTBufferTemp[--pos] = (val % 10) + '0';
        val /= 10;
    } while(val);
    uart_debug_addToBuffer(debugUARTBufferTemp + pos, DEBUG_UART_TEMP_BUFFER_LEN - pos);
}

void uart_debug_hexprint8(uint8_t val) {
    debugUARTBufferTemp[0] = '0';
    debugUARTBufferTemp[1] = 'x';
    for(int8_t i = 3; i > 1; --i) {
        uint8_t c = (0xF & val);
        if(10 <= c) {
            debugUARTBufferTemp[i] = (c - 10) + 'A';
        } else {
            debugUARTBufferTemp[i] = c + '0';
        }
        val >>= 4;
    }
    uart_debug_addToBuffer(debugUARTBufferTemp, 4);
}

void uart_debug_hexprint16(uint16_t val) {
    debugUARTBufferTemp[0] = '0';
    debugUARTBufferTemp[1] = 'x';
    for(int8_t i = 5; i > 1; --i) {
        uint8_t c = (0xF & val);
        if(10 <= c) {
            debugUARTBufferTemp[i] = (c - 10) + 'A';
        } else {
            debugUARTBufferTemp[i] = c + '0';
        }
        val >>= 4;
    }
    uart_debug_addToBuffer(debugUARTBufferTemp, 6);
}

void uart_debug_hexprint32(uint32_t val) {
    debugUARTBufferTemp[0] = '0';
    debugUARTBufferTemp[1] = 'x';
    for(int8_t i = 9; i > 1; --i) {
        uint8_t c = (0xF & val);
        if(10 <= c) {
            debugUARTBufferTemp[i] = (c - 10) + 'A';
        } else {
            debugUARTBufferTemp[i] = c + '0';
        }
        val >>= 4;
    }
    uart_debug_addToBuffer(debugUARTBufferTemp, 10);
}

void uart_debug_hexprint64(uint64_t val) {
    debugUARTBufferTemp[0] = '0';
    debugUARTBufferTemp[1] = 'x';
    for(int8_t i = 17; i > 1; --i) {
        uint8_t c = (0xF & val);
        if(10 <= c) {
            debugUARTBufferTemp[i] = (c - 10) + 'A';
        } else {
            debugUARTBufferTemp[i] = c + '0';
        }
        val >>= 4;
    }
    uart_debug_addToBuffer(debugUARTBufferTemp, 18);
}

void uart_debug_printBool(uint32_t b) {
    if(0 == b) {
        uart_debug_sendstring("False");
    } else {
        uart_debug_sendstring("True");
    }
}

void uart_debug_hexdump(uint8_t* buff, uint32_t len) {
    if(len > (DEBUG_UART_TRANSMIT_BUFFER_LEN / 3)) {
        len = DEBUG_UART_TRANSMIT_BUFFER_LEN / 3;
    }
    uint32_t blockSize = DEBUG_UART_HEXDUMP_BLOCK_SIZE; // for spaces between bytes
    debugUARTBufferTemp[(DEBUG_UART_HEXDUMP_BLOCK_SIZE * 3) - 1] = '\n'; //preload this
    uint32_t numBlocks = len / blockSize;
    uint32_t lastBlockSize = len % blockSize;
    for(uint32_t k = 0; k < (blockSize - 1); ++k) { //for our inter-byte spaces
        debugUARTBufferTemp[(k * 3) + 2] = ' '; //preload these as well as they won't change
    }
    if(lastBlockSize > 0) {
        numBlocks++;
    } else {
        lastBlockSize = blockSize;
    }
    for(uint32_t i = 0; i < numBlocks; i++) {
        uint32_t blockStart = i * blockSize;
        if(i == (numBlocks - 1)) {
            //last block
            blockSize = lastBlockSize;
        }
        for(uint32_t j = 0; j < blockSize; ++j) {
            uint8_t c1 = (0xF0 & buff[blockStart + j]) >> 4;
            uint8_t c2 = (0x0F & buff[blockStart + j]);
            if(10 <= c1) {
                debugUARTBufferTemp[j * 3] = (c1 - 10) + 'A';
            } else {
                debugUARTBufferTemp[j * 3] = c1 + '0';
            }
            if(10 <= c2) {
                debugUARTBufferTemp[(j * 3) + 1] = (c2 - 10) + 'A';
            } else {
                debugUARTBufferTemp[(j * 3) + 1] = c2 + '0';
            }
        }
        if(i != (numBlocks - 1)) {//if we're not on the last block
            uart_debug_addToBuffer(debugUARTBufferTemp, (blockSize * 3));
        }
        else { //on the last block, ignore the trailing space/newline
            uart_debug_addToBuffer(debugUARTBufferTemp, (blockSize * 3) - 1);
        }
        //HAL_Delay(50); //should rate-limit it enough to not overflow the buffer ever at 57600. disabled for now as it was causing issues as i was calling from an interrupt
    }
    uart_debug_newline(); //add final newline
}

/* puts a newline out cause i'm lazy*/
void uart_debug_newline(void) {
    uint8_t newlineChar = '\n';
    uart_debug_addToBuffer(&newlineChar, 1);
}

/* puts a newline-terminated character string on the wire (limited to the transmit buffer length for sanity) */
void uart_debug_sendline(char* str) {
    uint32_t len = 0;
    for(; len < DEBUG_UART_TRANSMIT_BUFFER_LEN; ++len) {
        if(str[len] == '\n') {
            break;
        }
    }
    uart_debug_addToBuffer((uint8_t*) str, len + 1); /* to account for the newline char*/
}

/* puts a null-terminated character string on the wire (limited to the transmit buffer length for sanity) */
void uart_debug_sendstring(char* str) {
    uint32_t len = 0;
    for(; len < DEBUG_UART_TRANSMIT_BUFFER_LEN; ++len) {
        if(str[len] == '\0') {
            break;
        }
    }
    uart_debug_addToBuffer((uint8_t*) str, len);
}


/* adds stuff to the debug buffer to be sent out.
 * returns the number of bytes added to the buffer
 * if the amount of data is too big, does not add the bit that won't fit in the buffer
 */
uint32_t uart_debug_addToBuffer(uint8_t* in, uint32_t len) {
    uint8_t a = 0;
    a += 1;
    if(len > debugUARTBufferRemaining) {
        len = debugUARTBufferRemaining; /*toss bit that won't fit*/
    }
    uint32_t remainingLen = (len + debugUARTBufferHead);
    if(DEBUG_UART_TRANSMIT_BUFFER_LEN > remainingLen) {
        /* not overflowing the end of the buffer*/
        memcpy(debugUARTBuffer + debugUARTBufferHead, in, len);
        debugUARTBufferHead += len;
    } else {
        /*overflowing end of buffer, wrap around*/
        uint32_t endLen = DEBUG_UART_TRANSMIT_BUFFER_LEN - debugUARTBufferHead;
        memcpy(debugUARTBuffer + debugUARTBufferHead, in, endLen);
        uint32_t startLen = len - endLen; /*what goes at the start of the buffer*/
        memcpy(debugUARTBuffer, in + endLen, startLen);
        debugUARTBufferHead = startLen;
    }
    debugUARTBufferRemaining -= len; /* update this*/
    if(!debugUARTBusy) {
        uart_debug_transmit(); /* start things*/
    }
    return len;
}

void uart_debug_putchar(unsigned char in) {
    uart_debug_addToBuffer(&in, 1);
}

/* internal helper function to handle the actual HAL call*/
void uart_debug_transmit(void) {
    uint32_t head = debugUARTBufferHead;
    if(head < debugUARTBufferTail) {
        /* we've wrapped around
         * head is at the end and we'll have to do another HAL call later
         */
        head = DEBUG_UART_TRANSMIT_BUFFER_LEN;
    }
    uint32_t len = head - debugUARTBufferTail;
    HAL_UART_Transmit_IT(debugUART, debugUARTBuffer + debugUARTBufferTail, len);
    debugUARTBusy = 1;
    /*update the transmit head*/
    debugUARTBufferTransmitHead = head;
}


/*void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
 * moved the callback to main, that callback then calls this function
 * */
void uart_debug_callback(void) {
    debugUARTBufferRemaining += (debugUARTBufferTransmitHead - debugUARTBufferTail);
    if(DEBUG_UART_TRANSMIT_BUFFER_LEN <= debugUARTBufferTransmitHead) {
        debugUARTBufferTransmitHead = 0; //wrap this
    }
    debugUARTBufferTail = debugUARTBufferTransmitHead; /*update tail*/
    if(debugUARTBufferHead == debugUARTBufferTransmitHead) {
        /*nothing new has been added, so nothing to do!*/
        debugUARTBusy = 0;
    } else {
        uart_debug_transmit();
    }
}
/* dumps a byte to the serial in binary format
 * handy for things like the status or access registers
 */
void uart_debug_dumpbyte(uint8_t b) {
    uint32_t len = 11;
    debugUARTBufferTemp[0] = '0';
    debugUARTBufferTemp[1] = 'b';
    uint8_t mask = 0x80;
    for(uint32_t i = 2; i < 10; ++i) {
        if(mask & b) {
            debugUARTBufferTemp[i] = '1';
        } else {
            debugUARTBufferTemp[i] = '0';
        }
        mask >>= 1;
    }
    debugUARTBufferTemp[10] = '\n';
    uart_debug_addToBuffer(debugUARTBufferTemp, len);
}

#endif /* UART_DEBUG_C */
