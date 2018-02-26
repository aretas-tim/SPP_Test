/*
 * uart_debug.c
 *
 *  Created on: Feb 2, 2016
 *      Author: me
 *
 *  make sure to call UartDebug_init with a valid (and initialized) UART_HandleTypeDef* before calling any other function
 *  call UartDebug_deinit to null the reference and disable things.
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
static void UartDebug_transmit(void);


uint8_t* UartDebug_buffer = (uint8_t*) SRAM2_UART_DEBUG_TX_BUFF;
uint8_t UartDebug_bufferTemp[DEBUG_UART_TEMP_BUFFER_LEN]; //= (uint8_t*) SRAM2_UART_DEBUG_TEMP_BUFF;
volatile uint32_t UartDebug_bufferHead = 0;
volatile uint32_t UartDebug_bufferTail = 0;
volatile uint32_t UartDebug_bufferTransmitHead = 0;
volatile uint32_t UartDebug_bufferRemaining = DEBUG_UART_TRANSMIT_BUFFER_LEN;
volatile int8_t UartDebug_busy = -1;

int8_t UartDebug_init(UART_HandleTypeDef* uart) {
    if(NULL == uart) {
        /* why are we being called with an null pointer? */
        return -1;
    }
    debugUART = uart;
    UartDebug_busy = 0;
    return 0;
}

void UartDebug_deinit(void) {
    debugUART = NULL;
    if(NULL != UartDebug_buffer) {
        free(UartDebug_buffer);
    }
    if(NULL != UartDebug_bufferTemp) {
        free(UartDebug_bufferTemp);
    }
    UartDebug_busy = -1;
    UartDebug_bufferHead = 0;
    UartDebug_bufferTail = 0;
    UartDebug_bufferTransmitHead = 0;
    UartDebug_bufferRemaining = DEBUG_UART_TRANSMIT_BUFFER_LEN;
}

void UartDebug_printuint8(uint8_t val) {
    UartDebug_bufferTemp[0] = (val / 100) + '0';
    val %= 100;
    UartDebug_bufferTemp[1] = (val / 10) + '0';
    UartDebug_bufferTemp[2] = (val % 10) + '0';
    UartDebug_addToBuffer(UartDebug_bufferTemp, 3);
}

void UartDebug_printuint32(uint32_t val) {
    uint32_t pos = DEBUG_UART_TEMP_BUFFER_LEN;
    do {
        UartDebug_bufferTemp[--pos] = (val % 10) + '0';
        val /= 10;
    } while(val);
    UartDebug_addToBuffer(UartDebug_bufferTemp + pos, DEBUG_UART_TEMP_BUFFER_LEN - pos);
}

void UartDebug_printuint64(uint64_t val) {
    uint64_t pos = DEBUG_UART_TEMP_BUFFER_LEN;
    do {
        UartDebug_bufferTemp[--pos] = (val % 10) + '0';
        val /= 10;
    } while(val);
    UartDebug_addToBuffer(UartDebug_bufferTemp + pos, DEBUG_UART_TEMP_BUFFER_LEN - pos);
}

void UartDebug_hexprint8(uint8_t val) {
    UartDebug_bufferTemp[0] = '0';
    UartDebug_bufferTemp[1] = 'x';
    for(int8_t i = 3; i > 1; --i) {
        uint8_t c = (0xF & val);
        if(10 <= c) {
            UartDebug_bufferTemp[i] = (c - 10) + 'A';
        } else {
            UartDebug_bufferTemp[i] = c + '0';
        }
        val >>= 4;
    }
    UartDebug_addToBuffer(UartDebug_bufferTemp, 4);
}

void UartDebug_hexprint16(uint16_t val) {
    UartDebug_bufferTemp[0] = '0';
    UartDebug_bufferTemp[1] = 'x';
    for(int8_t i = 5; i > 1; --i) {
        uint8_t c = (0xF & val);
        if(10 <= c) {
            UartDebug_bufferTemp[i] = (c - 10) + 'A';
        } else {
            UartDebug_bufferTemp[i] = c + '0';
        }
        val >>= 4;
    }
    UartDebug_addToBuffer(UartDebug_bufferTemp, 6);
}

void UartDebug_hexprint32(uint32_t val) {
    UartDebug_bufferTemp[0] = '0';
    UartDebug_bufferTemp[1] = 'x';
    for(int8_t i = 9; i > 1; --i) {
        uint8_t c = (0xF & val);
        if(10 <= c) {
            UartDebug_bufferTemp[i] = (c - 10) + 'A';
        } else {
            UartDebug_bufferTemp[i] = c + '0';
        }
        val >>= 4;
    }
    UartDebug_addToBuffer(UartDebug_bufferTemp, 10);
}

void UartDebug_hexprint64(uint64_t val) {
    UartDebug_bufferTemp[0] = '0';
    UartDebug_bufferTemp[1] = 'x';
    for(int8_t i = 17; i > 1; --i) {
        uint8_t c = (0xF & val);
        if(10 <= c) {
            UartDebug_bufferTemp[i] = (c - 10) + 'A';
        } else {
            UartDebug_bufferTemp[i] = c + '0';
        }
        val >>= 4;
    }
    UartDebug_addToBuffer(UartDebug_bufferTemp, 18);
}

void UartDebug_printBool(uint32_t b) {
    if(0 == b) {
        UartDebug_sendString("False");
    } else {
        UartDebug_sendString("True");
    }
}

void UartDebug_hexdump(uint8_t* buff, uint32_t len) {
    if(len > (DEBUG_UART_TRANSMIT_BUFFER_LEN / 3)) {
        len = DEBUG_UART_TRANSMIT_BUFFER_LEN / 3;
    }
    uint32_t blockSize = DEBUG_UART_HEXDUMP_BLOCK_SIZE; // for spaces between bytes
    UartDebug_bufferTemp[(DEBUG_UART_HEXDUMP_BLOCK_SIZE * 3) - 1] = '\n'; //preload this
    uint32_t numBlocks = len / blockSize;
    uint32_t lastBlockSize = len % blockSize;
    for(uint32_t k = 0; k < (blockSize - 1); ++k) { //for our inter-byte spaces
        UartDebug_bufferTemp[(k * 3) + 2] = ' '; //preload these as well as they won't change
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
                UartDebug_bufferTemp[j * 3] = (c1 - 10) + 'A';
            } else {
                UartDebug_bufferTemp[j * 3] = c1 + '0';
            }
            if(10 <= c2) {
                UartDebug_bufferTemp[(j * 3) + 1] = (c2 - 10) + 'A';
            } else {
                UartDebug_bufferTemp[(j * 3) + 1] = c2 + '0';
            }
        }
        if(i != (numBlocks - 1)) {//if we're not on the last block
            UartDebug_addToBuffer(UartDebug_bufferTemp, (blockSize * 3));
        }
        else { //on the last block, ignore the trailing space/newline
            UartDebug_addToBuffer(UartDebug_bufferTemp, (blockSize * 3) - 1);
        }
        //HAL_Delay(50); //should rate-limit it enough to not overflow the buffer ever at 57600. disabled for now as it was causing issues as i was calling from an interrupt
    }
    UartDebug_newline(); //add final newline
}

/* puts a newline out cause i'm lazy*/
void UartDebug_newline(void) {
#ifdef UART_DEBUG_OSX
	uint8_t carriageChar = '\r';
	UartDebug_addToBuffer(&carriageChar, 1);
#endif
    uint8_t newlineChar = '\n';
    UartDebug_addToBuffer(&newlineChar, 1);
}



/* puts a newline-terminated character string on the wire (limited to the transmit buffer length for sanity) */
/*void UartDebug_sendline(char* str) {
    uint32_t len = 0;
    for(; len < DEBUG_UART_TRANSMIT_BUFFER_LEN; ++len) {
        if(str[len] == '\n') {
            break;
        }
    }
    UartDebug_addToBuffer((uint8_t*) str, len + 1); // to account for the newline char/
}*/


void UartDebug_sendline(char* str) {
    uint32_t len = 0;
    for(; len < DEBUG_UART_TRANSMIT_BUFFER_LEN; ++len) {
        if(str[len] == '\n') {
            break;
        }
    }
    UartDebug_addToBuffer((uint8_t*) str, len + 1); // to account for the newline char/
    uart_debug_carriageReturn();
    UartDebug_newline();

}

/* puts a null-terminated character string on the wire (limited to the transmit buffer length for sanity) */
void UartDebug_sendString(char* str) {
    uint32_t len = 0;
    for(; len < DEBUG_UART_TRANSMIT_BUFFER_LEN; ++len) {
        if(str[len] == '\0') {
            break;
        }
    }
    UartDebug_addToBuffer((uint8_t*) str, len);
}


/* adds stuff to the debug buffer to be sent out.
 * returns the number of bytes added to the buffer
 * if the amount of data is too big, does not add the bit that won't fit in the buffer
 */
uint32_t UartDebug_addToBuffer(uint8_t* in, uint32_t len) {
    uint8_t a = 0;
    a += 1;
    if(len > UartDebug_bufferRemaining) {
        len = UartDebug_bufferRemaining; /*toss bit that won't fit*/
    }
    uint32_t remainingLen = (len + UartDebug_bufferHead);
    if(DEBUG_UART_TRANSMIT_BUFFER_LEN > remainingLen) {
        /* not overflowing the end of the buffer*/
        memcpy(UartDebug_buffer + UartDebug_bufferHead, in, len);
        UartDebug_bufferHead += len;
    } else {
        /*overflowing end of buffer, wrap around*/
        uint32_t endLen = DEBUG_UART_TRANSMIT_BUFFER_LEN - UartDebug_bufferHead;
        memcpy(UartDebug_buffer + UartDebug_bufferHead, in, endLen);
        uint32_t startLen = len - endLen; /*what goes at the start of the buffer*/
        memcpy(UartDebug_buffer, in + endLen, startLen);
        UartDebug_bufferHead = startLen;
    }
    UartDebug_bufferRemaining -= len; /* update this*/
    if(!UartDebug_busy) {
        UartDebug_transmit(); /* start things*/
    }
    return len;
}

void UartDebug_putchar(unsigned char in) {
    UartDebug_addToBuffer(&in, 1);
}

/* internal helper function to handle the actual HAL call*/
void UartDebug_transmit(void) {
    uint32_t head = UartDebug_bufferHead;
    if(head < UartDebug_bufferTail) {
        /* we've wrapped around
         * head is at the end and we'll have to do another HAL call later
         */
        head = DEBUG_UART_TRANSMIT_BUFFER_LEN;
    }
    uint32_t len = head - UartDebug_bufferTail;
    HAL_UART_Transmit_IT(debugUART, UartDebug_buffer + UartDebug_bufferTail, len);
    UartDebug_busy = 1;
    /*update the transmit head*/
    UartDebug_bufferTransmitHead = head;
}


/*void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
 * moved the callback to main, that callback then calls this function
 * */
void UartDebug_callback(void) {
    UartDebug_bufferRemaining += (UartDebug_bufferTransmitHead - UartDebug_bufferTail);
    if(DEBUG_UART_TRANSMIT_BUFFER_LEN <= UartDebug_bufferTransmitHead) {
        UartDebug_bufferTransmitHead = 0; //wrap this
    }
    UartDebug_bufferTail = UartDebug_bufferTransmitHead; /*update tail*/
    if(UartDebug_bufferHead == UartDebug_bufferTransmitHead) {
        /*nothing new has been added, so nothing to do!*/
        UartDebug_busy = 0;
    } else {
        UartDebug_transmit();
    }
}
/* dumps a byte to the serial in binary format
 * handy for things like the status or access registers
 */
void UartDebug_dumpbyte(uint8_t b) {
    uint32_t len = 11;
    UartDebug_bufferTemp[0] = '0';
    UartDebug_bufferTemp[1] = 'b';
    uint8_t mask = 0x80;
    for(uint32_t i = 2; i < 10; ++i) {
        if(mask & b) {
            UartDebug_bufferTemp[i] = '1';
        } else {
            UartDebug_bufferTemp[i] = '0';
        }
        mask >>= 1;
    }
    UartDebug_bufferTemp[10] = '\n';
    UartDebug_addToBuffer(UartDebug_bufferTemp, len);
}

/* dumps memory to the debug UART*/
void UartDebug_memDump(uint8_t* start, size_t end) {
    size_t pos = 0;
    size_t chunk = 256; //how many bytes to dump at once before delaying so the UART can catch up
    uint32_t delay = 60; /* roughly how long it should take to dump a chunk*/
    while(pos < end) {
        if((end - pos) < chunk) {
            chunk = end - pos; //last chunk
        }
        UartDebug_sendString("MemDump: Pos ");
        UartDebug_hexprint32((uint32_t)start + pos);
        UartDebug_sendString(" Len ");
        UartDebug_hexprint32(chunk);
        UartDebug_newline();
        UartDebug_hexdump(start + pos, chunk);
        pos += chunk;
        HAL_Delay(delay);
    }
}

#endif /* UART_DEBUG_C */
