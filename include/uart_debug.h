
#ifndef UART_DEBUG_H
#define UART_DEBUG_H

#include <stdlib.h>
#include <string.h> /*memcpy*/
#include "stm32l4xx_hal.h"

#define DEBUG_UART_HEXDUMP_BLOCK_SIZE 16 /*bytes per dump line, can be a maximum of DEBUG_UART_TEMP_BUFFER_LEN / 3 */

int8_t UART_Debug_Init(UART_HandleTypeDef*);
void UART_Debug_Deinit(void);
uint32_t uart_debug_addToBuffer(uint8_t*, uint32_t);
void uart_debug_hexdump(uint8_t*, uint32_t);
void uart_debug_hexprint8(uint8_t val);
void uart_debug_hexprint16(uint16_t);
void uart_debug_hexprint32(uint32_t);
void uart_debug_hexprint64(uint64_t val);
void uart_debug_printuint8(uint8_t);
void uart_debug_printuint32(uint32_t);
void uart_debug_printuint64(uint64_t);
void uart_debug_putchar(const unsigned char);
void uart_debug_callback(void);
void uart_debug_newline(void);
void uart_debug_dumpbyte(uint8_t);
void uart_debug_sendline(char*);
void uart_debug_sendstring(char* str);
void uart_debug_printBool(uint32_t);

#endif /* UART_DEBUG_H */
