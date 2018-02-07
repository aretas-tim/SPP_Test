/*
 * sram2.h
 *
 *  Created on: Apr 29, 2016
 *      Author: me
 */

#ifndef SRAM2_H_
#define SRAM2_H_

/* header for the allocation of SRAM2 areas */

#include "stm32l476xx.h" /* SRAM2_BASE */

/**
 * SRAM2 Layout
 * 0x7000 - 0x7FFF: Debug transmit buffer
 * 0x6800 - 0x6FFF: Unallocated
 * 0x6000 - 0x67FF: ICC Hotkey Tx (Secure->ICC->USB) buffer
 * 0x4000 - 0x5FFF: ICC U2F Tx (Secure->ICC->USB) buffer
 * 0x2000 - 0x3FFF: ICC U2F Rx (USB->ICC->Secure) buffer
 * 0x1000 - 0x1FFF: ICC Tunnel Tx (Secure->ICC->USB) buffer
 * 0x0000 - 0x0FFF: ICC Tunnel Rx (USB->ICC->Secure) buffer
 */


#define SRAM2_UART_DEBUG_TX_BUFF (SRAM2_BASE + 0x00007000)
#define SRAM2_UART_DEBUG_TX_BUFF_LEN 0xFC0 /* 4kB - 64 bytes */

#define SRAM2_UART_DEBUG_TEMP_BUFF (SRAM2_BASE + 0x00007FC0)
#define SRAM2_UART_DEBUG_TEMP_BUFF_LEN 0x40 /* 64 bytes on a line */

#define SRAM2_HOTKEY_IN_BUFF (SRAM2_BASE + 0x00006000)
#define SRAM2_HOTKEY_IN_BUFF_LEN 0x800 /* 1024 KeyWithModifier structs */

#define SRAM2_U2F_IN_BUFF (SRAM2_BASE + 0x00004000)
#define SRAM2_U2F_IN_BUFF_LEN 0x2000 /* 8kB, fits maximum-length U2FHID message */

#define SRAM2_U2F_OUT_BUFF (SRAM2_BASE + 0x00002000)
#define SRAM2_U2F_OUT_BUFF_LEN 0x2000 /* 8kB, fits maximum-length U2FHID message */

#define SRAM2_TUNNEL_IN_BUFF (SRAM2_BASE + 0x00001000)
#define SRAM2_TUNNEL_IN_BUFF_LEN 0x1000 /* 4kB */

#define SRAM2_TUNNEL_OUT_BUFF (SRAM2_BASE + 0x00000000)
#define SRAM2_TUNNEL_OUT_BUFF_LEN 0x1000 /* 4kB */

#endif /* SRAM2_H_ */
