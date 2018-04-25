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



#define SRAM2_UART_DEBUG_TX_BUFF (SRAM2_BASE + 0x00007000)
#define SRAM2_UART_DEBUG_TX_BUFF_LEN 0x1000 /* 4kB */

#define SRAM2_TUNNEL_FIFO (SRAM2_BASE + 0x00005F00)
#define SRAM2_TUNNEL_FIFO_LEN 0x1100 /* 4.25kB */ /* to handle encryption overhead when passing 4kB of data */

#define SRAM2_TUNNEL_COMMAND_BUFF (SRAM2_BASE + 0x00004E00)
#define SRAM2_TUNNEL_COMMAND_BUFF_LEN 0x1100 /* 4.25kB */ /* to handle encryption overhead when passing 4kB of data */

#define SRAM2_BFSS_RECORD_CACHE (SRAM2_BASE + 0x00003D00)
#define SRAM2_BFSS_RECORD_CAHCE_LEN 0x1100 /* 4.25kB */

#define SRAM2_SPI_SECTOR_CACHE (SRAM2_BASE + 0x00000000)
#define SRAM2_SPI_SECTOR_CACHE_LEN 0x2400 /* 9kB, currently enough for two cache structs */
#define SRAM2_SPI_PAGE_BUFF (SRAM2_BASE + 0x00002400)
#define SRAM2_SPI_PAGE_BUFF_LEN 0xA00 /* 2.5kB */ /* currently, this tops out at 2.275kB of possibly-concurrent use. */
#define SRAM2_SPI_SECTOR_BITMAP (SRAM2_BASE + 0x00002E00)
#define SRAM2_SPI_SECTOR_BITMAP_LEN 0x200 /* 512B */


#endif /* SRAM2_H_ */
