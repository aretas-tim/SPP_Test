/*
 * backup_regs.h
 *
 *  Created on: May 3, 2016
 *      Author: me
 */

#ifndef BACKUP_REGS_H_
#define BACKUP_REGS_H_

#include "stm32l476xx.h"
#include "uart_debug.h"
#include "authdata_store.h"
#include "aes.h"
#include "utilities.h"
#include <stdbool.h>


#define BACKUP_REGS_COUNT 32
#define BACKUP_REGS_BASE &(RTC->BKP0R)
#define BACKUP_REGS_OWNERSHIP_REG &(RTC->BKP0R)
#define BACKUP_REGS_UNLOCK_ITERS_REG &(RTC->BKP1R)
#define BACKUP_REGS_TUNNEL_ITERS_REG &(RTC->BKP2R)
#define BACKUP_REGS_RELOCK_REG &(RTC->BKP3R)
#define BACKUP_REGS_AM_INFO_REG &(RTC->BKP4R)
#define BACKUP_REGS_AM_COUNT_REG &(RTC->BKP5R)
#define BACKUP_REGS_AM_LOCKOUT_UNTIL_TIME &(RTC->BKP6R)
#define BACKUP_REGS_U2F_COUNT_REG &(RTC->BKP11R) /* single 32 bit count. if you do 2 billion ops, congratulations?*/
#define BACKUP_REGS_PASSWORD_HASH_BASE &(RTC->BKP12R)
#define BACKUP_REGS_STORAGE_KEY_IV_BASE &(RTC->BKP20R)
#define BACKUP_REGS_STORAGE_KEY_BASE &(RTC->BKP24R)
#define BACKUP_REGS_KEYS_VALID 0x0000000F
#define BACKUP_REGS_AM_VALID 0x000000F0
#define KEY_ENCRYPTING_KEY_LEN STORAGE_KEY_LEN
#define KEY_ENCRYPTING_KEY_LEN_BITS (KEY_ENCRYPTING_KEY_LEN * 8)
#define AES_BLOCK_LEN 16

void BackupRegs_clear(void); /* use carefully*/
uint8_t BackupRegs_isValid(void);
void BackupRegs_packRegs(volatile uint32_t* baseReg, uint8_t* data, size_t numRegs);
void BackupRegs_extractRegs(volatile uint32_t* baseReg, uint8_t* dataOut, size_t numRegs);

#ifdef DEBUG
void BackupRegs_dump(void);
#endif /*DEBUG*/


#endif /* BACKUP_REGS_H_ */
