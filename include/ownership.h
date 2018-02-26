/*
 * ownership.h
 *
 *  Created on: Nov 3, 2016
 *      Author: me
 */

#ifndef OWNERSHIP_H_
#define OWNERSHIP_H_

#include "backup_regs.h"
#include "uart_debug.h"
#include "utilities.h"
#include <stdbool.h>

#define OWNERSHIP_CTRL_OWNERSHIP    0x0000000F
#define OWNERSHIP_CTRL_IS_OWNED        0x0000000F
#define OWNERSHIP_CTR_IS_UNOWNED    0x00000000
#define OWNERSHIP_CTRL_MIN_PIN_LEN_MASK 0x00FF0000
#define OWNERSHIP_CTRL_MIN_PIN_LEN_SHIFT 16
#define OWNERSHIP_CTRL_MIN_OTC_LEN_MASK 0x0000FF00
#define OWNERSHIP_CTRL_MIN_OTC_LEN_SHIFT 8

/* multiplies the given count to give a wider range of options*/
/* should be smaller than 2^24 otherwise there's a risk of rollover.. and really, really long unlock times */
#define OWNERSHIP_CTRL_UNLOCK_KDF_COUNT_MULTIPLIER 128
#define OWNERSHIP_CTRL_UNLOCK_KDF_COUNT_DEFAULT 32 /* default, used when the value of 0 is present */
#define KDF_ITERATIONS_PER_SECOND 1200 /* as measured for SHA256 PBKDF2 on STM32L476 at 80MHz with interrupts disabled, rounded slightly */
                                       /* results should be broadly comparable across any Cortex-M core after compensating for clock frequency */
#define DEFAULT_UNLOCK_TIME 2 /* seconds */
#define DEFAULT_RELOCK_TIME 3600 /* seconds */

#define OWNERSHIP_PIN_HASH_LEN 32 /* bytes */
#define OWNERSHIP_KEY_LEN 32 /* bytes*/
#define OWNERSHIP_IV_LEN 16 /* bytes*/
#define PIN_HASH_LEN 32 /* bytes */
//attack mitigation constants
#define SECONDS_PER_MINUTE 60
#define SECONDS_PER_HOUR 3600
#define SECONDS_PER_DAY 86400
#define MONTH_1_SECONDS_AFTER_YEAR         0
#define MONTH_2_SECONDS_AFTER_YEAR   2678400 /* leap years accounted for when we hande the years */
#define MONTH_3_SECONDS_AFTER_YEAR   5097600
#define MONTH_4_SECONDS_AFTER_YEAR   7776000
#define MONTH_5_SECONDS_AFTER_YEAR  10368000
#define MONTH_6_SECONDS_AFTER_YEAR  13046400
#define MONTH_7_SECONDS_AFTER_YEAR  15638400
#define MONTH_8_SECONDS_AFTER_YEAR  18316800
#define MONTH_9_SECONDS_AFTER_YEAR  20995200
#define MONTH_10_SECONDS_AFTER_YEAR 23587200
#define MONTH_11_SECONDS_AFTER_YEAR 26265600
#define MONTH_12_SECONDS_AFTER_YEAR 28857600
#define SECONDS_PER_365_DAY_YEAR (SECONDS_PER_DAY * 365)
#define SECONDS_PER_CENTURY            3155760000

#define AM_START_COUNT_MASK        0xFF000000
#define AM_END_COUNT_MASK        0x00FF0000
#define AM_ERASE_COUNT_MASK        0x0000FF00
#define AM_AGGRESSIVENESS        0x000000FF
#define AM_START_COUNT_SHIFT     24
#define AM_END_COUNT_SHIFT        16
#define AM_ERASE_COUNT_SHIFT    8
#define AM_BASE_TIME_IN_SECONDS 1
#define AM_ACTIVE_COUNT_MAX        24 /* prevents gross overflows */
#define AM_MINIMUM_MAXIMUM_LOCKOUT_TIME (SECONDS_PER_HOUR * 2) /* one trial every two hours gives ~4380 per year, under the maximum of 5k/year that is a common number to use */
#define AM_DEFAULT_START_COUNT 10
#define AM_DEFAULT_END_COUNT 23
#define AM_DEFAULT_ERASE_COUNT 0
#define AM_DEFAULT_AGGRESSIVENESS 2
#define MINIMUM_PIN_LEN 4
#define MINIMUM_NO_ERASE_PIN_LEN 6 /* minimum PIN length if device is not set to erase itself */
#define MINIMUM_OTC_LEN 4
#define MINIMUM_UNLOCK_TIME 250 /* milliseconds, i.e. a quarter of a second */
#define MAXIMUM_UNLOCK_TIME 3600000 /* milliseconds, i.e. an hour, this should remain under 4.2 million to prevent overflow issues*/
#define MINIMUM_TUNNEL_TIME 250 /* milliseconds */
#define MAXIMUM_TUNNEL_TIME 3600000 /* milliseconds, i.e. an hour, this should remain under 4.2 million to prevent overflow issues */

#define RELOCK_SECONDS_MASK 0x0000FFFF
#define RELOCK_TYPE_MASK 0x00010000
#define RELOCK_TYPE_SHIFT 16
#define RELOCK_TYPE_PIN_ONLY 0x00000000
#define RELOCK_TYPE_ANY_AUTH 0x00010000

typedef struct tdOwnershipInfo {
    uint8_t am_threshold;
    uint8_t am_maxCount;
    uint8_t am_eraseCount;
    uint8_t am_baseTime;
    uint32_t unlockTime;
    uint32_t tunnelTime;
    uint8_t minPINLen;
    uint8_t minOTCLen;
    uint16_t relockTimeout;
    uint8_t relockMode;
} Ownership_SetupInfo;

/* holds info for PIN changes between the tunnel call to update this and the actual PIN change procedure. */
typedef struct tdPINUpdateInfo {
    bool valid;
    uint8_t minLen;
    uint32_t workFactor; /* KDF iterations */

} Ownership_PinUpdateInfo;




void Ownership_Init(void (*clearFunc)(void));
bool Ownership_HasBeenUnlocked(void);
bool Ownership_checkAttackMitigation(void);
void Ownership_increaseAttackMitigation(void);
void Ownership_clearAttackMitigation(void);
bool Ownership_clearOwner(void);
bool Ownership_isOwned(void);
uint8_t Ownership_installOwner(uint8_t* pin, size_t pinLen, uint8_t* salt, size_t saltLen, uint8_t storageKey[OWNERSHIP_KEY_LEN]);
bool Ownership_ownerUnlock(uint8_t* pin, size_t pinLen, uint8_t* salt, size_t saltLen, uint8_t storageKey[OWNERSHIP_KEY_LEN]);
uint32_t Ownership_doInitialSetup(Ownership_SetupInfo* setupInfo);
uint32_t Ownership_getConfiguration(Ownership_SetupInfo* configInfo);
uint32_t Ownership_setupAttackMitigation(uint8_t threshold, uint8_t maxCount, uint8_t eraseCount, uint8_t baseTime);
uint32_t Ownership_setupRelock(uint16_t timeout, uint8_t mode);
uint32_t Ownership_setupTunnel(uint8_t minOTCLen, uint32_t tunnelTime);
uint32_t Ownership_setupPINChange(uint8_t minPINLen, uint32_t unlockTime);
bool Ownership_checkRelockTimeout(void);
void Ownership_updateRelockTimeout(void);
void Ownership_resetRelockTimeout(void);

#endif /* OWNERSHIP_H_ */
