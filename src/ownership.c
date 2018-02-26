/*
 * ownership.c
 *
 *    handles all the fancy ownership stuff
 *
 *  Created on: Nov 3, 2016
 *      Author: me
 */

#include "ownership.h"
#include "pkcs5.h"
#include "tunnel.h" /* so correct tunnel codes can be returned */

static uint32_t Ownership_relockTicks = 0;
bool Ownership_hasBeenUnlocked = false;

void (*Ownership_clearFunc)(void) = NULL;

Ownership_PinUpdateInfo Ownership_pinUpdateInfo = {false, 0, 0}; /* holds info to be used in changing the PIN */

/* initializes the ownership system.
 * takes a pointer to a function that will be called when attack mitigation is set to erase the device after a certain number of failures
 * function must not be NULL or it will enter an infinite loop
 */
void Ownership_Init(void (*clearFunc)(void)) {
    Ownership_hasBeenUnlocked = false;
    if(clearFunc == NULL) {
        uart_debug_sendline("Ownership Clear Callback cannot be NULL. Entering Infinite Loop.\n");
        while(true);
    }
    Ownership_clearFunc = clearFunc;
}

/* returns if this device has been unlocked since the last reset
 * a precaution against accidental force clears
 */
bool Ownership_HasBeenUnlocked(void) {
    return Ownership_hasBeenUnlocked;
}

/*
 * returns true if this device is owned
 * returns false if it does not have an owner (
 */
bool Ownership_isOwned(void) {
    if((*BACKUP_REGS_OWNERSHIP_REG & OWNERSHIP_CTRL_OWNERSHIP) == OWNERSHIP_CTRL_IS_OWNED) {
        return true;
    } else {
        return false;
    }
}

/**
 *
 *
 * param pin the users pin
 * param pinLen the length of the pin
 * param salt the data used to salt the KDF run
 * param saltLen the length of the salt
 * param storageKey output: the storage key, valid if the return value is 0
 */
uint8_t Ownership_installOwner(uint8_t* pin, size_t pinLen, uint8_t* salt, size_t saltLen, uint8_t storageKey[OWNERSHIP_KEY_LEN]) {
    //uart_debug_hexdump(salt, saltLen);
    //uart_debug_newline();
    size_t combinedDataLen = OWNERSHIP_PIN_HASH_LEN + OWNERSHIP_KEY_LEN;
    uint8_t combinedData[combinedDataLen];
    uint8_t hashedPIN[PIN_HASH_LEN]; //@TODO move these in to a quick malloc?


    if(*BACKUP_REGS_UNLOCK_ITERS_REG == 0) {
        *BACKUP_REGS_UNLOCK_ITERS_REG = KDF_ITERATIONS_PER_SECOND * DEFAULT_UNLOCK_TIME;
    }
    if(*BACKUP_REGS_TUNNEL_ITERS_REG == 0) {
        *BACKUP_REGS_TUNNEL_ITERS_REG = KDF_ITERATIONS_PER_SECOND * DEFAULT_UNLOCK_TIME;
    }
    if(*BACKUP_REGS_AM_INFO_REG == 0) {
        *BACKUP_REGS_AM_INFO_REG =  (((uint32_t) AM_DEFAULT_START_COUNT) << AM_START_COUNT_SHIFT) |
                                    (((uint32_t) AM_DEFAULT_END_COUNT) << AM_END_COUNT_SHIFT) |
                                    (((uint32_t) AM_DEFAULT_ERASE_COUNT) << AM_ERASE_COUNT_SHIFT) |
                                    ((uint32_t) AM_DEFAULT_AGGRESSIVENESS);
    }
    if(*BACKUP_REGS_OWNERSHIP_REG == 0) {
        *BACKUP_REGS_OWNERSHIP_REG = (((uint32_t) MINIMUM_NO_ERASE_PIN_LEN) << OWNERSHIP_CTRL_MIN_PIN_LEN_SHIFT) |
                                     (((uint32_t) MINIMUM_OTC_LEN) << OWNERSHIP_CTRL_MIN_OTC_LEN_SHIFT);
    }



    if(*BACKUP_REGS_RELOCK_REG == 0) {
        *BACKUP_REGS_RELOCK_REG = DEFAULT_RELOCK_TIME | RELOCK_TYPE_ANY_AUTH;
    }


    mbedtls_md_context_t ctx;
    mbedtls_md_init(&ctx);
    mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), true);
    //TIM2->CNT = 0x0; //reinitialize timer2
    //TIM2->CR1 |= 0x0001; //start timer 2
    //hash the PIN once. this was required at one point to avoid a timing bug
    mbedtls_md(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), pin, pinLen, hashedPIN);
    mbedtls_pkcs5_pbkdf2_hmac(&ctx, hashedPIN, PIN_HASH_LEN, salt, saltLen, *BACKUP_REGS_UNLOCK_ITERS_REG, combinedDataLen, combinedData);
    //TIM2->CR1 &= ~(0x0001); //stop timer 2

    uart_debug_hexdump(combinedData, combinedDataLen);
    uart_debug_newline();
    //uart_debug_addToBuffer("Derivation Cycles: ", 19);
    //uart_debug_printuint32(TIM2->CNT);
    //uart_debug_newline();
    mbedtls_md_free(&ctx);

    uint8_t* ownerAuthentication = combinedData;
    uint8_t* keyEncryptingKey = combinedData + OWNERSHIP_PIN_HASH_LEN;

    uint8_t iv[OWNERSHIP_IV_LEN];
    uint8_t encryptedStorageKey[OWNERSHIP_KEY_LEN];

    getRandomBuff(NULL, storageKey, OWNERSHIP_KEY_LEN); //random storage key
    getRandomBuff(NULL, iv, OWNERSHIP_IV_LEN); //random IV



    mbedtls_aes_context aesctx;
    mbedtls_aes_init(&aesctx);
    mbedtls_aes_setkey_enc(&aesctx, keyEncryptingKey, OWNERSHIP_KEY_LEN * 8);


    //int encResult =
    mbedtls_aes_crypt_cbc(&aesctx, MBEDTLS_AES_ENCRYPT, OWNERSHIP_KEY_LEN, iv, storageKey, encryptedStorageKey);
    mbedtls_aes_free(&aesctx);

    //if(encResult == 0) {
        //success
        BackupRegs_packRegs(BACKUP_REGS_STORAGE_KEY_BASE, encryptedStorageKey, OWNERSHIP_KEY_LEN  / 4);
        *BACKUP_REGS_OWNERSHIP_REG = OWNERSHIP_CTRL_IS_OWNED | (*BACKUP_REGS_OWNERSHIP_REG & ~OWNERSHIP_CTRL_OWNERSHIP);
        BackupRegs_packRegs(BACKUP_REGS_PASSWORD_HASH_BASE, ownerAuthentication, OWNERSHIP_PIN_HASH_LEN / 4);
        BackupRegs_packRegs(BACKUP_REGS_STORAGE_KEY_IV_BASE, iv, OWNERSHIP_IV_LEN / 4);
        BackupRegs_packRegs(BACKUP_REGS_STORAGE_KEY_BASE, encryptedStorageKey, OWNERSHIP_KEY_LEN / 4);
    //}
    //return (encResult != 0);
    return true;
}

/*
 *
 *
 *
 */
bool Ownership_ownerUnlock(uint8_t* pin, size_t pinLen, uint8_t* salt, size_t saltLen, uint8_t storageKey[OWNERSHIP_KEY_LEN]) {
    if(!Ownership_checkAttackMitigation()) {
        //cannot unlock while under attack mitigation protection
        return false;
    }
#ifdef DEBUG_TIME_UNLOCK
    //__disable_irq();
    /*uart_debug_hexprint32(RTC->TR);
    uart_debug_newline();
    uart_debug_hexprint32(RTC->SSR);
    uart_debug_newline();*/
    uint32_t timer2KDFStart, timer2KDFEnd, timer2MatchEnd, timer2UnlockEnd;
    TIM2->CNT = 0x0; //reinitialize timer2
    TIM2->CR1 |= 0x0001; //start timer 2
#endif /* DEBUG_TIME_UNLOCK */

    size_t combinedDataLen = OWNERSHIP_PIN_HASH_LEN + OWNERSHIP_KEY_LEN;
    uint8_t combinedData[combinedDataLen];
    uint8_t hashedPIN[PIN_HASH_LEN]; //@TODO move these in to a quick malloc?


    mbedtls_md_context_t ctx;
    mbedtls_md_init(&ctx);
    mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), true);
    //TIM2->CNT = 0x0; //reinitialize timer2
    //TIM2->CR1 |= 0x0001; //start timer 2
#ifdef DEBUG_TIME_UNLOCK
    timer2KDFStart = TIM2->CNT;
#endif /* DEBUG_TIME_UNLOCK */
    //hash the PIN once. this was required at one point to avoid a timing bug
    mbedtls_md(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), pin, pinLen, hashedPIN);
    mbedtls_pkcs5_pbkdf2_hmac(&ctx, hashedPIN, PIN_HASH_LEN, salt, saltLen, *BACKUP_REGS_UNLOCK_ITERS_REG, combinedDataLen, combinedData);
    //TIM2->CR1 &= ~(0x0001); //stop timer 2
#ifdef DEBUG_TIME_UNLOCK
    timer2KDFEnd = TIM2->CNT;
#endif /* DEBUG_TIME_UNLOCK */
#ifdef DEBUG
    uart_debug_hexdump(combinedData, combinedDataLen);
    uart_debug_newline();
#endif /* DEBUG */
    uint8_t* ownerAuthentication = combinedData;
    uint8_t* keyEncryptingKey = combinedData + OWNERSHIP_PIN_HASH_LEN;
    uint8_t* storedOwnerAuthentication = hashedPIN; //reuse pointer

    BackupRegs_extractRegs(BACKUP_REGS_PASSWORD_HASH_BASE, storedOwnerAuthentication, OWNERSHIP_PIN_HASH_LEN / 4);
    bool match = compareDigests(ownerAuthentication, storedOwnerAuthentication, OWNERSHIP_PIN_HASH_LEN);
#ifdef DEBUG_TIME_UNLOCK
    timer2MatchEnd = TIM2->CNT;
#endif /* DEBUG_TIME_UNLOCK */
    if(match) {
        Ownership_hasBeenUnlocked = true;
        Ownership_clearAttackMitigation();
        Ownership_pinUpdateInfo.valid = false;
        Ownership_pinUpdateInfo.minLen = 0;
        Ownership_pinUpdateInfo.workFactor = 0;

        uint8_t iv[OWNERSHIP_IV_LEN];
        uint8_t encryptedStorageKey[OWNERSHIP_KEY_LEN];

        BackupRegs_extractRegs(BACKUP_REGS_STORAGE_KEY_IV_BASE, encryptedStorageKey, OWNERSHIP_KEY_LEN / 4);
        BackupRegs_extractRegs(BACKUP_REGS_STORAGE_KEY_BASE, iv, OWNERSHIP_IV_LEN / 4);

        mbedtls_aes_context aesctx;
        mbedtls_aes_init(&aesctx);
        mbedtls_aes_setkey_enc(&aesctx, keyEncryptingKey, OWNERSHIP_KEY_LEN * 8);

        //int decResult =
        mbedtls_aes_crypt_cbc(&aesctx, MBEDTLS_AES_DECRYPT, OWNERSHIP_KEY_LEN, iv, encryptedStorageKey, storageKey);
    } else {
        Ownership_increaseAttackMitigation();
    }
#ifdef DEBUG_TIME_UNLOCK
    TIM2->CR1 &= ~(0x0001); //stop timer 2
    //__enable_irq();
    timer2UnlockEnd = TIM2->CNT;
    /*uart_debug_hexprint32(RTC->TR);
    uart_debug_newline();
    uart_debug_hexprint32(RTC->SSR);
    uart_debug_newline(); */
    uart_debug_sendline("Unlock Timing (in clocks):\n");
    uart_debug_addToBuffer("KDF Start: ", 11);
    uart_debug_printuint32(timer2KDFStart);
    uart_debug_addToBuffer("\nKDF End: ", 10);
    uart_debug_printuint32(timer2KDFEnd);
    uart_debug_addToBuffer("\nMatch End: ", 12);
    uart_debug_printuint32(timer2MatchEnd);
    uart_debug_addToBuffer("\nUnlock End: ", 13);
    uart_debug_printuint32(timer2UnlockEnd);
    uart_debug_newline();
#endif /* DEBUG_TIME_UNLOCK*/

    return match;
}

/*
 * clears the owner from the device. there are NO checks on this function.
 * better be sure you know what you're doing
 * always returns true
 */
bool Ownership_clearOwner(void) {
    Ownership_clearAttackMitigation(); //clear this
    *BACKUP_REGS_OWNERSHIP_REG = (*BACKUP_REGS_OWNERSHIP_REG & ~OWNERSHIP_CTRL_OWNERSHIP); //clear our owned bits
    volatile uint32_t* regPtr = BACKUP_REGS_PASSWORD_HASH_BASE; //@TODO update this if additional sensitive data is stored
    do {
        *(regPtr++) = 0;
    } while(regPtr <= (&RTC->BKP31R));
    return true;
}






/* returns true if attack mitigation allows an action to proceed.
 * returns false if attack mitigation is in progress and the action must not proceed
 */
bool Ownership_checkAttackMitigation(void) {
    uint32_t lockoutUntilTime = *BACKUP_REGS_AM_LOCKOUT_UNTIL_TIME;
    uint32_t currentTime = getSecondsSinceEpoch();
    //check the lockout time to see if its in to the next century
    if(lockoutUntilTime >= SECONDS_PER_CENTURY) {
        //check if our current time is not
        if(currentTime < SECONDS_PER_CENTURY) {
            currentTime += SECONDS_PER_CENTURY;
        }
    }
    //compare
    return (currentTime >= lockoutUntilTime);
}

void Ownership_clearAttackMitigation(void) {
    *BACKUP_REGS_AM_COUNT_REG = 0; //clear the reg
    *BACKUP_REGS_AM_LOCKOUT_UNTIL_TIME = 0; //remove any existing lockout
}

void Ownership_increaseAttackMitigation(void) {
    uint32_t count = ++(*BACKUP_REGS_AM_COUNT_REG);
    uint32_t startCount, endCount, eraseCount, aggressiveness;

    startCount = (*BACKUP_REGS_AM_INFO_REG & AM_START_COUNT_MASK) >> AM_START_COUNT_SHIFT;
    endCount = (*BACKUP_REGS_AM_INFO_REG & AM_END_COUNT_MASK) >> AM_END_COUNT_SHIFT;
    eraseCount = (*BACKUP_REGS_AM_INFO_REG & AM_ERASE_COUNT_MASK) >> AM_ERASE_COUNT_SHIFT;
    aggressiveness = (*BACKUP_REGS_AM_INFO_REG & AM_AGGRESSIVENESS);

    if(count < startCount) {
        return; //nothing to do
    } else if ((count >= eraseCount) && (eraseCount > 0)) { //erase count of 0 means no erase
        Ownership_clearFunc();
    } else {
        //calculate and set the lockout time
        uint32_t lockoutSeconds = AM_BASE_TIME_IN_SECONDS;
        uint32_t activeCount = count - startCount;
        if(count > endCount) {
            activeCount = endCount - startCount;
            *BACKUP_REGS_AM_COUNT_REG = UINT8_MAX - 1; //prevent this from overflowing
        }


        lockoutSeconds *= aggressiveness;
        if(activeCount > AM_ACTIVE_COUNT_MAX) {
            activeCount = AM_ACTIVE_COUNT_MAX;
        }
        lockoutSeconds <<= activeCount;
        uint32_t epoch = getSecondsSinceEpoch();
        *BACKUP_REGS_AM_LOCKOUT_UNTIL_TIME = epoch + lockoutSeconds;
        if(*BACKUP_REGS_AM_LOCKOUT_UNTIL_TIME < epoch) { //check for overflow
            *BACKUP_REGS_AM_LOCKOUT_UNTIL_TIME = -1; //set to max
        }
    }


}
/*
 * increments the relock counter and checks if it exceeds its timeout.
 * call this once per second
 */
bool Ownership_checkRelockTimeout(void) {
    Ownership_relockTicks++;
    return (Ownership_relockTicks > (*BACKUP_REGS_RELOCK_REG & RELOCK_SECONDS_MASK));

}
/*
 * updates the relock timeout. right now this just resets it to 0.
 *
 * the intent here is that this is called when an authorization-required function is called
 * if the owner has indicated they do not want a "keep alive" style function, this function can be updated so that it does nothing
 */
void Ownership_updateRelockTimeout(void) {
    if((*BACKUP_REGS_RELOCK_REG & RELOCK_TYPE_MASK) == RELOCK_TYPE_ANY_AUTH) {
        Ownership_relockTicks = 0;
    } //else do nothing
}
/*
 * resets the relock timeout.
 * call this on successful unlock.
 */
void Ownership_resetRelockTimeout(void) {
    Ownership_relockTicks = 0;
}


uint32_t Ownership_doInitialSetup(Ownership_SetupInfo* setupInfo){
    if(*BACKUP_REGS_OWNERSHIP_REG & OWNERSHIP_CTRL_OWNERSHIP) {
        return TUNNEL_RSP_DEVICE_OWNED;
    }
    //how the hell we got an attack mitigation count while unowned.. but it should never happen. check anyway, probably indicates corruption
    //also check for null here
    if(*BACKUP_REGS_AM_COUNT_REG || (setupInfo == NULL)) {
        return TUNNEL_RSP_FAILED;
    }

    //confirm variables are consistent and in acceptable ranges
    if(setupInfo->minOTCLen < MINIMUM_OTC_LEN) {
        return TUNNEL_RSP_BAD_PARAMETER;
    }
    if(setupInfo->minPINLen < MINIMUM_PIN_LEN) {
        return TUNNEL_RSP_BAD_PARAMETER;
    }
    if(setupInfo->am_baseTime == 0) {
            return TUNNEL_RSP_BAD_PARAMETER;
        }
    if(setupInfo->am_maxCount == UINT8_MAX) {
        return TUNNEL_RSP_BAD_PARAMETER; //cannot be 255 for internal reasons
    }
    if(setupInfo->am_maxCount <= setupInfo->am_threshold) { //maxCount cannot be less than threshold
        return TUNNEL_RSP_BAD_PARAMETER;
    } else if (setupInfo->am_eraseCount == 0) {
        //if we're set not to erase, ensure the PIN is long enough and the lockout time is long enough to maintain adequate security
        uint8_t maxActiveCount = setupInfo->am_maxCount - setupInfo->am_threshold;
        if(maxActiveCount > AM_ACTIVE_COUNT_MAX) {
            return TUNNEL_RSP_BAD_PARAMETER; //prevents overflow-related errors
        }
        uint32_t maxLockoutTime = setupInfo->am_baseTime << maxActiveCount;
        if(maxLockoutTime < AM_MINIMUM_MAXIMUM_LOCKOUT_TIME) {
            return TUNNEL_RSP_BAD_PARAMETER; //lockout time is not long enough
        }
        if(setupInfo->minPINLen < MINIMUM_NO_ERASE_PIN_LEN) {
            return TUNNEL_RSP_BAD_PARAMETER;
        }
    }
    if((setupInfo->unlockTime < MINIMUM_UNLOCK_TIME) || (setupInfo->unlockTime > MAXIMUM_UNLOCK_TIME)) {
        return TUNNEL_RSP_BAD_PARAMETER;
    }
    if((setupInfo->tunnelTime < MINIMUM_TUNNEL_TIME) || (setupInfo->tunnelTime > MAXIMUM_TUNNEL_TIME)) {
        return TUNNEL_RSP_BAD_PARAMETER;
    }
    //verified parameters

    *BACKUP_REGS_UNLOCK_ITERS_REG = (setupInfo->unlockTime * KDF_ITERATIONS_PER_SECOND) / 1000;
    *BACKUP_REGS_TUNNEL_ITERS_REG = (setupInfo->tunnelTime * KDF_ITERATIONS_PER_SECOND) / 1000;

    *BACKUP_REGS_AM_INFO_REG =  (((uint32_t) setupInfo->am_threshold) << AM_START_COUNT_SHIFT) |
                                (((uint32_t) setupInfo->am_maxCount) << AM_END_COUNT_SHIFT) |
                                (((uint32_t) setupInfo->am_eraseCount) << AM_ERASE_COUNT_SHIFT) |
                                ((uint32_t) setupInfo->am_baseTime);

    *BACKUP_REGS_OWNERSHIP_REG = (((uint32_t) setupInfo->minPINLen) << OWNERSHIP_CTRL_MIN_PIN_LEN_SHIFT) |
                                 (((uint32_t) setupInfo->minOTCLen) << OWNERSHIP_CTRL_MIN_OTC_LEN_SHIFT);

    *BACKUP_REGS_RELOCK_REG = setupInfo->relockTimeout & (setupInfo->relockMode ? RELOCK_TYPE_ANY_AUTH : RELOCK_TYPE_PIN_ONLY);

    return TUNNEL_RSP_SUCCESS;
}

uint32_t Ownership_getConfiguration(Ownership_SetupInfo* configInfo) {
    if(configInfo == NULL) {
        return TUNNEL_RSP_FAILED;
    }

    configInfo->am_baseTime = *BACKUP_REGS_AM_INFO_REG & AM_AGGRESSIVENESS;
    configInfo->am_eraseCount = (*BACKUP_REGS_AM_INFO_REG & AM_ERASE_COUNT_MASK) >> AM_ERASE_COUNT_SHIFT;
    configInfo->am_threshold = (*BACKUP_REGS_AM_INFO_REG & AM_START_COUNT_MASK) >> AM_START_COUNT_SHIFT;
    configInfo->am_maxCount = (*BACKUP_REGS_AM_INFO_REG & AM_END_COUNT_MASK) >> AM_END_COUNT_SHIFT;

    configInfo->unlockTime = (*BACKUP_REGS_UNLOCK_ITERS_REG * 1000) / KDF_ITERATIONS_PER_SECOND;
    configInfo->tunnelTime = (*BACKUP_REGS_TUNNEL_ITERS_REG * 1000) / KDF_ITERATIONS_PER_SECOND;

    configInfo->minPINLen = (*BACKUP_REGS_OWNERSHIP_REG & OWNERSHIP_CTRL_MIN_PIN_LEN_MASK) >> OWNERSHIP_CTRL_MIN_PIN_LEN_SHIFT;
    configInfo->minOTCLen = (*BACKUP_REGS_OWNERSHIP_REG & OWNERSHIP_CTRL_MIN_OTC_LEN_MASK) >> OWNERSHIP_CTRL_MIN_OTC_LEN_SHIFT;

    configInfo->relockTimeout = (*BACKUP_REGS_RELOCK_REG & RELOCK_SECONDS_MASK);
    configInfo->relockMode = (*BACKUP_REGS_RELOCK_REG & RELOCK_TYPE_MASK) >> RELOCK_TYPE_SHIFT;

    return TUNNEL_RSP_SUCCESS;
}

uint32_t Ownership_setupAttackMitigation(uint8_t threshold, uint8_t maxCount, uint8_t eraseCount, uint8_t baseTime) {
    if(*BACKUP_REGS_AM_COUNT_REG) {
        //cannot change the attack mitigation if we're under attack mitigation protection
        return TUNNEL_RSP_FAILED;
    }
    if(baseTime == 0) {
        return TUNNEL_RSP_BAD_PARAMETER;
    }

    if(maxCount == UINT8_MAX) {
        return TUNNEL_RSP_BAD_PARAMETER; //cannot be 255 for internal reasons
    }
    if(maxCount <= threshold) { //maxCount cannot be less than threshold
        return TUNNEL_RSP_BAD_PARAMETER;
    } else if (eraseCount == 0) {
        //if we're set not to erase, ensure the PIN is long enough and the lockout time is long enough to maintain adequate security
        uint8_t maxActiveCount = maxCount - threshold;
        if(maxActiveCount > AM_ACTIVE_COUNT_MAX) {
            return TUNNEL_RSP_BAD_PARAMETER; //prevents overflow-related errors
        }
        uint32_t maxLockoutTime = baseTime << maxActiveCount;
        if(maxLockoutTime < AM_MINIMUM_MAXIMUM_LOCKOUT_TIME) {
            return TUNNEL_RSP_BAD_PARAMETER; //lockout time is not long enough
        }
        uint8_t minPINLen = (*BACKUP_REGS_OWNERSHIP_REG & OWNERSHIP_CTRL_MIN_PIN_LEN_MASK) >> OWNERSHIP_CTRL_MIN_PIN_LEN_SHIFT;
        if(minPINLen < MINIMUM_NO_ERASE_PIN_LEN) {
            return TUNNEL_RSP_BAD_PARAMETER;
        }
    }

    *BACKUP_REGS_AM_INFO_REG =  (((uint32_t) threshold) << AM_START_COUNT_SHIFT) |
                                (((uint32_t) maxCount) << AM_END_COUNT_SHIFT) |
                                (((uint32_t) eraseCount) << AM_ERASE_COUNT_SHIFT) |
                                ((uint32_t) baseTime);
    return TUNNEL_RSP_SUCCESS;
}

uint32_t Ownership_setupRelock(uint16_t timeout, uint8_t mode) {
    *BACKUP_REGS_RELOCK_REG = timeout | (mode ? RELOCK_TYPE_ANY_AUTH : RELOCK_TYPE_PIN_ONLY);

    return TUNNEL_RSP_SUCCESS;

}

/*
 * any changes here will only effect the next tunnel set up
 * param minOTCLen    the minimum one-time code length the device considers acceptable
 * param tunnelTime the number of milliseconds the device will spend on the KDF (turns in to the work factor
 */
uint32_t Ownership_setupTunnel(uint8_t minOTCLen, uint32_t tunnelTime) {
    if((tunnelTime < MINIMUM_TUNNEL_TIME) || (tunnelTime > MAXIMUM_TUNNEL_TIME)) {
        return TUNNEL_RSP_BAD_PARAMETER;
    }
    if(minOTCLen < MINIMUM_OTC_LEN) {
        return TUNNEL_RSP_BAD_PARAMETER;
    }

    *BACKUP_REGS_TUNNEL_ITERS_REG = (tunnelTime * 1000) / KDF_ITERATIONS_PER_SECOND;
    *BACKUP_REGS_OWNERSHIP_REG = (*BACKUP_REGS_OWNERSHIP_REG & ~OWNERSHIP_CTRL_MIN_OTC_LEN_MASK) | (((uint32_t) minOTCLen) << OWNERSHIP_CTRL_MIN_OTC_LEN_SHIFT);

    return TUNNEL_RSP_SUCCESS;
}

/*
 * any changes here will only take effect if a PIN change is initiated before the next unlock.
 *
 * param minPINLen    the minimum PIN length the device considers acceptable
 * param unlockTime the number of milliseconds the device will spend on the KDF during unlock
 */
uint32_t Ownership_SetupPINChange(uint8_t minPINLen, uint32_t unlockTime) {
    if((unlockTime < MINIMUM_UNLOCK_TIME) || (unlockTime > MAXIMUM_UNLOCK_TIME)) {
        return TUNNEL_RSP_BAD_PARAMETER;
    }
    uint8_t am_eraseCount = (*BACKUP_REGS_AM_INFO_REG & AM_ERASE_COUNT_MASK) >> AM_ERASE_COUNT_SHIFT;
    if(am_eraseCount) { //if we're set up to erase, shorter PINs are acceptable
        if(minPINLen < MINIMUM_PIN_LEN) {
            return TUNNEL_RSP_BAD_PARAMETER;
        }
    } else {
        if(minPINLen < MINIMUM_NO_ERASE_PIN_LEN) {
            return TUNNEL_RSP_BAD_PARAMETER;
        }
    }
    Ownership_pinUpdateInfo.minLen = minPINLen;
    Ownership_pinUpdateInfo.workFactor = (unlockTime * 1000) / KDF_ITERATIONS_PER_SECOND;

    return TUNNEL_RSP_SUCCESS;
}
