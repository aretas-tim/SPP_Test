/*
 * backup_regs.c
 *
 *  Created on: May 3, 2016
 *      Author: me
 */
#include "backup_regs.h"

void BACKUP_REGS_Clear(void) {
    /* use carefully*/
    for(int i = 0; i < BACKUP_REGS_COUNT; ++i) {
        *(BACKUP_REGS_BASE + i) = 0;
    }
}


#ifdef DEBUG
void BACKUP_REGS_Dump(void) {
    for(int i = 0; i < BACKUP_REGS_COUNT; ++i) {
            uart_debug_hexprint32(*(BACKUP_REGS_BASE + i));
            uart_debug_newline();
    }
}
#endif /*DEBUG*/


/*
 * assumptions:
 * storageKey of length STORAGE_KEY_LEN
 * transportKey and transportSalt are of length TRANSPORT_KEY_LEN
 * keyEncryptingKey is of length KEY_ENCRYPTING_KEY_LEN
 */ /*
void BACKUP_REGS_SetKeys(uint8_t* keyEncryptingKey, uint8_t* storageKey, uint8_t* transportKey, uint8_t* transportSalt) {
    uint8_t encryptedStorageKey[STORAGE_KEY_LEN];
    uint8_t encryptedTransportKey[TRANSPORT_KEY_LEN];
    uint8_t counter[AES_BLOCK_LEN];
    getRandomBuff(NULL, counter, AES_BLOCK_LEN);

    BACKUP_REGS_PackRegs(BACKUP_REGS_COUNTER_BASE, counter, AES_BLOCK_LEN / 4); //needs to go in before we use (and update!) the counter
    BACKUP_REGS_PackRegs(BACKUP_REGS_TRANSPORT_SALT_BASE, transportSalt, TRANSPORT_KEY_LEN / 4);


    mbedtls_aes_context ctx; // doesn't appear to have any dynamic memory in the initialization routine
    mbedtls_aes_init(&ctx);
    mbedtls_aes_setkey_enc(&ctx, keyEncryptingKey, KEY_ENCRYPTING_KEY_LEN_BITS);
    TPM_AES_CTR_Crypt(&ctx, STORAGE_KEY_LEN, counter, storageKey, encryptedStorageKey);
    TPM_AES_CTR_Crypt(&ctx, TRANSPORT_KEY_LEN, counter, transportKey, encryptedTransportKey);

    BACKUP_REGS_PackRegs(BACKUP_REGS_STORAGE_KEY_BASE, encryptedStorageKey, STORAGE_KEY_LEN / 4);
    BACKUP_REGS_PackRegs(BACKUP_REGS_TRANSPORT_KEY_BASE, encryptedTransportKey, TRANSPORT_KEY_LEN / 4);

    mbedtls_aes_free(&ctx);
    zeroize(encryptedStorageKey, STORAGE_KEY_LEN);
    zeroize(encryptedTransportKey, TRANSPORT_KEY_LEN); //clear these

    *BACKUP_REGS_CONF_INFO_REG |= BACKUP_REGS_KEYS_VALID;
}

void BACKUP_REGS_GetKeys(uint8_t* keyEncryptingKey, uint8_t* storageKey, uint8_t* transportKey, uint8_t* transportSalt) {
    uint8_t encryptedStorageKey[STORAGE_KEY_LEN];
    uint8_t encryptedTransportKey[TRANSPORT_KEY_LEN];
    uint8_t counter[AES_BLOCK_LEN];
    BACKUP_REGS_ExtractRegs(BACKUP_REGS_COUNTER_BASE, counter, AES_BLOCK_LEN / 4);
    BACKUP_REGS_ExtractRegs(BACKUP_REGS_TRANSPORT_SALT_BASE, transportSalt, TRANSPORT_KEY_LEN / 4);
    BACKUP_REGS_ExtractRegs(BACKUP_REGS_STORAGE_KEY_BASE, encryptedStorageKey, STORAGE_KEY_LEN / 4);
    BACKUP_REGS_ExtractRegs(BACKUP_REGS_TRANSPORT_KEY_BASE, encryptedTransportKey, TRANSPORT_KEY_LEN / 4);

    mbedtls_aes_context ctx; // doesn't appear to have any dynamic memory in the initialization routine
    mbedtls_aes_init(&ctx);
    mbedtls_aes_setkey_enc(&ctx, keyEncryptingKey, KEY_ENCRYPTING_KEY_LEN_BITS);
    TPM_AES_CTR_Crypt(&ctx, STORAGE_KEY_LEN, counter, encryptedStorageKey, storageKey);
    TPM_AES_CTR_Crypt(&ctx, TRANSPORT_KEY_LEN, counter, encryptedTransportKey, transportKey);

    mbedtls_aes_free(&ctx);
    zeroize(encryptedStorageKey, STORAGE_KEY_LEN);
    zeroize(encryptedTransportKey, TRANSPORT_KEY_LEN); //clear these

}*/

/* numRegs is the number of registers (32 bits)
 * the length of the data paramater is assumed to be (numRegs * 4)
 *
 * this is mostly because the registers are not byte accessible so memcpy doesn't work
 */
void BACKUP_REGS_PackRegs(volatile uint32_t* baseReg, uint8_t* dataIn, size_t numRegs) {
    size_t dataPos = 0;
    for(size_t i = 0; i < numRegs; ++i) {
        *(baseReg + i) = dataIn[dataPos] + (dataIn[dataPos + 1] << 8) + (dataIn[dataPos + 2] << 16) + (dataIn[dataPos + 3] << 24);
        dataPos += 4;
    }
}

void BACKUP_REGS_ExtractRegs(volatile uint32_t* baseReg, uint8_t* dataOut, size_t numRegs) {
    size_t dataPos = 0;
    for(size_t i = 0; i < numRegs; ++i) {
        dataOut[dataPos] = *(baseReg + i) & 0x000000FF;
        dataOut[dataPos + 1] = (*(baseReg + i) & 0x0000FF00) >> 8;
        dataOut[dataPos + 2] = (*(baseReg + i) & 0x00FF0000) >> 16;
        dataOut[dataPos + 3] = (*(baseReg + i) & 0xFF000000) >> 24;
        dataPos += 4;
    }
}

/* returns TRUE if the registers are (theoretically) valid, FALSE if they are not */
uint8_t BACKUP_REGS_Valid(void) {
    if((*BACKUP_REGS_BASE & BACKUP_REGS_KEYS_VALID) != BACKUP_REGS_KEYS_VALID) {
        return false;
    } else if (false /*(*BACKUP_REGS_BASE & BACKUP_REGS_AM_VALID) != BACKUP_REGS_AM_VALID*/) {
        return false; /* @TODO once attack mitigation is in, decomment the above line */
    }
    return true;
}



