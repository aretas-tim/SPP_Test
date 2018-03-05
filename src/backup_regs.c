/*
 * backup_regs.c
 *
 *  Created on: May 3, 2016
 *      Author: me
 */
#include "backup_regs.h"

void BackupRegs_clear(void) {
    /* use carefully*/
    for(int i = 0; i < BACKUP_REGS_COUNT; ++i) {
        *(BACKUP_REGS_BASE + i) = 0;
    }
}


#ifdef DEBUG
void BackupRegs_dump(void) {
    for(int i = 0; i < BACKUP_REGS_COUNT; ++i) {
            UartDebug_hexprint32(*(BACKUP_REGS_BASE + i));
            UartDebug_newline();
    }
}
#endif /*DEBUG*/



/* numRegs is the number of registers (32 bits)
 * the length of the data paramater is assumed to be (numRegs * 4)
 *
 * this is mostly because the registers are not byte accessible so memcpy doesn't work
 */
void BackupRegs_packRegs(volatile uint32_t* baseReg, uint8_t* dataIn, size_t numRegs) {
    size_t dataPos = 0;
    for(size_t i = 0; i < numRegs; ++i) {
        *(baseReg + i) = dataIn[dataPos] + (dataIn[dataPos + 1] << 8) + (dataIn[dataPos + 2] << 16) + (dataIn[dataPos + 3] << 24);
        dataPos += 4;
    }
}

void BackupRegs_extractRegs(volatile uint32_t* baseReg, uint8_t* dataOut, size_t numRegs) {
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
uint8_t BackupRegs_isValid(void) {
    if((*BACKUP_REGS_BASE & BACKUP_REGS_KEYS_VALID) != BACKUP_REGS_KEYS_VALID) {
        return false;
    } else if (false /*(*BACKUP_REGS_BASE & BACKUP_REGS_AM_VALID) != BACKUP_REGS_AM_VALID*/) {
        return false; /* @TODO once attack mitigation is in, decomment the above line */
    }
    return true;
}



