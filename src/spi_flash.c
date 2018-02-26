/*
 * spi_flash.c
 *
 *  Created on: Mar 7, 2016
 *      Author: me
 */

#include "spi_flash.h"

GPIO_TypeDef* SpiFlash_currentPort = NULL;
uint16_t SpiFlash_currentPin = 0;
volatile bool SpiFlash_interruptRxCompleteFlag = false; /* set to true by the receive complete callback in interrupt mode, which will let the receive function proceed */
uint8_t SpiFlash_pageBuff[SPI_FLASH_PAGE_LEN];



static void SpiFlash_select(void);
static void SpiFlash_deselect(void);
static uint16_t SpiFlash_readDataInterrupt(SPI_HandleTypeDef*, uint8_t*, uint16_t, uint32_t);
static uint16_t SpiFlash_pageProgram(SPI_HandleTypeDef*, uint8_t*, uint16_t, uint32_t);
static bool SpiFlash_readSR1UntilWriteEnabled(SPI_HandleTypeDef*);
static uint8_t SpiFlash_writeEnable(SPI_HandleTypeDef*);






void SpiFlash_select(void) {
    if(SpiFlash_currentPort != NULL) {
        HAL_GPIO_WritePin(SpiFlash_currentPort, SpiFlash_currentPin, RESET);
    } else {
        uart_debug_sendline("SPI Flash Selected but port is null.\n");
    }
}

void SpiFlash_deselect(void) {
    if(SpiFlash_currentPort != NULL) {
        HAL_GPIO_WritePin(SpiFlash_currentPort, SpiFlash_currentPin, SET);
    }
}

void SpiFlash_selectChip0(void) {
    //HAL_GPIO_WritePin(SPI_FLASH_CS0_PORT, SPI_FLASH_CS0_PIN, RESET);
    //HAL_GPIO_WritePin(SPI_FLASH_CS1_PORT, SPI_FLASH_CS1_PIN, SET);
    SpiFlash_selectNone(); //clear any outstanding selections
    SpiFlash_currentPort = SPI_FLASH_CS0_PORT;
    SpiFlash_currentPin = SPI_FLASH_CS0_PIN;
}
void SpiFlash_selectChip1(void) {
    //HAL_GPIO_WritePin(SPI_FLASH_CS0_PORT, SPI_FLASH_CS0_PIN, SET);
    //HAL_GPIO_WritePin(SPI_FLASH_CS1_PORT, SPI_FLASH_CS1_PIN, RESET);
    SpiFlash_selectNone(); //clear any outstanding selections
    SpiFlash_currentPort = SPI_FLASH_CS1_PORT;
    SpiFlash_currentPin = SPI_FLASH_CS1_PIN;
}
void SpiFlash_selectNone(void) {
    HAL_GPIO_WritePin(SPI_FLASH_CS0_PORT, SPI_FLASH_CS0_PIN, SET);
    HAL_GPIO_WritePin(SPI_FLASH_CS1_PORT, SPI_FLASH_CS1_PIN, SET);
    SpiFlash_currentPort = NULL;
    SpiFlash_currentPin = 0;
}
void SpiFlash_reset(void) {
    HAL_GPIO_WritePin(SPI_FLASH_RESET_PORT, SPI_FLASH_RESET_PIN, RESET);
    HAL_Delay(1); //* wait so we get a good reset @TODO: make this a 1 microsecond delay (check datasheet)*/
    HAL_GPIO_WritePin(SPI_FLASH_RESET_PORT, SPI_FLASH_RESET_PIN, SET);
}
void SpiFlash_setHold(uint8_t hold) {
    if(hold) {
        HAL_GPIO_WritePin(SPI_FLASH_WP_PORT, SPI_FLASH_WP_PIN, RESET);
    } else {
        HAL_GPIO_WritePin(SPI_FLASH_WP_PORT, SPI_FLASH_WP_PIN, SET);
    }
}

/* must already have selected desired chip to read*/
size_t SpiFlash_getUniqueID(SPI_HandleTypeDef* hspi, uint8_t* buff, size_t buffLen) {
    SpiFlash_select();
    size_t cmdLen = SPI_FLASH_CMDLEN_GET_UNIQUE_ID;
    uint8_t outBuff[cmdLen];
    uint8_t inBuff[cmdLen];
    outBuff[0] = SPI_FLASH_CMD_GET_UNIQUE_ID;
    memset(outBuff + 1, 0, SPI_FLASH_CMDLEN_GET_UNIQUE_ID - 1);
    uint16_t dummy = 0;
    while(hspi->Instance->SR & SPI_SR_FRLVL) {
        dummy = hspi->Instance->DR; //flush the RX FIFO
    }
    HAL_StatusTypeDef rc = HAL_SPI_TransmitReceive(hspi, outBuff, inBuff, cmdLen, 1000);
    SpiFlash_deselect();
    if(HAL_OK == rc) {
        size_t lenOut = SPI_FLASH_UNIQUE_ID_LEN;
        if(lenOut > buffLen) {
            lenOut = buffLen;
        }
        memcpy(buff, inBuff + SPI_FLASH_UNIQUE_ID_POS, SPI_FLASH_UNIQUE_ID_LEN);
        return SPI_FLASH_UNIQUE_ID_LEN;
    }
    else return 0;
}

uint8_t SpiFlash_readStatusRegister(SPI_HandleTypeDef* hspi, uint8_t statusReg) {
    SpiFlash_select();
    size_t cmdLen = SPI_FLASH_CMDLEN_READ_STATUS_REG;
    uint8_t outBuff[cmdLen];
    uint8_t inBuff[cmdLen];
    switch(statusReg) {
        case 1:
            outBuff[0] = SPI_FLASH_CMD_READ_STATUS_REG_1;
            break;
        case 2:
            outBuff[0] = SPI_FLASH_CMD_READ_STATUS_REG_2;
            break;
        case 3:
            outBuff[0] = SPI_FLASH_CMD_READ_STATUS_REG_3;
            break;
        default:
            SpiFlash_deselect();
            return 0;
    }
    uint16_t dummy = 0;
    while(hspi->Instance->SR & SPI_SR_FRLVL) {
        dummy = hspi->Instance->DR; //flush the RX FIFO
    }
    HAL_StatusTypeDef rc = HAL_SPI_TransmitReceive(hspi, outBuff, inBuff, cmdLen, 1000);
    SpiFlash_deselect();
    /*uart_debug_addToBuffer("SPI Flash Status Register ", 26);
    uart_debug_printuint8(statusReg);
    uart_debug_addToBuffer(": ", 2);
    uart_debug_hexprint16(inBuff[1]);
    uart_debug_newline();*/
    if(HAL_OK == rc) {
        return inBuff[1];
    }
    else return 0x1; //return BUSY
}
/*
 * reads the given status register number, using interrupt mode for the response (slower due to overhead but should not be bugged)
 */
uint8_t SpiFlash_readStatusRegisterInterrupt(SPI_HandleTypeDef* hspi, uint8_t statusRegNum) {
    size_t cmdLen = 1; //SPI_FLASH_CMDLEN_READ_STATUS_REG; 2017-09-06 splitting this up for interrupt operation
    uint8_t cmd;
    uint8_t rsp = 0x99;
    switch(statusRegNum) {
        case 1:
            cmd = SPI_FLASH_CMD_READ_STATUS_REG_1;
            break;
        case 2:
            cmd = SPI_FLASH_CMD_READ_STATUS_REG_2;
            break;
        case 3:
            cmd = SPI_FLASH_CMD_READ_STATUS_REG_3;
            break;
        default:
            return 0x1;
    }
    SpiFlash_select();
    HAL_SPIEx_FlushRxFifo(hspi);
    HAL_StatusTypeDef rc = HAL_SPI_Transmit(hspi, &cmd, cmdLen, 100); //send command
    if(HAL_OK != rc) {
#ifdef DEBUG
        uart_debug_sendstring("SPI Flash Read SR IT Returned at Header: ");
        uart_debug_hexprint32(rc);
        uart_debug_newline();
#endif
        SpiFlash_deselect();
        return 0x1; //BUSY, write disabled
    }
    HAL_SPIEx_FlushRxFifo(hspi);
    SpiFlash_interruptRxCompleteFlag = false;
    rc = HAL_SPI_Receive_IT(hspi, &rsp, 1); //start receive data
    while(!SpiFlash_interruptRxCompleteFlag); //spin until we have it
    SpiFlash_deselect();
    HAL_SPIEx_FlushRxFifo(hspi);
    if(HAL_OK == rc) {
        return rsp;
    } else {
#ifdef DEBUG
        uart_debug_sendstring("SPI Flash Read SR IT Returned at Data: ");
        uart_debug_hexprint32(rc);
        uart_debug_newline();
#endif
        return 0x1;
    }
}

/* for stupid HAL reasons, len is limited to a uint16_t and i don't feel like breaking it up yet
 * @TODO: break it up in to multiple HAL calls so we can read out the entire chip.. if we had the RAM. 64kByte is probably fine >_>
 */
uint16_t SpiFlash_fastRead(SPI_HandleTypeDef* hspi, uint8_t* inBuff, uint16_t len, uint32_t startAddress) {
    size_t cmdLen = SPI_FLASH_CMDLEN_FAST_READ; //this is going to be split up in to two HAL calls
    uint8_t cmdOutBuff[cmdLen];
    packToBuffer32(cmdOutBuff, 0, startAddress);
    cmdOutBuff[0] = SPI_FLASH_CMD_FAST_READ;
    SpiFlash_select();
    HAL_StatusTypeDef rc = HAL_SPI_Transmit(hspi, cmdOutBuff, cmdLen, 1000); //send command, address and dummy byte
    uint16_t dummy = 0;
    while(hspi->Instance->SR & SPI_SR_FRLVL) {
        dummy = hspi->Instance->DR; //flush the RX FIFO
    }
    if(HAL_OK != rc) {
        SpiFlash_deselect();
        return 0; //no data read
    }
    rc = HAL_SPI_Receive(hspi, inBuff, len, 10000); //receive data
    SpiFlash_deselect();
    if(HAL_OK == rc) {
        return len;
    } else return 0;
}
/* for stupid HAL reasons, len is limited to a uint16_t and i don't feel like breaking it up yet
 *
 * 2017-03-02 redirected this to the interrupt version, which actually works reliably (~2500 pages read with no issue, vs >50% failure rates in some cases)
 * @TODO: break it up in to multiple HAL calls so we can read out the entire chip.. if we had the RAM. 64kByte is probably fine >_>
 */
uint16_t SpiFlash_readData(SPI_HandleTypeDef* hspi, uint8_t* inBuff, uint16_t len, uint32_t startAddress) {

    //return SpiFlash_readDataInterrupt(hspi, inBuff, len, startAddress); /* guess what the interrupt version is better*/
    size_t cmdLen = SPI_FLASH_CMDLEN_READ_DATA; //this is going to be split up in to two HAL calls
    uint8_t cmdOutBuff[cmdLen];
    packToBuffer32(cmdOutBuff, 0, startAddress);
    cmdOutBuff[0] = SPI_FLASH_CMD_READ_DATA;
    SpiFlash_select();
    HAL_SPIEx_FlushRxFifo(hspi);
    HAL_StatusTypeDef rc = HAL_SPI_Transmit(hspi, cmdOutBuff, cmdLen, 100); //send command and address
    HAL_SPIEx_FlushRxFifo(hspi);
    if(HAL_OK != rc) {
#ifdef DEBUG
        uart_debug_sendstring("SPI Flash Read Data Returned at Header: ");
        uart_debug_hexprint32(rc);
        uart_debug_newline();
#endif
        SpiFlash_deselect();
        return 0; //no data read
    }
    rc = HAL_SPI_Receive(hspi, inBuff, len, 1000); //receive data
    SpiFlash_deselect();
    HAL_SPIEx_FlushRxFifo(hspi);
    if(HAL_OK == rc) {
        return len;
    } else {
#ifdef DEBUG
        uart_debug_sendstring("SPI Flash Read Data Returned at Data: ");
        uart_debug_hexprint32(rc);
        uart_debug_newline();
#endif
        return 0;
    }
}

/* reads data from the flash in interrupt mode.
 * given the issues with actual blocking mode, blocking mode (SpiFlash_readData) now redirects to this function
 * uses blocking mode to transmit the command then interrupt mode to get the data back
 * still waits until the data is returned or an error occurs, but uses interrupt receive.
 */
uint16_t SpiFlash_readDataInterrupt(SPI_HandleTypeDef* hspi, uint8_t* inBuff, uint16_t len, uint32_t startAddress) {
    size_t cmdLen = SPI_FLASH_CMDLEN_READ_DATA; //this is going to be split up in to two HAL calls
    uint8_t cmdOutBuff[cmdLen];
    packToBuffer32(cmdOutBuff, 0, startAddress);
    cmdOutBuff[0] = SPI_FLASH_CMD_READ_DATA;
    SpiFlash_select();
    HAL_SPIEx_FlushRxFifo(hspi);
    HAL_StatusTypeDef rc = HAL_SPI_Transmit(hspi, cmdOutBuff, cmdLen, 100); //send command and address
    HAL_SPIEx_FlushRxFifo(hspi);
    if(HAL_OK != rc) {
#ifdef DEBUG
        uart_debug_sendstring("SPI Flash Read Data Returned at Header: ");
        uart_debug_hexprint32(rc);
        uart_debug_newline();
#endif
        SpiFlash_deselect();
        return 0; //no data read
    }
    SpiFlash_interruptRxCompleteFlag = false;
    rc = HAL_SPI_Receive_IT(hspi, inBuff, len); //start receive data
    while(!SpiFlash_interruptRxCompleteFlag); //spin until we have it
    SpiFlash_deselect();
    HAL_SPIEx_FlushRxFifo(hspi);
    if(HAL_OK == rc) {
        return len;
    } else {
#ifdef DEBUG
        uart_debug_sendstring("SPI Flash Read Data Returned at Data: ");
        uart_debug_hexprint32(rc);
        uart_debug_newline();
#endif
        return 0;
    }
}


/* len can be a maximum of 256 (full page) */
uint16_t SpiFlash_pageProgram(SPI_HandleTypeDef* hspi, uint8_t* outBuff, uint16_t len, uint32_t startAddress) {
    size_t cmdLen = SPI_FLASH_CMDLEN_PAGE_PROGRAM; //this is going to be split up in to two HAL calls
    uint8_t cmdOutBuff[cmdLen];
    packToBuffer32(cmdOutBuff, 0, startAddress);
    cmdOutBuff[0] = SPI_FLASH_CMD_PAGE_PROGRAM;
    SpiFlash_select();
    HAL_StatusTypeDef rc = HAL_SPI_Transmit(hspi, cmdOutBuff, cmdLen, 1000); //send command and address
    if(HAL_OK != rc) {
        SpiFlash_deselect();
        return 0; //no data read
    }
    rc = HAL_SPI_Transmit(hspi, outBuff, len, 10000); //send data
    SpiFlash_deselect();
    if(HAL_OK == rc) {
        return len;
    } else return 0;
}

/* writes data across page boundaries, if required
 * preconditions:
 *   Flash chip must be selected
 *
 * this function will enable write for itself, as required.
 *
 * param hspi: spi handle to use to communicate on
 * param outBuff: buffer of data to write
 * param len: length of data to write
 * param startAddress: first byte of data is written here
 *
 * returns: 0 or the count of errors if there were any.
 */
uint32_t SpiFlash_write(SPI_HandleTypeDef* hspi, uint8_t* outBuff, uint32_t len, uint32_t startAddress) {
    uint32_t endAddress = startAddress + (len) - 1;
    uint32_t numPages = ((endAddress >> 8) - (startAddress >> 8)) + 1; //must program at least one page. Order of Operations is critical, do not change!
    uint32_t thisPageBytes = SPI_FLASH_PAGE_LEN - (startAddress % SPI_FLASH_PAGE_LEN); //length to write in this page, special handling for first page
    if(thisPageBytes > len) {
        thisPageBytes = len;
    }
    uint32_t bytesToWrite = len;
    uint32_t addressOffset = 0;
    uint8_t errorCount = 0; //cumulative count of errors, we'll push on then report this at the end
    uint32_t rcPrim;
    do {
        if(!SpiFlash_writeEnableAndWait(hspi, SPI_FLASH_WRITE_ENABLE_TIMEOUT)) {
            uart_debug_sendline("Unable to enable write for page program.\n");
            errorCount++;
            rcPrim = 0;
        } else {
            rcPrim = SpiFlash_pageProgram(hspi, outBuff + addressOffset, thisPageBytes, startAddress + addressOffset);
            //SpiFlash_spinUntilNotBusy(hspi); //wait for page program to complete
            SpiFlash_readSR1UntilNotBusy(hspi);
            if(rcPrim != thisPageBytes) {
                errorCount++;
                uart_debug_sendline("Error in page program.\n");
            }

            uart_debug_sendstring("Wrote ");
            uart_debug_printuint32(thisPageBytes);
            uart_debug_sendstring(" bytes at location ");
            uart_debug_printuint32(startAddress + addressOffset);
            uart_debug_sendstring(".\n");
            bytesToWrite -= thisPageBytes;
            addressOffset += thisPageBytes;
            thisPageBytes = bytesToWrite;
            if(thisPageBytes > SPI_FLASH_PAGE_LEN) {
                thisPageBytes = SPI_FLASH_PAGE_LEN;
            }
        }
        //HAL_Delay(100);
    } while(bytesToWrite > 0);
    return errorCount;
}

/*bool SPI_FLASH_IsBusy(SPI_HandleTypeDef* hspi) { //2017-09-06 not used so commented out
    //GPIOB->BSRR = GPIO_PIN_10;
    uint8_t sr1 = SpiFlash_readStatusRegister(hspi, 1);
    //uart_debug_putchar(sr1);
    //uart_debug_newline();
    bool busy = ((sr1 & SPI_FLASH_STS_1_BUSY) == SPI_FLASH_STS_1_BUSY);
    //GPIOB->BRR = GPIO_PIN_10;
    return busy;
}*/


/*bool SPI_FLASH_IsWriteEnabled(SPI_HandleTypeDef* hspi) { //2017-09-06 not used so commented out
    uint8_t sr1 = SpiFlash_readStatusRegister(hspi, 1);
    return (sr1 & SPI_FLASH_STS_1_WEL);
}*/

/**
 * continually reads SR1 as a single transaction until write enabled
 * chip must already be selected
 * returns true if not busy
 * false if there's an error or a timeout
 */
bool SpiFlash_readSR1UntilWriteEnabled(SPI_HandleTypeDef* hspi) {
    size_t cmdLen = SPI_FLASH_CMDLEN_READ_STATUS_REG ;
    uint8_t outBuff[cmdLen];
    uint8_t inBuff[cmdLen];
    outBuff[0] = SPI_FLASH_CMD_READ_STATUS_REG_1;

    HAL_SPIEx_FlushRxFifo(hspi);
    SpiFlash_select();
    HAL_StatusTypeDef rc = HAL_SPI_TransmitReceive(hspi, outBuff, inBuff, cmdLen, 1000);
    if(HAL_OK != rc) {
        SpiFlash_deselect();
        return false;
    }
    while(((inBuff[1] & SPI_FLASH_STS_1_WEL) != SPI_FLASH_STS_1_WEL)) {
        HAL_SPIEx_FlushRxFifo(hspi); //so many buffer flushes
        rc = HAL_SPI_Receive(hspi, inBuff + 1, 1, 100); //read a futher byte
        if(HAL_OK != rc) {
            SpiFlash_deselect();
            return false;
        }
    }
    SpiFlash_deselect();
    /*uart_debug_addToBuffer("SPI Flash Status Register ", 26);
    uart_debug_printuint8(statusReg);
    uart_debug_addToBuffer(": ", 2);
    uart_debug_hexprint16(inBuff[1]);
    uart_debug_newline();*/

    return true;
}



/**
 * continually reads SR1 as a single transaction until not busy
 * chip must already be selected
 * returns true if not busy
 * false if there's an error or a timeout
 */
bool SpiFlash_readSR1UntilNotBusy(SPI_HandleTypeDef* hspi) {
    size_t cmdLen = SPI_FLASH_CMDLEN_READ_STATUS_REG ;
    uint8_t outBuff[cmdLen];
    uint8_t inBuff[cmdLen];
    outBuff[0] = SPI_FLASH_CMD_READ_STATUS_REG_1;

    HAL_SPIEx_FlushRxFifo(hspi);
    SpiFlash_select();
    HAL_StatusTypeDef rc = HAL_SPI_TransmitReceive(hspi, outBuff, inBuff, cmdLen, 1000);
    if(HAL_OK != rc) {
        SpiFlash_deselect();
        return false;
    }
    uint32_t iterCount = 0;
    while(((inBuff[1] & SPI_FLASH_STS_1_BUSY) == SPI_FLASH_STS_1_BUSY) || ((inBuff[1] & SPI_FLASH_STS_1_WEL) == SPI_FLASH_STS_1_WEL)) {
        HAL_SPIEx_FlushRxFifo(hspi); //so many buffer flushes
        rc = HAL_SPI_Receive(hspi, inBuff + 1, 1, 100); //read a futher byte
        //uart_debug_putchar('.');
        if(HAL_OK != rc) {
            SpiFlash_deselect();
            return false;
        }
        /*if((++iterCount & 0x3FF) == 0) {
            uart_debug_hexprint16(inBuff[1]);
            uart_debug_newline();
        }*/
    }
    SpiFlash_deselect();
    /*uart_debug_addToBuffer("SPI Flash Status Register ", 26);
    uart_debug_printuint8(statusReg);
    uart_debug_addToBuffer(": ", 2);
    uart_debug_hexprint16(inBuff[1]);
    uart_debug_newline();*/

    return true;
}

/* erases a chip. use with care
 * will not work if the chip is locked in any way (locked by the chip, that is, this will still run)
 * returns true if the erase has started (unless locked out by the chip)
 * returns false if there was an error enabling write or with the SPI subsystem.
 */
bool SpiFlash_eraseChip(SPI_HandleTypeDef* hspi) {
    if(!SpiFlash_writeEnableAndWait(hspi, SPI_FLASH_WRITE_ENABLE_TIMEOUT)) {
#ifdef DEBUG
        uart_debug_sendline("Unable to enable write for chip erase.\n");
#endif /* DEBG */
        return false;
    }
    SpiFlash_select();
    uint8_t cmdOut = SPI_FLASH_CMD_CHIP_ERASE;
    HAL_StatusTypeDef rc = HAL_SPI_Transmit(hspi, &cmdOut, 1, 1000); //send command
    SpiFlash_deselect();
    if(HAL_OK != rc) {
        return false; //HAL error
    }
    return true;
}

/* returns 1 on success
 * 0 for timeout or error
 */
uint8_t SpiFlash_writeEnableAndWait(SPI_HandleTypeDef* hspi, uint32_t timeout) {
    if(!SpiFlash_writeEnable(hspi)) { //as success is 0..
        //uart_debug_sendline("Waiting for WEL bit...\n");
        /*uint32_t timeoutAt = HAL_GetTick() + timeout;
        while(HAL_GetTick() < timeoutAt) {
            if(SPI_FLASH_IsWriteEnabled(hspi)) {
                //uart_debug_sendline("WEL bit set.\n");
                return true;
            }
        }*/
        return SpiFlash_readSR1UntilWriteEnabled(hspi);
        //uart_debug_sendline("Timeout waiting for WEL bit.\n");
    }
    //uart_debug_sendline("WEL bit not set.\n");
    return false; //timed out or error on write enable
}

uint8_t SpiFlash_writeEnable(SPI_HandleTypeDef* hspi) {
    size_t cmdLen = SPI_FLASH_CMDLEN_WRITE_ENABLE;
    uint8_t outBuff[cmdLen];
    outBuff[0] = SPI_FLASH_CMD_WRITE_ENABLE;
    SpiFlash_select();
    HAL_StatusTypeDef rc = HAL_SPI_Transmit(hspi, outBuff, cmdLen, 1000);
    SpiFlash_deselect();
    if(HAL_OK == rc) {
        //uart_debug_sendline("Write Enable returning success (0).\n");
        return 0;
    } else {
        //uart_debug_sendline("Write Enable returning failure (1).\n");
        return 1;
    }
}

/* erases a sector (4kB) */
/* param sector is the full address, not the sector number! */
uint8_t SpiFlash_eraseSector(SPI_HandleTypeDef* hspi, uint32_t sector) {
    size_t cmdLen = SPI_FLASH_CMDLEN_SECTOR_ERASE; //this is going to be split up in to two HAL calls
    uint8_t cmdOutBuff[cmdLen];
    packToBuffer32(cmdOutBuff, 0, (sector & SPI_FLASH_SECTOR_MASK));
    cmdOutBuff[0] = SPI_FLASH_CMD_SECTOR_ERASE;
    SpiFlash_select();
    HAL_StatusTypeDef rc = HAL_SPI_Transmit(hspi, cmdOutBuff, cmdLen, 1000); //send command and address
    SpiFlash_deselect();
    if(HAL_OK == rc) {
        return 0;
    }
    else return 1;
}

/* reads the SFDP */
uint8_t SpiFlash_readSFDP(SPI_HandleTypeDef* hspi, uint8_t* data, size_t maxLen, size_t offset) {
    if(maxLen > SPI_FLASH_SFDP_MAX_LEN) {
        maxLen = SPI_FLASH_SFDP_MAX_LEN;
    }
    size_t cmdLen = SPI_FLASH_CMDLEN_READ_SFDP; //this is going to be split up in to two HAL calls
    uint8_t cmdOutBuff[cmdLen];

    cmdOutBuff[0] = SPI_FLASH_CMD_READ_SFDP;
    cmdOutBuff[1] = 0;
    cmdOutBuff[2] = 0;
    cmdOutBuff[3] = 0;
    cmdOutBuff[4] = 0;
    //cmdOutBuff[3] = (cmdOutBuff, 0, (offset & SPI_FLASH_SFDP_MASK));
    SpiFlash_select();
    HAL_StatusTypeDef rc = HAL_SPI_Transmit(hspi, cmdOutBuff, cmdLen, 1000); //send command and address
    if(HAL_OK != rc) {
        SpiFlash_deselect();
        uart_debug_sendline("HAL Failure on SPI transmit.\n");
        return 0; //no data read
    }
    uint16_t dummy = 0;
    while(hspi->Instance->SR & SPI_SR_FRLVL) {
        dummy = hspi->Instance->DR; //flush the RX FIFO
    }
    rc = HAL_SPI_Receive(hspi, data, maxLen, 10000); //receive data
    SpiFlash_deselect();
    if(HAL_OK == rc) {

        return maxLen;
    } else {
        uart_debug_sendline("HAL Failure on SPI receive.\n");
        return 0;
    }
}

//hexdumps flash with line starters for the location
//aligns start on a 16 byte boundary by masking off 4 LSbits
//goes for at least len and always dumps in 16 byte chunks
void SpiFlash_hexDumpFlash(SPI_HandleTypeDef* hspi, uint32_t start, uint32_t len) {
    uint32_t pageLen = SPI_FLASH_PAGE_LEN; //how many bytes to fetch at once. reduces SPI overhead
    uint8_t* pageBuffer = SpiFlash_pageBuff;

    uint32_t lineLen = DEBUG_UART_HEXDUMP_BLOCK_SIZE;
    uint32_t bytesDumped = 0;
    uint32_t dumpStart = start & 0xFFFFFFF0; //mask off LSbits to align
    uint32_t dumpLen = (start - dumpStart) + len;
    if(dumpLen & 0xF) {
        dumpLen = (dumpLen + 16) & 0xFFFFFFF0; //crude end alignment, add 16 and mask off the LSbits
    }
    while(bytesDumped < dumpLen) {
        uint32_t thisPageLen = pageLen;
        if(thisPageLen > (dumpLen - bytesDumped)) {
            thisPageLen = dumpLen - bytesDumped;
        }
        uint16_t rc = SpiFlash_readData(hspi, pageBuffer, thisPageLen, dumpStart + bytesDumped);
        if(rc) {
            for(uint32_t i = 0; i < (thisPageLen / lineLen); ++i) {
                uart_debug_hexprint32(dumpStart + bytesDumped + lineLen * i);
                uart_debug_sendstring(": ");
                uart_debug_hexdump(pageBuffer + lineLen * i, lineLen);
            }
        }
        bytesDumped += thisPageLen;
        HAL_Delay(75); //pause between pages to ensure it all makes it out
    }
}

/* hexdumps flash in interrupt mode
 * as of 2017-03-02 this is intended for testing as the regular readData isn't working properly for unknown reasons (stalling out)
 */
void SpiFlash_hexDumpFlashInterrupt(SPI_HandleTypeDef* hspi, uint32_t start, uint32_t len) {
    uint32_t pageLen = SPI_FLASH_PAGE_LEN; //how many bytes to fetch at once. reduces SPI overhead
    uint8_t* pageBuffer = SpiFlash_pageBuff;

    uint32_t lineLen = DEBUG_UART_HEXDUMP_BLOCK_SIZE;
    uint32_t bytesDumped = 0;
    uint32_t dumpStart = start & 0xFFFFFFF0; //mask off LSbits to align
    uint32_t dumpLen = (start - dumpStart) + len;
    if(dumpLen & 0xF) {
        dumpLen = (dumpLen + 16) & 0xFFFFFFF0; //crude end alignment, add 16 and mask off the LSbits
    }
    while(bytesDumped < dumpLen) {
        uint32_t thisPageLen = pageLen;
        if(thisPageLen > (dumpLen - bytesDumped)) {
            thisPageLen = dumpLen - bytesDumped;
        }
        uint16_t rc = SpiFlash_readDataInterrupt(hspi, pageBuffer, thisPageLen, dumpStart + bytesDumped);
        if(rc == thisPageLen) {
            for(uint32_t i = 0; i < (thisPageLen / lineLen); ++i) {
                uart_debug_hexprint32(dumpStart + bytesDumped + lineLen * i);
                uart_debug_sendstring(": ");
                uart_debug_hexdump(pageBuffer + lineLen * i, lineLen);
            }
        } else if(rc) {
            uint32_t errorPageLen = rc;
            if(errorPageLen & 0xF) {
                errorPageLen = (errorPageLen + 16) & 0xFFFFFFF0; //crude end alignment, add 16 and mask off the LSbits
            }
            for(uint32_t i = 0; i < (errorPageLen / lineLen); ++i) {
                uart_debug_hexprint32(dumpStart + bytesDumped + lineLen * i);
                uart_debug_sendstring("! ");
                uart_debug_hexdump(pageBuffer + lineLen * i, lineLen);
            }
        }
        bytesDumped += thisPageLen;
        HAL_Delay(75); //pause between pages to ensure it all makes it out
    }
}

void SpiFlash_interruptRxComplete(SPI_HandleTypeDef* hspi) {
    (void*) hspi; //suppress unused parameter warning
    SpiFlash_interruptRxCompleteFlag = true;
    uart_debug_sendline("SPI Flash Rx Complete Interrupt!\n"); //2017-09-07 this is apparently a critical part of the code. doesn't seem to want to work without it. *sigh*
}

