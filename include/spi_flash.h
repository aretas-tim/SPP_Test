
#ifndef SPI_FLASH_H
#define SPI_FLASH_H

#include "stm32l476xx.h"
#include "stm32l4xx_hal.h"
#include "uart_debug.h"
#include <string.h> /* memset*/
#include <stdbool.h>
#include "pat-config.h"
#include "utilities.h" /* packToBuffer32 */
#include "sram2.h"

#ifdef DEVBOARD

#define SPI_FLASH_CS0_PIN GPIO_PIN_15
#define SPI_FLASH_CS0_PORT GPIOA

#define SPI_FLASH_CS1_PIN GPIO_PIN_2
#define SPI_FLASH_CS1_PORT GPIOC

#define SPI_FLASH_RESET_PIN GPIO_PIN_2
#define SPI_FLASH_RESET_PORT GPIOD

#define SPI_FLASH_WP_PIN GPIO_PIN_7
#define SPI_FLASH_WP_PORT GPIOB

#elif defined DEVICE_0_A_0_1

#define SPI_FLASH_CS0_PIN GPIO_PIN_2
#define SPI_FLASH_CS0_PORT GPIOA

#define SPI_FLASH_CS1_PIN GPIO_PIN_3
#define SPI_FLASH_CS1_PORT GPIOA

#define SPI_FLASH_RESET_PIN GPIO_PIN_1
#define SPI_FLASH_RESET_PORT GPIOA

#define SPI_FLASH_WP_PIN GPIO_PIN_4
#define SPI_FLASH_WP_PORT GPIOC

#elif defined DEVICE_0_A_1_1_S

#define SPI_FLASH_CS0_PIN GPIO_PIN_4
#define SPI_FLASH_CS0_PORT GPIOA

#define SPI_FLASH_CS1_PIN GPIO_PIN_4 /* duplicating this as there IS no CS1 in this version */
#define SPI_FLASH_CS1_PORT GPIOA

#define SPI_FLASH_RESET_PIN GPIO_PIN_0
#define SPI_FLASH_RESET_PORT GPIOB

#define SPI_FLASH_WP_PIN GPIO_PIN_1
#define SPI_FLASH_WP_PORT GPIOB

#elif defined DEVICE_0_A_1_1_U

#define SPI_FLASH_CS0_PIN GPIO_PIN_4
#define SPI_FLASH_CS0_PORT GPIOB

#define SPI_FLASH_CS1_PIN GPIO_PIN_4 /* duplicating this as there IS no CS1 in this version */
#define SPI_FLASH_CS1_PORT GPIOB

#define SPI_FLASH_RESET_PIN GPIO_PIN_5
#define SPI_FLASH_RESET_PORT GPIOB

#define SPI_FLASH_WP_PIN GPIO_PIN_6
#define SPI_FLASH_WP_PORT GPIOB

#else
#error "Must define SPI Flash Pins"
#endif /* DEVICE_0_A_0_1 */

#define SPI_FLASH_WRITE_ENABLE_TIMEOUT 30 //milliseconds, get this value from the datasheet and double it just to be sure

#define SPI_FLASH_STS_1_BUSY 0x01 //bitmask values
#define SPI_FLASH_STS_1_WEL 0x02
#define SPI_FLASH_STS_1_BP0 0x04
#define SPI_FLASH_STS_1_BP1 0x08
#define SPI_FLASH_STS_1_BP2 0x10
#define SPI_FLASH_STS_1_TB 0x20
#define SPI_FLASH_STS_1_SEC 0x40
#define SPI_FLASH_STS_1_SRP0 0x80

#define SPI_FLASH_CMD_READ_STATUS_REG_1 0x05
#define SPI_FLASH_CMD_READ_STATUS_REG_2 0x35
#define SPI_FLASH_CMD_READ_STATUS_REG_3 0x15
#define SPI_FLASH_CMD_GET_UNIQUE_ID 0x4B

#define SPI_FLASH_CMD_WRITE_ENABLE 0x06
#define SPI_FLASH_CMD_WRITE_ENABLE_VOLATILE 0x50
#define SPI_FLASH_CMD_WRITE_DISABLE 0x04
#define SPI_FLASH_CMD_PAGE_PROGRAM 0x02
#define SPI_FLASH_CMD_SECTOR_ERASE 0x20
#define SPI_FLASH_CMD_CHIP_ERASE 0x60

#define SPI_FLASH_CMD_READ_DATA 0x03
#define SPI_FLASH_CMD_FAST_READ 0x0B


#define SPI_FLASH_UNIQUE_ID_LEN 8 //length of ID in bytes
#define SPI_FLASH_UNIQUE_ID_POS 5 //position of first byte (from datasheet)
#define SPI_FLASH_CMDLEN_GET_UNIQUE_ID 13

#define SPI_FLASH_CMDLEN_READ_STATUS_REG 2 //byte out, byte in

#define SPI_FLASH_CMDLEN_FAST_READ 5 //command, 24 bit address, dummy, plus length of actual read
#define SPI_FLASH_CMDLEN_READ_DATA 4 //command, 24 bit address plus length of actual read

#define SPI_FLASH_CMDLEN_PAGE_PROGRAM 4 //command, 24 bit address plus length of actual write

#define SPI_FLASH_CMDLEN_SECTOR_ERASE 4 //command, 24 bit sector address
#define SPI_FLASH_SECTOR_MASK 0x00FFF000 //masks off the 12 bits actually used for the sector address.

#define SPI_FLASH_CMDLEN_WRITE_ENABLE 1

#define SPI_FLASH_PAGE_LEN 256
#define SPI_FLASH_SECTOR_LEN 4096
#define SPI_FLASH_CHIP_LEN 0x01000000 //16777216

#define SPI_FLASH_CMD_READ_SFDP 0x5A
#define SPI_FLASH_CMDLEN_READ_SFDP 5 //outgoing length
#define SPI_FLASH_SFDP_MASK 0x000000FF //mask off any extraneous address bits
#define SPI_FLASH_SFDP_MAX_LEN 0x100 //maximum incoming length

void SpiFlash_reset(void);
void SpiFlash_setHold(uint8_t hold);
void SpiFlash_selectChip0(void);
void SpiFlash_selectChip1(void);
void SpiFlash_selectNone(void);
size_t SpiFlash_getUniqueID(SPI_HandleTypeDef* hspi, uint8_t* buff, size_t buffLen);
uint8_t SpiFlash_readSFDP(SPI_HandleTypeDef* hspi, uint8_t* data, size_t maxLen, size_t offset);
uint16_t SpiFlash_fastRead(SPI_HandleTypeDef* hspi, uint8_t* inBuff, uint16_t len, uint32_t startAddress);
uint16_t SpiFlash_readData(SPI_HandleTypeDef* hspi, uint8_t* inBuff, uint16_t len, uint32_t startAddress);
uint16_t SpiFlash_readDataInterrupt(SPI_HandleTypeDef* hspi, uint8_t* inBuff, uint16_t len, uint32_t startAddress);
uint8_t SpiFlash_writeEnable(SPI_HandleTypeDef* hspi);
uint8_t SpiFlash_writeEnableAndWait(SPI_HandleTypeDef* hspi, uint32_t timeout);
void SpiFlash_spinUntilNotBusy(SPI_HandleTypeDef* hspi);
bool SpiFlash_readSR1UntilNotBusy(SPI_HandleTypeDef* hspi);
uint16_t SpiFlash_pageProgram(SPI_HandleTypeDef* hspi, uint8_t* outBuff, uint16_t len, uint32_t startAddress);
uint8_t SpiFlash_eraseSector(SPI_HandleTypeDef* hspi, uint32_t sector);
bool SpiFlash_eraseChip(SPI_HandleTypeDef* hspi);
uint32_t SpiFlash_write(SPI_HandleTypeDef* hspi, uint8_t* outBuff, uint32_t len, uint32_t startAddress);
void SpiFlash_hexDumpFlash(SPI_HandleTypeDef* hspi, uint32_t start, uint32_t len);
void SpiFlash_hexDumpFlashInterrupt(SPI_HandleTypeDef* hspi, uint32_t start, uint32_t len);
void SpiFlash_interruptRxComplete(SPI_HandleTypeDef* hspi);


#endif /* SPI_FLASH_H */
