/*
 * bfss.h
 *
 *  Created on: Jul 5, 2016
 *      Author: me
 */
#include "sram2.h"
#include "spi_flash.h"
#include <stdbool.h>
//#include "tpm_nonce.h" /* getRandomBuff() */
//#include "tpm_authsession.h" /*compareDigests*/
#include "gcm.h"

#ifndef BFSS_H_
#define BFSS_H_



typedef struct tdBFSS_FlashInfo {
    uint32_t usableSectorCount;
    uint32_t unallocatedSectorCount;
    uint32_t clearedSectorCount;
    uint8_t* sectorBitmap;
} BFSS_FlashInfo;


#define BFSS_RECORD_OFFSET_LEN 0
#define BFSS_RECORD_OFFSET_ID 2
#define BFSS_RECORD_OFFSET_HEADER_LEN 4
#define BFSS_RECORD_BACK_OFFSET_TAG 16 /*this is the distance backwards from the end */

#define BFSS_RECORD_LEN_LEN 2
#define BFSS_RECORD_LEN_ID 2
#define BFSS_RECORD_LEN_HEADER_LEN 1
#define BFSS_RECORD_LEN_IV 12
#define BFSS_RECORD_LEN_TAG 16

#define BFSS_RECORD_MAX_HEADER_LEN 255
#define BFSS_RECORD_MAX_LEN 1024 /* maximum overall length*/

#define BFSS_RECORD_MIN_LEN (BFSS_RECORD_LEN_LEN + BFSS_RECORD_LEN_ID + BFSS_RECORD_LEN_HEADER_LEN) /*absolute minimum, overall length, type, header length*/
#define BFSS_RECORD_NON_HEADER_MIN_LEN (BFSS_RECORD_LEN_IV + BFSS_RECORD_LEN_TAG + 1) /*absolute minimum to have a single byte of encrypted data*/

#define BFSS_ENC_CUSTOM_ENTRY_TYPE_ENTRY_ENC_BLOCK_TYPE_POS 0 /*encrypted custom entry type entry, encrypted block, position of type*/
#define BFSS_ENC_CUSTOM_ENTRY_TYPE_ENTRY_ENC_BLOCK_DESC_POS 2 /*encrypted custom entry type entry, encrypted block, position of description*/

#define BFSS_ENC_CUSTOM_ENTRY_TYPE_ENTRY_AUTH_BLOCK_LEN 4 /* encrypted custom entry type entry, length of header (authenticated but not encrypted) data */

#define BFSS_ENC_CUSTOM_ENTRY_TYPE_ENTRY_IV_POS 4 /* encrypted custom entry type entry, IV position */
#define BFSS_ENC_CUSTOM_ENTRY_TYPE_ENC_BLOCK_POS 16 /* encrypted custom entry type entry, encrypted block position*/


#define BFSS_CUSTOM_ENTRY_TYPE_NEXT_SECTOR_ENTRY_SECTOR_NUM_POS 4 /*custom entry type next sector entry, position of next sector number*/
#define BFSS_CLEAR_CUSTOM_ENTRY_TYPE_TYPE_POS 4 /* clear custom entry type, type being described position */
#define BFSS_CLEAR_CUSTOM_ENTRY_TYPE_DESC_POS 6 /* clear custom entry type, description start position*/


typedef enum tdAuthState {
    AUTH_STATE_NOT_PRESENT,
    AUTH_STATE_FAILED,
    AUTH_STATE_OKAY
} AuthState;

typedef struct tdBFSS_Record {
    uint16_t id;
    AuthState state;
    uint8_t headerLen;
    uint8_t* header;
    uint16_t dataLen; //length of encData
    uint8_t* data;

} BFSS_Record;

#define BFSS_SECTOR_MAX_RECORDS 127 //limits size of our offset array

typedef struct tdBFSS_SectorCacheRecord {
    uint16_t offset;
    uint16_t id;
} BFSS_SectorCacheRecord;

/* generally, do NOT instantiate one of these (they're 4608 bytes in length!)
 * just cast a location in SRAM2 to them
 * hides some uglyness and stuff.
 */
typedef struct tdBFSS_SectorCache {
    uint16_t sectorNum;
    uint8_t valid;
    uint8_t numRecords;
    BFSS_SectorCacheRecord records[BFSS_SECTOR_MAX_RECORDS];
    uint8_t data[SPI_FLASH_SECTOR_LEN];
} BFSS_SectorCache;

/* return codes */
#define BFSS_SUCCESS 0
#define BFSS_FAILED 1
#define BFSS_INVALID_SECTOR 2
#define BFSS_FLASH_READ_FAILED 3
#define BFSS_SECTOR_TABLE_ERROR 4
#define BFSS_FLASH_WRITE_FAILED 5
#define BFSS_NEEDS_INITIALIZATION 6
#define BFSS_NO_SPACE 7
#define BFSS_NO_KEY 8
#define BFSS_CRYPT_FAILED 9
#define BFSS_NO_SUCH_RECORD 10
#define BFSS_RECORD_LENGTH_UNEXPECTED 11
#define BFSS_AUTHFAIL 12
#define BFSS_RECORD_TOO_LONG 13
#define BFSS_OUT_OF_MEMORY 14
#define BFSS_MUST_READ_TYPE_INFO 15
#define BFSS_CACHE_FAILED 16
#define BFSS_NO_COMPRESSION_POSSIBLE 17
#define BFSS_COMPRESSION_WILL_NOT_FREE_ENOUGH_SPACE 18
#define BFSS_CLEAR_IN_PROGRESS 19 /* technically a success, but returning this indicates that the clear is still in progress, as it can be a long-running task. */

#define BFSS_SECTOR_NUM_TO_LOCATION_SHIFT_AMOUNT 12

#define BFSS_SECTOR_TABLE_LOCATION 0

#define BFSS_SECTOR_TABLE_HEADER_LEN 4 /* number of bytes in the header*/
#define BFSS_SECTOR_TABLE_HEADER_MAGIC_NUMBER 0x53534642



#define BFSS_SECTOR_COUNT (SPI_FLASH_CHIP_LEN / SPI_FLASH_SECTOR_LEN) //count of sectors
#define BFSS_SECTOR_BITMAP_LEN (BFSS_SECTOR_COUNT / 8)

#define BFSS_SECTOR_TABLE_SECTOR_LEN 2 //length of the sector table itself, in sectors

#define BFSS_SECTOR_LOCATION_UNAVAILABLE 0xFFFFFFFF
#define BFSS_SECTOR_LOCATION_LAST_VALID (SPI_FLASH_CHIP_LEN - SPI_FLASH_SECTOR_LEN)

#define BFSS_SECTOR_NUM_CLEARTEXT BFSS_SECTOR_TABLE_SECTOR_LEN //cleartext sector comes right after the sector tables.
#define BFSS_SECTOR_NUM_HOTKEY (BFSS_SECTOR_NUM_CLEARTEXT + 1) //hotkey storage sector is after the cleartext
#define BFSS_SECTOR_NUM_CUSTOM_TYPE_START BFSS_SECTOR_LOCATION_LAST_VALID
#define BFSS_SECTOR_NUM_INVALID 0xFFFF

#define BFSS_SECTOR_STATE_UNALLOCATED 0xFFFF
#define BFSS_SECTOR_STATE_BAD 0xFFFE
#define BFSS_SECTOR_STATE_CLEARED 0x0000

#define BFSS_SECTOR_STATE_ENTRY_TYPES 0x8000

#define BFSS_NUM_FIXED_SECTORS 2 /* number of fixed sectors, currently cleartext and hotkey sectors */
#define BFSS_INIT_FIXED_LEN (BFSS_SECTOR_TABLE_HEADER_LEN + ( BFSS_NUM_FIXED_SECTORS * 2 )) /* length of fixed initialization section */

/*these are used to return additional information to the host on record write */
#define BFSS_WRITE_FLAGS_COMPRESSION_PERFORMED 0x01
#define BFSS_WRITE_FLAGS_SYSTEM_FUNCTION_PERFORMED 0x02



#define BFSS_RECORD_LEN_NOT_PRESENT 0xFFFF

#define BFSS_BYTE_EMPTY 0xFF

/* system reserved entry types */
#define BFSS_RECORD_ID_NONE 0x0000
#define BFSS_RECORD_ID_SYSTEM_RESERVED_BASE 0xFF00
#define BFSS_RECORD_ID_SYSTEM_RESERVED_TOP 0xFFFF
#define BFSS_RECORD_ID_SYSTEM_RESERVED_MASK 0x00FF
#define BFSS_RECORD_ID_ALWAYS_RESERVED 0xFFFF

/*these get extra special handling*/
#define BFSS_RECORD_ID_TYPE_DESCRIPTION_CLEAR 0xFF00 /*entry type description, cleartext*/
#define BFSS_RECORD_ID_TYPE_DESCRIPTION_ENC 0xFF01 /*entry type description, encrypted*/
#define BFSS_RECORD_ID_TYPE_DESCRIPTION_NEXT_SECTOR 0xFF02 /*points to the next sector of entry type descriptions*/

#define BFSS_RECORD_LEN_TYPE_DESCRIPTION_CLEAR 6 /*length with zero-length description*/
#define BFSS_RECORD_LEN_TYPE_DESCRIPTION_ENC 34 /*length with zero-length description*/
#define BFSS_RECORD_LEN_TYPE_DESCRIPTION_NEXT_SECTOR 6 /*length with zero-length description*/

/* hotkey record IDs for internal use by the hotkey subsystem */
#define BFSS_RECORD_ID_HOTKEY_BASE 0xFF10
#define BFSS_RECORD_ID_HOTKEY_TOP 0xFF1F
#define BFSS_RECORD_ID_HOTKEY_0 (BFSS_RECORD_ID_HOTKEY_BASE + 0x0)
#define BFSS_RECORD_ID_HOTKEY_1 (BFSS_RECORD_ID_HOTKEY_BASE + 0x1)
#define BFSS_RECORD_ID_HOTKEY_2 (BFSS_RECORD_ID_HOTKEY_BASE + 0x2)
#define BFSS_RECORD_ID_HOTKEY_3 (BFSS_RECORD_ID_HOTKEY_BASE + 0x3)
#define BFSS_RECORD_ID_HOTKEY_4 (BFSS_RECORD_ID_HOTKEY_BASE + 0x4)
#define BFSS_RECORD_ID_HOTKEY_5 (BFSS_RECORD_ID_HOTKEY_BASE + 0x5)
#define BFSS_RECORD_ID_HOTKEY_6 (BFSS_RECORD_ID_HOTKEY_BASE + 0x6)
#define BFSS_RECORD_ID_HOTKEY_7 (BFSS_RECORD_ID_HOTKEY_BASE + 0x7)
#define BFSS_RECORD_ID_HOTKEY_8 (BFSS_RECORD_ID_HOTKEY_BASE + 0x8)
#define BFSS_RECORD_ID_HOTKEY_9 (BFSS_RECORD_ID_HOTKEY_BASE + 0x9)
#define BFSS_RECORD_ID_HOTKEY_A (BFSS_RECORD_ID_HOTKEY_BASE + 0xA)
#define BFSS_RECORD_ID_HOTKEY_B (BFSS_RECORD_ID_HOTKEY_BASE + 0xB)
#define BFSS_RECORD_ID_HOTKEY_C (BFSS_RECORD_ID_HOTKEY_BASE + 0xC)
#define BFSS_RECORD_ID_HOTKEY_D (BFSS_RECORD_ID_HOTKEY_BASE + 0xD)
#define BFSS_RECORD_ID_HOTKEY_E (BFSS_RECORD_ID_HOTKEY_BASE + 0xE)
#define BFSS_RECORD_ID_HOTKEY_F (BFSS_RECORD_ID_HOTKEY_BASE + 0xF)







#define BFSS_OUTPUT_LEN_RECORD_TYPE_INFO 4 /*length of an output entry, used for ReadTypeInfo*/

#define BFSS_KEY_LEN 32


uint32_t BFSS_Init(SPI_HandleTypeDef* hspi, void (*chipSelFunc)(void), void (*chipDeselFunc)(void));
void BFSS_ClearKey(void);
bool BFSS_IsInitialized(void);

bool BFSS_IsBusy(void);

void BFSS_InitFlashInfo(BFSS_FlashInfo* info);
void BFSS_DumpInfo();
void BFSS_InitSectorCache(BFSS_SectorCache* cache);
void BFSS_InitRecord(BFSS_Record* record);
void BFSS_FreeRecord(BFSS_Record* record);
uint32_t BFSS_CheckFileSystem();
uint32_t BFSS_InitFileSystem();
uint32_t BFSS_ClearFileSystem();
uint32_t BFSS_GetFlashInfo(BFSS_FlashInfo* info);

uint32_t BFSS_WriteAllocationTable(uint16_t sectorNum, uint32_t count);
uint32_t BFSS_AllocateSector(uint16_t sectorCount, uint16_t* sectorNum);
uint32_t BFSS_EraseSector(uint16_t sectorNum);
uint32_t BFSS_CompressSector(BFSS_SectorCache* sector, uint16_t idToRemove, uint32_t minFreeSpace);

BFSS_FlashInfo* BFSS_GetFlashInfoPtr();

uint32_t BFSS_SetKey(uint8_t key[BFSS_KEY_LEN]);

uint32_t BFSS_GetSectorHeaderLocation(uint16_t sectorNum);
BFSS_SectorCache* BFSS_CacheSector(uint16_t sectorNum);
void BFSS_CachePurge(uint16_t sectorNum);
void BFSS_DumpCache();
uint32_t BFSS_CalcFreeSpace(BFSS_SectorCache* sector);

uint32_t BFSS_ReadRecord(uint16_t sectorNum, BFSS_Record* entry, uint8_t historicalDepth);
uint32_t BFSS_WriteRecord(uint16_t sectorNum, BFSS_Record* entry, uint8_t* compressionPerformed);
uint32_t BFSS_WriteCleartextRecord(uint16_t sectorNum, BFSS_Record* entry, uint8_t* compressionPerformed);
uint32_t BFSS_ReadCleartextRecord(uint16_t sectorNum, BFSS_Record* entry, uint8_t historicalDepth);

void BFSS_DumpSectorCacheRecords(BFSS_SectorCache* cache);
void BFSS_DumpRecord(BFSS_Record* entry);

uint32_t BFSS_WriteTypeInfo(uint16_t type, uint16_t len, uint8_t* desc, uint8_t doEncrypt);
uint32_t BFSS_ReadTypeInfo(uint8_t initialRead, uint8_t* moreEntries, uint8_t* entryCount, uint8_t* outputBuff, uint16_t* outputBuffLen);

#ifdef DEBUG
void BFSS_TEST_AddEntry(void);
#endif /* DEBUG */

#endif /* BFSS_H_ */
