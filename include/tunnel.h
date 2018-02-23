/*
 * tunnel.h
 *
 *  Created on: Apr 5, 2016
 *      Author: me
 */

#ifndef TUNNEL_H_
#define TUNNEL_H_

#include "tpm_utils.h"
//#include "tpm_nonce.h" /*getRandomBuff*/
#include "md.h"
#include "sha256.h"
#include "sha512.h"
#include "aes.h"
#include "pkcs5.h" /* pbkdf2*/
//#include "tpm_authsession.h" /* TPM_AES_CTR_Crypt*/
#include "authdata_store.h"
//#include "tpm_structures.h"
//#include "tpm_i2c.h"
#include "tunnel_structures.h"
#include "led.h"
#include "adc.h"
#include "ownership.h"

#define TUNNEL_MINIMUM_OTC_LEN 6
#define TUNNEL_MAXIMUM_OTC_LEN 64


#define TUNNEL_HEADER 0xAAAA
#define TUNNEL_HEADER_BYTE 0xAA
#define TUNNEL_TAG_NULL 0xFF00
#define TUNNEL_TAG_CMD_CLEAR 0xFF01
#define TUNNEL_TAG_CMD_ENC 0xFF02
#define TUNNEL_TAG_RSP_CLEAR 0xFF81
#define TUNNEL_TAG_RSP_ENC 0xFF82


#define TUNNEL_ORD_BAD_TAG 0xFFFFFFFF /*flag for bad tag */
#define TUNNEL_ORD_BAD_LENGTH 0xFFFFFFFE /*flag for bad length */
#define TUNNEL_ORD_TEST 0x7FFFFFFF /* debug builds only*/
#define TUNNEL_ORD_NONE 0x80000000
#define TUNNEL_ORD_INIT TUNNEL_ORD_NONE + 0x01
#define TUNNEL_ORD_END TUNNEL_ORD_NONE + 0x02
#define TUNNEL_ORD_GET_OTC_SETUP TUNNEL_ORD_NONE + 0x03
#define TUNNEL_ORD_TIME_GET TUNNEL_ORD_NONE + 0x04
#define TUNNEL_ORD_TIME_SET TUNNEL_ORD_NONE + 0x05
#define TUNNEL_ORD_GET_VOLTAGES TUNNEL_ORD_NONE + 0x06
#define TUNNEL_ORD_CLEAR TUNNEL_ORD_NONE + 0x08
#define TUNNEL_ORD_CLEAR_FORCE TUNNEL_ORD_NONE + 0x09
#define TUNNEL_ORD_SETUP_INITIAL TUNNEL_ORD_NONE + 0x0A
#define TUNNEL_ORD_OWNER_SETUP_AM TUNNEL_ORD_NONE + 0x0B
#define TUNNEL_ORD_OWNER_SETUP_TUNNEL TUNNEL_ORD_NONE + 0x0C
#define TUNNEL_ORD_OWNER_SETUP_UNLOCK TUNNEL_ORD_NONE + 0x0D
#define TUNNEL_ORD_OWNER_SETUP_RELOCK TUNNEL_ORD_NONE + 0x0E
#define TUNNEL_ORD_OWNER_GET_CONFIG TUNNEL_ORD_NONE + 0x0F



#define TUNNEL_ORD_TPM_GET_CAPABILITY TUNNEL_ORD_NONE + 0x10
#define TUNNEL_ORD_TPM_GET_CAPABILITY_OWNER TUNNEL_ORD_NONE + 0x11
#define TUNNEL_ORD_TPM_SET_CAPABILITY TUNNEL_ORD_NONE + 0x12
#define TUNNEL_ORD_TPM_SEAL TUNNEL_ORD_NONE + 0x20
#define TUNNEL_ORD_TPM_UNSEAL TUNNEL_ORD_NONE + 0x21
#define TUNNEL_ORD_TPM_UNBIND TUNNEL_ORD_NONE + 0x22
#define TUNNEL_ORD_TPM_CREATE_WRAP_KEY TUNNEL_ORD_NONE + 0x23
#define TUNNEL_ORD_TPM_LOAD_KEY TUNNEL_ORD_NONE + 0x24
#define TUNNEL_ORD_TPM_GET_PUB_KEY TUNNEL_ORD_NONE + 0x25
#define TUNNEL_ORD_TPM_NV_DEFINE_SPACE TUNNEL_ORD_NONE + 0x30
#define TUNNEL_ORD_TPM_NV_READ TUNNEL_ORD_NONE + 0x31
#define TUNNEL_ORD_TPM_NV_WRITE TUNNEL_ORD_NONE + 0x32
#define TUNNEL_ORD_SETUP_SRK TUNNEL_ORD_NONE + 0x100
#define TUNNEL_ORD_SETUP_DAM TUNNEL_ORD_NONE + 0x101


/*expected (minimum) parameter lengths for commands */
#define TUNNEL_ORD_FORCE_CLEAR_PLEN 8
#define TUNNEL_ORD_TPM_GET_CAPABILITY_PLEN 8
#define TUNNEL_PLEN_ORD_TPM_SETUP_DAM 2

#define TUNNEL_FORCE_CLEAR_KEY_1 0xF0E1D2C3
#define TUNNEL_FORCE_CLEAR_KEY_2 0xB4A59687

#define TUNNEL_RSP_SUCCESS 0x0
#define TUNNEL_RSP_AUTHFAIL 0x1
#define TUNNEL_RSP_BAD_LENGTH 0x2
#define TUNNEL_RSP_NO_TRANSPORT_KEY 0x3 /* as of 2016-07-26, this is not used anywhere */
#define TUNNEL_RSP_BAD_HEADER 0x4
#define TUNNEL_RSP_OUT_OF_MEMORY 0x5
#define TUNNEL_RSP_BAD_TAG 0x6
#define TUNNEL_RSP_ORD_UNKNOWN 0x7
#define TUNNEL_RSP_FAILED 0x8 /* general didn't work error when there isn't a more specific one or we don't want to give a more specific one */
#define TUNNEL_RSP_BAD_ATTRIBUTE 0x9 /* for NV Define Space attributes (and others?)*/
#define TUNNEL_RSP_PCR_INFO_INVALID 0xA
#define TUNNEL_RSP_ORD_NOT_IMPLEMENTED 0xB
#define TUNNEL_RSP_TPM_OWNED 0xC /*if we're trying to do something on a TPM that requires no owner, but the TPM already has an owner*/
#define TUNNEL_RSP_BAD_PARAMETER 0xD
#define TUNNEL_RSP_RESERVED_INDEX 0xE /* if a reserved index value is specified*/
#define TUNNEL_RSP_SESSION_EXISTS 0xF
#define TUNNEL_RSP_TIMEOUT 0x10
#define TUNNEL_RSP_DEVICE_OWNED 0x11 /* if we're trying to do something that requires the device to have no owner, but an owner is present */


#define TUNNEL_RSP_TPM_CODE 0x00001000 /* passing on a TPM response code */
#define TUNNEL_RSP_BFSS_CODE 0x00002000 /* passing on a BFSS response code */






#define TUNNEL_CMD_INIT_LEN 44 /*incoming number of bytes*/

#define TUNNEL_MAX_BLOCKS_ENCRYPTED ((uint64_t) 0x100000000)



/*typedef struct TDCommandBuffer {
    uint32_t command; // command ordinal
    uint8_t authorized; //boolean, non-zero means an authorized/encrypted command
    size_t paramLen; // length of the params field, in bytes
    size_t extractHead; //position in the params field to extract the next byte(s)
    uint8_t* params; // parameters, interpretation is command-dependent.
    //uint8_t nonceOdd[TUNNEL_NONCE_LENGTH]; // these two aren't apparently used anywhere
    //uint8_t hmac[TUNNEL_HMAC_LENGTH];
} CommandBuffer;*/


void TUNNEL_BufferInit(TUNNEL_BUFFER_CTX*);
TUNNEL_BUFFER_CTX* TUNNEL_GetBufferPtr();

size_t TUNNEL_BufferExtend(TUNNEL_BUFFER_CTX*, uint8_t*, size_t);
size_t TUNNEL_BufferExtend8(TUNNEL_BUFFER_CTX*, uint8_t);
size_t TUNNEL_BufferExtend16(TUNNEL_BUFFER_CTX*, uint16_t);
size_t TUNNEL_BufferExtend32(TUNNEL_BUFFER_CTX*, uint32_t);
void TUNNEL_BufferExtendDigestOnly(TUNNEL_BUFFER_CTX* buff, uint8_t* in, size_t len);
void TUNNEL_BufferExtendDigestOnly32(TUNNEL_BUFFER_CTX* buff, uint32_t in);

uint8_t TUNNEL_BufferExtract8(TUNNEL_BUFFER_CTX* buff);
uint16_t TUNNEL_BufferExtract16(TUNNEL_BUFFER_CTX* buff);
uint32_t TUNNEL_BufferExtract32(TUNNEL_BUFFER_CTX* buff);
size_t TUNNEL_BufferExtractBuffer(TUNNEL_BUFFER_CTX* buff, uint8_t* out, size_t len);
int32_t TUNNEL_BufferExtractStructure(TUNNEL_BUFFER_CTX* buff, int32_t (*extractFunc)(void*, uint8_t*), void* structure);
size_t TUNNEL_BufferLengthField(TUNNEL_BUFFER_CTX*);
void TUNNEL_BufferDigest(TUNNEL_BUFFER_CTX*);

int32_t TUNNEL_BufferMakeAuthSection(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* buff);
int32_t TUNNEL_BufferHandleAuthSection(TUNNEL_BUFFER_CTX*, uint8_t* authSection, TransportTunnel*);
size_t TUNNEL_BufferCalcLength(TUNNEL_BUFFER_CTX*);
size_t TUNNEL_BufferGet(TUNNEL_BUFFER_CTX*, uint8_t*, size_t);
void TUNNEL_BufferFree(TUNNEL_BUFFER_CTX*);
void TUNNEL_BufferSend(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* buff, uint32_t resposneCode, uint32_t commandCode, uint8_t authorized);




/*void initCommandBuffer(CommandBuffer* buffer);
void freeCommandBuffer(CommandBuffer* buffer);*/


void TUNNEL_Init(TransportTunnel* tunnel, uint16_t (*tunnelSendFunc)(uint8_t* data, uint16_t len));
void TUNNEL_End(TransportTunnel* tunnel);
void TUNNEL_Dump(TransportTunnel* tunnel);
int32_t TUNNEL_SendShortClear(TransportTunnel* tunnel, uint32_t responseCode);
int32_t TUNNEL_SendShortEnc(TransportTunnel* tunnel, uint32_t commandCode, uint32_t responseCode);

int32_t TUNNEL_InitHandler(TransportTunnel* tunnel, uint8_t* nonceIn, uint8_t* otc, size_t otcLen);

int32_t TUNNEL_VerifyAuthCommand(TransportTunnel* tunnel, uint8_t* authSection, uint8_t* digest);
int32_t TUNNEL_MakeAuthSection(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* buff);

size_t TUNNEL_AES_CTR_CryptInPlace(TransportTunnel* tunnel, uint8_t* buffer, size_t len);
void TUNNEL_Test(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff);
int32_t TUNNEL_ORD_InitHandler(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff, volatile uint8_t* codeEntryCompleteFlag, size_t (*getOTCFunc)(unsigned char*, size_t), uint32_t timeout);
uint32_t TUNNEL_GetOTCSetup(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff);

uint32_t TUNNEL_TPM_GetPubKey(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff, I2C_HandleTypeDef* hi2c, uint8_t* keyAuthData/*, AuthSession* oiap*/);
uint32_t TUNNEL_TPM_Seal(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff, I2C_HandleTypeDef* hi2c, uint8_t* keyAuthData);
uint32_t TUNNEL_TPM_Unseal(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff, I2C_HandleTypeDef* hi2c, uint8_t* keyAuthData/*, AuthSession* keyAuthSession, AuthSession* dataAuthSession*/);
uint32_t TUNNEL_TPM_CreateWrapKey(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff, I2C_HandleTypeDef* hi2c, uint8_t* keyAuthData);
uint32_t TUNNEL_TPM_LoadKey(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff, I2C_HandleTypeDef* hi2c/*, AuthSession* as*/, uint8_t* keyAuthData);

uint32_t TUNNEL_TPM_NV_DefineSpace(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff, I2C_HandleTypeDef* hi2c, AuthData_tdAuthDataStore* authDataStore);
uint32_t TUNNEL_TPM_NV_Read(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff, I2C_HandleTypeDef* hi2c, AuthData_tdAuthDataStore* authDataStore/*, AuthSession* as*/);
uint32_t TUNNEL_TPM_NV_Write(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff, I2C_HandleTypeDef* hi2c, AuthData_tdAuthDataStore* authDataStore/*, AuthSession* as*/);

uint32_t TUNNEL_TPM_GetCapability(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff, I2C_HandleTypeDef* hi2c);
uint32_t TUNNEL_TPM_GetCapabilityOwner(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff, I2C_HandleTypeDef* hi2c, AuthData_tdAuthDataStore* authDataStore/*, AuthSession* as*/);

uint32_t TUNNEL_RTC_Set(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff, RTC_HandleTypeDef* hrtc);
uint32_t TUNNEL_RTC_Get(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff, RTC_HandleTypeDef* hrtc);

uint32_t TUNNEL_Setup_SRK(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff/*, TPM_KEY12* srk*/);
uint32_t TUNNEL_TPM_DAMSetup(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff, I2C_HandleTypeDef* hi2c);

uint32_t TUNNEL_GetVoltages(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff);

uint32_t TUNNEL_SetupInitial(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff);
uint32_t TUNNEL_SetupAttackMitigation(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff);
uint32_t TUNNEL_SetupTunnel(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff);
uint32_t TUNNEL_SetupPINChange(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff);
uint32_t TUNNEL_SetupRelock(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff);
uint32_t TUNNEL_OwnerGetConfiguration(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff);

#endif /* TUNNEL_H_ */
