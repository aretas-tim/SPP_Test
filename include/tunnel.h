/*
 * tunnel.h
 *
 *  Created on: Apr 5, 2016
 *      Author: me
 */

#ifndef TUNNEL_H_
#define TUNNEL_H_

#include "utilities.h"
//#include "tpm_nonce.h" /*Utilities_getRandomBuff*/
#include "md.h"
#include "sha256.h"
#include "sha512.h"
#include "aes.h"
#include "pkcs5.h" /* pbkdf2*/
//#include "tpm_authsession.h" /* Utilities_tpmAesCtrCrypt*/
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


void TUNNEL_BufferInit(TunnelStructures_TunnelBufferCtx*);
TunnelStructures_TunnelBufferCtx* TUNNEL_GetBufferPtr();

size_t TUNNEL_BufferExtend(TunnelStructures_TunnelBufferCtx*, uint8_t*, size_t);
size_t TUNNEL_BufferExtend8(TunnelStructures_TunnelBufferCtx*, uint8_t);
size_t TUNNEL_BufferExtend16(TunnelStructures_TunnelBufferCtx*, uint16_t);
size_t TUNNEL_BufferExtend32(TunnelStructures_TunnelBufferCtx*, uint32_t);
void TUNNEL_BufferExtendDigestOnly(TunnelStructures_TunnelBufferCtx* buff, uint8_t* in, size_t len);
void TUNNEL_BufferExtendDigestOnly32(TunnelStructures_TunnelBufferCtx* buff, uint32_t in);

uint8_t TUNNEL_BufferExtract8(TunnelStructures_TunnelBufferCtx* buff);
uint16_t TUNNEL_BufferExtract16(TunnelStructures_TunnelBufferCtx* buff);
uint32_t TUNNEL_BufferExtract32(TunnelStructures_TunnelBufferCtx* buff);
size_t TUNNEL_BufferExtractBuffer(TunnelStructures_TunnelBufferCtx* buff, uint8_t* out, size_t len);
int32_t TUNNEL_BufferExtractStructure(TunnelStructures_TunnelBufferCtx* buff, int32_t (*extractFunc)(void*, uint8_t*), void* structure);
size_t TUNNEL_BufferLengthField(TunnelStructures_TunnelBufferCtx*);
void TUNNEL_BufferDigest(TunnelStructures_TunnelBufferCtx*);

int32_t TUNNEL_BufferMakeAuthSection(TunnelStructures_TransportTunnel* tunnel, TunnelStructures_TunnelBufferCtx* buff);
int32_t TUNNEL_BufferHandleAuthSection(TunnelStructures_TunnelBufferCtx*, uint8_t* authSection, TunnelStructures_TransportTunnel*);
size_t TUNNEL_BufferCalcLength(TunnelStructures_TunnelBufferCtx*);
size_t TUNNEL_BufferGet(TunnelStructures_TunnelBufferCtx*, uint8_t*, size_t);
void TUNNEL_BufferFree(TunnelStructures_TunnelBufferCtx*);
void TUNNEL_BufferSend(TunnelStructures_TransportTunnel* tunnel, TunnelStructures_TunnelBufferCtx* buff, uint32_t resposneCode, uint32_t commandCode, uint8_t authorized);




/*void initCommandBuffer(CommandBuffer* buffer);
void freeCommandBuffer(CommandBuffer* buffer);*/


void TUNNEL_Init(TunnelStructures_TransportTunnel* tunnel, uint16_t (*tunnelSendFunc)(uint8_t* data, uint16_t len));
void TUNNEL_End(TunnelStructures_TransportTunnel* tunnel);
void TUNNEL_Dump(TunnelStructures_TransportTunnel* tunnel);
int32_t TUNNEL_SendShortClear(TunnelStructures_TransportTunnel* tunnel, uint32_t responseCode);
int32_t TUNNEL_SendShortEnc(TunnelStructures_TransportTunnel* tunnel, uint32_t commandCode, uint32_t responseCode);

int32_t TUNNEL_InitHandler(TunnelStructures_TransportTunnel* tunnel, uint8_t* nonceIn, uint8_t* otc, size_t otcLen);

int32_t TUNNEL_VerifyAuthCommand(TunnelStructures_TransportTunnel* tunnel, uint8_t* authSection, uint8_t* digest);
int32_t TUNNEL_MakeAuthSection(TunnelStructures_TransportTunnel* tunnel, TunnelStructures_TunnelBufferCtx* buff);

size_t TUNNEL_AES_CTR_CryptInPlace(TunnelStructures_TransportTunnel* tunnel, uint8_t* buffer, size_t len);
void TUNNEL_Test(TunnelStructures_TransportTunnel* tunnel, TunnelStructures_TunnelBufferCtx* cbuff);
int32_t TUNNEL_ORD_InitHandler(TunnelStructures_TransportTunnel* tunnel, TunnelStructures_TunnelBufferCtx* cbuff, volatile uint8_t* codeEntryCompleteFlag, size_t (*getOTCFunc)(unsigned char*, size_t), uint32_t timeout);
uint32_t TUNNEL_GetOTCSetup(TunnelStructures_TransportTunnel* tunnel, TunnelStructures_TunnelBufferCtx* cbuff);

uint32_t TUNNEL_TPM_GetPubKey(TunnelStructures_TransportTunnel* tunnel, TunnelStructures_TunnelBufferCtx* cbuff, I2C_HandleTypeDef* hi2c, uint8_t* keyAuthData/*, AuthSession* oiap*/);
uint32_t TUNNEL_TPM_Seal(TunnelStructures_TransportTunnel* tunnel, TunnelStructures_TunnelBufferCtx* cbuff, I2C_HandleTypeDef* hi2c, uint8_t* keyAuthData);
uint32_t TUNNEL_TPM_Unseal(TunnelStructures_TransportTunnel* tunnel, TunnelStructures_TunnelBufferCtx* cbuff, I2C_HandleTypeDef* hi2c, uint8_t* keyAuthData/*, AuthSession* keyAuthSession, AuthSession* dataAuthSession*/);
uint32_t TUNNEL_TPM_CreateWrapKey(TunnelStructures_TransportTunnel* tunnel, TunnelStructures_TunnelBufferCtx* cbuff, I2C_HandleTypeDef* hi2c, uint8_t* keyAuthData);
uint32_t TUNNEL_TPM_LoadKey(TunnelStructures_TransportTunnel* tunnel, TunnelStructures_TunnelBufferCtx* cbuff, I2C_HandleTypeDef* hi2c/*, AuthSession* as*/, uint8_t* keyAuthData);

uint32_t TUNNEL_TPM_NV_DefineSpace(TunnelStructures_TransportTunnel* tunnel, TunnelStructures_TunnelBufferCtx* cbuff, I2C_HandleTypeDef* hi2c, AuthData_AuthDataStore* authDataStore);
uint32_t TUNNEL_TPM_NV_Read(TunnelStructures_TransportTunnel* tunnel, TunnelStructures_TunnelBufferCtx* cbuff, I2C_HandleTypeDef* hi2c, AuthData_AuthDataStore* authDataStore/*, AuthSession* as*/);
uint32_t TUNNEL_TPM_NV_Write(TunnelStructures_TransportTunnel* tunnel, TunnelStructures_TunnelBufferCtx* cbuff, I2C_HandleTypeDef* hi2c, AuthData_AuthDataStore* authDataStore/*, AuthSession* as*/);

uint32_t TUNNEL_TPM_GetCapability(TunnelStructures_TransportTunnel* tunnel, TunnelStructures_TunnelBufferCtx* cbuff, I2C_HandleTypeDef* hi2c);
uint32_t TUNNEL_TPM_GetCapabilityOwner(TunnelStructures_TransportTunnel* tunnel, TunnelStructures_TunnelBufferCtx* cbuff, I2C_HandleTypeDef* hi2c, AuthData_AuthDataStore* authDataStore/*, AuthSession* as*/);

uint32_t TUNNEL_RTC_Set(TunnelStructures_TransportTunnel* tunnel, TunnelStructures_TunnelBufferCtx* cbuff, RTC_HandleTypeDef* hrtc);
uint32_t TUNNEL_RTC_Get(TunnelStructures_TransportTunnel* tunnel, TunnelStructures_TunnelBufferCtx* cbuff, RTC_HandleTypeDef* hrtc);

uint32_t TUNNEL_Setup_SRK(TunnelStructures_TransportTunnel* tunnel, TunnelStructures_TunnelBufferCtx* cbuff/*, TPM_KEY12* srk*/);
uint32_t TUNNEL_TPM_DAMSetup(TunnelStructures_TransportTunnel* tunnel, TunnelStructures_TunnelBufferCtx* cbuff, I2C_HandleTypeDef* hi2c);

uint32_t TUNNEL_GetVoltages(TunnelStructures_TransportTunnel* tunnel, TunnelStructures_TunnelBufferCtx* cbuff);

uint32_t TUNNEL_SetupInitial(TunnelStructures_TransportTunnel* tunnel, TunnelStructures_TunnelBufferCtx* cbuff);
uint32_t TUNNEL_SetupAttackMitigation(TunnelStructures_TransportTunnel* tunnel, TunnelStructures_TunnelBufferCtx* cbuff);
uint32_t TUNNEL_SetupTunnel(TunnelStructures_TransportTunnel* tunnel, TunnelStructures_TunnelBufferCtx* cbuff);
uint32_t TUNNEL_SetupPINChange(TunnelStructures_TransportTunnel* tunnel, TunnelStructures_TunnelBufferCtx* cbuff);
uint32_t TUNNEL_SetupRelock(TunnelStructures_TransportTunnel* tunnel, TunnelStructures_TunnelBufferCtx* cbuff);
uint32_t TUNNEL_OwnerGetConfiguration(TunnelStructures_TransportTunnel* tunnel, TunnelStructures_TunnelBufferCtx* cbuff);

#endif /* TUNNEL_H_ */
