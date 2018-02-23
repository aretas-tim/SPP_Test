/*
 * tunnel.c
 *
 *  Created on: Apr 5, 2016
 *      Author: me
 */

#include "tunnel.h"
#include "status.h"
#include "tunnel_lengths.h"
#include "sram2.h"
#include "md.h"
#include <stdbool.h>
#include "tpm_utils.h"
#include "backup_regs.h"

extern LockStatus statusLock;
extern WorkStatus statusWork;
extern ADC_HandleTypeDef hadc;
TUNNEL_BUFFER_CTX tunnelBuffer; /* there can be only one!*/

//@TODO remove these when we gut the tunnel handler
#define SRAM2_TUNNEL_COMMAND_BUFF_LEN 2048
uint8_t SRAM2_TUNNEL_FIFO[SRAM2_TUNNEL_COMMAND_BUFF_LEN];
uint8_t SRAM2_TUNNEL_COMMAND_BUFF[SRAM2_TUNNEL_COMMAND_BUFF_LEN];


TUNNEL_BUFFER_CTX* TUNNEL_GetBufferPtr() {
    return &tunnelBuffer;
}


void TUNNEL_Init(TransportTunnel* tunnel, uint16_t (*tunnelSendFunc)(uint8_t* data, uint16_t len)) {
    memset(tunnel->nonceEven, 0, TUNNEL_NONCE_LENGTH);
    memset(tunnel->nonceOdd, 0, TUNNEL_NONCE_LENGTH);
    memset(tunnel->sessionCounter, 0, TUNNEL_COUNTER_LENGTH);
    memset(tunnel->sessionKey, 0, TUNNEL_SESSION_KEY_LENGTH);
    tunnel->sessionAlive = 0;
    tunnel->blocksEncrypted = 0;
    mbedtls_aes_init(&(tunnel->aesctx));
    tunnel->sendFunc = tunnelSendFunc; //send stuff out here. function pointer to decouple it from the UART or USB
}

void TUNNEL_End(TransportTunnel* tunnel) {
    mbedtls_aes_free(&(tunnel->aesctx));
    TUNNEL_Init(tunnel, tunnel->sendFunc); /*change this if the init routine changes*/
    //don't overwrite the function pointer though!
    LED_SetTunnelEstablished(false);
}

void TUNNEL_Dump(TransportTunnel* tunnel) {
#ifdef DEBUG
    uart_debug_sendline("Transport Tunnel Dump:\n");
    uart_debug_sendline("Session Key:\n");
    uart_debug_hexdump(tunnel->sessionKey, TUNNEL_SESSION_KEY_LENGTH);
    uart_debug_sendline("Session Counter:\n");
    uart_debug_hexdump(tunnel->sessionCounter, TUNNEL_COUNTER_LENGTH);
    uart_debug_sendline("Even Nonce:\n");
    uart_debug_hexdump(tunnel->nonceEven, TUNNEL_NONCE_LENGTH);
    uart_debug_sendline("Odd Nonce:\n");
    uart_debug_hexdump(tunnel->nonceOdd, TUNNEL_NONCE_LENGTH);
    uart_debug_addToBuffer("Alive: ", 7);
    uart_debug_printBool(tunnel->sessionAlive);
    uart_debug_newline();
    uart_debug_addToBuffer("Blocks Encrypted: ", 18);
    uart_debug_printuint64(tunnel->blocksEncrypted);
    uart_debug_newline();
#endif /* DEBUG */
}

/* basic loopback test. will take the decrypted parameters in cbuff and return them */
void TUNNEL_Test(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff) {
#ifdef DEBUG
    if(!cbuff->authOkay) {
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_AUTHFAIL);
        return;
    } else if (0 == cbuff->paramHead) {
        //nothing to do, so return an error, otherwise this could be used to roll the nonces
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_BAD_LENGTH);
        return;
    }
    uint8_t* temp = malloc(sizeof(uint8_t) * cbuff->paramHead);
    uint32_t tempLen = cbuff->paramHead;
    if(NULL == temp) {
        TUNNEL_SendShortEnc(tunnel, TUNNEL_ORD_TEST, TUNNEL_RSP_OUT_OF_MEMORY); //welp
        return;
    }
    memcpy(temp, cbuff->params, cbuff->paramHead); //gotta use a temp buffer now
    TUNNEL_BufferInit(&tunnelBuffer);
    TUNNEL_BufferExtend(&tunnelBuffer, temp, tempLen);
    free(temp);
    TUNNEL_BufferSend(tunnel, &tunnelBuffer, TUNNEL_RSP_SUCCESS, TUNNEL_ORD_TEST, true);
    TUNNEL_BufferFree(&tunnelBuffer);
#else
    TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_ORD_UNKNOWN); //if this somehow gets run in a non-debug build, just fail like it doesn't exist
#endif

}

/* okay this is a bit funky due to function pointers and volatile stuff, but bear with me here
 * basically forced this way so no other commands can be interpreted while this is in progress (slow human in the loop for this one)
 *
 * cbuff should contain the incoming command and the session odd nonce from the host
 * codeEntryCompleteFlag should go TRUE when the getOTCFunc has the code ready to go
 * this function will spin while codeEntryCompleteFlag is FALSE
 * timeout sets a maximum time out, in milliseconds (via HAL calls). if this is exceeded, it returns failure to the host
 */
int32_t TUNNEL_ORD_InitHandler(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff, volatile uint8_t* codeEntryCompleteFlag, size_t (*getOTCFunc)(unsigned char*, size_t), uint32_t timeout) {
    if(cbuff->hasAuth) {
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_BAD_TAG);
        return TUNNEL_RSP_BAD_TAG;
    } else if (cbuff->paramHead != TUNNEL_NONCE_LENGTH) {
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_BAD_LENGTH);
        return TUNNEL_RSP_BAD_LENGTH;
    } else if (tunnel->sessionAlive) {
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_SESSION_EXISTS);
        return TUNNEL_RSP_SESSION_EXISTS;
    }
    LED_SetTunnelEstablishing();
#ifndef DEBUG_BYPASS_SECURITY
    uint32_t timeoutAt = HAL_GetTick() + timeout;
    while(HAL_GetTick() < timeoutAt) {
        statusWork = WORK_AWAKE; //@TODO something to indicate waiting on user?
        if(*codeEntryCompleteFlag) {
            statusWork = WORK_WORKING;
            unsigned char otcBuffer[TUNNEL_MAXIMUM_OTC_LEN];
            uint32_t otcLen = getOTCFunc(otcBuffer, TUNNEL_MAXIMUM_OTC_LEN);
            *codeEntryCompleteFlag = false;
            if(TUNNEL_MINIMUM_OTC_LEN <= otcLen) {
                int32_t rc = TUNNEL_InitHandler(tunnel, cbuff->params, otcBuffer, otcLen);
                memset(otcBuffer, 0, TUNNEL_MAXIMUM_OTC_LEN);
                return rc;
            } else {
                statusWork = WORK_AWAKE; //too short, could not process
                ledSetErrorOn();
            }
        }
    }
#else /* DEBUG_BYPASS_SECURITY defined */
    int32_t rc = TUNNEL_InitHandler(tunnel, cbuff->params, NULL, 6); //@TODO put everything else back so there's security
#endif /* DEBUG_BYPASS_SECURITY */
    LED_SetTunnelEstablished(false); //turn off blinking light
    //return TUNNEL_RSP_TIMEOUT;
    TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_TIMEOUT);
    return TUNNEL_RSP_TIMEOUT;

}

uint32_t TUNNEL_DeriveSessionKey(uint8_t* derivedKey, uint8_t* derivedCounter, uint8_t* salt, uint32_t saltLen, uint8_t* otc, uint32_t otcLen) {
    uart_debug_sendline("OTC and Salt:\n");
    uart_debug_hexdump(otc, otcLen);
    uart_debug_hexdump(salt, saltLen);
    uart_debug_newline();
    size_t combinedDataLen = TRANSPORT_KEY_LEN + TUNNEL_COUNTER_LENGTH;
    uint8_t combinedData[combinedDataLen];
    size_t hashedOTCLen = 32;
    uint8_t hashedOTC[hashedOTCLen];
    mbedtls_md_context_t ctx;
    mbedtls_md_init(&ctx);
    mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), true);
    //TIM2->CNT = 0x0; //reinitialize timer2
    //TIM2->CR1 |= 0x0001; //start timer 2
    mbedtls_md(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), otc, otcLen, hashedOTC);
    mbedtls_pkcs5_pbkdf2_hmac(&ctx, hashedOTC, hashedOTCLen, salt, saltLen, *BACKUP_REGS_TUNNEL_ITERS_REG, combinedDataLen, combinedData);
    //TIM2->CR1 &= ~(0x0001); //stop timer 2

    uart_debug_hexdump(combinedData, combinedDataLen);
    uart_debug_newline();
    //uart_debug_addToBuffer("Derivation Cycles: ", 19);
    //uart_debug_printuint32(TIM2->CNT);
    //uart_debug_newline();
    mbedtls_md_free(&ctx);

    memcpy(derivedKey, combinedData, TRANSPORT_KEY_LEN);
    memcpy(derivedCounter, combinedData + TRANSPORT_KEY_LEN, TUNNEL_COUNTER_LENGTH);

    memset(combinedData, 0, combinedDataLen);
    return combinedDataLen;
}

/* TUNNEL_InitHandler
 * nonceIn must be the nonce incoming from the host and of length 32
 * transportAuthData_tdKeyStore is initialized and valid and contains the transport key and the salt used to derive it
 *
 * this function then computes a sessionKey, used for both the AES encryption and the HMAC authentication
 * it then returns to the host:
 *         a sessionNonceEven of 256 bits, from which the host can derive the sessionKey
 *         a commandNonceEven of 256 bits, which is uses as part of the incoming HMAC for the first authorized/authenticated command
 *         an initial counter value of 128 bits, used as part of the CTR mode AES encryption on the tunnel.
 *
 * if there is no transportKey or the transportKey is not valid it returns an error message.
 */
int32_t TUNNEL_InitHandler(TransportTunnel* tunnel, uint8_t* nonceIn, uint8_t* otc, size_t otcLen) {
    if (tunnel->sessionAlive) {
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_SESSION_EXISTS);
        LED_SetTunnelEstablished(true);
        return TUNNEL_RSP_SESSION_EXISTS;
    }
    //set up our end of the tunnel
    uint8_t salt[TRANSPORT_KEY_LEN];
    getRandomBuff(NULL, salt, TRANSPORT_KEY_LEN); //random salt
    uint8_t transportKey[TRANSPORT_KEY_LEN];
#ifdef DEBUG_BYPASS_SECURITY
    uint8_t testOTC[6] = {'1', '2', '3', '4', '5', '6'};
    otcLen = 6;
    TUNNEL_DeriveSessionKey(transportKey, tunnel->sessionCounter, salt, TRANSPORT_KEY_LEN, testOTC, otcLen);
#else
    TUNNEL_DeriveSessionKey(transportKey, tunnel->sessionCounter, salt, TRANSPORT_KEY_LEN, otc, otcLen);
#endif /*DEBUG_BYPASS_SECURITY */
    uint8_t localNonces[TUNNEL_NONCE_LENGTH * 2];
    getRandomBuff(NULL, localNonces, TUNNEL_NONCE_LENGTH); //get a random nonce for the session even nonce
    memcpy(localNonces + TUNNEL_NONCE_LENGTH, nonceIn, TUNNEL_NONCE_LENGTH); //concatenate the incoming session odd nonce to the end of ours
    //HMAC the combined nonces, with the shared secret as the key to derive our sessionKey
    mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), transportKey, TRANSPORT_KEY_LEN, localNonces, (TUNNEL_NONCE_LENGTH * 2), tunnel->sessionKey);
    //initialize the aes context with the session key
    mbedtls_aes_setkey_enc(&(tunnel->aesctx), tunnel->sessionKey, TRANSPORT_KEY_LEN * 8);

    getRandomBuff(NULL, tunnel->nonceEven, TUNNEL_NONCE_LENGTH); //even nonce that will form part of the HMAC for the next authorized command from the host

    tunnel->sessionAlive = 1; //we're good
    TUNNEL_BufferInit(&tunnelBuffer); //set up buffer
    TUNNEL_BufferExtend(&tunnelBuffer, salt, TRANSPORT_KEY_LEN); //salt
    TUNNEL_BufferExtend(&tunnelBuffer, localNonces, TUNNEL_NONCE_LENGTH); //even session nonce that's part of the HMAC session key derivation
    TUNNEL_BufferExtend(&tunnelBuffer, tunnel->nonceEven, TUNNEL_NONCE_LENGTH); //HMAC even nonce
    //TUNNEL_BufferCalcLength(&tunnelBuffer);
    TUNNEL_BufferSend(tunnel, &tunnelBuffer, TUNNEL_RSP_SUCCESS, TUNNEL_ORD_INIT, false);
#ifdef DEBUG
    TUNNEL_Dump(tunnel);
#endif /* DEBUG */
    LED_SetTunnelEstablished(true);
    /* there's no way to fail once we're in this function, a tunnel is always established
     * it may be different than what was established on the other end and fail at the first command
     * but from our perspective its always established.
     */
    return 0;
}


/*
 * sends a short (no data, unencrypted) response back to the host
 * basically a handy way to send error messages
 * if the responseCode is *not* TUNNEL_RSP_SUCCESS or TUNNEL_RSP_SESSION_EXISTS, it will end the tunnel
 */
int32_t TUNNEL_SendShortClear(TransportTunnel* tunnel, uint32_t responseCode) {
    TUNNEL_BufferInit(&tunnelBuffer);
    //TUNNEL_BufferExtend16(&tunnelBuffer, TUNNEL_TAG_RSP_CLEAR, 0);
    //TUNNEL_BufferExtend32(&tunnelBuffer, 12, 0); /* fixed length*/
    //TUNNEL_BufferExtend32(&tunnelBuffer, responseCode, 0);
    TUNNEL_BufferSend(tunnel, &tunnelBuffer, responseCode, 0, true); //command code doesn't matter for a cleartext response
    TUNNEL_BufferFree(&tunnelBuffer);
    if((TUNNEL_RSP_SUCCESS != responseCode && TUNNEL_RSP_SESSION_EXISTS != responseCode) && (TUNNEL_RSP_SESSION_EXISTS != responseCode)) {
        TUNNEL_End(tunnel);
        //@TODO lock the device?
    }
    return 0;
}

/*
 * sends a short (no data) response back to the host
 * basically a handy way to send TUNNEL_RSP_SUCCESS or pass on error codes that don't call for the tunnel to be ended
 */
int32_t TUNNEL_SendShortEnc(TransportTunnel* tunnel, uint32_t commandCode, uint32_t responseCode) {
    TUNNEL_BufferInit(&tunnelBuffer);
    /*TUNNEL_BufferExtend16(&tunnelBuffer, TUNNEL_TAG_RSP_ENC, 0);
    TUNNEL_BufferExtend32(&tunnelBuffer, 76, 0); // fixed length
    TUNNEL_BufferExtend32(&tunnelBuffer, responseCode, TRUE);
    TUNNEL_BufferExtendDigestOnly32(&tunnelBuffer, commandCode); //this is part of the digest, but NOT the parameters
    TUNNEL_BufferMakeAuthSection(tunnel, &tunnelBuffer);*/
    TUNNEL_BufferSend(tunnel, &tunnelBuffer, responseCode, commandCode, true);
    TUNNEL_BufferFree(&tunnelBuffer);
    return 0;
}

void TUNNEL_BufferSend(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* buff, uint32_t responseCode, uint32_t commandCode, uint8_t authorized) {
    uint32_t SIZE_OF_UINT32 = 4; //bytes. @TODO clean this up
    uint8_t temp[SIZE_OF_UINT32];
    uint8_t* fifo = (uint8_t*) SRAM2_TUNNEL_FIFO;
    fifo[0] = 0xAA;
    fifo[1] = 0xAA;
    size_t fifoHead = 2;
    size_t length = TUNNEL_HEADER_LEN + SIZE_OF_UINT32 + buff->paramHead;

    if(authorized) {
        fifoHead += packToBuffer16(fifo, fifoHead, TUNNEL_TAG_RSP_ENC);
        length += TUNNEL_LEN_RSP_ENC;
    } else {
        fifoHead += packToBuffer16(fifo, fifoHead, TUNNEL_TAG_RSP_CLEAR);
    }
    fifoHead += packToBuffer32(fifo, fifoHead, length);

    //this is kinda ugly, but at least its only happening once
    packToBuffer32(temp, 0, responseCode);
    memcpy(fifo + fifoHead, temp, SIZE_OF_UINT32);
    fifoHead += SIZE_OF_UINT32;

    memcpy(fifo + fifoHead, buff->params, buff->paramHead);
    fifoHead += buff->paramHead;

    if(authorized) {
        // 2016-05-17 removed local stuff, was causing problems and i'm not immediately sure how to fix it. may be able to re-implement in the future
        mbedtls_sha256_context digestCtx;
        mbedtls_sha256_init(&digestCtx);
        mbedtls_sha256_starts(&digestCtx, false);
        mbedtls_sha256_update(&digestCtx, temp, SIZE_OF_UINT32); //response Code, already in temp
        uart_debug_sendline("Tunnel Digest Parameters:\n");
        uart_debug_hexdump(temp, SIZE_OF_UINT32);
        //.. twice
        packToBuffer32(temp, 0, commandCode);
        mbedtls_sha256_update(&digestCtx, temp, SIZE_OF_UINT32); //command Code (included in the digest but not part of the return parameters)
        uart_debug_hexdump(temp, SIZE_OF_UINT32);

        mbedtls_sha256_update(&digestCtx, buff->params, buff->paramHead); //parameters
        uart_debug_hexdump(buff->params, buff->paramHead);

        uint8_t digest[TUNNEL_HASH_LENGTH]; /* temp buffer for the digest */
        mbedtls_sha256_finish(&digestCtx, digest);
        mbedtls_sha256_free(&digestCtx);


        mbedtls_md_context_t hmacCtx;
        mbedtls_md_init(&hmacCtx);
        mbedtls_md_setup(&hmacCtx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), true);
        mbedtls_md_hmac_starts(&hmacCtx, tunnel->sessionKey, TUNNEL_SESSION_KEY_LENGTH); //init HMAC with the tunnel session key

        mbedtls_md_hmac_update(&hmacCtx, digest, TUNNEL_HASH_LENGTH);

        getRandomBuff(NULL, tunnel->nonceEven, TUNNEL_NONCE_LENGTH); // get a new nonce
        mbedtls_md_hmac_update(&hmacCtx, tunnel->nonceEven, TUNNEL_NONCE_LENGTH); // add it to the HMAC
        memcpy(fifo + fifoHead, tunnel->nonceEven, TUNNEL_NONCE_LENGTH); // also add it to the authsection to send back to the host
        fifoHead += TUNNEL_NONCE_LENGTH; //update our head location

        mbedtls_md_hmac_update(&hmacCtx, tunnel->nonceOdd, TUNNEL_NONCE_LENGTH); /* add the nonce from the host*/

        mbedtls_md_hmac_finish(&hmacCtx, fifo + fifoHead);
        mbedtls_md_free(&hmacCtx);
        fifoHead += TUNNEL_HMAC_LENGTH; // update this

        buff->hasAuth = 1;
        //uart_debug_addToBuffer("HMAC RC: ", 9);
        //uart_debug_printuint32(rc);
        //uart_debug_newline();
        uart_debug_sendline("Digest:\n");
        uart_debug_hexdump(digest, TUNNEL_HASH_LENGTH);
        uart_debug_sendline("Even Nonce:\n");
        uart_debug_hexdump(tunnel->nonceEven, TUNNEL_NONCE_LENGTH);
        uart_debug_sendline("Odd Nonce:\n");
        uart_debug_hexdump(tunnel->nonceOdd, TUNNEL_NONCE_LENGTH);
        uart_debug_sendline("HMAC:\n");
        uart_debug_hexdump(fifo + (fifoHead - TUNNEL_HMAC_LENGTH), TUNNEL_HMAC_LENGTH);


        //TUNNEL_BufferExtend(&tunnelBuffer, buff->authSection, TUNNEL_LEN_RSP_ENC, FALSE); /* copy this to our parameter buffer so its all in one place*/
        //TUNNEL_BufferCalcLength(buff);
        //uart_debug_sendline("Clear Response Dump:\n");
        //uart_debug_hexdump(buff->params, buff->paramHead);
        //TUNNEL_BufferMakeAuthSection(tunnel, buff);


        TUNNEL_AES_CTR_CryptInPlace(tunnel, fifo + TUNNEL_CMD_ENC_HEADER_LEN, fifoHead - TUNNEL_CMD_ENC_HEADER_LEN); //encrypt things
    } //else {
        //TUNNEL_BufferCalcLength(buff);
        //uart_debug_sendline("Clear Response Dump:\n");
        //uart_debug_hexdump(buff->params, buff->paramHead);
    //}
    //TUNNEL_BufferCalcLength(buff);
    //uart_comm_addToBuffer(fifo, fifoHead);
    if(tunnel->sendFunc == NULL) {
        uart_debug_sendline("Null pointer for tunnel send!\n");
    } else {
        uint16_t rsp = tunnel->sendFunc(fifo, fifoHead);
        uart_debug_addToBuffer("USB Transmitted Bytes: ", 23);
        uart_debug_printuint8(rsp);
        uart_debug_newline();
    }
    if(buff->hasAuth) {
        //uart_debug_sendline("Encrypted Response Dump:\n");
        //uart_debug_hexdump(buff->params, buff->paramHead);
    }
    TUNNEL_BufferFree(buff);
    return;
}

void TUNNEL_BufferInit(TUNNEL_BUFFER_CTX* buff) {
    buff->paramHead = 0;
    buff->params = (uint8_t*) SRAM2_TUNNEL_COMMAND_BUFF;

    buff->digested = 0;
    buff->hasAuth = 0;
    memset(buff->authSection, 0, TUNNEL_LEN_RSP_ENC);
    memset(buff->params, 0, SRAM2_TUNNEL_COMMAND_BUFF_LEN);
    memset(buff->digest, 0, TUNNEL_NONCE_LENGTH);
    mbedtls_sha256_init(&(buff->digestctx));
    mbedtls_sha256_starts(&(buff->digestctx), 0);
    /* this is where it gets kinda sketchy and. basically, to avoid dynamic allocation, we're going to statically allocate these
     * as part of the buffer context itself, then give the hmac context a pointer to them so it can operate
     */
    mbedtls_sha256_init(&(buff->hmac_digest_ctx));
    memset(buff->hmac_hmac_ctx, 0, 64 * 2);
    buff->hmacctx.hmac_ctx = buff->hmac_hmac_ctx;
    buff->hmacctx.md_ctx = &(buff->hmac_digest_ctx);
    buff->hmacctx.md_info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
}

void TUNNEL_BufferFree(TUNNEL_BUFFER_CTX* buff) {
    buff->digested = 0;
    buff->hasAuth = 0;
    buff->paramHead = 0;
    mbedtls_sha256_free(&(buff->digestctx));
    memset(buff->authSection, 0, TUNNEL_LEN_RSP_ENC);
    memset(buff->params, 0, SRAM2_TUNNEL_COMMAND_BUFF_LEN);
    memset(buff->digest, 0, TUNNEL_NONCE_LENGTH);
}
/* 2016-05-16 digestable removed. parameters are ALWAYS digestable and this is now handled by TUNNEL_BufferSend() directly */
size_t TUNNEL_BufferExtend(TUNNEL_BUFFER_CTX* buff, uint8_t* in, size_t len) {
    if(SRAM2_TUNNEL_COMMAND_BUFF_LEN < (buff->paramHead + len)) {
        len = SRAM2_TUNNEL_COMMAND_BUFF_LEN - buff->paramHead; //prevent buffer overflow
#ifdef DEBUG
        uart_debug_sendline("Tunnel Buffer has overflowed!\n");
#endif /* DEBUG */
    }
    memcpy(buff->params + buff->paramHead, in, len);
    buff->paramHead += len;
    return len;
}
void TUNNEL_BufferExtendDigestOnly(TUNNEL_BUFFER_CTX* buff, uint8_t* in, size_t len) {
    mbedtls_sha256_update(&(buff->digestctx), in, len);
    return;
}
size_t TUNNEL_BufferExtend8(TUNNEL_BUFFER_CTX* buff, uint8_t in) {
    size_t len = 1;
    return TUNNEL_BufferExtend(buff, &in, len);
}
size_t TUNNEL_BufferExtend16(TUNNEL_BUFFER_CTX* buff, uint16_t in) {
    size_t len = 2;
    uint8_t temp[len];
    packToBuffer16(temp, 0, in);
    return TUNNEL_BufferExtend(buff, temp, len);
}
size_t TUNNEL_BufferExtend32(TUNNEL_BUFFER_CTX* buff, uint32_t in) {
    size_t len = 4;
    uint8_t temp[len];
    packToBuffer32(temp, 0, in);
    return TUNNEL_BufferExtend(buff, temp, len);
}
/* does not check for overflow*/
size_t TUNNEL_BufferExtendStructure(TUNNEL_BUFFER_CTX* buff, size_t (*packFunction)(uint8_t*, void*), void* structure) {
    size_t len = packFunction(buff->params + buff->paramHead, structure);
    buff->paramHead += len;
    return len;
}
void TUNNEL_BufferExtendDigestOnly32(TUNNEL_BUFFER_CTX* buff, uint32_t in) {
    size_t len = 4;
    uint8_t temp[len];
    packToBuffer32(temp, 0, in);
    TUNNEL_BufferExtendDigestOnly(buff, temp, len);
    return;
}
//legacy code, no longer used, will just put 4 bytes of 0x00 in the parameter buffer
size_t TUNNEL_BufferLengthField(TUNNEL_BUFFER_CTX* buff) {
    return TUNNEL_BufferExtend32(buff, 0);
}

/* these three do not check for overrun */
/* gets a uint8_t from the paramater buffer, advances the internal extractHead */
uint8_t TUNNEL_BufferExtract8(TUNNEL_BUFFER_CTX* buff) {
    return buff->params[(buff->extractHead)++];
}

uint16_t TUNNEL_BufferExtract16(TUNNEL_BUFFER_CTX* buff) {
    uint16_t val = buff->params[(buff->extractHead)++] << 8;
    val += buff->params[(buff->extractHead)++];
    return val;
}

uint32_t TUNNEL_BufferExtract32(TUNNEL_BUFFER_CTX* buff) {
    uint32_t val = buff->params[(buff->extractHead)++] << 24;
    val += buff->params[(buff->extractHead)++] << 16;
    val += buff->params[(buff->extractHead)++] << 8;
    val += buff->params[(buff->extractHead)++];
    return val;
}

/* extracts a buffer of length len from the parameter buffer
 * will check for overrun and not exceed the length of its internal data
 * returns the actual extracted length */
size_t TUNNEL_BufferExtractBuffer(TUNNEL_BUFFER_CTX* buff, uint8_t* out, size_t len) {
    if(buff->paramHead < (len + buff->extractHead)) { //paramHead holds the length of the param buffer
        len = buff->paramHead - buff->extractHead;
    }
    memcpy(out, buff->params + buff->extractHead, len);
    buff->extractHead += len;
    return len;
}
/* let the fun begin!
 * takes a TPM_BUFFER_CTX, a pointer to an extraction function (which takes a pointer to the passed-in structure and a uint8_t*) and a pointer to a structure
 * it is up to the caller to ensure the extraction function matches the structure!
 * returns the structure extraction result (usually the length, or a negative error code)
 * @TODO: if we ever move to C++, this is a prime place to rework it so the structure can extract itself with a .extract(buff) function
 */
int32_t TUNNEL_BufferExtractStructure(TUNNEL_BUFFER_CTX* buff, int32_t (*extractFunc)(void*, uint8_t*), void* structure) {
    int32_t extractedLen = extractFunc(structure, buff->params + buff->extractHead);
    if(extractedLen >= 0) {
        buff->extractHead += extractedLen;
    }  /* sorry for the ugly formatting, only way to surround it in preprocessor directives to eliminate the else */
#ifdef DEBUG
    else {
        uart_debug_addToBuffer("Structure Extraction Error: ", 28);
        uart_debug_hexprint32(extractedLen);
        uart_debug_newline();
    }
#endif /* DEBUG */
    return extractedLen;
}
 /* legacy code */
void TUNNEL_BufferDigest(TUNNEL_BUFFER_CTX* buff) {
    mbedtls_sha256_finish(&(buff->digestctx), buff->digest);
    buff->digested = 1;
}

/* legacy code */
int32_t TUNNEL_BufferMakeAuthSection(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* buff) {
    if(!(buff->digested)) {
        TUNNEL_BufferDigest(buff);
    }

    return TUNNEL_MakeAuthSection(tunnel, buff);
}

/*
 * makes the authorization section
 * tunnel must be valid and initialized
 * buffer must have been digested
 * fills in the nonceEven field in tunnel
 * and the authSection
 */
int32_t TUNNEL_MakeAuthSection(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* buff) {
    uint8_t hmacBuff[TUNNEL_HASH_LENGTH + (TUNNEL_NONCE_LENGTH * 2)]; /* temp buffer for the hmac*/
    memcpy(hmacBuff, buff->digest, TUNNEL_HASH_LENGTH); /* start with the (already computed) digest*/
    getRandomBuff(NULL, tunnel->nonceEven, TUNNEL_NONCE_LENGTH); /* get a new nonce*/
    memcpy(hmacBuff + TUNNEL_HASH_LENGTH, tunnel->nonceEven, TUNNEL_NONCE_LENGTH); /* add that to the temp buffer*/
    memcpy(buff->authSection, tunnel->nonceEven, TUNNEL_NONCE_LENGTH); /*also add it to the authsection to send back to the host*/
    memcpy(hmacBuff + TUNNEL_HASH_LENGTH + TUNNEL_NONCE_LENGTH, tunnel->nonceOdd, TUNNEL_NONCE_LENGTH); /* add the nonce from the host*/
    mbedtls_md_hmac_starts(&(buff->hmacctx), tunnel->sessionKey, TUNNEL_SESSION_KEY_LENGTH);
    mbedtls_md_hmac_update(&(buff->hmacctx), hmacBuff, TUNNEL_HASH_LENGTH + (TUNNEL_NONCE_LENGTH * 2));
    mbedtls_md_hmac_finish(&(buff->hmacctx), buff->authSection + TUNNEL_NONCE_LENGTH);

    //int rc = mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), tunnel->sessionKey, TUNNEL_SESSION_KEY_LENGTH, hmacBuff, (TUNNEL_HASH_LENGTH + TUNNEL_NONCE_LENGTH * 2), buff->authSection + TUNNEL_NONCE_LENGTH);
    buff->hasAuth = 1;
    //uart_debug_addToBuffer("HMAC RC: ", 9);
    //uart_debug_printuint32(rc);
    //uart_debug_newline();
    /*uart_debug_sendline("Digest:\n");
    uart_debug_hexdump(hmacBuff, TUNNEL_HASH_LENGTH);
    uart_debug_sendline("Even Nonce:\n");
    uart_debug_hexdump(hmacBuff + TUNNEL_HASH_LENGTH, TUNNEL_NONCE_LENGTH);
    uart_debug_sendline("Odd Nonce:\n");
    uart_debug_hexdump(hmacBuff + TUNNEL_HASH_LENGTH + TUNNEL_NONCE_LENGTH, TUNNEL_NONCE_LENGTH);
    uart_debug_sendline("HMAC:\n");
    uart_debug_hexdump(buff->authSection + TUNNEL_NONCE_LENGTH, TUNNEL_HMAC_LENGTH);*/
    return 0;
}

int32_t TUNNEL_BufferHandleAuthSection(TUNNEL_BUFFER_CTX* buff, uint8_t* authSection, TransportTunnel* tunnel) {
    if(!(buff->digested)) {
        TUNNEL_BufferDigest(buff);
    }
    return TUNNEL_VerifyAuthCommand(tunnel, authSection, buff->digest);
}

size_t TUNNEL_BufferGet(TUNNEL_BUFFER_CTX* buff, uint8_t* out, size_t lenMax) {
    size_t head = 0;
    size_t len = buff->paramHead;
    if(buff->hasAuth) {
        len += TUNNEL_LEN_RSP_ENC;
    }
    if(len > lenMax) {
        return 0;
    }
    packToBuffer32(buff->params, TUNNEL_POS_LEN, len);
    memcpy(out, buff->params, buff->paramHead);
    head += buff->paramHead;
    if(buff->hasAuth) {
        memcpy(out + head, buff->authSection, TUNNEL_LEN_RSP_ENC);
        head += TUNNEL_LEN_RSP_ENC;
    }
    return head;
}

size_t TUNNEL_BufferCalcLength(TUNNEL_BUFFER_CTX* buff) {
    packToBuffer32(buff->params, TUNNEL_POS_LEN, buff->paramHead);
    return buff->paramHead;
}


/*void initCommandBuffer(CommandBuffer* buffer) {
    buffer->authorized = 0;
    buffer->command = TUNNEL_ORD_NONE;
    buffer->paramLen = 0;
    buffer->extractHead = 0;
    buffer->params = NULL;
}
void freeCommandBuffer(CommandBuffer* buffer) {
    buffer->authorized = 0;
    buffer->command = TUNNEL_ORD_NONE;
    zeroize(buffer->params, buffer->paramLen);
    buffer->paramLen = 0;
    buffer->extractHead = 0;
    //free(buffer->params); //how this didn't cause issues before i'm not sure

}*/

size_t TUNNEL_AES_CTR_CryptInPlace(TransportTunnel* tunnel, uint8_t* buffer, size_t len) {
    uint32_t AES_BLOCK_SIZE = 16; //@TODO de-magic this again
    size_t pos = 0;
    uint8_t temp[AES_BLOCK_SIZE];
    while(pos < len) {
        size_t blockLen = AES_BLOCK_SIZE;
        if(blockLen > (len - pos)) {
            blockLen = len - pos;
        }
        TPM_AES_CTR_Crypt(&(tunnel->aesctx), blockLen, tunnel->sessionCounter, buffer + pos, temp);
        memcpy(buffer + pos, temp, blockLen);
        pos += blockLen;
        tunnel->blocksEncrypted++;
    }
    return pos;
}

/* commands (besides test) start here! */

#ifdef TPM_TUNNEL_COMMANDS
/* define space makes its own authsession, as it ends immediately once the space is defined (due to authdata insertion)*/
uint32_t TUNNEL_TPM_NV_DefineSpace(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff, I2C_HandleTypeDef* hi2c, AuthData_tdAuthDataStore* authDataStore) {
    if(!cbuff->hasAuth) {
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_BAD_TAG);
        return TUNNEL_RSP_BAD_TAG;
        /* the only way to get here should be through sending this as a cleartext command, so in that case its bad tag*/
    } else if (!cbuff->authOkay) {
        //this should already be caught before being called, but we'll double-check it
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_AUTHFAIL);
        return TUNNEL_RSP_AUTHFAIL;
    }
    /*
    TPM_NV_DATA_PUBLIC nvData;
    initTPM_NV_DATA_PUBLIC(&nvData);
    // extract the incoming data from the buffer
    // unfortunately, i've "simplified" the information, so its no longer just a simple "NV_DATA_PUBLIC" struct
    nvData.nvIndex = TUNNEL_BufferExtract32(cbuff);
    nvData.dataSize = TUNNEL_BufferExtract32(cbuff);
    nvData.permission.attributes = TUNNEL_BufferExtract32(cbuff);
    uint16_t pcrSize = TUNNEL_BufferExtract16(cbuff);
    if(0 == pcrSize) {
        //nothing to do in here
        //the TPM_PCR_INFO_SHORT struct is initialized to zero, which means PCRs are not checked by the TPM.
    } else if (TPM_PCR_COUNT_BYTES == pcrSize) {
        nvData.pcrInfoRead.pcrSelection.sizeOfSelect = TPM_PCR_COUNT_BYTES;
        TUNNEL_BufferExtractBuffer(cbuff, nvData.pcrInfoRead.pcrSelection.pcrSelect, TPM_PCR_COUNT_BYTES);
        TUNNEL_BufferExtractBuffer(cbuff, nvData.pcrInfoRead.digestAtRelease.digest, digestSize);

        nvData.pcrInfoWrite.pcrSelection.sizeOfSelect = TPM_PCR_COUNT_BYTES;
        TUNNEL_BufferExtractBuffer(cbuff, nvData.pcrInfoWrite.pcrSelection.pcrSelect, TPM_PCR_COUNT_BYTES);
        TUNNEL_BufferExtractBuffer(cbuff, nvData.pcrInfoWrite.digestAtRelease.digest, digestSize);
    } else {
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_PCR_INFO_INVALID);
        return TUNNEL_RSP_PCR_INFO_INVALID;
        //cannot interpret PCR info
    }
    if((!(nvData.permission.attributes & (TPM_NV_PER_OWNERREAD | TPM_NV_PER_OWNERWRITE))) || \
        (nvData.permission.attributes & (TPM_NV_PER_AUTHREAD | TPM_NV_PER_AUTHWRITE))) {
        //this is a *little* convoluted, but it basically boils down to say Owner Read/Write and NOT auth Read/Write
        //other flags are fine
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_BAD_ATTRIBUTE);
        return TUNNEL_RSP_BAD_ATTRIBUTE;
    }

    uint8_t spaceAuthData[digestSize];
    memset(spaceAuthData, 0, digestSize); //TPM ignores this as its owner-authorized space, always
    //uart_debug_sendline("Owner AuthData:\n");
    //uart_debug_hexdump(authDataStore->ownerAuthData, digestSize);
    AuthSession localOSAPSession;
    TPM_AuthSessionInit(&localOSAPSession);
    int32_t tpm_rc = TPM_GetOSAPSession_Owner(hi2c, &localOSAPSession, TPM_ET_AES128_CTR, authDataStore->ownerAuthData);
    if(TPM_SUCCESS != tpm_rc) {
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_TPM_CODE | tpm_rc);
        return TUNNEL_RSP_TPM_CODE + tpm_rc;
    }
    TPM_SetCommandReadyAndWait(hi2c, 1000);
    tpm_rc = TPM_NV_DefineSpace(hi2c, &localOSAPSession, &nvData, spaceAuthData);
    if(TPM_SUCCESS != tpm_rc) {
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_TPM_CODE | tpm_rc);
        return TUNNEL_RSP_TPM_CODE + tpm_rc;
    }*/
    TUNNEL_SendShortEnc(tunnel, cbuff->command, TUNNEL_RSP_SUCCESS);
    TPM_SetCommandReadyAndWait(hi2c, 1000);
    return TUNNEL_RSP_SUCCESS;
}

uint32_t TUNNEL_TPM_GetCapability(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff, I2C_HandleTypeDef* hi2c) {
    if(cbuff->paramHead < TUNNEL_ORD_TPM_GET_CAPABILITY_PLEN) {
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_BAD_LENGTH);
    } /*
    TPM_CAPABILITY_AREA capArea = TUNNEL_BufferExtract32(cbuff);
    uint32_t subCapSize = TUNNEL_BufferExtract32(cbuff);
    uint8_t* capResp = NULL;
    size_t capRespSize = 0;
    int32_t tpm_rc =  TPM_GetCapability(hi2c, capArea, cbuff->params + cbuff->extractHead, subCapSize, &capResp, &capRespSize); //cheat a little to just pass the subcapability buffer directly
    if(TPM_SUCCESS != tpm_rc) {
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_TPM_CODE | tpm_rc);
    }
    //uart_debug_sendline("Response from TPM:\n");
    //uart_debug_hexdump(capResp, capRespSize);
    TUNNEL_BufferInit(&tunnelBuffer);
    TUNNEL_BufferExtend32(&tunnelBuffer, capRespSize);
    TUNNEL_BufferExtend(&tunnelBuffer, capResp, capRespSize);
    TUNNEL_BufferSend(tunnel, &tunnelBuffer, TUNNEL_RSP_SUCCESS, TUNNEL_ORD_TPM_GET_CAPABILITY, FALSE);
    TUNNEL_BufferFree(&tunnelBuffer);

    free(capResp); /* as TPM_GetCapability will allocate this, or should*/
    //TPM_SetCommandReadyAndWait(hi2c, 1000);
    return TUNNEL_RSP_SUCCESS;
}

uint32_t TUNNEL_TPM_GetCapabilityOwner(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff, I2C_HandleTypeDef* hi2c, AuthData_tdAuthDataStore* authDataStore/*, AuthSession* as*/) {
    if(!cbuff->hasAuth) {
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_BAD_TAG);
        return TUNNEL_RSP_BAD_TAG;
        // the only way to get here should be through sending this as a cleartext command, so in that case its bad tag
    } else if (!cbuff->authOkay) {
        //this should already be caught before being called, but we'll double-check it
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_AUTHFAIL);
        return TUNNEL_RSP_AUTHFAIL;
    }
    /*if(NULL == as || NULL == authDataStore) {
        return -1;
    }
    TPM_VERSION v;
    uint32_t nvflags;
    uint32_t vflags;
    if(!(as->alive)) {
        TPM_GetOIAPSession(hi2c, as);
        TPM_SetCommandReadyAndWait(hi2c, 1000);
    }
    int32_t tpm_rc = TPM_GetCapabilityOwner(hi2c, as, authDataStore->ownerAuthData, &v, &nvflags,  &vflags);
    TPM_SetCommandReadyAndWait(hi2c, 1000);
    if(TPM_SUCCESS != tpm_rc) {
        TUNNEL_SendShortEnc(tunnel, cbuff->command, TUNNEL_RSP_TPM_CODE | tpm_rc);
        return TUNNEL_RSP_TPM_CODE + tpm_rc;
    }
    TUNNEL_BufferInit(&tunnelBuffer);
    TUNNEL_BufferExtendStructure(&tunnelBuffer, packTPM_VERSION, &v);
    TUNNEL_BufferExtend32(&tunnelBuffer, nvflags);
    TUNNEL_BufferExtend32(&tunnelBuffer, vflags);
    TUNNEL_BufferSend(tunnel, &tunnelBuffer, TUNNEL_RSP_SUCCESS, TUNNEL_ORD_TPM_GET_CAPABILITY_OWNER, TRUE);
    TUNNEL_BufferFree(&tunnelBuffer);
    */

    return TUNNEL_RSP_SUCCESS;
}
uint32_t TUNNEL_TPM_NV_Write(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff, I2C_HandleTypeDef* hi2c, AuthData_tdAuthDataStore* authDataStore/*, AuthSession* as*/) {
    if(!cbuff->hasAuth) {
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_BAD_TAG);
        return TUNNEL_RSP_BAD_TAG;
        /* the only way to get here should be through sending this as a cleartext command, so in that case its bad tag*/
    } else if (!cbuff->authOkay) {
        //this should already be caught before being called, but we'll double-check it
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_AUTHFAIL);
        return TUNNEL_RSP_AUTHFAIL;
    }
    /*if(NULL == as || NULL == authDataStore) {
        return -1;
    }
    TPM_NV_INDEX idx = TUNNEL_BufferExtract32(cbuff);
    uint32_t offset = TUNNEL_BufferExtract32(cbuff);
    uint32_t dataLen = TUNNEL_BufferExtract32(cbuff);
    int32_t tpm_rc = TPM_NV_WriteValue(hi2c, as, authDataStore->ownerAuthData, idx, offset, dataLen, cbuff->params + cbuff->extractHead); //more extraction cheating
    TPM_SetCommandReadyAndWait(hi2c, 1000);
    if(TPM_SUCCESS != tpm_rc) {
        TUNNEL_SendShortEnc(tunnel, cbuff->command, TUNNEL_RSP_TPM_CODE | tpm_rc);
        return TUNNEL_RSP_TPM_CODE + tpm_rc;
    } else {
        TUNNEL_SendShortEnc(tunnel, cbuff->command, TUNNEL_RSP_SUCCESS);
        return TUNNEL_RSP_SUCCESS;
    }
    */
    return -1;
}

uint32_t TUNNEL_TPM_NV_Read(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff, I2C_HandleTypeDef* hi2c, AuthData_tdAuthDataStore* authDataStore/*, AuthSession* as*/) {
    if(!cbuff->hasAuth) {
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_BAD_TAG);
        return TUNNEL_RSP_BAD_TAG;
        /* the only way to get here should be through sending this as a cleartext command, so in that case its bad tag*/
    } else if (!cbuff->authOkay) {
        //this should already be caught before being called, but we'll double-check it
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_AUTHFAIL);
        return TUNNEL_RSP_AUTHFAIL;
    }
    /*
    if(NULL == as || NULL == authDataStore) {
        return -1; //coding error, these should never be null in practice
    }
    TPM_NV_INDEX idx = TUNNEL_BufferExtract32(cbuff);
    uint32_t offset = TUNNEL_BufferExtract32(cbuff);
    size_t dataLen = TUNNEL_BufferExtract32(cbuff);
    uint8_t* data = malloc(sizeof(uint8_t) * dataLen); //not sure how to easily avoid dynamic memory here. @TODO think harder next time. command temp buff?
    //uart_debug_hexprint32(dataLen);
    //uart_debug_newline();
    if(NULL == data) {
        TUNNEL_SendShortEnc(tunnel, cbuff->command, TUNNEL_RSP_OUT_OF_MEMORY);
        return TUNNEL_RSP_OUT_OF_MEMORY;
    }
    int32_t tpm_rc = TPM_NV_ReadValue(hi2c, as, authDataStore->ownerAuthData, idx, offset, &dataLen, data);
    TPM_SetCommandReadyAndWait(hi2c, 1000);
    //uart_debug_hexprint32(dataLen);
    //uart_debug_newline();
    if(TPM_SUCCESS != tpm_rc) {
        TUNNEL_SendShortEnc(tunnel, cbuff->command, TUNNEL_RSP_TPM_CODE | tpm_rc);
        free(data);
        return TUNNEL_RSP_TPM_CODE + tpm_rc;
    }
    TUNNEL_BufferInit(&tunnelBuffer);
    TUNNEL_BufferExtend32(&tunnelBuffer, dataLen);
    TUNNEL_BufferExtend(&tunnelBuffer, data, dataLen);
    TUNNEL_BufferSend(tunnel, &tunnelBuffer, TUNNEL_RSP_SUCCESS, TUNNEL_ORD_TPM_NV_READ, TRUE);
    TUNNEL_BufferFree(&tunnelBuffer);


    free(data);*/
    return TUNNEL_RSP_SUCCESS;
}
#endif /* TPM_TUNNEL_COMMANDS */

uint32_t TUNNEL_RTC_Set(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff, RTC_HandleTypeDef* hrtc) {
    if(!cbuff->hasAuth) {
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_BAD_TAG);
        return TUNNEL_RSP_BAD_TAG;
        /* the only way to get here should be through sending this as a cleartext command, so in that case its bad tag*/
    } else if (!cbuff->authOkay) {
        //this should already be caught before being called, but we'll double-check it
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_AUTHFAIL);
        return TUNNEL_RSP_AUTHFAIL;
    }

    RTC_TimeTypeDef time;
    RTC_DateTypeDef date;

    date.Year = TUNNEL_BufferExtract8(cbuff);
    date.Month = TUNNEL_BufferExtract8(cbuff);
    date.Date = TUNNEL_BufferExtract8(cbuff);
    date.WeekDay = TUNNEL_BufferExtract8(cbuff);

    time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    time.StoreOperation = RTC_STOREOPERATION_RESET;
    time.Hours = TUNNEL_BufferExtract8(cbuff);
    time.Minutes = TUNNEL_BufferExtract8(cbuff);
    time.Seconds = TUNNEL_BufferExtract8(cbuff);
    time.SubSeconds = 0;

    HAL_StatusTypeDef halStatus = HAL_RTC_SetDate(hrtc, &date, FORMAT_BCD);
    if(HAL_OK != halStatus) {
        TUNNEL_SendShortEnc(tunnel, cbuff->command, TUNNEL_RSP_FAILED);
        return TUNNEL_RSP_FAILED;
    }
    halStatus = HAL_RTC_SetTime(hrtc, &time, FORMAT_BCD);
    if(HAL_OK != halStatus) {
        TUNNEL_SendShortEnc(tunnel, cbuff->command, TUNNEL_RSP_FAILED);
        return TUNNEL_RSP_FAILED;
    } else {
        TUNNEL_SendShortEnc(tunnel, cbuff->command, TUNNEL_RSP_SUCCESS);
        return TUNNEL_RSP_SUCCESS;
    }
}

uint32_t TUNNEL_RTC_Get(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff, RTC_HandleTypeDef* hrtc) {
    if(cbuff->hasAuth) {
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_BAD_TAG);
        return TUNNEL_RSP_BAD_TAG;
        /* the only way to get here should be through sending this as an encrypted command, so in that case its bad tag*/
    }
    RTC_TimeTypeDef rtcTime;
    RTC_DateTypeDef date;

    HAL_StatusTypeDef halStatus = HAL_RTC_GetTime(hrtc, &rtcTime, FORMAT_BCD);
    if(HAL_OK != halStatus) {
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_FAILED);
        return TUNNEL_RSP_FAILED;
    }
    halStatus = HAL_RTC_GetDate(hrtc, &date, FORMAT_BCD);
    if(HAL_OK != halStatus) {
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_FAILED);
        return TUNNEL_RSP_FAILED;
    }
    TUNNEL_BufferInit(&tunnelBuffer);
    TUNNEL_BufferExtend8(&tunnelBuffer, date.Year);
    TUNNEL_BufferExtend8(&tunnelBuffer, date.Month);
    TUNNEL_BufferExtend8(&tunnelBuffer, date.Date);
    TUNNEL_BufferExtend8(&tunnelBuffer, date.WeekDay);
    TUNNEL_BufferExtend8(&tunnelBuffer, rtcTime.Hours);
    TUNNEL_BufferExtend8(&tunnelBuffer, rtcTime.Minutes);
    TUNNEL_BufferExtend8(&tunnelBuffer, rtcTime.Seconds);
    TUNNEL_BufferSend(tunnel, &tunnelBuffer, TUNNEL_RSP_SUCCESS, 0, false);
    TUNNEL_BufferFree(&tunnelBuffer);
    return TUNNEL_RSP_SUCCESS;
}

#ifdef TPM_TUNNEL_COMMANDS
/* sets up the SRK to specified parameters that are packed in to cbuff
 * srk must not be NULL (and ideally already initialized by a call to initStorageKeyInfo(...) )
 * any checking of if the TPM is already owned is left to the caller, this simply reads the command parameters and updates the passed-in TPM_KEY12 struct
 */

uint32_t TUNNEL_Setup_SRK(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff/*, TPM_KEY12* srk*/) {
    if(cbuff->hasAuth) {
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_BAD_TAG);
        return TUNNEL_RSP_BAD_TAG;
        /* the only way to get here should be through sending this as an encrypted command, so in that case its bad tag*/
        /* remember that this is only effective on an un-owned device! */
    /*} else if(NULL == srk) {
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_FAILED);
        return TUNNEL_RSP_FAILED;
        // avoids stupid programmer syndrome */
    } else if(cbuff->paramHead < TUNNEL_PLEN_ORD_SETUP_SRK_NOPCRS) { /*will check again once we know if PCRs are included */
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_BAD_LENGTH);
        return TUNNEL_RSP_BAD_LENGTH;
    }
    /*extract data first*/ /*
    TPM_KEY_FLAGS flags = TUNNEL_BufferExtract32(cbuff);
    TPM_AUTH_DATA_USAGE usage = TUNNEL_BufferExtract8(cbuff); // one byte
    uint32_t pcrInfoSize = TUNNEL_BufferExtract32(cbuff);
    if(cbuff->paramHead != (TUNNEL_PLEN_ORD_SETUP_SRK_NOPCRS + pcrInfoSize)) { // check if there's enough length with the PCRs included
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_BAD_LENGTH);
        return TUNNEL_RSP_BAD_LENGTH;
    }
    TPM_PCR_INFO_LONG pcrInfo;
    initTPM_PCR_INFO_LONG(&pcrInfo);
    if(pcrInfoSize) { // if we have PCR info
        //extractHead += extractTPM_PCR_INFO_LONG(&pcrInfo, cbuff->params + extractHead); // extract it
        TUNNEL_BufferExtractStructure(cbuff, extractTPM_PCR_INFO_LONG, &pcrInfo); //extract the PCR info
    }
    // won't fail due to length, check parameters
    uint8_t parametersOkay = TRUE;
    if(flags & (TPM_KEY_FLAG_MIGRATABLE | TPM_KEY_FLAG_MIGRATE_AUTHORITY)) {
        parametersOkay = FALSE; //cannot be migratable nor a migration authority. presumably isVolatile will also pose a problem but its not on the TPM docs.
    }
    if(usage != TPM_AUTH_ALWAYS || usage != TPM_NO_READ_PUBKEY_AUTH) {
        parametersOkay = FALSE; //must authorize to use the private portion of the key
    }
    if(pcrInfoSize) { // we were passed PCR Info, validate it
        if(!(validateTPM_PCR_INFO_LONG(&pcrInfo))) {
            parametersOkay = FALSE;
        }
    }
    if(!parametersOkay) {
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_BAD_PARAMETER);
        return TUNNEL_RSP_BAD_PARAMETER;
    }

    // parameters are okay, fill in struct!
    srk->keyFlags = flags;
    srk->authDataUsage = usage;
    if(pcrInfoSize) {
        memcpy(&(srk->PCRInfo), &pcrInfo, sizeof(TPM_PCR_INFO_LONG)); //copy the struct
    }
    */
    TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_SUCCESS);
    return TUNNEL_RSP_SUCCESS;
}

uint32_t TUNNEL_TPM_DAMSetup(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff, I2C_HandleTypeDef* hi2c) {
    if(cbuff->hasAuth) {
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_BAD_TAG);
        return TUNNEL_RSP_BAD_TAG;
        /* the only way to get here should be through sending this as an encrypted command, so in that case its bad tag*/
        /* remember that this is only effective on a TPM that has no owner!! */
    } else if(cbuff->paramHead != TUNNEL_PLEN_ORD_TPM_SETUP_DAM) {
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_BAD_LENGTH);
        return TUNNEL_RSP_BAD_LENGTH;
    } else if (statusLock != LOCK_LOCKED) {
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_FAILED);
        return TUNNEL_RSP_FAILED; //only works on a locked (unowned) device
    }
    /*
    uint8_t DAM_Threshold = TUNNEL_BufferExtract8(cbuff);
    uint8_t DAM_MaxCount = TUNNEL_BufferExtract8(cbuff);
    int32_t rc = TPM_DAM_SetParameters(hi2c, DAM_Threshold, DAM_MaxCount);
    if(TPM_SUCCESS != rc) {
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_TPM_CODE | rc);
        return TUNNEL_RSP_TPM_CODE + rc;
    }
    TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_SUCCESS);
    TPM_SetCommandReadyAndWait(hi2c, 1000);
    */
    return TUNNEL_RSP_SUCCESS;
}

uint32_t TUNNEL_TPM_GetPubKey(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff, I2C_HandleTypeDef* hi2c, uint8_t* keyAuthData/*, AuthSession* oiap*/) {
    if(!cbuff->hasAuth) {
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_BAD_TAG);
        return TUNNEL_RSP_BAD_TAG;
        /* the only way to get here should be through sending this as a cleartext command, so in that case its bad tag*/
    } else if (!cbuff->authOkay) {
        //this should already be caught before being called, but we'll double-check it
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_AUTHFAIL);
        return TUNNEL_RSP_AUTHFAIL;
    }
    if(TUNNEL_PLEN_ORD_GET_PUB_KEY != cbuff->paramHead) {
        TUNNEL_SendShortEnc(tunnel, cbuff->command, TUNNEL_RSP_BAD_LENGTH);
        return TUNNEL_RSP_BAD_LENGTH;
    }
    /*
    TPM_KEY_HANDLE kh = TUNNEL_BufferExtract32(cbuff);
    uint8_t authData[digestSize]; //local copy so we don't overwrite anything by accident
    if(TPM_KH_SRK == kh) {
        memcpy(authData, keyAuthData, digestSize);
    } else {
        //memcpy(authData, (cbuff->params + extractHead), digestSize);
        TUNNEL_BufferExtractBuffer(cbuff, authData, digestSize);
    }
    TPM_PUBKEY key;
    initTPM_PUBKEY(&key); //set this up to receive the key in to
    int32_t rc = 0;
    if(!oiap->alive) {
        rc = TPM_GetOIAPSession(hi2c, oiap);
        if(rc != TPM_SUCCESS) {
            TUNNEL_SendShortEnc(tunnel, cbuff->command, TUNNEL_RSP_TPM_CODE + rc);
            return TUNNEL_RSP_TPM_CODE + rc;
        }
    }
    rc = TPM_GetPubKey(hi2c, oiap, authData, TPM_TRUE, kh, &key);
    TPM_SetCommandReadyAndWait(hi2c, 1000);
    if(rc != TPM_SUCCESS) {
        TUNNEL_SendShortEnc(tunnel, cbuff->command, TUNNEL_RSP_TPM_CODE + rc);
        return TUNNEL_RSP_TPM_CODE + rc;
    }



    size_t packedKeyLen = sizeofTPM_PUBKEY(&key);
    uint8_t* packedKey = malloc(sizeof(uint8_t) * packedKeyLen);
    if(NULL == packedKey) {
            TUNNEL_SendShortEnc(tunnel, cbuff->command, TUNNEL_RSP_OUT_OF_MEMORY);
            freeTPM_PUBKEY(&key); // just in case
            return TUNNEL_RSP_OUT_OF_MEMORY;
        }
    packTPM_PUBKEY(packedKey, &key);

    TUNNEL_BufferInit(&tunnelBuffer);
    TUNNEL_BufferExtendStructure(&tunnelBuffer, packTPM_PUBKEY, &key);
    TUNNEL_BufferSend(tunnel, &tunnelBuffer, TUNNEL_RSP_SUCCESS, TUNNEL_ORD_TPM_GET_PUB_KEY, TRUE);
    TUNNEL_BufferFree(&tunnelBuffer);

    free(packedKey);
    freeTPM_PUBKEY(&key);
    */
    return TUNNEL_RSP_SUCCESS;
}

uint32_t TUNNEL_TPM_Seal(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX *cbuff, I2C_HandleTypeDef* hi2c, uint8_t* keyAuthData) {
    if(!cbuff->hasAuth) {
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_BAD_TAG);
        return TUNNEL_RSP_BAD_TAG;
        /* the only way to get here should be through sending this as a cleartext command, so in that case its bad tag*/
    } else if (!cbuff->authOkay) {
        //this should already be caught before being called, but we'll double-check it
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_AUTHFAIL);
        return TUNNEL_RSP_AUTHFAIL;
    }
    if(TUNNEL_PLEN_ORD_SEAL_NOPCRS > cbuff->paramHead) { //make sure we have at least enough length
        TUNNEL_SendShortEnc(tunnel, cbuff->command, TUNNEL_RSP_BAD_LENGTH);
        return TUNNEL_RSP_BAD_LENGTH;
    }
    /*
    TPM_ENTITY_TYPE et = (uint16_t) TPM_ET_AES128_CTR << 8; //use AES for the ADIP
    uint8_t localKeyAuthData[digestSize]; //local copy so we don't overwrite anything by accident
    uint8_t dataAuthData[digestSize];
    TPM_PCR_INFO_LONG pcrInfo;
    initTPM_PCR_INFO_LONG(&pcrInfo); //we may be able to just use the pcrInfo in the outData struct for optimization. @TODO ?
    TPM_STORED_DATA12 outData;
    initTPM_STORED_DATA12(&outData);

    TPM_KEY_HANDLE kh = TUNNEL_BufferExtract32(cbuff);
    if(TPM_KH_SRK == kh) {
        //extractHead += extractBuffer(localKeyAuthData, keyAuthData, digestSize);
        memcpy(localKeyAuthData, keyAuthData, digestSize);
        cbuff->extractHead += digestSize; //force it to skip ahead, over the empty keyAuthData field
        et |= TPM_ET_SRK;
    } else {
        //memcpy(localKeyAuthData, (cbuff->params + extractHead), digestSize);
        //extractHead += extractBuffer(localKeyAuthData, (cbuff->params + extractHead), digestSize);
        TUNNEL_BufferExtractBuffer(cbuff, localKeyAuthData, digestSize);
        et |= TPM_ET_KEY;
    }
    TUNNEL_BufferExtractBuffer(cbuff, dataAuthData, digestSize);
    //extractHead += extractBuffer(dataAuthData, (cbuff->params + extractHead), digestSize);
    uint32_t pcrInfoSize = TUNNEL_BufferExtract32(cbuff);
    if(pcrInfoSize) { //if we have PCRs specified
        if (cbuff->paramHead < (TUNNEL_PLEN_ORD_SEAL_NOPCRS + pcrInfoSize)) {
            TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_BAD_LENGTH);
            freeTPM_STORED_DATA12(&outData); // just in case
            return TUNNEL_RSP_BAD_LENGTH;
        } else { //have the right amount of data to extract the TPM_PCR_INFO_LONG structure
            TUNNEL_BufferExtractStructure(cbuff, extractTPM_PCR_INFO_LONG, &pcrInfo); //extract the PCR info
        }
    } // else no PCRs specified, default null set is fine
    uint32_t dataSize = TUNNEL_BufferExtract32(cbuff); */
    /*uint8_t* inData = malloc(sizeof(uint8_t) * dataSize);
    if(NULL == inData) {
        TUNNEL_SendShortEnc(tunnel, cbuff->command, TUNNEL_RSP_BAD_LENGTH);
        freeTPM_STORED_DATA12(&outData); // just in case
        return TUNNEL_RSP_OUT_OF_MEMORY;
    }
    extractHead += extractBuffer(inData, (cbuff->params + extractHead), dataSize);*/
    /*
    AuthSession localOSAPSession; //use a local session as it ends immediately due to authdata insertion
    TPM_AuthSessionInit(&localOSAPSession);
    int32_t rc = TPM_GetOSAPSession(hi2c, &localOSAPSession, et, kh, localKeyAuthData);
    TPM_SetCommandReadyAndWait(hi2c, 1000);
    if(rc != TPM_SUCCESS) {
        TUNNEL_SendShortEnc(tunnel, cbuff->command, TUNNEL_RSP_TPM_CODE + rc);
        return TUNNEL_RSP_TPM_CODE + rc;
    }

    //cheat with the inData again
    rc = TPM_Seal(hi2c, &localOSAPSession, kh, dataAuthData, &pcrInfo, dataSize, cbuff->params + cbuff->extractHead, &outData);
    TPM_SetCommandReadyAndWait(hi2c, 1000);
    //free(inData);
    if(rc != TPM_SUCCESS) {
        TUNNEL_SendShortEnc(tunnel, cbuff->command, TUNNEL_RSP_TPM_CODE + rc);
        freeTPM_STORED_DATA12(&outData);
        return TUNNEL_RSP_TPM_CODE + rc;
    }
    //size_t outDataBuffLen = sizeofTPM_STORED_DATA12(&outData); */
    /*uint8_t* outDataBuff = malloc(sizeof(uint8_t) * outDataBuffLen);
    if(NULL == outDataBuff) {
        TUNNEL_SendShortEnc(tunnel, cbuff->command, TUNNEL_RSP_OUT_OF_MEMORY);
        freeTPM_STORED_DATA12(&outData); // just in case
        return TUNNEL_RSP_OUT_OF_MEMORY;
    }
    packTPM_STORED_DATA12(outDataBuff, &outData);*/
/*
    TUNNEL_BufferInit(&tunnelBuffer);
    TUNNEL_BufferExtendStructure(&tunnelBuffer, packTPM_STORED_DATA12, &outData);
    TUNNEL_BufferSend(tunnel, &tunnelBuffer, TUNNEL_RSP_SUCCESS, TUNNEL_ORD_TPM_SEAL, TRUE);
    TUNNEL_BufferFree(&tunnelBuffer);

    freeTPM_STORED_DATA12(&outData);
    //free(outDataBuff);
     */
    return TUNNEL_RSP_SUCCESS;
}

uint32_t TUNNEL_TPM_Unseal(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff, I2C_HandleTypeDef* hi2c, uint8_t* keyAuthData/*, AuthSession* keyAuthSession, AuthSession* dataAuthSession*/) {
    if(!cbuff->hasAuth) {
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_BAD_TAG);
        return TUNNEL_RSP_BAD_TAG;
        /* the only way to get here should be through sending this as a cleartext command, so in that case its bad tag*/
    } else if (!cbuff->authOkay) {
        //this should already be caught before being called, but we'll double-check it
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_AUTHFAIL);
        return TUNNEL_RSP_AUTHFAIL;
    }
    /*
    if(NULL == dataAuthSession || NULL == keyAuthSession) {
        TUNNEL_SendShortEnc(tunnel, cbuff->command, TUNNEL_RSP_FAILED);
        return TPM_ERROR_NULL_PARAMETER;
    }
    //HAL_Delay(200); //?
    uint8_t localKeyAuthData[digestSize];
    uint8_t dataAuthData[digestSize];
    TPM_STORED_DATA12 inData;
    initTPM_STORED_DATA12(&inData);
    uint8_t* outData = NULL;
    uint32_t outDataLen = 0;
    int32_t rc = 0xDDDDDDDD; //don't want this to match anything by accident

    TPM_KEY_HANDLE kh = TUNNEL_BufferExtract32(cbuff);
    if(TPM_KH_SRK == kh) {
        memcpy(localKeyAuthData, keyAuthData, digestSize);
        cbuff->extractHead += digestSize; //skip over the empty keyAuthData in the incoming parameters
    } else {
        TUNNEL_BufferExtractBuffer(cbuff, localKeyAuthData, digestSize);
    }

    TUNNEL_BufferExtractBuffer(cbuff, dataAuthData, digestSize);
    TUNNEL_BufferExtractStructure(cbuff, extractTPM_STORED_DATA12, &inData);

    if(!keyAuthSession->alive) {
        rc = TPM_GetOIAPSession(hi2c, keyAuthSession);
        TPM_SetCommandReadyAndWait(hi2c, 1000);
        if(TPM_SUCCESS != rc) {
            TUNNEL_SendShortEnc(tunnel, cbuff->command, TUNNEL_RSP_FAILED);
            return TUNNEL_RSP_FAILED;
        }
    }
    if(!dataAuthSession->alive) {
        rc = TPM_GetOIAPSession(hi2c, dataAuthSession);
        TPM_SetCommandReadyAndWait(hi2c, 1000);
        if(TPM_SUCCESS != rc) {
            TUNNEL_SendShortEnc(tunnel, cbuff->command, TUNNEL_RSP_FAILED);
            return TUNNEL_RSP_FAILED;
        }
    }
    //HAL_Delay(200); //?
    rc = TPM_Unseal(hi2c, keyAuthSession, localKeyAuthData, dataAuthSession, dataAuthData, kh, &inData, &outDataLen, &outData);
    TPM_SetCommandReadyAndWait(hi2c, 1000);
    freeTPM_STORED_DATA12(&inData); // no longer needed
    if(TPM_SUCCESS != rc) {
        TUNNEL_SendShortEnc(tunnel, cbuff->command, TUNNEL_RSP_TPM_CODE + rc);
        free(outData);
        return TUNNEL_RSP_TPM_CODE + rc;
    }
    TUNNEL_BufferInit(&tunnelBuffer);
    TUNNEL_BufferExtend32(&tunnelBuffer, outDataLen);
    TUNNEL_BufferExtend(&tunnelBuffer, outData, outDataLen);
    TUNNEL_BufferSend(tunnel, &tunnelBuffer, TUNNEL_RSP_SUCCESS, TUNNEL_ORD_TPM_UNSEAL, TRUE);
    TUNNEL_BufferFree(&tunnelBuffer);

    free(outData);
    */
    return TUNNEL_RSP_SUCCESS;
}

uint32_t TUNNEL_TPM_CreateWrapKey(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff, I2C_HandleTypeDef* hi2c, uint8_t* keyAuthData) {
    if(!cbuff->hasAuth) {
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_BAD_TAG);
        return TUNNEL_RSP_BAD_TAG;
        /* the only way to get here should be through sending this as a cleartext command, so in that case its bad tag*/
    } else if (!cbuff->authOkay) {
        //this should already be caught before being called, but we'll double-check it
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_AUTHFAIL);
        return TUNNEL_RSP_AUTHFAIL;
    }
    /*
    TPM_ENTITY_TYPE et = ((uint16_t) TPM_ET_AES128_CTR) << 8; //use AES for the ADIP
    uint8_t localKeyAuthData[digestSize];
    uint8_t usageAuthData[digestSize];
    uint8_t migrationAuthData[digestSize];
    TPM_KEY12 createdKey;
    initTPM_KEY12(&createdKey);

    TPM_KEY_HANDLE kh = TUNNEL_BufferExtract32(cbuff);

    if(TPM_KH_SRK == kh) {
        memcpy(localKeyAuthData, keyAuthData, digestSize);
        cbuff->extractHead += digestSize; //skip over the empty keyAuthData in the incoming parameters
        et |= TPM_ET_SRK;
    } else {
        TUNNEL_BufferExtractBuffer(cbuff, localKeyAuthData, digestSize);
        et |= TPM_ET_KEY;
    }
    TUNNEL_BufferExtractBuffer(cbuff, usageAuthData, digestSize);
    TUNNEL_BufferExtractBuffer(cbuff, migrationAuthData, digestSize);
    TUNNEL_BufferExtractStructure(cbuff, extractTPM_KEY12, &createdKey);

    AuthSession localOSAPSession; // use a local session as it ends immediately due to authdata insertion
    TPM_AuthSessionInit(&localOSAPSession);
    int32_t rc = TPM_GetOSAPSession(hi2c, &localOSAPSession, et, kh, localKeyAuthData);
    TPM_SetCommandReadyAndWait(hi2c, 1000);
    if(rc != TPM_SUCCESS) {
        TUNNEL_SendShortEnc(tunnel, cbuff->command, TUNNEL_RSP_TPM_CODE + rc);
        freeTPM_KEY12(&createdKey);
        return TUNNEL_RSP_TPM_CODE + rc;
    }

    rc = TPM_CreateWrapKey(hi2c, &localOSAPSession, kh, &createdKey, usageAuthData, migrationAuthData);
    TPM_SetCommandReadyAndWait(hi2c, 1000);
    if(TPM_SUCCESS != rc) {
        TUNNEL_SendShortEnc(tunnel, cbuff->command, TUNNEL_RSP_TPM_CODE + rc);
        freeTPM_KEY12(&createdKey);
        return TUNNEL_RSP_TPM_CODE + rc;
    }

    TUNNEL_BufferInit(&tunnelBuffer);
    TUNNEL_BufferExtendStructure(&tunnelBuffer, packTPM_KEY12, &createdKey);
    TUNNEL_BufferSend(tunnel, &tunnelBuffer, TUNNEL_RSP_SUCCESS, TUNNEL_ORD_TPM_CREATE_WRAP_KEY, TRUE);
    TUNNEL_BufferFree(&tunnelBuffer);

    freeTPM_KEY12(&createdKey);
    */
    return TUNNEL_RSP_SUCCESS;
}

uint32_t TUNNEL_TPM_LoadKey(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff, I2C_HandleTypeDef* hi2c/*, AuthSession* as*/, uint8_t* keyAuthData) {
    if(!cbuff->hasAuth) {
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_BAD_TAG);
        return TUNNEL_RSP_BAD_TAG;
        /* the only way to get here should be through sending this as a cleartext command, so in that case its bad tag*/
    } else if (!cbuff->authOkay) {
        //this should already be caught before being called, but we'll double-check it
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_AUTHFAIL);
        return TUNNEL_RSP_AUTHFAIL;
    }
    /*
    if(NULL == as) {
        TUNNEL_SendShortEnc(tunnel, cbuff->command, TUNNEL_RSP_FAILED);
        return TPM_ERROR_NULL_PARAMETER;
    }
    uint8_t parentKeyAuthData[digestSize];
    TPM_KEY12 keyToLoad;
    initTPM_KEY12(&keyToLoad);
    TPM_KEY_HANDLE loadedKeyHandle;

    TPM_KEY_HANDLE kh = TUNNEL_BufferExtract32(cbuff);

    if(TPM_KH_SRK == kh) {
        memcpy(parentKeyAuthData, keyAuthData, digestSize);
        cbuff->extractHead += digestSize; //skip over the parent Key AuthData field
    } else {
        //memcpy(localKeyAuthData, (cbuff->params + extractHead), digestSize);
        TUNNEL_BufferExtractBuffer(cbuff, parentKeyAuthData, digestSize);
    }
    //extractHead += extractTPM_KEY12(&keyToLoad, (cbuff->params + extractHead));
    TUNNEL_BufferExtractStructure(cbuff, extractTPM_KEY12, &keyToLoad);

    int32_t rc = TPM_LoadKey(hi2c, as, parentKeyAuthData, kh, &keyToLoad, &loadedKeyHandle);
    TPM_SetCommandReadyAndWait(hi2c, 1000);
    if(TPM_SUCCESS != rc) {
        TUNNEL_SendShortEnc(tunnel, cbuff->command, TUNNEL_RSP_TPM_CODE + rc);
        freeTPM_KEY12(&keyToLoad);
        return TUNNEL_RSP_TPM_CODE + rc;
    }

    TUNNEL_BufferInit(&tunnelBuffer);
    TUNNEL_BufferExtend32(&tunnelBuffer, loadedKeyHandle);
    TUNNEL_BufferSend(tunnel, &tunnelBuffer, TUNNEL_RSP_SUCCESS, TUNNEL_ORD_TPM_LOAD_KEY, TRUE);
    TUNNEL_BufferFree(&tunnelBuffer);
    */
    return TUNNEL_RSP_SUCCESS;
}
#endif /* TPM_TUNNEL_COMMANDS */

uint32_t TUNNEL_GetOTCSetup(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff) {
    if(cbuff->hasAuth) {
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_BAD_TAG);
        return TUNNEL_RSP_BAD_TAG;
        /* the only way to get here should be through sending this as an encrypted command, so in that case its bad tag*/
    }
    TUNNEL_BufferInit(&tunnelBuffer);
    TUNNEL_BufferExtend8(&tunnelBuffer, NUM_KEYS);
    TUNNEL_BufferExtend8(&tunnelBuffer, TUNNEL_MINIMUM_OTC_LEN);
    TUNNEL_BufferExtend8(&tunnelBuffer, TUNNEL_MAXIMUM_OTC_LEN - 1); //
    TUNNEL_BufferExtend32(&tunnelBuffer, *BACKUP_REGS_TUNNEL_ITERS_REG);
    TUNNEL_BufferSend(tunnel, &tunnelBuffer, TUNNEL_RSP_SUCCESS, 0, false);
    TUNNEL_BufferFree(&tunnelBuffer);
    return TUNNEL_RSP_SUCCESS;
}

uint32_t TUNNEL_GetVoltages(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff) {
    uint16_t resultCounts = Adc_getVBat(&hadc);
    uint8_t chargeEnabled = false;
    uint8_t batteryLow = false;
    if(resultCounts < ADC_VBAT_LOW_COUNTS) {
        batteryLow = true;
    } //else false
    if(PWR->CR4 & PWR_CR4_VBE) {
        chargeEnabled = true;
    } //else false

    TUNNEL_BufferInit(&tunnelBuffer); //no issue with initializing this then not using it.

    TUNNEL_BufferExtend8(&tunnelBuffer, batteryLow);
    TUNNEL_BufferExtend8(&tunnelBuffer, chargeEnabled);

    if(!(cbuff->hasAuth)) {
        TUNNEL_BufferSend(tunnel, &tunnelBuffer, TUNNEL_RSP_SUCCESS, 0, false);
        TUNNEL_BufferFree(&tunnelBuffer);
        return TUNNEL_RSP_SUCCESS;
    } else if(!(cbuff->authOkay)) {
        //has auth, not okay, should have been picked up already but didn't..
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_AUTHFAIL);
        return TUNNEL_RSP_AUTHFAIL;
    } else {
        uint32_t vbatCounts = Adc_getVBat(&hadc);
        uint32_t vrefintCounts = Adc_getVRefInt(&hadc);
        uint32_t pcieCounts = Adc_getPCIE12VSense(&hadc);
        uint32_t vbusCounts = Adc_getVBus(&hadc); //@TODO massage vbus appropriately
        /*uart_debug_addToBuffer("VBus Counts: ", 13);
        uart_debug_printuint32(vbusCounts);
        uart_debug_newline();
        uart_debug_addToBuffer("VRefInt Counts: ", 16);
        uart_debug_printuint32(vrefintCounts);
        uart_debug_newline();
        uart_debug_addToBuffer("VPCI-E Counts: ", 15);
        uart_debug_printuint32(pcieCounts);
        uart_debug_newline();
        uart_debug_addToBuffer("VBat Counts: ", 13);
        uart_debug_printuint32(vbatCounts);
        uart_debug_newline();*/



        uint16_t vbatmv = (vbatCounts * ADC_VBAT_PCI_MULTIPLIER) / ADC_VBAT_RESULT_TO_MV_DIVISOR;
        uint16_t vccmv = (uint16_t) ((uint32_t) ADC_VREFINT_RESULT_TO_VCC_MV_DIVIDEND / vrefintCounts);
        uint16_t pciemv = (pcieCounts * ADC_VBAT_PCI_MULTIPLIER) / ADC_VBAT_RESULT_TO_MV_DIVISOR;

        TUNNEL_BufferExtend16(&tunnelBuffer, vccmv);
        TUNNEL_BufferExtend16(&tunnelBuffer, vbatmv);
        TUNNEL_BufferExtend16(&tunnelBuffer, pciemv);
        TUNNEL_BufferExtend16(&tunnelBuffer, vbusCounts); //@TODO put massaged vbus in here
        TUNNEL_BufferSend(tunnel, &tunnelBuffer, TUNNEL_RSP_SUCCESS, TUNNEL_ORD_GET_VOLTAGES, true);
        TUNNEL_BufferFree(&tunnelBuffer);
        return TUNNEL_RSP_SUCCESS;

    }

}


uint32_t TUNNEL_SetupInitial(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff) {
    if(cbuff->hasAuth) {
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_BAD_TAG);
        return TUNNEL_RSP_BAD_TAG;
        /* the only way to get here should be through sending this as an encrypted command, so in that case its bad tag*/
    }
    if(statusLock != LOCK_UNOWNED) {
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_DEVICE_OWNED);
        return TUNNEL_RSP_DEVICE_OWNED;
    }
    OwnershipSetupInfo setupInfo;

    setupInfo.am_threshold = TUNNEL_BufferExtract8(cbuff);
    setupInfo.am_maxCount = TUNNEL_BufferExtract8(cbuff);
    setupInfo.am_eraseCount = TUNNEL_BufferExtract8(cbuff);
    setupInfo.am_baseTime = TUNNEL_BufferExtract8(cbuff);
    setupInfo.unlockTime = TUNNEL_BufferExtract32(cbuff);
    setupInfo.tunnelTime = TUNNEL_BufferExtract32(cbuff);
    setupInfo.minPINLen = TUNNEL_BufferExtract8(cbuff);
    setupInfo.minOTCLen = TUNNEL_BufferExtract8(cbuff);
    setupInfo.relockTimeout = TUNNEL_BufferExtract16(cbuff);
    setupInfo.relockMode = TUNNEL_BufferExtract8(cbuff);

    uint32_t setupResult = Ownership_DoInitialSetup(&setupInfo);

    TUNNEL_SendShortClear(tunnel, setupResult);
    return setupResult;
}

uint32_t TUNNEL_OwnerGetConfiguration(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff) {
    if(!cbuff->hasAuth) {
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_BAD_TAG);
        return TUNNEL_RSP_BAD_TAG;
        /* the only way to get here should be through sending this as a cleartext command, so in that case its bad tag*/
    } else if (!cbuff->authOkay) {
        //this should already be caught before being called, but we'll double-check it
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_AUTHFAIL);
        return TUNNEL_RSP_AUTHFAIL;
    }
    OwnershipSetupInfo setupInfo;

    uint32_t getConfigResult = Ownership_GetConfiguration(&setupInfo);

    if(getConfigResult == TUNNEL_RSP_SUCCESS) {
        TUNNEL_BufferInit(&tunnelBuffer);
        TUNNEL_BufferExtend8(&tunnelBuffer, setupInfo.am_threshold);
        TUNNEL_BufferExtend8(&tunnelBuffer, setupInfo.am_maxCount);
        TUNNEL_BufferExtend8(&tunnelBuffer, setupInfo.am_eraseCount);
        TUNNEL_BufferExtend8(&tunnelBuffer, setupInfo.am_baseTime);
        TUNNEL_BufferExtend32(&tunnelBuffer, setupInfo.unlockTime);
        TUNNEL_BufferExtend32(&tunnelBuffer, setupInfo.tunnelTime);
        TUNNEL_BufferExtend8(&tunnelBuffer, setupInfo.minPINLen);
        TUNNEL_BufferExtend8(&tunnelBuffer, setupInfo.minOTCLen);
        TUNNEL_BufferExtend16(&tunnelBuffer, setupInfo.relockTimeout);
        TUNNEL_BufferExtend8(&tunnelBuffer, setupInfo.relockMode);
        TUNNEL_BufferSend(tunnel, &tunnelBuffer, TUNNEL_RSP_SUCCESS, TUNNEL_ORD_OWNER_GET_CONFIG, true);
        TUNNEL_BufferFree(&tunnelBuffer);
    } else {
        TUNNEL_SendShortEnc(tunnel, TUNNEL_ORD_OWNER_GET_CONFIG, getConfigResult);
    }

    return getConfigResult;

}

uint32_t TUNNEL_SetupAttackMitigation(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff) {
    if(!cbuff->hasAuth) {
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_BAD_TAG);
        return TUNNEL_RSP_BAD_TAG;
        /* the only way to get here should be through sending this as a cleartext command, so in that case its bad tag*/
    } else if (!cbuff->authOkay) {
        //this should already be caught before being called, but we'll double-check it
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_AUTHFAIL);
        return TUNNEL_RSP_AUTHFAIL;
    }

    uint8_t am_threshold, am_maxCount, am_eraseCount, am_baseTime;

    am_threshold = TUNNEL_BufferExtract8(cbuff);
    am_maxCount = TUNNEL_BufferExtract8(cbuff);
    am_eraseCount = TUNNEL_BufferExtract8(cbuff);
    am_baseTime = TUNNEL_BufferExtract8(cbuff);

    uint32_t result = Ownership_SetupAttackMitigation(am_threshold, am_maxCount, am_eraseCount, am_baseTime);

    TUNNEL_SendShortEnc(tunnel, TUNNEL_ORD_OWNER_SETUP_AM, result);

    return result;

}
uint32_t TUNNEL_SetupTunnel(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff) {
    if(!cbuff->hasAuth) {
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_BAD_TAG);
        return TUNNEL_RSP_BAD_TAG;
        /* the only way to get here should be through sending this as a cleartext command, so in that case its bad tag*/
    } else if (!cbuff->authOkay) {
        //this should already be caught before being called, but we'll double-check it
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_AUTHFAIL);
        return TUNNEL_RSP_AUTHFAIL;
    }

    uint32_t tunnelTime = TUNNEL_BufferExtract32(cbuff);
    uint8_t minOTCLen = TUNNEL_BufferExtract8(cbuff);

    uint32_t result = Ownership_SetupTunnel(minOTCLen, tunnelTime);

    TUNNEL_SendShortEnc(tunnel, TUNNEL_ORD_OWNER_SETUP_TUNNEL, result);

    return result;
}
uint32_t TUNNEL_SetupPINChange(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff) {
    if(!cbuff->hasAuth) {
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_BAD_TAG);
        return TUNNEL_RSP_BAD_TAG;
        /* the only way to get here should be through sending this as a cleartext command, so in that case its bad tag*/
    } else if (!cbuff->authOkay) {
        //this should already be caught before being called, but we'll double-check it
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_AUTHFAIL);
        return TUNNEL_RSP_AUTHFAIL;
    }

    uint32_t unlockTime = TUNNEL_BufferExtract32(cbuff);
    uint8_t minPINLen = TUNNEL_BufferExtract8(cbuff);

    uint32_t result = Ownership_SetupPINChange(minPINLen, unlockTime);

    TUNNEL_SendShortEnc(tunnel, TUNNEL_ORD_OWNER_SETUP_UNLOCK, result);

    return result;
}


uint32_t TUNNEL_SetupRelock(TransportTunnel* tunnel, TUNNEL_BUFFER_CTX* cbuff) {
    if(!cbuff->hasAuth) {
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_BAD_TAG);
        return TUNNEL_RSP_BAD_TAG;
        /* the only way to get here should be through sending this as a cleartext command, so in that case its bad tag*/
    } else if (!cbuff->authOkay) {
        //this should already be caught before being called, but we'll double-check it
        TUNNEL_SendShortClear(tunnel, TUNNEL_RSP_AUTHFAIL);
        return TUNNEL_RSP_AUTHFAIL;
    }

    uint16_t timeout = TUNNEL_BufferExtract16(cbuff);
    uint8_t mode = TUNNEL_BufferExtract8(cbuff);

    uint32_t result = Ownership_SetupRelock(timeout, mode);

    TUNNEL_SendShortEnc(tunnel, TUNNEL_ORD_OWNER_SETUP_UNLOCK, result);

    return result;
}
