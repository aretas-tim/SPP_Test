/*
 * tpm_utils.c
 *
 *  Created on: Feb 12, 2016
 *      Author: me
 */

#include "utilities.h"
#include "string.h"

RNG_HandleTypeDef* util_rng = NULL;

/*
 * @TODO
 * look up inline keyword, as these may be candidates?
 *
 */

/* takes a 16 bit value and packs it in to a buffer at the given position
 * does not check for overrun or anything. be careful!
 */
size_t Utilities_packToBuffer16(uint8_t* buff, uint32_t pos, uint16_t data) {
    buff[pos] = data >> 8;
    buff[pos + 1] = (data & 0xFF);
    return 2;
}

/* takes a 32 bit value and packs it in to a buffer at the given position
 * does not check for overrun or anything. be careful!
 */
size_t Utilities_packToBuffer32(uint8_t* buff, uint32_t pos, uint32_t data) {
    buff[pos]     = data >> 24;
    buff[pos + 1] = (data & 0x00FF0000) >> 16;
    buff[pos + 2] = (data & 0x0000FF00) >> 8;
    buff[pos + 3] = (data & 0x000000FF);
    return 4;
}
/* extracts four bytes starting at pos from buff, returns as a uint32_t */
uint32_t Utilities_extract32(uint8_t* buff, uint32_t pos) {
    uint32_t val = buff[pos] << 24;
    val += buff[pos + 1] << 16;
    val += buff[pos + 2] << 8;
    val += buff[pos + 3];
    return val;
}
/* extracts two bytes starting at pos from buff, returns as a uint16_t */
uint16_t Utilities_extract16(uint8_t* buff, uint32_t pos) {
    uint16_t val = buff[pos] << 8;
    val += buff[pos + 1];
    return val;
}
/* basically a wrapper around memcpy that returns the buffer length instead of the to pointer*/
size_t Utilities_extractBuffer(uint8_t* outBuff, uint8_t* inBuff, size_t len) {
    memcpy(outBuff, inBuff, len);
    return len;
}

/* inspired by mbedtls aes.c, simplified to only work on uint8_t*s.
 * zeroes out memory, might need to be re-implemented if its optimized out somehow*/

void Utilities_zeroize(uint8_t* d, uint32_t len) {
    volatile uint8_t* e = d;
     for(uint32_t i = 0; i < len; ++i) {
         *(e + i) = 0;
     }
 }

uint32_t Utilities_init(RNG_HandleTypeDef* hrng) {
    if(NULL != hrng) {
        util_rng = hrng;
        return 0;
    } else {
        return 1;
    }
}

/* to interact with the mbedtls library for RSA encryption, need to hook it in to the RNG
 * param is ignored as i have no idea what its for, but the mbedtls requires it. just pass it NULL
 */
int Utilities_getRandomBuff(void* param, unsigned char* buff, size_t len) {
    if(NULL == util_rng) {
        return -1;
    }
    uint32_t r; //holds the current random number
    uint8_t rBytesLeft = 0;
    for(size_t i = 0; i < len; ++i) {
        if(0 == rBytesLeft) {
            //while(HAL_OK != HAL_RNG_GenerateRandomNumber(nonce_rng, &r));
            /* @TODO maybe add a timeout here or something so it doesn't spin forever*/

            while(!(RNG->SR & RNG_SR_DRDY)); //spin until the RNG has data
            //get more bits from the RNG
            r = RNG->DR;
            rBytesLeft = 4;
        }
        buff[i] = r & 0xFF;
        r >>= 8;
        rBytesLeft--;
    }
    return 0;
}

/* compares two digests of length len
 * hopefully in constant-time
 * returns 1 if they match, 0 if they do not.
 */
uint8_t Utilities_compareDigests(const uint8_t const *digestOne, const uint8_t const *digestTwo, const uint32_t len) {
    uint8_t match = 0;
    for(uint32_t i = 0; i < len; ++i) {
        match |= digestOne[i] ^ digestTwo[i];
    }
    return (match == 0);
}

/*
 * modified copy of the AES CTR crypt function from mbedtls, to handle the 32-bit wrap functionality as required by the TPM
 * removed reference to stream position, always start at the beginning
 * fixed to 32 bit nonce rollover as per TPM part 1 31.1.3 (i.e. most significant 96 bits are fixed, least significant 32 rollover after 0x...FFFFFFFF
 *
 */
int32_t Utilities_tpmAesCtrCrypt(mbedtls_aes_context* ctx, size_t length, uint8_t nonce_counter[16], uint8_t* input, uint8_t* output) {
    int c, i;
    int blockSize = 16; //@TODO demagic this
    uint8_t stream_block[blockSize];
    size_t n = 0;
    int counter_bytes = 4;

    while( length-- ) {
        if( n == 0 ) {
            mbedtls_aes_crypt_ecb( ctx, MBEDTLS_AES_ENCRYPT, nonce_counter, stream_block );
            for( i = blockSize; i > (blockSize - counter_bytes); i-- ) {
                if( ++nonce_counter[i - 1] != 0 ) {
                    break;
                }
            }
        }
        c = *input++;
        *output++ = (unsigned char)( c ^ stream_block[n] );
        n = ( n + 1 ) & 0x0F; /*this is based off of block size, remember to update it if the block size changes for some reason*/
    }
    return( 0 );
}

/* TPM2 KDFa(...)
 * see TPM2 Part 4, section B.8.6.2 and TPM2 Part 1 Section 11.4.9.1. as well as NIST SP800-108
 * note that currently there is support *only* for SHA1, SHA256 and SHA512
 *
 * param hashAlg     the TPM algorithm ID number used for the underlying hash
 * param key        the key to the HMAC function
 * param label        a label, null terminated character array or NULL
 * param contextU    contextU
 * param contextV     contextV
 * param sizeInBits    the number of bits of output to produce, ignored in case of once being TRUE
 * param keyStrem    output is placed here
 * param counterInOut    iteration counter, if required
 * param once        TRUE to perform the generation function only once, sizeInBits is thus the length of the underlying hash
 *
 */
/*uint16_t KDFa(TPM_ALG_ID hashAlg, TPM2B* key, const char* label, TPM2B* contextU, TPM2B* contextV, uint32_t sizeInBits, uint8_t* keyStream, uint32_t* counterInOut, BOOL once) {
    uint32_t counter = 0;
    int32_t lLen = 0;
    int16_t hLen;
    int16_t bytes;
    uint8_t* stream = keyStream;
    uint8_t marshaledUint32[4];
    mbedtls_md_context_t ctx; //aka hashState
    TPM2B_MAX_HASH_BLOCK hmacKey;

    if((key == NULL) || (keyStream == NULL)) {
        uart_debug_sendline("KDFa error. Key or keyStream is null.\n");
        return 0;
    }
    if((once == FALSE) && ((sizeInBits & 7) == 0)) {
        uart_debug_sendline("KDFa error in requested output length.\n");
    }
    if(label != NULL) {
        for(lLen = 0; label[lLen++] != 0;); //clever. gets the length of a null-terminated string
    }
    //translate TPM_ALG_ID in to the related mbedtls mbedtls_md_info_t

    mbedtls_md_init(&ctx);
    //mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), TRUE);
    mbedtls_md_info_t* mdInfo;
    switch(hashAlg) {
        case TPM_ALG_SHA1:
            mdInfo = mbedtls_md_info_from_type(MBEDTLS_MD_SHA1);
            break;
        case TPM_ALG_SHA256:
            mdInfo = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
            break;
        case TPM_ALG_SHA512:
            mdInfo = mbedtls_md_info_from_type(MBEDTLS_MD_SHA512);
            break;
        default:
            uart_debug_sendline("KDFa error. Unsupported Hash Algorithm.\n");
            return 0;
    }
    if(NULL == mdInfo) {

    }
    mbedtls_md_setup(&ctx, mdInfo, TRUE);

    hLen = mbedtls_md_get_size(mdInfo);

    if(((sizeInBits + 7) / 8) >= INT16_MAX) {
        uart_debug_sendline("KDFa error. Too many bits requested.\n");
        return 0;
    }
    uint8_t* hmacOut = malloc(hLen); //as mbedtls always returns the hash length, we have to make a temp to prevent overrunning keyStream
    if(hmacOut == NULL) {
        uart_debug_sendline("KDFa error. Unable to create temp buffer.\n");
        return 0;
    }

    bytes = once ? hLen : ((int16_t) ((sizeInBits + 7) / 8));

    while(bytes > 0) {
        if(bytes < hLen) {
            hLen = bytes;
        }
        counter++;
        mbedtls_md_hmac_starts(&ctx, key->buffer, key->size); //init HMAC
        Utilities_packToBuffer32(marshaledUint32, 0, counter);
        uart_debug_hexdump(marshaledUint32, 4);
        mbedtls_md_hmac_update(&ctx, marshaledUint32, sizeof(uint32_t));
        if(label != NULL) {
            mbedtls_md_hmac_update(&ctx, label, lLen);
        }
        if(contextU != NULL) {
            mbedtls_md_hmac_update(&ctx, contextU->buffer, contextU->size);
        }
        if(contextV != NULL) {
            mbedtls_md_hmac_update(&ctx, contextV->buffer, contextV->size);
        }
        Utilities_packToBuffer32(marshaledUint32, 0, sizeInBits);
        uart_debug_hexdump(marshaledUint32, 4);
        mbedtls_md_hmac_update(&ctx, marshaledUint32, sizeof(uint32_t));
        mbedtls_md_hmac_finish(&ctx, hmacOut);

        memcpy(stream, hmacOut, hLen); //copy out of our temp buffer

        stream += hLen; //update the pointer
        bytes -= hLen; //decrement bytes left
    }
    free(hmacOut); //done with this
    if((sizeInBits % 8) != 0) {
        keyStream[0] &= ((1 << (sizeInBits %8)) -1); //mask off non-required bits in MSO
    }
    if(counterInOut != NULL) {
        *counterInOut = counter; //update counter
    }
    return ((sizeInBits + 7) / 8);

}*/
