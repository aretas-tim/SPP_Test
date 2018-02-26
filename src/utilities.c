/*
 * tpm_utils.c
 *
 *  Created on: Feb 12, 2016
 *      Author: me
 */
#include <stdbool.h>
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

void setBitInMap(uint16_t bitNum, uint8_t* map, uint16_t mapByteLen) {
    uint16_t byteNum = bitNum >> 3;
    if(byteNum >= mapByteLen) {
        return;
    } else {
        map[byteNum] |= (uint8_t) 1 << (bitNum & 0x7);
    }
}

void clearBitInMap(uint16_t bitNum, uint8_t* map, uint16_t mapByteLen) {
    uint16_t byteNum = bitNum >> 3;
    if(byteNum >= mapByteLen) {
        return;
    } else {
        map[byteNum] &= ~((uint8_t) 1 << (bitNum & 0x7));
    }
}

bool checkBitInMap(uint16_t bitNum, uint8_t* map, uint16_t mapByteLen) {
    uint16_t byteNum = bitNum >> 3;
    if(byteNum >= mapByteLen) {
        return false;
    } else {
        return (map[byteNum] & ((uint8_t) 1 << (bitNum & 0x7))) ? true : false; //abuse conditional so we're not returning "128" for true or something
    }
}

/**
 * gets the bit number of the lowest unset bit in the map
 * just a linear search.
 * this is not designed for use on large maps
 * returns BITMAP_NO_BIT_FOUND (0xFFFFFFFF) if there are no unset bits in the map
 * otherwise returns from 0 to 524,287 (2^19 less one) depending on the length of the map
 */
uint32_t getLowestUnsetBitInMap(uint8_t* map, uint16_t mapByteLen) {
    uint16_t byteNum = 0;
    while((map[byteNum] == 0xFF) && (byteNum < mapByteLen)) {
        byteNum++;
    }
    if(byteNum < mapByteLen) { //at least one bit in this byte is 0
        uint32_t bitNum = 0;
        uint8_t byte = map[byteNum];
        //search linearly through the byte, LSBit to MSBit
        for(uint8_t b = 1; b; b <<= 1) {
            if(byte & b) {
                bitNum++; //increment this
            } else {
                //not set, found it!
                break;
            }
        }
        bitNum += byteNum << 3; //add our byte number
        return bitNum;

    } else {
        return UTILITIES_BITMAP_NO_BIT_FOUND; //error, no bit found
    }
}

