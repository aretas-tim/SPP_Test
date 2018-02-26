/*
 * tpm_utils.h
 *
 *  Created on: Feb 12, 2016
 *      Author: me
 */

#ifndef TPM_UTILS_H_
#define TPM_UTILS_H_

#include <stdlib.h>
#include <stdint.h>
#include "stm32l4xx.h"
#include "aes.h"
#include "stm32l4xx_hal_rng.h"
#include "md.h"

#ifndef UTILITIES_AES_BLOCK_SIZE
#define UTILITIES_AES_BLOCK_SIZE 16 /* in bytes */
#endif /* not defined UTILITIES_AES_BLOCK_SIZE */

#define UTILITIES_BITMAP_NO_BIT_FOUND 0xFFFFFFFF

size_t Utilities_packToBuffer16(uint8_t*, uint32_t, uint16_t);
size_t Utilities_packToBuffer32(uint8_t*, uint32_t, uint32_t);
uint32_t Utilities_extract32(uint8_t*, uint32_t);
uint16_t Utilities_extract16(uint8_t*, uint32_t);
size_t Utilities_extractBuffer(uint8_t* outBuff, uint8_t* inBuff, size_t len);

void Utilities_zeroize(uint8_t* d, uint32_t len);

uint8_t Utilities_compareDigests(const uint8_t const *digestOne, const uint8_t const *digestTwo, const uint32_t len);
int32_t Utilities_aesCtrCrypt(mbedtls_aes_context*, size_t, uint8_t nonce_counter[16], uint8_t*, uint8_t*);

int Utilities_getRandomBuff(void* param, unsigned char* buff, size_t len);
uint32_t Utilities_init(RNG_HandleTypeDef* hrng);



#endif /* TPM_UTILS_H_ */
