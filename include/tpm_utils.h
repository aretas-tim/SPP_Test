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

size_t packToBuffer16(uint8_t*, uint32_t, uint16_t);
size_t packToBuffer32(uint8_t*, uint32_t, uint32_t);
uint32_t extract32(uint8_t*, uint32_t);
uint16_t extract16(uint8_t*, uint32_t);
size_t extractBuffer(uint8_t* outBuff, uint8_t* inBuff, size_t len);

void zeroize(uint8_t* d, uint32_t len);

uint8_t compareDigests(uint8_t*, uint8_t*, uint32_t);
int32_t TPM_AES_CTR_Crypt(mbedtls_aes_context*, size_t, uint8_t nonce_counter[16], uint8_t*, uint8_t*);

int getRandomBuff(void* param, unsigned char* buff, size_t len);
int32_t TPM_Utils_Init(RNG_HandleTypeDef* hrng);



#endif /* TPM_UTILS_H_ */
