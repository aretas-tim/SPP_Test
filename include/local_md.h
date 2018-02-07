/*
 * local_md.h
 *
 *  Created on: May 2, 2016
 *      Author: me
 */

#ifndef LOCAL_MD_H_
#define LOCAL_MD_H_

/* this is basically a "wrapper" class so we can avoid mbedtls' malloc calls, which seem to be causing errors*/
#include "md.h"
#include "sha256.h"
#include "sha512.h"
#include "sha1.h"

#define LOCAL_MD_HMAC_CTX_LEN 128 /* see md.c line 235*/

mbedtls_sha256_context localmd_sha256digestctx;
mbedtls_sha1_context localmd_sha1digestctx;
mbedtls_md_context_t localmd_hmacctx;
uint8_t localmd_hmac_hmac_ctx[LOCAL_MD_HMAC_CTX_LEN]; //see md.c line 235. can't use it directly so we have to fix it. this is fixed for SHA256

void local_md_init(void);
int32_t localHMAC_SHA1(uint8_t* key, size_t keyLen, uint8_t* message, size_t messageLen, uint8_t* hmac);
int32_t localHMAC_SHA256(uint8_t* key, size_t keyLen, uint8_t* message, size_t messageLen, uint8_t* hmac);
int32_t localDigestSHA256(uint8_t* message, size_t messageLen, uint8_t* digest);
int32_t localDigestSHA1(uint8_t* message, size_t messageLen, uint8_t* digest);


#endif /* LOCAL_MD_H_ */
