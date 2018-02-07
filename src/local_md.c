/*
 * local_md.h
 *
 *  Created on: May 2, 2016
 *      Author: me
 */

#include "local_md.h"
#include "string.h"


void local_md_init(void) {
    mbedtls_sha256_init(&localmd_sha256digestctx);
    mbedtls_sha1_init(&localmd_sha1digestctx);
    memset(localmd_hmac_hmac_ctx, 0, LOCAL_MD_HMAC_CTX_LEN);
    localmd_hmacctx.hmac_ctx = localmd_hmac_hmac_ctx;
    /* localmd_hmacctx.md_ctx and localmd_hmacctx.md_info are set in the direct hmac wrappers*/
}

int32_t localDigestSHA256(uint8_t* message, size_t messageLen, uint8_t* digest) {
    mbedtls_sha256_init(&localmd_sha256digestctx);
    mbedtls_sha256_starts(&localmd_sha256digestctx, 0);
    mbedtls_sha256_update(&localmd_sha256digestctx, message, messageLen);
    mbedtls_sha256_finish(&localmd_sha256digestctx, digest);
    mbedtls_sha256_free(&localmd_sha256digestctx);
    return 0;
}

int32_t localDigestSHA1(uint8_t* message, size_t messageLen, uint8_t* digest) {
    mbedtls_sha1_init(&localmd_sha1digestctx);
    mbedtls_sha1_starts(&localmd_sha1digestctx);
    mbedtls_sha1_update(&localmd_sha1digestctx, message, messageLen);
    mbedtls_sha1_finish(&localmd_sha1digestctx, digest);
    mbedtls_sha1_free(&localmd_sha1digestctx);
    return 0;
}

int32_t localHMAC_SHA256(uint8_t* key, size_t keyLen, uint8_t* message, size_t messageLen, uint8_t* hmac) {
    mbedtls_sha256_init(&localmd_sha256digestctx);
    localmd_hmacctx.md_ctx = &localmd_sha256digestctx;
    localmd_hmacctx.md_info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
    /*this avoids the call to setup*/

    mbedtls_md_hmac_starts(&localmd_hmacctx, key, keyLen);
    mbedtls_md_hmac_update(&localmd_hmacctx, message, messageLen);
    mbedtls_md_hmac_finish(&localmd_hmacctx, hmac);
    mbedtls_md_free(&localmd_hmacctx);
    return 0;
}

int32_t localHMAC_SHA1(uint8_t* key, size_t keyLen, uint8_t* message, size_t messageLen, uint8_t* hmac) {
    mbedtls_sha1_init(&localmd_sha1digestctx);
    localmd_hmacctx.md_ctx = &localmd_sha1digestctx;
    localmd_hmacctx.md_info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA1);
    /*this avoids the call to setup*/

    mbedtls_md_hmac_starts(&localmd_hmacctx, key, keyLen);
    mbedtls_md_hmac_update(&localmd_hmacctx, message, messageLen);
    mbedtls_md_hmac_finish(&localmd_hmacctx, hmac);
    mbedtls_md_free(&localmd_hmacctx);
    return 0;
}
