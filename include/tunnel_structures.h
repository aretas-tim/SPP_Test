#ifndef TUNNEL_STRUCTURES_H
#define TUNNEL_STRUCTURES_H

#include "aes.h"
#include "md.h"
#include "sha256.h"
#include <stdbool.h>

#define TUNNEL_SESSION_KEY_LENGTH 32 /*32 bytes = 256 bits for AES256 encryption*/
#define TUNNEL_NONCE_LENGTH 32
#define TUNNEL_HMAC_LENGTH 32
#define TUNNEL_COUNTER_LENGTH 16
#define TUNNEL_HASH_LENGTH 32

#define TUNNEL_LEN_RSP_ENC (TUNNEL_HMAC_LENGTH + TUNNEL_NONCE_LENGTH)
#define TUNNEL_POS_LEN 4 /*position of the length field*/
#define TUNNEL_LEN_LEN 4 /*length of the length field*/

#define TUNNEL_POS_TAG 2 /* position of the tag */

#define TUNNEL_CMD_CLEAR_HEADER_LEN 12 /* length of the header field for clear or decyrpted commands*/
#define TUNNEL_CMD_ENC_HEADER_LEN 8 /* length of the header field for encryption (command / response ordinal is encrypted)*/

#define TUNNEL_HEADER_LEN 8
#define TUNNEL_FIELD_ORD_LEN 4

typedef struct TDTransportTunnel {
    uint8_t sessionKey[TUNNEL_SESSION_KEY_LENGTH];
    uint8_t sessionAlive;
    uint8_t nonceEven[TUNNEL_NONCE_LENGTH];
    uint8_t nonceOdd[TUNNEL_NONCE_LENGTH];
    uint8_t sessionCounter[TUNNEL_COUNTER_LENGTH];
    uint64_t blocksEncrypted; /* keep track of how much we encrypt so we know when to toss the session for fear of reusing a counter value*/
    mbedtls_aes_context aesctx;
    uint16_t (*sendFunc)(uint8_t* data, uint16_t len);
} TunnelStructures_TransportTunnel;

typedef struct tdTUNNEL_BUFFER_CTX {
    uint16_t tag;
    /* length is calculated automatically*/
    uint32_t command; /* command ordinal*/
    size_t paramHead;
    size_t extractHead; /*dual-purpose, position in the params field to extract the next byte(s) when processing the command, or how many bytes have been received when receiving the command */
    uint8_t* params;
    mbedtls_sha256_context digestctx;
    uint8_t authSection[TUNNEL_LEN_RSP_ENC];
    uint8_t digest[TUNNEL_NONCE_LENGTH];
    uint8_t digested;
    uint8_t hasAuth;
    uint8_t authOkay; /* true if authorization has passed */
    mbedtls_md_context_t hmacctx;
    mbedtls_sha256_context hmac_digest_ctx;
    uint8_t hmac_hmac_ctx[64 * 2]; //see md.c line 235. can't use it directly so we have to fix it.
} TunnelStructures_TunnelBufferCtx;



#endif /*TUNNEL_STRUCTURES_H*/
