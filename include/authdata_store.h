/*
 * authdata_store.h
 *
 *  Created on: Mar 24, 2016
 *      Author: me
 */

#ifndef AUTHDATA_STORE_H_
#define AUTHDATA_STORE_H_

#include <stdlib.h>
#include <stdint.h>

#include <string.h> /* memset*/
#include "uart_debug.h"


#define AUTHDATA_LEN 20 /* @TODO make it not a dirty hack again. //was a dirty hack because of a compile-problem with not being able to find digestSize*/
/* @TODO fix this */
#define STORAGE_KEY_LEN 32
#define TRANSPORT_KEY_LEN STORAGE_KEY_LEN

typedef struct TDAuthDataStore {
    uint8_t valid;
    uint8_t ownerAuthData[AUTHDATA_LEN];
    uint8_t srkAuthData[AUTHDATA_LEN];
} AuthData_AuthDataStore;

typedef struct TDKeyStore {
    uint8_t valid;
    uint8_t key[STORAGE_KEY_LEN];
} AuthData_KeyStore;

/* use this one when we need to store the salt.
 * mostly this is for the transport keys, as we'll need to provide the salt to the host so they can run their own KDF
 */
typedef struct TDSaltedKeyStore {
    uint8_t valid;
    uint8_t key[STORAGE_KEY_LEN];
    uint8_t salt[STORAGE_KEY_LEN];
} AuthData_SaltedKeyStore;

typedef struct TDCombinedStore {
    AuthData_AuthDataStore authData;
    AuthData_KeyStore storageKey;
    AuthData_SaltedKeyStore transportKey;
} AuthData_CombinedStore;


void AuthData_initCombinedStore(AuthData_CombinedStore* combinedStore);
void AuthData_validateCombinedStore(AuthData_CombinedStore* combinedStore);
void AuthData_freeCombinedStore(AuthData_CombinedStore* combinedStore);
void AuthData_dumpSaltedKeyStore(AuthData_SaltedKeyStore* saltedAuthData_tdKeyStore);
#endif /* AUTHDATA_STORE_H_ */
