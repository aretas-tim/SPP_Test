/*
 * authdata_store.c
 *
 *  Created on: Apr 6, 2016
 *      Author: me
 */

#include "authdata_store.h"

static void AuthData_initAuthDataStore(AuthData_tdAuthDataStore*);
static void AuthData_initKeyStore(AuthData_tdKeyStore*);
static void AuthData_initSaltedKeyStore(AuthData_tdSaltedKeyStore* saltedKeyStore);

void AuthData_initAuthDataStore(AuthData_tdAuthDataStore* authData) {
    memset(authData->ownerAuthData, 0, 20);
    memset(authData->srkAuthData, 0, 20); //@TODO de-magic this.. again.. siiigh
    authData->valid = 0;
}

void AuthData_initKeyStore(AuthData_tdKeyStore* keyStore) {
    keyStore->valid = 0;
    memset(keyStore->key, 0, STORAGE_KEY_LEN);
}

void AuthData_initSaltedKeyStore(AuthData_tdSaltedKeyStore* saltedAuthData_tdKeyStore) {
    saltedAuthData_tdKeyStore->valid = 0;
    memset(saltedAuthData_tdKeyStore->key, 0, TRANSPORT_KEY_LEN);
    memset(saltedAuthData_tdKeyStore->salt, 0, TRANSPORT_KEY_LEN);
}

/* initializes everything to zero, including the valid bits*/
void AuthData_initCombinedStore(AuthData_tdCombinedStore* combinedStore) {
    AuthData_initAuthDataStore(&(combinedStore->authData));
    AuthData_initKeyStore(&(combinedStore->storageKey));
    AuthData_initSaltedKeyStore(&(combinedStore->transportKey));
}


void AuthData_validateCombinedStore(AuthData_tdCombinedStore* combinedStore) {
    combinedStore->authData.valid = 1;
    combinedStore->storageKey.valid = 1;
    combinedStore->transportKey.valid = 1;
}

void AuthData_freeCombinedStore(AuthData_tdCombinedStore* combinedStore) {
    AuthData_initCombinedStore(combinedStore);
}

void AuthData_dumpSaltedKeyStore(AuthData_tdSaltedKeyStore* saltedAuthData_tdKeyStore) {
    UartDebug_sendline("Salted Key Store Dump:\n");
    UartDebug_sendline("Key:\n");
    UartDebug_hexdump(saltedAuthData_tdKeyStore->key, TRANSPORT_KEY_LEN);
    UartDebug_sendline("Salt:\n");
    UartDebug_hexdump(saltedAuthData_tdKeyStore->salt, TRANSPORT_KEY_LEN);
    UartDebug_sendString("Valid: ");
    UartDebug_printBool(saltedAuthData_tdKeyStore->valid);
    UartDebug_newline();
}
