/*
 * authdata_store.c
 *
 *  Created on: Apr 6, 2016
 *      Author: me
 */

#include "authdata_store.h"


void initAuthDataStore(AuthDataStore* authData) {
    memset(authData->ownerAuthData, 0, 20);
    memset(authData->srkAuthData, 0, 20); //@TODO de-magic this.. again.. siiigh
    authData->valid = 0;
}

void initKeyStore(KeyStore* keyStore) {
    keyStore->valid = 0;
    memset(keyStore->key, 0, STORAGE_KEY_LEN);
}

void initSaltedKeyStore(SaltedKeyStore* saltedKeyStore) {
    saltedKeyStore->valid = 0;
    memset(saltedKeyStore->key, 0, TRANSPORT_KEY_LEN);
    memset(saltedKeyStore->salt, 0, TRANSPORT_KEY_LEN);
}

/* initializes everything to zero, including the valid bits*/
void initCombinedStore(CombinedStore* combinedStore) {
    initAuthDataStore(&(combinedStore->authData));
    initKeyStore(&(combinedStore->storageKey));
    initSaltedKeyStore(&(combinedStore->transportKey));
}


void validateCombinedStore(CombinedStore* combinedStore) {
    combinedStore->authData.valid = 1;
    combinedStore->storageKey.valid = 1;
    combinedStore->transportKey.valid = 1;
}

void freeCombinedStore(CombinedStore* combinedStore) {
    initCombinedStore(combinedStore);
}

void dumpSaltedKeyStore(SaltedKeyStore* saltedKeyStore) {
    uart_debug_sendline("Salted Key Store Dump:\n");
    uart_debug_sendline("Key:\n");
    uart_debug_hexdump(saltedKeyStore->key, TRANSPORT_KEY_LEN);
    uart_debug_sendline("Salt:\n");
    uart_debug_hexdump(saltedKeyStore->salt, TRANSPORT_KEY_LEN);
    uart_debug_sendstring("Valid: ");
    uart_debug_printBool(saltedKeyStore->valid);
    uart_debug_newline();
}
