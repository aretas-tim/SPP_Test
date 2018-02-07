/*
 * tunnel_lengths.h
 *
 *  Created on: Apr 26, 2016
 *      Author: me
 */

#ifndef TUNNEL_LENGTHS_H_
#define TUNNEL_LENGTHS_H_

//#include "tpm_st33tpm12i2c.h"
//#include "tpm_defines.h"

#define TUNNEL_PLEN_ORD_SETUP_SRK_NOPCRS 9 /* length if PCR Count is 0 */

#define TUNNEL_PLEN_ORD_GET_PUB_KEY 24

#define TUNNEL_PLEN_ORD_SEAL_NOPCRS 52 /* keyHandle, keyAuthData, dataAuthData, pcrSize, inDataSize */

#define TUNNEL_PLEN_ORD_UNSEAL_NODATA 44 /* keyHandle, keyAuthData, dataAuthData*/

#define TUNNEL_PLEN_ORD_CREATE_WRAP_KEY_NODATA 64 /* keyHandle, keyAuthData, dataAuthData*/

#define TUNNEL_PLEN_ORD_LOAD_KEY_NODATA 24 /* parentKeyHandle, parentKeyAuthData*/

#endif /* TUNNEL_LENGTHS_H_ */
