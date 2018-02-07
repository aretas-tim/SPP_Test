/*
 * status.h
 *
 *  Created on: Mar 23, 2016
 *      Author: me
 */

#ifndef STATUS_H_
#define STATUS_H_

typedef enum TDLockStatus {
    LOCK_LOCKED,
    LOCK_UNLOCKING,
    LOCK_UNLOCKED,
    LOCK_UNOWNED,
    LOCK_PIN_VERIFY,
    LOCK_ERROR, /*for fatal errors*/
    LOCK_PIN_CHANGE_VERIFY_CURRENT,
    LOCK_PIN_CHANGE_ENTER_NEW,
    LOCK_PIN_CHANGE_VERIFY_NEW
} LockStatus;



typedef enum TDWorkStatus {
    WORK_OFF,
    WORK_SLEEP,
    WORK_AWAKE,
    WORK_WORKING,
    WORK_ERROR /* this is bad */
} WorkStatus;

typedef enum TDCommStatus {
    COMM_DISCONNECTED,
    COMM_CONNECTING,
    COMM_CONNECTED,
    COMM_ERROR
    /* activity must be handled outside of this struct */
} CommStatus;

typedef enum TDTunnelStatus {
    TUNNEL_DISCONNECTED,
    TUNNEL_CONNECTED,
    TUNNEL_CONNECTING,
    TUNNEL_ERROR
} TunnelStatus;

#endif /* STATUS_H_ */
