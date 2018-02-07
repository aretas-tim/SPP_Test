/*
 * usbd_pat_comp.h
 *
 *  Created on: Nov 15, 2016
 *      Author: me
 */


#ifndef USBD_PAT_COMP_H_
#define USBD_PAT_COMP_H_


#include "usbd_msc.h"
#include "usbd_tunnel_hid.h"
#include "usbd_u2f_hid.h"
#include "usbd_hotkey_hid.h"
#include "usb_hid.h"
#include <stdint.h>

#define USB_EP_NUM_MASK 0x7F

#define INTERFACE_NUM_MSC 0x00
#define INTERFACE_NUM_TUNNEL 0x01
#define INTERFACE_NUM_U2F 0x02
#define INTERFACE_NUM_HOTKEY 0x03

extern USBD_ClassTypeDef USBD_PAT_COMP;


/*pointers are good here */
typedef struct tdPAT_COMP_Callbacks {
    USBD_StorageTypeDef* storageCallbacks;
    USBD_TUNNEL_HID_ItfTypeDef* tunnelHIDCallbacks;
    USBD_U2F_HID_ItfTypeDef* U2FHIDCallbacks;
    USBD_HOTKEY_HID_ItfTypeDef* hotkeyHIDCallbacks;
} PAT_COMP_Callbacks;


/* pointers would be very, very bad here, unless we change how the memory is allocated*/
typedef struct tdPAT_COMP_Data {
    USBD_MSC_BOT_HandleTypeDef storageData;
    USBD_TUNNEL_HID_HandleTypeDef tunnelHIDData;
    USBD_U2F_HID_HandleTypeDef U2FHIDData;
    USBD_HOTKEY_HID_HandleTypeDef hotkeyHIDData;
} PAT_COMP_Data;

uint8_t  USBD_PAT_COMP_RegisterCallbacks(USBD_HandleTypeDef *pdev, USBD_StorageTypeDef* storageCallbacks, USBD_TUNNEL_HID_ItfTypeDef* tunnelHIDCallbacks, USBD_U2F_HID_ItfTypeDef* U2FHIDCallbacks, USBD_HOTKEY_HID_ItfTypeDef* hotkeyHIDCallbacks);


#endif /* USBD_PAT_COMP_H_ */
