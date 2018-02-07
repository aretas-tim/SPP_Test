/*
 * hotkey.h
 *
 *  Created on: Jan 24, 2017
 *      Author: me
 *
 *      holds all the hotkey stuff and dispatches hotkey requests
 */

#ifndef HOTKEY_H_
#define HOTKEY_H_


#include "usbd_hotkey_hid_if.h"
#include "usb_hid.h" /* KeyWithModifiers struct */

#define HOTKEY_MAX_NUM_HOTKEYS 10
#define HOTKEY_DATA_LEN_MAX_KEYS 127
#define HOTKEY_DATA_LEN_MAX_TEXT (HOTKEY_DATA_LEN_MAX_KEYS * 2)

#define HOTKEY_STORAGE_TYPE_POS 0
#define HOTKEY_STORAGE_DATA_POS 1

#define HOTKEY_STORAGE_BYTES_PER_LEN_ASCII 1 /* 1 assuming char is one byte */
#define HOTKEY_STORAGE_BYTES_PER_LEN_KEYS 2 /* bytes per KeysWithModifier */



#define HOTKEY_SUCCESS 0
#define HOTKEY_INIT_ERROR 1
#define HOTKEY_UNKNOWN_DATA_TYPE 2
#define HOTKEY_DOES_NOT_EXIST 3

#define HOTKEY_BFSS_ERROR 0x01000000 /* AND with the BFSS error */

#define HOTKEY_TYPE_ASCII 0
#define HOTKEY_TYPE_KEY_WITH_MODIFIERS 1

//workaround for definition problems. should be in usb_hid.h but its not picking it up
#ifndef KEY_WITH_MODIFIERS_DEFINED
#define KEY_WITH_MODIFIERS_DEFINED
typedef struct tdKEY_WITH_MODIFIERS {
    uint8_t modifiers;
    uint8_t scancode;
} KeyWithModifiers;
#endif /* KEY_WITH_MODIFIERS_DEFINED */

typedef struct tdHotkeyData {
    uint8_t len;
    uint8_t dataType;
    union {
        char text[HOTKEY_DATA_LEN_MAX_TEXT]; //as long as the below
        KeyWithModifiers keys[HOTKEY_DATA_LEN_MAX_KEYS];
    };
} HotkeyData;

#ifdef DEVICE_0_A_1_1_S
uint32_t HOTKEY_Init(void);
void HOTKEY_DeInit(void);
uint32_t HOTKEY_SetHotkey(BFSS_Record* record);
#endif /*DEVICE_0_A_1_1_S */




#endif /* HOTKEY_H_ */
