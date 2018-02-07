/*
 * usb_hid.h
 *
 *  Created on: Jan 20, 2017
 *      Author: me
 */

#ifndef INCLUDE_USB_USB_HID_H_
#define INCLUDE_USB_USB_HID_H_

#include <stdint.h>

#define HID_PROTOCOL_BOOT 0
#define HID_PROTOCOL_REPORT 1


#define HID_KB_DEFAULT_IDLE_RATE 500

#define MOD_KEY_LEFT_SHIFT 0x02
#define MOD_SHIFT MOD_KEY_LEFT_SHIFT

#define SCANCODE_NULL 0


#ifndef KEY_WITH_MODIFIERS_DEFINED
#define KEY_WITH_MODIFIERS_DEFINED
typedef struct tdKEY_WITH_MODIFIERS {
	uint8_t modifiers;
	uint8_t scancode;
} KeyWithModifiers;
#endif /* KEY_WITH_MODIFIERS_DEFINED */

/**
 * gets a scancode and modifiers from the LUT
 */
void getKeyFromASCII(char in, KeyWithModifiers* out);

#endif /* INCLUDE_USB_USB_HID_H_ */
