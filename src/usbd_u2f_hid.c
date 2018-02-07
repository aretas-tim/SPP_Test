/*
 * usbd_u2f_hid.c
 *
 * this is basically a stub to ensure there are HID Descriptors available and not just empty memory
 *
 *  Created on: Jan 20, 2017
 *      Author: me
 */

#include "usbd_u2f_hid.h"

/* USB HOTKEY HID device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_U2F_HID_Desc[USB_U2F_HID_DESC_SIZ] __ALIGN_END =
{
  /* 18 */
  0x09,         /*bLength: Hotkey HID Descriptor size*/
  HID_DESCRIPTOR_TYPE, /*bDescriptorType: Hotkey HID*/
  0x11,         /*b Hotkey HID  Class Spec release number*/
  0x01,
  0x00,         /*bCountryCode: Hardware target country, U2F is not localized*/
  0x01,         /*bNumDescriptors: Number of Hotkey HID class descriptors to follow*/
  0x22,         /*bDescriptorType*/
  USBD_U2F_HID_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
  0x00,
};


