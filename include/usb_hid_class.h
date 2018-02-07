/*
 * usb_hid_class.h
 *
 * basically holds some constants for the USB HID interface class
 *
 *  Created on: Nov 15, 2016
 *      Author: me
 */

#ifndef USB_HID_CLASS_H_
#define USB_HID_CLASS_H_

#define HID_DESCRIPTOR_TYPE           0x21
#define HID_REPORT_DESC               0x22

#define HID_REQ_SET_PROTOCOL          0x0B
#define HID_REQ_GET_PROTOCOL          0x03

#define HID_REQ_SET_IDLE              0x0A
#define HID_REQ_GET_IDLE              0x02

#define HID_REQ_SET_REPORT            0x09
#define HID_REQ_GET_REPORT            0x01

typedef enum
{
  HID_IDLE = 0,
  HID_BUSY,
  HID_FULL
}
HID_StateTypeDef;

#endif /* USB_HID_CLASS_H_ */
