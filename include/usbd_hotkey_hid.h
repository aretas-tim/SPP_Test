/**
  ******************************************************************************
  * @file    usbd_hotkey_hid.h
  * @author  MCD Application Team
  * @version V2.4.1
  * @date    19-June-2015
  * @brief   header file for the usbd_tunnel_hid.c file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>portions &copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  *  edited by AM 2017-01-20 to support hotkey interface (boot keyboard emulation)
  */
 
/* Define to prevent recursive inclusion -------------------------------------*/ 
#ifndef __USB_HOTKEY_HID_H
#define __USB_HOTKEY_HID_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"
#include "usb_hid_class.h"
#include "usb_hid.h"
#include "pat-config.h"
#include "stdbool.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */
  
/** @defgroup USBD_CUSTOM_HID
  * @brief This file is the Header file for USBD_customhid.c
  * @{
  */ 


/** @defgroup USBD_CUSTOM_HID_Exported_Defines
  * @{
  */ 

#ifdef USBD_COMPOSITE
#define HOTKEY_HID_EPIN_ADDR                 0x84
#elif defined USBD_STANDALONE_HOTKEY
#define HOTKEY_HID_EPIN_ADDR                 0x81
#else
#error "USB Device Not Defined"
#endif
#define HOTKEY_HID_EPIN_SIZE                 0x08 /* boot KB is all we need */

 /* no OUT EP for hotkey
#define HOTKEY_HID_EPOUT_ADDR                0x04
#define HOTKEY_HID_EPOUT_SIZE                0x40
*/
#define USB_HOTKEY_HID_CONFIG_DESC_SIZ       34
#define USB_HOTKEY_HID_DESC_SIZ              9

#define USB_KB_DEFAULT_IDLE_RATE             500

/**
  * @}
  */ 

#define USBD_HOTKEY_HID_REPORT_DESC_SIZE     0x40 /* 64 bytes */
#define USBD_HOTKEY_HID_OUTREPORT_BUF_SIZE     0x1 /* even without an out EP, we can still receive reports via the control pipe */
#define USBD_HOTKEY_HID_INREPORT_BUF_SIZE    HOTKEY_HID_EPIN_SIZE /* need to keep one buffer around */


#define HOTKEY_IN_REPORT_MODIFIER_BYTE         0 /* where the modifier keys go*/
#define HOTKEY_IN_REPORT_SCANCODE_BYTE         2 /* where the scancode goes */

//#define TUNNEL_HID_DATA_BUFF_SIZE (USBD_TUNNEL_HID_OUTREPORT_BUF_SIZE * 0x10) /* holds sixteen full buffers for now */

static uint8_t USBD_HOTKEY_HID_Desc[USB_HOTKEY_HID_DESC_SIZ]; /* warning is BS, this is used */

/** @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */




typedef struct _USBD_HOTKEY_HID_Itf
{
  uint8_t                  *pReport;
  int8_t (* Init)          (void);
  int8_t (* DeInit)        (void);
  int8_t (* OutEvent)      (uint8_t*);

} USBD_HOTKEY_HID_ItfTypeDef;

typedef struct
{
  uint8_t               Report_buf[USBD_HOTKEY_HID_OUTREPORT_BUF_SIZE];
  uint8_t                transmitReportBuffer[USBD_HOTKEY_HID_INREPORT_BUF_SIZE];
  uint32_t              Protocol;
  uint32_t              IdleState;
  uint32_t              AltSetting;
  uint32_t              IsReportAvailable;
  HID_StateTypeDef      transmitState;
  HID_StateTypeDef         receiveState;
  KeyWithModifiers*        transmitBuffer;
  size_t                transmitBufferLen;
  bool                    isFirstChar;
}
USBD_HOTKEY_HID_HandleTypeDef;
/**
  * @}
  */ 



/** @defgroup USBD_CORE_Exported_Macros
  * @{
  */ 

/**
  * @}
  */ 

/** @defgroup USBD_CORE_Exported_Variables
  * @{
  */ 

extern USBD_ClassTypeDef  USBD_HOTKEY_HID;
#define USBD_HOTKEY_HID_CLASS    &USBD_HOTKEY_HID
/**
  * @}
  */ 

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */ 
//uint8_t USBD_HOTKEY_HID_SendReport (USBD_HandleTypeDef *pdev, uint8_t *report, uint16_t len);



uint8_t  USBD_HOTKEY_HID_RegisterCallbacks(USBD_HandleTypeDef *pdev, USBD_HOTKEY_HID_ItfTypeDef* hotkeyHIDCallbacks);

/**
  * @}
  */ 

#ifdef __cplusplus
}
#endif

#endif  /* __USB_TUNNEL_HID_H */
/**
  * @}
  */ 

/**
  * @}
  */ 
  
/************************ portions (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
