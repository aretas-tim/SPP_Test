/**
  ******************************************************************************
  * @file    usbd_usf_hid.h
  * @author  MCD Application Team
  * @version V2.4.1
  * @date    19-June-2015
  * @brief   header file for the usbd_usf_hid.c file.
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
  *  edited by AM for U2F interface jan 2017
  */
 
/* Define to prevent recursive inclusion -------------------------------------*/ 
#ifndef __USB_U2F_HID_H
#define __USB_U2F_HID_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"
#include "usb_hid_class.h"
#include "stdbool.h"

#ifdef DEBUG
#define DEBUG_U2F
#endif /* DEBUG */

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
#define U2F_HID_EPIN_ADDR                 0x83
#define U2F_HID_EPIN_SIZE                 0x40

#define U2F_HID_EPOUT_ADDR                0x03
#define U2F_HID_EPOUT_SIZE                0x40

#define USB_U2F_HID_CONFIG_DESC_SIZ       41
#define USB_U2F_HID_DESC_SIZ              9

/**
  * @}
  */ 

#define U2F_HID_MAX_PAYLOAD_LEN 7609 /* from U2F HID protocol v 1.1, section 2.4 */
#define U2F_HID_INIT_FRAME_DATA_LEN (HID_RPT_SIZE-7)
#define U2F_HID_CONT_FRAME_DATA_LEN (HID_RPT_SIZE-5)


#define USBD_U2F_HID_REPORT_DESC_SIZE     0x1B /* 27 bytes */
#define USBD_U2F_HID_OUTREPORT_BUF_SIZE     0x40


#define USBD_U2F_HID_MAX_CHANNELS 64

static uint8_t USBD_U2F_HID_Desc[USB_U2F_HID_DESC_SIZ]; /* warning is BS, this is used */



/** @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */


typedef enum {
    U2F_HID_IDLE,
    U2F_HID_RECEIVE,
    U2F_HID_MESSAGE_READY,
    U2F_HID_PROCESSING,
    U2F_HID_TRANSMIT,
    U2F_HID_ERROR
} U2F_HID_StateTypeDef;

typedef struct _USBD_U2F_HID_Itf {
  uint8_t                  *pReport;
  int8_t (* Init)          (void);
  int8_t (* DeInit)        (void);
  int8_t (* OutEvent)      (uint8_t*);
  void (* TransmitComplete) (void);
} USBD_U2F_HID_ItfTypeDef;

typedef struct
{
  uint8_t               Report_buf[USBD_U2F_HID_OUTREPORT_BUF_SIZE];
  uint8_t                transmitReportBuffer[U2F_HID_EPIN_SIZE]; //holds a single report, used so we don't overrun the passed-in transmitBuffer when sending data that isn't a multiple of the packet length
  uint8_t                transmitImmediateBuffer[U2F_HID_EPIN_SIZE]; //holds a single report for immediate sending
  uint32_t              Protocol;
  uint32_t              IdleState;
  uint32_t              AltSetting;
  uint32_t              IsReportAvailable;
  HID_StateTypeDef      transmitState;
  HID_StateTypeDef         transmitImmediateState;
  uint8_t*                 transmitBuffer;
  size_t                transmitBufferLen;
  uint8_t                 nextSequenceNum; /* the sequence number of the next packet to be received or sent */
} USBD_U2F_HID_HandleTypeDef;
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

extern USBD_ClassTypeDef  USBD_U2F_HID;
#define USBD_U2F_HID_CLASS    &USBD_U2F_HID
/**
  * @}
  */ 

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */ 
uint8_t USBD_U2F_HID_SendReport (USBD_HandleTypeDef *pdev, uint8_t *report, uint16_t len);



uint8_t  USBD_U2F_HID_RegisterInterface  (USBD_HandleTypeDef   *pdev, USBD_U2F_HID_ItfTypeDef *fops);

/**
  * @}
  */ 

#ifdef __cplusplus
}
#endif

#endif  /* __USB_U2F_HID_H */
/**
  * @}
  */ 

/**
  * @}
  */ 
  
/************************ portions (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
