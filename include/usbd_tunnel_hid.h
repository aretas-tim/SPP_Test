/**
  ******************************************************************************
  * @file    usbd_tunnel_hid.h
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
  *  edited by AM
  */
 
/* Define to prevent recursive inclusion -------------------------------------*/ 
#ifndef __USB_TUNNEL_HID_H
#define __USB_TUNNEL_HID_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"
#include "usb_hid_class.h"
#include "tunnel_shim.h"
#include <stdbool.h>

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
#define TUNNEL_HID_EPIN_ADDR                 0x82
#define TUNNEL_HID_EPIN_SIZE                 0x40

#define TUNNEL_HID_EPOUT_ADDR                0x02
#define TUNNEL_HID_EPOUT_SIZE                0x40

#define USB_TUNNEL_HID_CONFIG_DESC_SIZ       41
#define USB_TUNNEL_HID_DESC_SIZ              9

/**
  * @}
  */ 

#define USBD_TUNNEL_HID_REPORT_DESC_SIZE     0x1C /* 28 bytes */
#define USBD_TUNNEL_HID_OUTREPORT_BUF_SIZE     0x40

#define TUNNEL_HID_DATA_BUFF_SIZE (USBD_TUNNEL_HID_OUTREPORT_BUF_SIZE * 0x10) /* holds sixteen full buffers for now */

#define TUNNEL_HID_IMMEDIATE_PACKET_BUFFER_LEN 2

static uint8_t USBD_TUNNEL_HID_Desc[USB_TUNNEL_HID_DESC_SIZ]; /* warning is BS, this is used */

/** @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */


typedef struct _USBD_TUNNEL_HID_Itf
{
  uint8_t                  *pReport;
  int8_t (* Init)          (void);
  int8_t (* DeInit)        (void);
  int8_t (* OutEvent)      (uint8_t*);
  int8_t (* InEvent)       (bool*);
  //void (*TransmitComplete) (void);

}USBD_TUNNEL_HID_ItfTypeDef;

typedef struct
{
  uint8_t               Report_buf[USBD_TUNNEL_HID_OUTREPORT_BUF_SIZE];
  //uint8_t                transmitReportBuffer[TUNNEL_HID_EPIN_SIZE]; //holds a single report, used so we don't overrun the passed-in transmitBuffer when sending data that isn't a multiple of the packet length
  //volatile TunnelShimPacket        transmitImmediatePacket[TUNNEL_HID_IMMEDIATE_PACKET_BUFFER_LEN]; //holds packets for immediate send
  //volatile uint8_t            transmitImmediateNum;
  uint32_t              Protocol;
  uint32_t              IdleState;
  uint32_t              AltSetting;
  uint32_t              IsReportAvailable;
  HID_StateTypeDef      transmitState;
  //HID_StateTypeDef        transmitInitiationState;
  //HID_StateTypeDef        transmitImmediateState;
  //HID_StateTypeDef         receiveState;
  union {
      TunnelShimPacket        inPacketBuffer;
      uint8_t                inByteBuffer[TUNNEL_HID_EPIN_SIZE];
  };
  //size_t                transmitPacketBufferLen;
  //volatile bool            packetTransmitted[TUNNEL_SHIM_MAX_SEQUENCE_NUM + 1];
  //volatile uint8_t        nextTransmitPacketNum;
}
USBD_TUNNEL_HID_HandleTypeDef;
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

extern USBD_ClassTypeDef  USBD_TUNNEL_HID;
#define USBD_TUNNEL_HID_CLASS    &USBD_TUNNEL_HID
/**
  * @}
  */ 

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */ 
uint8_t USBD_TUNNEL_HID_SendReport (USBD_HandleTypeDef *pdev, uint8_t *report, uint16_t len);



uint8_t  USBD_TUNNEL_HID_RegisterInterface  (USBD_HandleTypeDef   *pdev, USBD_TUNNEL_HID_ItfTypeDef *fops);

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
