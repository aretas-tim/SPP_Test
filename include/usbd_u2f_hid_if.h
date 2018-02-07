/**
  ******************************************************************************
  * @file    usbd_usf_if.h
  * @author  MCD Application Team
  * @version V2.4.1
  * @date    19-June-2015
  * @brief   Header for usbd_usf_if..c file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
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
  * See the License for th?
  e specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  ******************************************************************************
  ******************************************************************************
  * modified by AM jan 2017 for U2F
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_USF_IF_H
#define __USBD_USF_IF_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_u2f_hid.h"
#include "uart_debug.h"
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

//where the first 1 is the control byte and the second 1 is the key handle length byte
//max length is set by the length of the authentication request
#define U2F_HID_OUT_BUFFER_LEN (1 + U2F_CHAL_SIZE + U2F_APPID_SIZE + 1 + U2F_MAX_KH_SIZE)


#ifndef U2F_HID_DEVICE_VERSION_MAJOR
#define U2F_HID_DEVICE_VERSION_MAJOR 0
#endif /* U2F_HID_DEVICE_VERSION_MAJOR */

#ifndef U2F_HID_DEVICE_VERSION_MINOR
#define U2F_HID_DEVICE_VERSION_MINOR 1
#endif /* U2F_HID_DEVICE_VERSION_MINOR */

#ifndef U2F_HID_DEVICE_VERSION_BUILD
#define U2F_HID_DEVICE_VERSION_BUILD 1
#endif /* U2F_HID_DEVICE_VERSION_BUILD */


#define U2F_HID_RECEIVE_TIMEOUT_SECONDS (U2FHID_TRANS_TIMEOUT / 1000) /* since the spec is given in ms */



/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
extern USBD_U2F_HID_ItfTypeDef  USBD_U2F_HID_Callbacks;
//extern USBD_TUNNEL_HID_ItfTypeDef USBD_Tunnel_HID_template_fops;


void U2F_HID_SecondTick(void);
bool U2F_HID_IsMessageWaiting(void);
uint16_t U2F_HID_GetMessageLength(void);
uint16_t U2F_HID_ReadMessage(uint8_t* msg, uint16_t maxMsgLen);
uint16_t U2F_HID_SendResponse(uint8_t response, uint8_t* data, uint16_t dataLen);

#ifdef __cplusplus
}
#endif

#endif /* __USBD_USF_IF_H */

/************************ portions (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
