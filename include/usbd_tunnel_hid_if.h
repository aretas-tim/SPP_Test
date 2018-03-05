/**
  ******************************************************************************
  * @file    usbd_customhid_if_template.h
  * @author  MCD Application Team
  * @version V2.4.1
  * @date    19-June-2015
  * @brief   Header for usbd_customhid_if_template.c file.
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
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_CUSTOMHID_IF_TEMPLATE_H
#define __USBD_CUSTOMHID_IF_TEMPLATE_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_tunnel_hid.h"
#include "uart_debug.h"
#include "tunnel_shim.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/


/**
* how did we get this crazy number?
* need at least TUNNEL_SHIM_MIN_BUFFER_SPACE
* TUNNEL_HID_EPIN_SIZE = TUNNEL_HID_EPOUT_SIZE < TUNNEL_SHIM_MAX_IMM_BUFFER_LEN
* need one packet buffer to send from, two immediates to prevent backlogging of those
* and then round it all up to a whole continuation packet payload
*/
/* preprocessor abuse */
#if (TUNNEL_SHIM_MIN_BUFFER_SPACE & (TUNNEL_HID_EPIN_SIZE - 1)) /* if the minimum is not divisible by the EPIN size */
#define TUNNEL_HID_BUFFER_LEN (((TUNNEL_SHIM_MIN_BUFFER_SPACE / TUNNEL_HID_EPIN_SIZE) + 4) * TUNNEL_HID_EPIN_SIZE) /*account for the extra packet */
#else
#define TUNNEL_HID_BUFFER_LEN (TUNNEL_SHIM_MIN_BUFFER_SPACE + (3 * TUNNEL_HID_EPIN_SIZE)) /*  everything lines up, no extra packet needed */
#endif



/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
extern USBD_TUNNEL_HID_ItfTypeDef  USBD_TUNNEL_HID_Callbacks;
//extern USBD_TUNNEL_HID_ItfTypeDef USBD_Tunnel_HID_template_fops;


#ifndef TUNNEL_HID_VERBOSE
#define TUNNEL_HID_VERBOSE 1 /* ALL THE VERBOSITY */
#endif /* TUNNEL_HID_VERBOSE */

void TunnelHid_setDisconnectedCallback(void (*callback)(void));
//void TUNNEL_HID_Reset(void);

//size_t TUNNEL_HID_MessageAvailable(void);
//size_t TUNNEL_HID_GetMessage(uint8_t** message, uint8_t* channel);
//uint16_t TUNNEL_HID_SendResponse(uint8_t* response, uint16_t len);

//uint16_t TUNNEL_HID_SendData(uint8_t* data, uint16_t len);
//size_t TUNNEL_HID_ReceiveData(uint8_t* data, size_t maxLen);
//uint8_t TUNNEL_HID_ReceiveByte(void);

//void TUNNEL_HID_DoTick(void);
//void TUNNEL_HID_CheckTimeout(void);
//void TUNNEL_HID_TransmitInitTimeoutCallback(void);

TunnelShim_Context* TunnelHid_getShimContext(void);

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CUSTOMHID_IF_TEMPLATE_H */

/************************ portions (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
