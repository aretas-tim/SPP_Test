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
#ifndef __USBD_HOTKEY_HID_IF_H
#define __USBD_HOTKEY_HID_IF_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_hotkey_hid.h"
#include "uart_debug.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/


/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
extern USBD_HOTKEY_HID_ItfTypeDef  USBD_HOTKEY_HID_Callbacks;
//extern USBD_TUNNEL_HID_ItfTypeDef USBD_Tunnel_HID_template_fops;

uint16_t HOTKEY_HID_SendString(char* text);
uint16_t HOTKEY_HID_SendScancodes(KeyWithModifiers* codes, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __USBD_HOTKEY_HID_IF_H */

/************************ portions (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
