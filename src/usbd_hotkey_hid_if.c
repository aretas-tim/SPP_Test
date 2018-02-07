/**
  ******************************************************************************
  * @file    usbd_customhid_if_template.c
  * @author  MCD Application Team
  * @version V2.4.1
  * @date    19-June-2015
  * @brief   USB Device Custom HID interface file.
  *             This template should be copied to the user folder, renamed and customized
  *          following user needs.
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
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <usbd_hotkey_hid_if.h>
#include "usbd_pat_comp.h"
#include <stdbool.h>
#include "usb_hid.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

static int8_t HOTKEY_HID_Init     (void);
static int8_t HOTKEY_HID_DeInit   (void);
static int8_t HOTKEY_HID_OutEvent (uint8_t* report);

#define HOTKEY_INTERNAL_BUFFER_LEN 256

KeyWithModifiers HOTKEY_InternalBuffer[HOTKEY_INTERNAL_BUFFER_LEN];

/* Private variables ---------------------------------------------------------*/

extern USBD_HandleTypeDef hUsbDeviceFS; //so we can send from this file


__ALIGN_BEGIN static uint8_t HOTKEY_HID_ReportDesc[USBD_HOTKEY_HID_REPORT_DESC_SIZE] __ALIGN_END =
{
  /* USER CODE BEGIN 0 */
  0x05, 0x01,  /* usage page 0x01 generic desktop, 2*/
  0x09, 0x06,  /* usage 0x06, 4*/
  0xA1, 0x01, /* collection collection application, 6*/
  0x75, 0x01, /* 1 bit report size, 8*/
  0x95, 0x08, /* report count 8, 10*/
  0x05, 0x07, /* usage page 0x07 key codes, 12*/
  0x19, 0xE0, /* usage minimum 224 (left control), 14*/
  0x29, 0xE7, /* usage maximum 231 (right GUI (windows key)), 16*/
  0x15, 0x00, /* logical minimum 0, 18 */
  0x25, 0x01, /* logical maximum 1, 20 */
  0x81, 0x02, /* input (data, variable, absolute), 22 */
  0x95, 0x01, /* report count 1, 24*/
  0x75, 0x08, /* 8 bit report size, 26*/
  0x81, 0x01, /* input (constant), 28 */
  0x95, 0x05, /* report count 1, 30*/
  0x75, 0x01, /* 1 bit report size, 32*/
  0x05, 0x08, /* usage page 0x08 leds, 34*/
  0x19, 0x01, /* usage minimum 1 (num lock), 36*/
  0x29, 0x05, /* usage maximum 5 (kana (?)), 38*/
  0x91, 0x02, /* output (array), 40*/
  0x95, 0x01, /* report count 1, 42*/
  0x75, 0x03, /* 3 bit report size, 44*/
  0x91, 0x01, /* output (constant), 46*/
  0x95, 0x06, /* report count 6, 48*/
  0x75, 0x08, /* 8 bit report size, 50*/
  0x15, 0x00, /* logical minimum 0, 52 */
  0x26, 0xFF, 0x00, /* logical maximum 255 (signed), 55 */
  0x05, 0x07, /* usage page 0x07 key codes, 57*/
  0x19, 0x00, /* usage minimum 0, 59*/
  0x29, 0xFF, /* usage maximum 255, 61*/
  0x81, 0x00, /* input (data, array, absolute), 63 */
  0xC0    /*     END_COLLECTION,                 64*/

};

USBD_HOTKEY_HID_ItfTypeDef USBD_HOTKEY_HID_Callbacks =
{
        HOTKEY_HID_ReportDesc,
        HOTKEY_HID_Init,
        HOTKEY_HID_DeInit,
        HOTKEY_HID_OutEvent,
};
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  TEMPLATE_CUSTOM_HID_Init
  *         Initializes the CUSTOM HID media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t HOTKEY_HID_Init(void) {

    memset(HOTKEY_InternalBuffer, 0, sizeof(KeyWithModifiers) * HOTKEY_INTERNAL_BUFFER_LEN);
    return (0);
}

/**
  * @brief  TEMPLATE_CUSTOM_HID_DeInit
  *         DeInitializes the CUSTOM HID media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t HOTKEY_HID_DeInit(void)
{
  /*
     Add your deinitialization code here 
  */  
    //clear the internal buffer
    memset(HOTKEY_InternalBuffer, 0, sizeof(KeyWithModifiers) * HOTKEY_INTERNAL_BUFFER_LEN);
    return (0);
}


/**
  * @brief  TEMPLATE_CUSTOM_HID_Control
  *         Manage the CUSTOM HID class events       
  * @param  event_idx: event index
  * @param  state: event state
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t HOTKEY_HID_OutEvent  (uint8_t* report )
{ 
    uart_debug_addToBuffer("USB Hotkey HID Received:\n", 25);
    uart_debug_hexdump(report, USBD_HOTKEY_HID_OUTREPORT_BUF_SIZE);
    uart_debug_newline();

    //? have to confirm this



  return (0);
}



/* sends a null-terminated character array out the HID as keystrokes
 * sends up to HOTKEY_INTERNAL_BUFFER_LEN in keys
 * unprintable characters are skipped over
 * returns the number of scancodes sent (usually equal to the number of characters)
 * if the text is longer than the internal buffer, text will be truncated.
 */
uint16_t HOTKEY_HID_SendString(char* text) {
    if(text == NULL) {
        return 0;
    }

#ifdef USBD_COMPOSITE
    USBD_HOTKEY_HID_HandleTypeDef *hhid = &(((PAT_COMP_Data*) hUsbDeviceFS.pClassData)->hotkeyHIDData);
#elif defined USBD_STANDALONE_HOTKEY
    USBD_HOTKEY_HID_HandleTypeDef *hhid = (USBD_HOTKEY_HID_HandleTypeDef*) hUsbDeviceFS.pClassData;
#endif

    if(hhid->transmitState != HID_IDLE) {
        return 0; //HID busy, nothing sent
    }
    //uart_debug_hexdump(text, 32);
    uint16_t len = 0;
    while((*text != '\0') && (len < HOTKEY_INTERNAL_BUFFER_LEN)) {
        getKeyFromASCII(*text, &(HOTKEY_InternalBuffer[len]));
        text++;
        if(HOTKEY_InternalBuffer[len].scancode == SCANCODE_NULL) {
            continue; //jump out early as it was an unprintable character
        }
        len++;
    }
    //uart_debug_hexprint32(len);
    //uart_debug_sendline("USB Hotkey HID Send:\n");
    //uart_debug_hexdump((uint8_t*) HOTKEY_InternalBuffer, sizeof(KeyWithModifiers) * len);

    HOTKEY_HID_SendScancodes(HOTKEY_InternalBuffer, len);
    return len;
}


/* sends keystrokes to the host, one at a time
 * will handle duplicated scancodes by inserting a null report between them */
uint16_t HOTKEY_HID_SendScancodes(KeyWithModifiers* codes, uint16_t len) {

#ifdef USBD_COMPOSITE
    USBD_HOTKEY_HID_HandleTypeDef *hhid = &(((PAT_COMP_Data*) hUsbDeviceFS.pClassData)->hotkeyHIDData);
#elif defined USBD_STANDALONE_HOTKEY
    USBD_HOTKEY_HID_HandleTypeDef *hhid = (USBD_HOTKEY_HID_HandleTypeDef*) hUsbDeviceFS.pClassData;
#endif

    if(hhid->transmitState != HID_IDLE || len == 0) {
        return 0; //HID busy or length of 0, nothing sent
    }
    uint8_t* report = hhid->transmitReportBuffer;
    memset(report, 0, USBD_HOTKEY_HID_INREPORT_BUF_SIZE); //clear buffer

    report[HOTKEY_IN_REPORT_MODIFIER_BYTE] = codes[0].modifiers;
    report[HOTKEY_IN_REPORT_SCANCODE_BYTE] = codes[0].scancode;

    hhid->transmitBuffer = codes + 1;
    hhid->transmitBufferLen = len - 1;
    hhid->isFirstChar = true;

    //uart_debug_sendline("USB Hotkey HID Send:\n");
    //uart_debug_hexdump((uint8_t*) codes, sizeof(KeyWithModifiers) * len);



    //experimental.
    if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED ) {
        if(hhid->transmitState == HID_IDLE) {
            hhid->transmitState = HID_BUSY;
            //uart_debug_sendline("USB HID Sent:\n");
            //uart_debug_hexdump(report, TUNNEL_HID_EPIN_SIZE);
            USBD_LL_Transmit (&hUsbDeviceFS, HOTKEY_HID_EPIN_ADDR, report, HOTKEY_HID_EPIN_SIZE);
            return len;
        } else {
            return 0;
        }
    } else {
        return 0;
    }
    return 0;

    //further packets will be send out by the dataIn function
}
/************************ portions (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
