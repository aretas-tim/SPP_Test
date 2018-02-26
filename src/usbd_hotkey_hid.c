/*
 * usbd_hotkey_hid.c
 *
 * this is basically a stub to ensure there are HID Descriptors available and not just empty memory
 *
 *  Created on: Jan 20, 2017
 *      Author: me
 */
#include "usbd_desc.h"
#include "usbd_ctlreq.h"

#include "uart_debug.h"
#include "usbd_hotkey_hid.h"

/* USB HOTKEY HID device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HOTKEY_HID_Desc[USB_HOTKEY_HID_DESC_SIZ] __ALIGN_END =
{
  /* 18 */
  0x09,         /*bLength: Hotkey HID Descriptor size*/
  HID_DESCRIPTOR_TYPE, /*bDescriptorType: Hotkey HID*/
  0x11,         /*b Hotkey HID  Class Spec release number*/
  0x01,
  0x21,         /*bCountryCode: Hardware target country, 0x21 = 33 = US*/
  0x01,         /*bNumDescriptors: Number of Hotkey HID class descriptors to follow*/
  0x22,         /*bDescriptorType*/
  USBD_HOTKEY_HID_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
  0x00,
};

static uint8_t  USBD_HOTKEY_HID_Init (USBD_HandleTypeDef *pdev,
                               uint8_t cfgidx);

static uint8_t  USBD_HOTKEY_HID_DeInit (USBD_HandleTypeDef *pdev,
                                 uint8_t cfgidx);

static uint8_t  USBD_HOTKEY_HID_Setup (USBD_HandleTypeDef *pdev,
                                USBD_SetupReqTypedef *req);

static uint8_t  *USBD_HOTKEY_HID_GetCfgDesc (uint16_t *length);

static uint8_t  *USBD_HOTKEY_HID_GetDeviceQualifierDesc (uint16_t *length);

static uint8_t  USBD_HOTKEY_HID_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t  USBD_HOTKEY_HID_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t  USBD_HOTKEY_HID_EP0_RxReady (USBD_HandleTypeDef  *pdev);

USBD_ClassTypeDef  USBD_HOTKEY_HID =
{
  USBD_HOTKEY_HID_Init,
  USBD_HOTKEY_HID_DeInit,
  USBD_HOTKEY_HID_Setup,
  NULL, /*EP0_TxSent*/
  USBD_HOTKEY_HID_EP0_RxReady, /*EP0_RxReady*/ /* STATUS STAGE IN */
  USBD_HOTKEY_HID_DataIn, /*DataIn*/
  USBD_HOTKEY_HID_DataOut,
  NULL, /*SOF */
  NULL,
  NULL,
  USBD_HOTKEY_HID_GetCfgDesc,
  USBD_HOTKEY_HID_GetCfgDesc,
  USBD_HOTKEY_HID_GetCfgDesc,
  USBD_HOTKEY_HID_GetDeviceQualifierDesc,
};

/* USB CUSTOM_HID device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HOTKEY_HID_CfgDesc[USB_HOTKEY_HID_CONFIG_DESC_SIZ] __ALIGN_END =
{
  0x09, /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION, /* bDescriptorType: Configuration */
  USB_HOTKEY_HID_CONFIG_DESC_SIZ,
  /* wTotalLength: Bytes returned */
  0x00,
  0x01,         /*bNumInterfaces: 1 interface*/
  0x01,         /*bConfigurationValue: Configuration value*/
  0x00,         /*iConfiguration: Index of string descriptor describing
  the configuration*/
  0xC0,         /*bmAttributes: bus powered */
  0x32,         /*MaxPower 100 mA: this current is used for detecting Vbus*/

  /************** Descriptor of CUSTOM HID interface ****************/
  /* 09 */
  0x09,         /*bLength: Interface Descriptor size*/
  USB_DESC_TYPE_INTERFACE,/*bDescriptorType: Interface descriptor type*/
  0x00,         /*bInterfaceNumber: Number of Interface*/
  0x00,         /*bAlternateSetting: Alternate setting*/
  0x01,         /*bNumEndpoints*/
  0x03,         /*bInterfaceClass: HID*/
  0x01,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
  0x01,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
  0,            /*iInterface: Index of string descriptor*/
  /******************** Descriptor of CUSTOM_HID *************************/
  /* 18 */
  0x09,         /*bLength: CUSTOM_HID Descriptor size*/
  HID_DESCRIPTOR_TYPE, /*bDescriptorType: CUSTOM_HID*/
  0x11,         /*bCUSTOM_HIDUSTOM_HID: CUSTOM_HID Class Spec release number*/
  0x01,
  0x21,         /*bCountryCode: Hardware target country*/
  0x01,         /*bNumDescriptors: Number of CUSTOM_HID class descriptors to follow*/
  0x22,         /*bDescriptorType*/
  USBD_HOTKEY_HID_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
  0x00,
  /******************** Descriptor of Custom HID endpoints ********************/
  /* 27 */
  0x07,          /*bLength: Endpoint Descriptor size*/
  USB_DESC_TYPE_ENDPOINT, /*bDescriptorType:*/

  HOTKEY_HID_EPIN_ADDR,     /*bEndpointAddress: Endpoint Address (IN)*/
  0x03,          /*bmAttributes: Interrupt endpoint*/
  HOTKEY_HID_EPIN_SIZE, /*wMaxPacketSize: 8 Byte max */
  0x00,
  0x01          /*bInterval: Polling Interval (1 ms)*/
  /* 34 */
} ;

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HOTKEY_HID_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};




uint8_t  USBD_HOTKEY_HID_RegisterCallbacks(USBD_HandleTypeDef *pdev, USBD_HOTKEY_HID_ItfTypeDef* hotkeyHIDCallbacks) {

    pdev->pUserData = hotkeyHIDCallbacks;

    return 0;
}

static uint8_t  USBD_HOTKEY_HID_Init (USBD_HandleTypeDef *pdev,
                               uint8_t cfgidx)
{
  uint8_t ret = 0;
  USBD_HOTKEY_HID_HandleTypeDef     *hHotkeyHID;
  /* Open EP IN */
  USBD_LL_OpenEP(pdev,
                     HOTKEY_HID_EPIN_ADDR,
                 USBD_EP_TYPE_INTR,
                 HOTKEY_HID_EPIN_SIZE);



  pdev->pClassData = USBD_malloc(sizeof (USBD_HOTKEY_HID_HandleTypeDef));

  if(pdev->pClassData == NULL)
  {
    ret = 1;
  }
  else
  {
    hHotkeyHID = (USBD_HOTKEY_HID_HandleTypeDef*) pdev->pClassData;


    hHotkeyHID->IdleState = HID_KB_DEFAULT_IDLE_RATE;
    hHotkeyHID->Protocol = HID_PROTOCOL_REPORT;
    hHotkeyHID->transmitState = HID_IDLE;
    ((USBD_HOTKEY_HID_ItfTypeDef*) pdev->pUserData)->Init();
  }

  return ret;
}

uint8_t  USBD_HOTKEY_HID_DeInit (USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
    /* Close Hotkey HID EP IN */
    USBD_LL_CloseEP(pdev, HOTKEY_HID_EPIN_ADDR);

    /* Free allocated memory */
    if(pdev->pClassData != NULL) {
        ((USBD_HOTKEY_HID_ItfTypeDef*) pdev->pUserData)->DeInit();



        USBD_free(pdev->pClassData);
        pdev->pClassData = NULL;
    }
    return USBD_OK;
}

uint8_t  USBD_HOTKEY_HID_Setup (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req) {
    uint16_t len = 0;
    uint8_t  *pbuf = NULL;

    USBD_HOTKEY_HID_HandleTypeDef* hHotkeyHID = (USBD_HOTKEY_HID_HandleTypeDef*) pdev->pClassData;
    USBD_HOTKEY_HID_ItfTypeDef* hHotkeyHID_cb = (USBD_HOTKEY_HID_ItfTypeDef*) pdev->pUserData;

    UartDebug_sendline("Setup Request:\n");
    UartDebug_addToBuffer("bmRequestType: ", 15);
    UartDebug_hexprint32(req->bmRequest);
    UartDebug_newline();
    UartDebug_addToBuffer("bRequest: ", 10);
    UartDebug_hexprint32(req->bRequest);
    UartDebug_newline();
    UartDebug_addToBuffer("wIndex: ", 8);
    UartDebug_hexprint32(req->wIndex);
    UartDebug_newline();
    UartDebug_addToBuffer("wValue: ", 8);
    UartDebug_hexprint32(req->wValue);
    UartDebug_newline();
    UartDebug_addToBuffer("wLength: ", 9);
    UartDebug_hexprint32(req->wLength);
    UartDebug_newline();

    switch (req->bmRequest & USB_REQ_TYPE_MASK) {
        case USB_REQ_TYPE_CLASS :
            switch (req->bRequest) {
                case HID_REQ_SET_PROTOCOL:
                    hHotkeyHID->Protocol = (uint8_t)(req->wValue);
                    break;

                case HID_REQ_GET_PROTOCOL:
                    USBD_CtlSendData (pdev, (uint8_t *)&hHotkeyHID->Protocol, 1);
                    break;

                case HID_REQ_SET_IDLE:
                    hHotkeyHID->IdleState = (uint8_t)(req->wValue >> 8);
                    break;

                case HID_REQ_GET_IDLE:
                    USBD_CtlSendData (pdev, (uint8_t *)&hHotkeyHID->IdleState, 1);
                    break;

                case HID_REQ_SET_REPORT:
                    hHotkeyHID->IsReportAvailable = 1;
                    USBD_CtlPrepareRx (pdev, hHotkeyHID->Report_buf, (uint8_t)(req->wLength));
                    break;
                default:
                    USBD_CtlError (pdev, req);
                    return USBD_FAIL;
            }
            break;
        /* END CASE USB_REQ_TYPE_CLASS */
        case USB_REQ_TYPE_STANDARD: /* Interface & Endpoint request */
            switch (req->bRequest) {
                case USB_REQ_GET_DESCRIPTOR:
                    if( req->wValue >> 8 == HID_REPORT_DESC) {
                        len = MIN(USBD_HOTKEY_HID_REPORT_DESC_SIZE , req->wLength);
                        pbuf =  hHotkeyHID_cb->pReport;
                    } else if( req->wValue >> 8 == HID_DESCRIPTOR_TYPE)    {
                        pbuf = USBD_HOTKEY_HID_Desc;
                        len = MIN(USB_HOTKEY_HID_DESC_SIZ , req->wLength);
                    }
                    USBD_CtlSendData (pdev, pbuf, len);

                    break;

                case USB_REQ_GET_INTERFACE :
                    USBD_CtlSendData (pdev,    (uint8_t *)&hHotkeyHID->AltSetting, 1);
                    break;

                case USB_REQ_SET_INTERFACE :
                    hHotkeyHID->AltSetting = (uint8_t)(req->wValue);
                    break;

                case USB_REQ_CLEAR_FEATURE:
                    //determine what we are clearing
                    if((req->bmRequest & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_ENDPOINT) {
                        /* Flush the FIFO and Clear the stall status */
                        USBD_LL_FlushEP(pdev, (uint8_t)req->wIndex);

                        /* Reactivate the EP */
                        USBD_LL_CloseEP (pdev , (uint8_t)req->wIndex);
                        //wIndex has the EP number

                        UartDebug_sendline("Clear Feature on KB HID IN\n");
                        USBD_LL_OpenEP(pdev, HOTKEY_HID_EPIN_ADDR, USBD_EP_TYPE_INTR, HOTKEY_HID_EPIN_SIZE);
                        memset(hHotkeyHID->transmitReportBuffer, 0, HOTKEY_HID_EPIN_SIZE);
                        USBD_LL_Transmit (pdev, HOTKEY_HID_EPIN_ADDR, hHotkeyHID->transmitReportBuffer, HOTKEY_HID_EPIN_SIZE); //put a null report in the output

                    }
                    break;
            }
            break;
        default:
            break;
    }
    return USBD_OK;
}

uint8_t USBD_HOTKEY_HID_EP0_RxReady(USBD_HandleTypeDef *pdev) {
    USBD_HOTKEY_HID_HandleTypeDef* hHotkeyHID = (USBD_HOTKEY_HID_HandleTypeDef*) pdev->pClassData;

    if (hHotkeyHID->IsReportAvailable == 1) {
        ((USBD_HOTKEY_HID_ItfTypeDef*) pdev->pUserData)->OutEvent(hHotkeyHID->Report_buf);
        hHotkeyHID->IsReportAvailable = 0;
    }

    return USBD_OK;
}

uint8_t  USBD_HOTKEY_HID_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum) {
    /*UartDebug_addToBuffer("DataIn EP Num: ", 15);
    UartDebug_printuint8(epnum);
    UartDebug_newline();*/
    //why epnum comes through without its direction bit set i'm not sure
    //mask it off

    USBD_HOTKEY_HID_HandleTypeDef* hHotkeyHID = (USBD_HOTKEY_HID_HandleTypeDef*) pdev->pClassData;
    if(hHotkeyHID->transmitBufferLen) { //something left in the buffer
        if((hHotkeyHID->transmitReportBuffer[HOTKEY_IN_REPORT_MODIFIER_BYTE] == hHotkeyHID->transmitBuffer[0].modifiers) &&
                (hHotkeyHID->transmitReportBuffer[HOTKEY_IN_REPORT_SCANCODE_BYTE] == hHotkeyHID->transmitBuffer[0].scancode) ) {
            //if we've sent this modifier byte / scancode combo in the previous byte, send a null so the duplicate will be picked up next time
            memset(hHotkeyHID->transmitReportBuffer, 0, USBD_HOTKEY_HID_INREPORT_BUF_SIZE);
        } else { //not a duplicate
            hHotkeyHID->transmitReportBuffer[HOTKEY_IN_REPORT_MODIFIER_BYTE] = hHotkeyHID->transmitBuffer[0].modifiers;
            hHotkeyHID->transmitReportBuffer[HOTKEY_IN_REPORT_SCANCODE_BYTE] = hHotkeyHID->transmitBuffer[0].scancode;
            hHotkeyHID->transmitBuffer++; //increment pointer
            hHotkeyHID->transmitBufferLen--; //decrement count
        }
        USBD_LL_Transmit (pdev, HOTKEY_HID_EPIN_ADDR, hHotkeyHID->transmitReportBuffer, HOTKEY_HID_EPIN_SIZE);
    } else { //finished sending
        memset(hHotkeyHID->transmitReportBuffer, 0, USBD_HOTKEY_HID_INREPORT_BUF_SIZE);
        USBD_LL_Transmit (pdev, HOTKEY_HID_EPIN_ADDR, hHotkeyHID->transmitReportBuffer, HOTKEY_HID_EPIN_SIZE); //send out empty report
        hHotkeyHID->transmitState = HID_IDLE;
        hHotkeyHID->transmitBuffer = NULL;
    }

    return USBD_OK;
}


uint8_t  USBD_HOTKEY_HID_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum) {
    UartDebug_sendline("Hotkey HID DataOut called. This should not happen.\n");
    return USBD_OK;
}

uint8_t* USBD_HOTKEY_HID_GetDeviceQualifierDesc (uint16_t *length) {
    *length = sizeof (USBD_HOTKEY_HID_DeviceQualifierDesc);
    return USBD_HOTKEY_HID_DeviceQualifierDesc;
}

uint8_t* USBD_HOTKEY_HID_GetCfgDesc (uint16_t *length) {
  *length = sizeof (USBD_HOTKEY_HID_CfgDesc);
  return USBD_HOTKEY_HID_CfgDesc;
}
