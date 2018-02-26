/*
 * usbd_pat_comp.c
 *
 *  Created on: Nov 15, 2016
 *      Author: me
 */

#include "usbd_pat_comp.h"
#include "uart_debug.h"
#include "u2f_hid.h"
#include "usbd_tunnel_hid_if.h" /*TUNNEL_HID_VERBOSE */

#include "led.h"
#include "status.h"

#ifdef USBD_COMPOSITE

#define USB_PAT_COMP_CONFIG_DESC_SIZ 0x79 /* @TODO change this when the descriptors change */

uint8_t  USBD_PAT_COMP_Init (USBD_HandleTypeDef *pdev, uint8_t cfgidx);

uint8_t  USBD_PAT_COMP_DeInit (USBD_HandleTypeDef *pdev, uint8_t cfgidx);

uint8_t  USBD_PAT_COMP_Setup (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);

uint8_t  USBD_PAT_COMP_EP0_RxReady (USBD_HandleTypeDef  *pdev);

uint8_t  USBD_PAT_COMP_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum);


uint8_t  USBD_PAT_COMP_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum);


uint8_t  *USBD_PAT_COMP_GetCfgDesc (uint16_t *length);

uint8_t  *USBD_PAT_COMP_GetDeviceQualifierDesc (uint16_t *length);


//uint8_t USBD_PAT_COMP_EMPTY_HID_BUFFER[TUNNEL_HID_EPIN_SIZE]; //2017-11-24 commented out as i was seeing no references to this.


USBD_ClassTypeDef  USBD_PAT_COMP =
{
  USBD_PAT_COMP_Init,
  USBD_PAT_COMP_DeInit,
  USBD_PAT_COMP_Setup,
  NULL, /*EP0_TxSent*/
  USBD_PAT_COMP_EP0_RxReady, /*EP0_RxReady*/
  USBD_PAT_COMP_DataIn,
  USBD_PAT_COMP_DataOut,
  NULL, /*SOF */
  NULL,
  NULL,
  NULL, /* not going to be used at high speed */
  USBD_PAT_COMP_GetCfgDesc,
  USBD_PAT_COMP_GetCfgDesc, /* not going to be used at "other" speed, whatever that is (presuming low but it should never enumerate as a low-speed device */
  USBD_PAT_COMP_GetDeviceQualifierDesc,
};


PAT_COMP_Callbacks callbacks;

uint8_t  USBD_PAT_COMP_RegisterCallbacks(USBD_HandleTypeDef *pdev, USBD_StorageTypeDef* storageCallbacks, USBD_TUNNEL_HID_ItfTypeDef* tunnelHIDCallbacks, USBD_U2F_HID_ItfTypeDef* U2FHIDCallbacks, USBD_HOTKEY_HID_ItfTypeDef* hotkeyHIDCallbacks) {
    callbacks.storageCallbacks = storageCallbacks;
    callbacks.tunnelHIDCallbacks = tunnelHIDCallbacks;
    callbacks.U2FHIDCallbacks = U2FHIDCallbacks;
    callbacks.hotkeyHIDCallbacks = hotkeyHIDCallbacks;

    pdev->pUserData = &callbacks;

    return 0;
}

/*  device Configuration Descriptor */
/*   All Descriptors (Configuration, Interface, Endpoint, Class, Vendor */
const uint8_t USBD_PAT_COMP_CfgDesc[USB_PAT_COMP_CONFIG_DESC_SIZ]  __ALIGN_END =
{

    0x09,   /* bLength: Configuation Descriptor size */
    USB_DESC_TYPE_CONFIGURATION,   /* bDescriptorType: Configuration */
    USB_PAT_COMP_CONFIG_DESC_SIZ,

    0x00,
    0x04,   /* bNumInterfaces: 4 interfaces */
    0x01,   /* bConfigurationValue: */
    0x04,   /* iConfiguration: */
    0xC0,   /* bmAttributes: */
    0x32,   /* MaxPower 100 mA */

    /********************  Mass Storage interface ********************/
    0x09,   /* bLength: Interface Descriptor size */
    0x04,   /* bDescriptorType: */
    INTERFACE_NUM_MSC,   /* bInterfaceNumber: Number of Interface */
    0x00,   /* bAlternateSetting: Alternate setting */
    0x02,   /* bNumEndpoints*/
    0x08,   /* bInterfaceClass: MSC Class */
    0x06,   /* bInterfaceSubClass : SCSI transparent*/
    0x50,   /* nInterfaceProtocol */
    0x05,          /* iInterface: */
    /********************  Mass Storage Endpoints ********************/
    0x07,   /*Endpoint descriptor length = 7*/
    0x05,   /*Endpoint descriptor type */
    MSC_EPIN_ADDR,   /*Endpoint address (IN, address 1) */
    0x02,   /*Bulk endpoint type */
    LOBYTE(MSC_MAX_FS_PACKET),
    HIBYTE(MSC_MAX_FS_PACKET),
    0x00,   /*Polling interval in milliseconds */

    0x07,   /*Endpoint descriptor length = 7 */
    0x05,   /*Endpoint descriptor type */
    MSC_EPOUT_ADDR,   /*Endpoint address (OUT, address 1) */
    0x02,   /*Bulk endpoint type */
    LOBYTE(MSC_MAX_FS_PACKET),
    HIBYTE(MSC_MAX_FS_PACKET),
    0x00,     /*Polling interval in milliseconds*/
    /* 32 cumulative */

    /************** Descriptor of Tunnel HID interface ****************/
    0x09,         /*bLength: Interface Descriptor size*/
    USB_DESC_TYPE_INTERFACE,/*bDescriptorType: Interface descriptor type*/
    INTERFACE_NUM_TUNNEL,         /*bInterfaceNumber: Number of Interface*/
    0x00,         /*bAlternateSetting: Alternate setting*/
    0x02,         /*bNumEndpoints*/
    0x03,         /*bInterfaceClass: HID*/
    0x00,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
    0x00,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
    0,            /*iInterface: Index of string descriptor*/
    /******************** Descriptor of Tunnel_HID *************************/
    /* 41 */
    0x09,         /*bLength: TUNNEL_HID Descriptor size*/
    HID_DESCRIPTOR_TYPE, /*bDescriptorType: HID*/
    0x11,         /*bcdHID TUNNEL_HID Class Spec release number*/
    0x01,            /* 1.11, little-endian*/
    0x00,         /*bCountryCode: Hardware target country*/
    0x01,         /*bNumDescriptors: Number of TUNNEL_HID class descriptors to follow*/
    0x22,         /*bDescriptorType*/
    USBD_TUNNEL_HID_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
    0x00,
    /******************** Descriptor of Tunnel HID endpoints ********************/
    /* 50 */
    0x07,          /*bLength: Endpoint Descriptor size*/
    USB_DESC_TYPE_ENDPOINT, /*bDescriptorType:*/

    TUNNEL_HID_EPIN_ADDR,     /*bEndpointAddress: Endpoint Address (IN)*/
    0x03,          /*bmAttributes: Interrupt endpoint*/
    TUNNEL_HID_EPIN_SIZE, /*wMaxPacketSize: 64 Byte max */
    0x00,
    0x01,          /*bInterval: Polling Interval (1 ms)*/
    /* 57 */

    0x07,             /* bLength: Endpoint Descriptor size */
    USB_DESC_TYPE_ENDPOINT,    /* bDescriptorType: */
    TUNNEL_HID_EPOUT_ADDR,  /*bEndpointAddress: Endpoint Address (OUT)*/
    0x03,    /* bmAttributes: Interrupt endpoint */
    TUNNEL_HID_EPOUT_SIZE,    /* wMaxPacketSize: 64 Bytes max  */
    0x00,
    0x01,    /* bInterval: Polling Interval (1 ms) */
    /* 64 */
    /************** Descriptor of U2F HID interface ****************/
    0x09,         /*bLength: Interface Descriptor size*/
    USB_DESC_TYPE_INTERFACE,/*bDescriptorType: Interface descriptor type*/
    INTERFACE_NUM_U2F,         /*bInterfaceNumber: Number of Interface*/
    0x00,         /*bAlternateSetting: Alternate setting*/
    0x02,         /*bNumEndpoints*/
    0x03,         /*bInterfaceClass: HID*/
    0x00,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
    0x00,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
    0,            /*iInterface: Index of string descriptor*/
    /******************** Descriptor of U2F HID *************************/
    /* 73 */
    0x09,         /*bLength: U2F HID Descriptor size*/
    HID_DESCRIPTOR_TYPE, /*bDescriptorType: HID*/
    0x11,         /*bcdHID U2F HID Class Spec release number*/
    0x01,            /* 1.11, little-endian*/
    0x00,         /*bCountryCode: Hardware target country*/
    0x01,         /*bNumDescriptors: Number of U2F HID class descriptors to follow*/
    0x22,         /*bDescriptorType*/
    USBD_U2F_HID_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
    0x00,
    /******************** Descriptor of U2F HID endpoints ********************/
    /* 82 */
    0x07,          /*bLength: Endpoint Descriptor size*/
    USB_DESC_TYPE_ENDPOINT, /*bDescriptorType:*/

    U2F_HID_EPIN_ADDR,     /*bEndpointAddress: Endpoint Address (IN)*/
    0x03,          /*bmAttributes: Interrupt endpoint*/
    U2F_HID_EPIN_SIZE, /*wMaxPacketSize: 64 Byte max */
    0x00,
    0x05,          /*bInterval: Polling Interval (5 ms)*/
    /* 89 */

    0x07,             /* bLength: Endpoint Descriptor size */
    USB_DESC_TYPE_ENDPOINT,    /* bDescriptorType: */
    U2F_HID_EPOUT_ADDR,  /*bEndpointAddress: Endpoint Address (OUT)*/
    0x03,    /* bmAttributes: Interrupt endpoint */
    U2F_HID_EPOUT_SIZE,    /* wMaxPacketSize: 64 Bytes max  */
    0x00,
    0x05,    /* bInterval: Polling Interval (5 ms) */
    /* 96 */
    /************** Descriptor of Hotkey HID interface ****************/
    0x09,         /*bLength: Interface Descriptor size*/
    USB_DESC_TYPE_INTERFACE,/*bDescriptorType: Interface descriptor type*/
    INTERFACE_NUM_HOTKEY,         /*bInterfaceNumber: Number of Interface*/
    0x00,         /*bAlternateSetting: Alternate setting*/
    0x01,         /*bNumEndpoints*/
    0x03,         /*bInterfaceClass: HID*/
    0x01,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
    0x01,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
    0,            /*iInterface: Index of string descriptor*/
    /******************** Descriptor of Hotkey HID *************************/
    /* 105 */
    0x09,         /*bLength: Hotkey HID Descriptor size*/
    HID_DESCRIPTOR_TYPE, /*bDescriptorType: HID*/
    0x11,         /*bcdHID Hotkey HID Class Spec release number*/
    0x01,            /* 1.11, little-endian*/
    0x21,         /*bCountryCode: Hardware target country*/
    0x01,         /*bNumDescriptors: Number of Hotkey HID class descriptors to follow*/
    0x22,         /*bDescriptorType*/
    USBD_HOTKEY_HID_REPORT_DESC_SIZE, /*wItemLength: Total length of Report descriptor*/
    0x00,
    /******************** Descriptor of Hotkey HID endpoints ********************/
    /* 114 */
    0x07,          /*bLength: Endpoint Descriptor size*/
    USB_DESC_TYPE_ENDPOINT, /*bDescriptorType:*/

    HOTKEY_HID_EPIN_ADDR,     /*bEndpointAddress: Endpoint Address (IN)*/
    0x03,          /*bmAttributes: Interrupt endpoint*/
    HOTKEY_HID_EPIN_SIZE, /*wMaxPacketSize: 8 Byte max */
    0x00,
    0x01          /*bInterval: Polling Interval (10 ms)*/
    /* 121 */

};

/* USB Standard Device Descriptor */
__ALIGN_BEGIN const uint8_t USBD_PAT_COMP_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END = {
    USB_LEN_DEV_QUALIFIER_DESC, /* bLength */
    USB_DESC_TYPE_DEVICE_QUALIFIER, /* bDescriptorType */
    0x00, /* bcdUSB, little-endian 2.00 */
    0x02,
    0x00, /* bDeviceClass */
    0x00, /* bDeviceSubclass */
    0x00, /* bDeviceProtocol */
    MSC_MAX_FS_PACKET, /*bMaxPacketSize0, i.e. maximum packet size for Endpoint 0 */
    0x01, /* bNumConfigurations*/
    0x00, /* reserved */
};



uint8_t USBD_PAT_COMP_Init (USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
    (void) cfgidx; // supress unused variable warning
    uint8_t ret = 0;
    USBD_TUNNEL_HID_HandleTypeDef* hTunnelHID;
    USBD_HOTKEY_HID_HandleTypeDef* hHotkeyHID;
    USBD_U2F_HID_HandleTypeDef* hU2FHID;

    /* Open MSC EP OUT */
    USBD_LL_OpenEP(pdev, MSC_EPOUT_ADDR, USBD_EP_TYPE_BULK, MSC_MAX_FS_PACKET);

    /* Open MSC EP IN */
    USBD_LL_OpenEP(pdev, MSC_EPIN_ADDR, USBD_EP_TYPE_BULK, MSC_MAX_FS_PACKET);

    /* Open Tunnel HID EP IN */
    USBD_LL_OpenEP(pdev, TUNNEL_HID_EPIN_ADDR, USBD_EP_TYPE_INTR, TUNNEL_HID_EPIN_SIZE);

    /* Open Tunnel HID EP OUT */
    USBD_LL_OpenEP(pdev, TUNNEL_HID_EPOUT_ADDR, USBD_EP_TYPE_INTR, TUNNEL_HID_EPOUT_SIZE);

    /* Open U2F HID EP IN */
    USBD_LL_OpenEP(pdev, U2F_HID_EPIN_ADDR, USBD_EP_TYPE_INTR, U2F_HID_EPIN_SIZE);

    /* Open U2F HID EP OUT */
    USBD_LL_OpenEP(pdev, U2F_HID_EPOUT_ADDR, USBD_EP_TYPE_INTR, U2F_HID_EPOUT_SIZE);

    /* Open Hotkey HID EP IN */
    USBD_LL_OpenEP(pdev, HOTKEY_HID_EPIN_ADDR, USBD_EP_TYPE_INTR, HOTKEY_HID_EPIN_SIZE);

    pdev->pClassData = USBD_malloc(sizeof (PAT_COMP_Data));

    if(pdev->pClassData == NULL) {
        ret = 1;
        //given this is statically allocated, shouldn't ever fail.
    } else {

        hTunnelHID = &(((PAT_COMP_Data*) pdev->pClassData)->tunnelHIDData);
        hHotkeyHID = &(((PAT_COMP_Data*) pdev->pClassData)->hotkeyHIDData);
        hU2FHID = &(((PAT_COMP_Data*) pdev->pClassData)->U2FHIDData);

        hTunnelHID->transmitState = HID_IDLE;
        ((PAT_COMP_Callbacks*) pdev->pUserData)->tunnelHIDCallbacks->Init();
              /* Prepare Out endpoint to receive 1st packet */
        USBD_LL_PrepareReceive(pdev, TUNNEL_HID_EPOUT_ADDR, hTunnelHID->Report_buf, USBD_TUNNEL_HID_OUTREPORT_BUF_SIZE);

        hU2FHID->transmitState = HID_IDLE;
        hU2FHID->transmitImmediateState = HID_IDLE;
        hU2FHID->IsReportAvailable = 0; /* legacy code for this, i guess */
        hU2FHID->nextSequenceNum = 0;
        hU2FHID->transmitBufferLen = 0;
        hU2FHID->transmitBuffer = NULL;
        ((PAT_COMP_Callbacks*) pdev->pUserData)->U2FHIDCallbacks->Init();
        USBD_LL_PrepareReceive(pdev, U2F_HID_EPOUT_ADDR, hU2FHID->Report_buf, USBD_U2F_HID_OUTREPORT_BUF_SIZE);

        hHotkeyHID->IdleState = HID_KB_DEFAULT_IDLE_RATE;
        hHotkeyHID->Protocol = HID_PROTOCOL_REPORT;
        hHotkeyHID->transmitState = HID_IDLE;
        ((PAT_COMP_Callbacks*) pdev->pUserData)->hotkeyHIDCallbacks->Init();
        //no out endpoint to set up

        MSC_BOT_Init(pdev);
    }

    LED_UpdateUSBStatus(COMM_CONNECTED);

    //memset(USBD_PAT_COMP_EMPTY_HID_BUFFER, 0, TUNNEL_HID_EPIN_SIZE); //2017-11-24 commented out as i was seeing no references to this
    return ret;
}

uint8_t  USBD_PAT_COMP_DeInit (USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
    (void) cfgidx; // supress unused variable warning
    /* Close TUNNEL HID EP IN */
    USBD_LL_CloseEP(pdev, TUNNEL_HID_EPIN_ADDR);

    /* Close TUNNEL HID EP OUT */
    USBD_LL_CloseEP(pdev, TUNNEL_HID_EPOUT_ADDR);

    /* Close MSC EP OUT */
    USBD_LL_CloseEP(pdev, MSC_EPOUT_ADDR);

    /* Close MSC EP IN */
    USBD_LL_CloseEP(pdev, MSC_EPIN_ADDR);

    /* Close U2F HID EP IN */
    USBD_LL_CloseEP(pdev, U2F_HID_EPIN_ADDR);

    /* Close U2F HID EP OUT */
    USBD_LL_CloseEP(pdev, U2F_HID_EPOUT_ADDR);

    /* Close Hotkey HID EP IN */
    USBD_LL_CloseEP(pdev, HOTKEY_HID_EPIN_ADDR);

    /* Free allocated memory */
    if(pdev->pClassData != NULL) {
        ((PAT_COMP_Callbacks *)pdev->pUserData)->tunnelHIDCallbacks->DeInit();

        MSC_BOT_DeInit(pdev);

        USBD_free(pdev->pClassData);
        pdev->pClassData = NULL;
    }
    LED_UpdateUSBStatus(COMM_DISCONNECTED);



    return USBD_OK;
}

uint8_t  USBD_PAT_COMP_Setup (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req) {
    uint16_t len = 0;
    uint8_t  *pbuf = NULL;

    USBD_HOTKEY_HID_HandleTypeDef* hHotkeyHID = &(((PAT_COMP_Data*) pdev->pClassData)->hotkeyHIDData);
    USBD_HOTKEY_HID_ItfTypeDef* hHotkeyHID_cb = ((PAT_COMP_Callbacks*)pdev->pUserData)->hotkeyHIDCallbacks;

    USBD_U2F_HID_HandleTypeDef* hU2FHID = &(((PAT_COMP_Data*) pdev->pClassData)->U2FHIDData);
    USBD_U2F_HID_ItfTypeDef* hU2FHID_cb = ((PAT_COMP_Callbacks*)pdev->pUserData)->U2FHIDCallbacks;

    USBD_TUNNEL_HID_HandleTypeDef* hTunnelHID = &(((PAT_COMP_Data*) pdev->pClassData)->tunnelHIDData);
    USBD_TUNNEL_HID_ItfTypeDef* hTunnelHID_cb = ((PAT_COMP_Callbacks*)pdev->pUserData)->tunnelHIDCallbacks;

    USBD_MSC_BOT_HandleTypeDef* hmsc = &(((PAT_COMP_Data*) pdev->pClassData)->storageData);
    USBD_StorageTypeDef* hmsc_cb = ((PAT_COMP_Callbacks*)pdev->pUserData)->storageCallbacks;

    /*UartDebug_sendline("Setup Request:\n");
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
    UartDebug_newline();*/

    switch (req->bmRequest & USB_REQ_TYPE_MASK) {
        case USB_REQ_TYPE_CLASS :
            switch (req->bRequest) {
                case HID_REQ_SET_PROTOCOL:
                    switch(req->wIndex) {
                        case INTERFACE_NUM_TUNNEL:
                            hTunnelHID->Protocol = (uint8_t)(req->wValue);
                            break;
                        case INTERFACE_NUM_U2F:
                            hU2FHID->Protocol = (uint8_t)(req->wValue);
                            break;
                        case INTERFACE_NUM_HOTKEY:
                            hHotkeyHID->Protocol = (uint8_t)(req->wValue);
                            break;
                        default:
                            USBD_CtlError (pdev, req);
                            return USBD_FAIL;
                    }
                    break;

                case HID_REQ_GET_PROTOCOL:
                    switch(req->wIndex) {
                        case INTERFACE_NUM_TUNNEL:
                            USBD_CtlSendData (pdev, (uint8_t *)&hTunnelHID->Protocol, 1);
                            break;
                        case INTERFACE_NUM_U2F:
                            USBD_CtlSendData (pdev, (uint8_t *)&hU2FHID->Protocol, 1);
                            break;
                        case INTERFACE_NUM_HOTKEY:
                            USBD_CtlSendData (pdev, (uint8_t *)&hHotkeyHID->Protocol, 1);
                            break;
                        default:
                            USBD_CtlError (pdev, req);
                            return USBD_FAIL;
                    }
                    break;

                case HID_REQ_SET_IDLE:
                    switch(req->wIndex) {
                        case INTERFACE_NUM_TUNNEL:
                            hTunnelHID->IdleState = (uint8_t)(req->wValue >> 8);
                            break;
                        case INTERFACE_NUM_U2F:
                            hU2FHID->IdleState = (uint8_t)(req->wValue >> 8);
                            break;
                        case INTERFACE_NUM_HOTKEY:
                            hHotkeyHID->IdleState = (uint8_t)(req->wValue >> 8);
                            break;
                        default:
                            USBD_CtlError (pdev, req);
                            return USBD_FAIL;
                    }
                    break;

                case HID_REQ_GET_IDLE:
                    switch(req->wIndex) {
                        case INTERFACE_NUM_TUNNEL:
                            USBD_CtlSendData (pdev, (uint8_t *)&hTunnelHID->IdleState, 1);
                            break;
                        case INTERFACE_NUM_U2F:
                            USBD_CtlSendData (pdev, (uint8_t *)&hU2FHID->IdleState, 1);
                            break;
                        case INTERFACE_NUM_HOTKEY:
                            USBD_CtlSendData (pdev, (uint8_t *)&hHotkeyHID->IdleState, 1);
                            break;
                        default:
                            USBD_CtlError (pdev, req);
                            return USBD_FAIL;
                    }
                    break;

                case HID_REQ_SET_REPORT:
                    switch(req->wIndex) {
                        case INTERFACE_NUM_TUNNEL:
                            hTunnelHID->IsReportAvailable = 1;
                            USBD_CtlPrepareRx (pdev, hTunnelHID->Report_buf, (uint8_t)(req->wLength));
                            break;
                        case INTERFACE_NUM_U2F:
                            hU2FHID->IsReportAvailable = 1;
                            USBD_CtlPrepareRx (pdev, hU2FHID->Report_buf, (uint8_t)(req->wLength));
                            break;
                        case INTERFACE_NUM_HOTKEY:
                            hHotkeyHID->IsReportAvailable = 1;
                            USBD_CtlPrepareRx (pdev, hHotkeyHID->Report_buf, (uint8_t)(req->wLength));
                            break;
                        default:
                            USBD_CtlError (pdev, req);
                            return USBD_FAIL;
                    }

                    break;

                case BOT_GET_MAX_LUN :
                    if((req->wValue  == 0) && (req->wLength == 1) && ((req->bmRequest & 0x80) == 0x80)) {
                        hmsc->max_lun = hmsc_cb->GetMaxLun();
                        USBD_CtlSendData (pdev, (uint8_t *)&hmsc->max_lun,  1);
                    } else {
                        USBD_CtlError(pdev , req);
                        return USBD_FAIL;
                    }
                    break;

                case BOT_RESET :
                    if((req->wValue  == 0) && (req->wLength == 0) && ((req->bmRequest & 0x80) != 0x80)) {
                        MSC_BOT_Reset(pdev);
                    } else {
                        USBD_CtlError(pdev , req);
                        return USBD_FAIL;
                    }
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
                    //interface is in wIndex
                    switch(req->wIndex) {
                        case INTERFACE_NUM_TUNNEL:
                            if( req->wValue >> 8 == HID_REPORT_DESC) {
                                len = MIN(USBD_TUNNEL_HID_REPORT_DESC_SIZE , req->wLength);
                                pbuf =  hTunnelHID_cb->pReport;
                            } else if( req->wValue >> 8 == HID_DESCRIPTOR_TYPE)    {
                                pbuf = USBD_TUNNEL_HID_Desc;
                                len = MIN(USB_TUNNEL_HID_DESC_SIZ , req->wLength);
                            }
                            break;
                        case INTERFACE_NUM_U2F:
                            if( req->wValue >> 8 == HID_REPORT_DESC) {
                                len = MIN(USBD_U2F_HID_REPORT_DESC_SIZE , req->wLength);
                                pbuf =  hU2FHID_cb->pReport;
                            } else if( req->wValue >> 8 == HID_DESCRIPTOR_TYPE)    {
                                pbuf = USBD_U2F_HID_Desc;
                                len = MIN(USB_U2F_HID_DESC_SIZ , req->wLength);
                            }
                            break;
                        case INTERFACE_NUM_HOTKEY:
                            if( req->wValue >> 8 == HID_REPORT_DESC) {
                                len = MIN(USBD_HOTKEY_HID_REPORT_DESC_SIZE , req->wLength);
                                pbuf =  hHotkeyHID_cb->pReport;
                            } else if( req->wValue >> 8 == HID_DESCRIPTOR_TYPE)    {
                                pbuf = USBD_HOTKEY_HID_Desc;
                                len = MIN(USB_HOTKEY_HID_DESC_SIZ , req->wLength);
                            }
                            break;
                        default:
                            break;

                    }
                    USBD_CtlSendData (pdev, pbuf, len);

                    break;

                case USB_REQ_GET_INTERFACE :
                    //@TODO possible endianness issue here
                    switch(req->wIndex) {
                        case INTERFACE_NUM_TUNNEL:
                            USBD_CtlSendData (pdev,    (uint8_t *)&hTunnelHID->AltSetting, 1);
                            break;
                        case INTERFACE_NUM_U2F:
                            USBD_CtlSendData (pdev,    (uint8_t *)&hU2FHID->AltSetting, 1);
                            break;
                        case INTERFACE_NUM_HOTKEY:
                            USBD_CtlSendData (pdev,    (uint8_t *)&hHotkeyHID->AltSetting, 1);
                            break;
                        default:
                            USBD_CtlError (pdev, req);
                            return USBD_FAIL;
                    }
                    break;

                case USB_REQ_SET_INTERFACE :
                    switch(req->wIndex) {
                        case INTERFACE_NUM_TUNNEL:
                            hTunnelHID->AltSetting = (uint8_t)(req->wValue);
                            break;
                        case INTERFACE_NUM_U2F:
                            hU2FHID->AltSetting = (uint8_t)(req->wValue);
                            break;
                        case INTERFACE_NUM_HOTKEY:
                            hHotkeyHID->AltSetting = (uint8_t)(req->wValue);
                            break;
                        default:
                            USBD_CtlError (pdev, req);
                            return USBD_FAIL;
                    }
                    break;

                case USB_REQ_CLEAR_FEATURE:
                    //determine what we are clearing
                    if((req->bmRequest & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_ENDPOINT) {
                        /* Flush the FIFO and Clear the stall status */
                        USBD_LL_FlushEP(pdev, (uint8_t)req->wIndex);

                        /* Reactivate the EP */
                        USBD_LL_CloseEP (pdev , (uint8_t)req->wIndex);
                        //wIndex has the EP number
                        switch(req->wIndex) {
                            case MSC_EPOUT_ADDR:
                                USBD_LL_OpenEP(pdev, MSC_EPOUT_ADDR, USBD_EP_TYPE_BULK, MSC_MAX_FS_PACKET);
                                MSC_BOT_CplClrFeature(pdev, (uint8_t)req->wIndex);
                                break;
                            case MSC_EPIN_ADDR:
                                USBD_LL_OpenEP(pdev, MSC_EPIN_ADDR, USBD_EP_TYPE_BULK, MSC_MAX_FS_PACKET);
                                MSC_BOT_CplClrFeature(pdev, (uint8_t)req->wIndex);
                                break;
                            case HOTKEY_HID_EPIN_ADDR:
                                UartDebug_sendline("Clear Feature on KB HID IN\n");
                                USBD_LL_OpenEP(pdev, HOTKEY_HID_EPIN_ADDR, USBD_EP_TYPE_INTR, HOTKEY_HID_EPIN_SIZE);
                                memset(hHotkeyHID->transmitReportBuffer, 0, HOTKEY_HID_EPIN_SIZE);
                                USBD_LL_Transmit (pdev, HOTKEY_HID_EPIN_ADDR, hHotkeyHID->transmitReportBuffer, HOTKEY_HID_EPIN_SIZE);
                                break;
                            default:
                                USBD_CtlError (pdev, req);
                                return USBD_FAIL;
                        }
                    }
                    break;
            }
            break;
        default:
            break;
    }
    return USBD_OK;
}


uint8_t USBD_PAT_COMP_EP0_RxReady(USBD_HandleTypeDef *pdev) {
    USBD_TUNNEL_HID_HandleTypeDef* hTunnelHID = &(((PAT_COMP_Data*) pdev->pClassData)->tunnelHIDData);
    USBD_U2F_HID_HandleTypeDef* hU2FHID = &(((PAT_COMP_Data*) pdev->pClassData)->U2FHIDData);
    USBD_HOTKEY_HID_HandleTypeDef* hHotkeyHID = &(((PAT_COMP_Data*) pdev->pClassData)->hotkeyHIDData);

    if (hTunnelHID->IsReportAvailable == 1) {
        ((PAT_COMP_Callbacks*) pdev->pUserData)->tunnelHIDCallbacks->OutEvent(hTunnelHID->Report_buf);
        hTunnelHID->IsReportAvailable = 0;
    }
    if (hU2FHID->IsReportAvailable == 1) {
        ((PAT_COMP_Callbacks*) pdev->pUserData)->U2FHIDCallbacks->OutEvent(hU2FHID->Report_buf);
        hU2FHID->IsReportAvailable = 0;
    }
    if (hHotkeyHID->IsReportAvailable == 1) {
        ((PAT_COMP_Callbacks*) pdev->pUserData)->hotkeyHIDCallbacks->OutEvent(hHotkeyHID->Report_buf);
        hHotkeyHID->IsReportAvailable = 0;
    }

    return USBD_OK;
}


uint8_t  USBD_PAT_COMP_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum) {
    /*UartDebug_addToBuffer("DataIn EP Num: ", 15);
    UartDebug_printuint8(epnum);
    UartDebug_newline();*/
    //why epnum comes through without its direction bit set i'm not sure
    //mask it off
    epnum = epnum & USB_EP_NUM_MASK; //mask off the direction bit

    if(epnum == (MSC_EPIN_ADDR & USB_EP_NUM_MASK)) {
        MSC_BOT_DataIn(pdev , epnum);
        //@TODO figure out how to add USB activity indication (LED)
    } else if(epnum == (TUNNEL_HID_EPIN_ADDR & USB_EP_NUM_MASK)) {
        //UartDebug_sendline("DataIn Event for Tunnel EP:\n");

        USBD_TUNNEL_HID_HandleTypeDef* hTunnelHID = &(((PAT_COMP_Data*) pdev->pClassData)->tunnelHIDData);
        bool sendPacket = false;
        ((PAT_COMP_Callbacks*) pdev->pUserData)->tunnelHIDCallbacks->InEvent(&sendPacket);
        if(sendPacket) {
            //UartDebug_sendline("Sending packet from data in callback.\n");
            hTunnelHID->transmitState = HID_BUSY;
            USBD_LL_Transmit(pdev, TUNNEL_HID_EPIN_ADDR, hTunnelHID->inByteBuffer, TUNNEL_HID_EPIN_SIZE);
            LED_UpdateUSBActivity();
#if defined DEBUG && (TUNNEL_HID_VERBOSE > 1)
            UartDebug_sendline("Tunnel USB HID Sent Data In Direct:\n");
            UartDebug_hexdump(hTunnelHID->inByteBuffer, TUNNEL_HID_EPIN_SIZE);
            UartDebug_newline();
#endif
        } else {
            hTunnelHID->transmitState = HID_IDLE;
        }
    } else if (epnum == (U2F_HID_EPIN_ADDR & USB_EP_NUM_MASK)) {
        //prep next packet and send to host if required
        USBD_U2F_HID_HandleTypeDef* hU2FHID = &(((PAT_COMP_Data*) pdev->pClassData)->U2FHIDData);
        bool sendNormalPacket = true;
        switch(hU2FHID->transmitImmediateState) {
            case HID_IDLE:
                //nothing to do
                break;
            case HID_BUSY:
                //busy, sent something out of here, transition this to idle and send a normal packet
                hU2FHID->transmitImmediateState = HID_IDLE;
                break;
            case HID_FULL:
                //something waiting to be sent
                sendNormalPacket = false;
                hU2FHID->transmitImmediateState = HID_BUSY; //sending
                USBD_LL_Transmit(pdev, U2F_HID_EPIN_ADDR, hU2FHID->transmitImmediateBuffer, U2F_HID_EPIN_SIZE);
                LED_UpdateUSBActivity();
                break;
            default:
                break;
        }
        switch(hU2FHID->transmitState) {
            case HID_IDLE:
                //nothing to do
                break;
            case HID_BUSY:
                if(sendNormalPacket) {
                    //UartDebug_sendline("In HID_BUSY send normal packet\n");
                    //UartDebug_printuint32(hU2FHID->transmitBufferLen = 0);
                    U2FHID_FRAME* frame = (U2FHID_FRAME*) hU2FHID->transmitReportBuffer;
                    frame->cont.seq = hU2FHID->nextSequenceNum & TYPE_CONT;
                    if(hU2FHID->transmitBufferLen > U2F_HID_CONT_FRAME_DATA_LEN) {
                        //*((uint32_t*) hU2FHID->transmitReportBuffer + U2F_HID_CONT_CID_POS) = U2F_HID_ActiveChannelID;
                        //channel ID should not need to be updated
                        memcpy(frame->cont.data, hU2FHID->transmitBuffer, U2F_HID_CONT_FRAME_DATA_LEN);
                        //update pointers and counts
                        hU2FHID->nextSequenceNum++; //sequence number increment
                        hU2FHID->transmitBufferLen -= U2F_HID_CONT_FRAME_DATA_LEN; //length decrement
                        hU2FHID->transmitBuffer += U2F_HID_CONT_FRAME_DATA_LEN; //pointer increment
                        USBD_LL_Transmit(pdev, U2F_HID_EPIN_ADDR, hU2FHID->transmitReportBuffer, U2F_HID_EPIN_SIZE); //send data
                        LED_UpdateUSBActivity();
                    } else if(hU2FHID->transmitBufferLen > 0) { //not a full packet
                        //*((uint32_t*) hU2FHID->transmitReportBuffer + U2F_HID_CONT_CID_POS) = U2F_HID_ActiveChannelID;
                        //channel ID should not need to be updated
                        memcpy(frame->cont.data, hU2FHID->transmitBuffer, hU2FHID->transmitBufferLen);
                        memset(frame->cont.data + hU2FHID->transmitBufferLen, 0, U2F_HID_CONT_FRAME_DATA_LEN - hU2FHID->transmitBufferLen);
                        //update pointers and counts
                        //ignore sequence number
                        hU2FHID->transmitBufferLen = 0; //zero this
                        hU2FHID->transmitBuffer = NULL; //null the pointer
                        USBD_LL_Transmit(pdev, U2F_HID_EPIN_ADDR, hU2FHID->transmitReportBuffer, U2F_HID_EPIN_SIZE); //send data
                        LED_UpdateUSBActivity();
                    } else { //must be zero
                        hU2FHID->transmitState = HID_IDLE; //finished sending
                        ((PAT_COMP_Callbacks*) pdev->pUserData)->U2FHIDCallbacks->TransmitComplete(); //run the tx complete callback
                    }
                } //else immediate is sending data, do nothing
                break;
            case HID_FULL:
                if(sendNormalPacket) {
                    //an init packet has been loaded in to the transmit report buffer for us, just send away!
                    hU2FHID->transmitState = HID_BUSY;
                    hU2FHID->nextSequenceNum = 0;
                    USBD_LL_Transmit(pdev, U2F_HID_EPIN_ADDR, hU2FHID->transmitReportBuffer, U2F_HID_EPIN_SIZE);
                    LED_UpdateUSBActivity();
                } //else immediate is sending data, do nothing
                break;
            default:
                break;
                //do nothing

        }

    } else if (epnum == (HOTKEY_HID_EPIN_ADDR & USB_EP_NUM_MASK)) {
        USBD_HOTKEY_HID_HandleTypeDef* hHotkeyHID = &(((PAT_COMP_Data*) pdev->pClassData)->hotkeyHIDData);
        if(hHotkeyHID->transmitBufferLen) { //something left in the buffer
            LED_UpdateUSBActivity();
            if(hHotkeyHID->isFirstChar) {
                hHotkeyHID->isFirstChar = false;
            } else if((hHotkeyHID->transmitReportBuffer[HOTKEY_IN_REPORT_MODIFIER_BYTE] == hHotkeyHID->transmitBuffer[0].modifiers) &&
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
    }
    return USBD_OK;
}

uint8_t  USBD_PAT_COMP_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum) {
    epnum = epnum & USB_EP_NUM_MASK; //mask off the direction bit

    if(epnum == (MSC_EPOUT_ADDR & USB_EP_NUM_MASK)) {
        MSC_BOT_DataOut(pdev , epnum);
        //@TODO figure out how to add USB activity indication (LED)
    } else if(epnum == (TUNNEL_HID_EPIN_ADDR & USB_EP_NUM_MASK)) {
        USBD_TUNNEL_HID_HandleTypeDef* hTunnelHID = &(((PAT_COMP_Data*) pdev->pClassData)->tunnelHIDData);
#if defined DEBUG && (TUNNEL_HID_VERBOSE > 1)
        UartDebug_sendline("Tunnel USB HID Received:\n");
        UartDebug_hexdump(hTunnelHID->Report_buf, USBD_TUNNEL_HID_OUTREPORT_BUF_SIZE);
        UartDebug_newline();
#endif

        ((PAT_COMP_Callbacks*) pdev->pUserData)->tunnelHIDCallbacks->OutEvent(hTunnelHID->Report_buf);
        LED_UpdateUSBActivity();

        USBD_LL_PrepareReceive(pdev, TUNNEL_HID_EPOUT_ADDR , hTunnelHID->Report_buf, USBD_TUNNEL_HID_OUTREPORT_BUF_SIZE);
    } else if(epnum == (U2F_HID_EPOUT_ADDR & USB_EP_NUM_MASK)) {
        USBD_U2F_HID_HandleTypeDef* hU2FHID = &(((PAT_COMP_Data*) pdev->pClassData)->U2FHIDData);
        /*UartDebug_addToBuffer("U2F USB HID Received:\n", 22);
        UartDebug_hexdump(hU2FHID->Report_buf, USBD_U2F_HID_OUTREPORT_BUF_SIZE);
        UartDebug_newline();*/

        ((PAT_COMP_Callbacks*) pdev->pUserData)->U2FHIDCallbacks->OutEvent(hU2FHID->Report_buf);
        LED_UpdateUSBActivity();

        USBD_LL_PrepareReceive(pdev, U2F_HID_EPOUT_ADDR , hU2FHID->Report_buf, USBD_U2F_HID_OUTREPORT_BUF_SIZE);
    } //hotkey has no OUT EP. out reports have to travel over the command endpoint (since its pretty much ignored)

    return USBD_OK;
}

uint8_t* USBD_PAT_COMP_GetDeviceQualifierDesc (uint16_t *length) {
    *length = sizeof (USBD_PAT_COMP_DeviceQualifierDesc);
    return USBD_PAT_COMP_DeviceQualifierDesc;
}

uint8_t* USBD_PAT_COMP_GetCfgDesc (uint16_t *length) {
  *length = sizeof (USBD_PAT_COMP_CfgDesc);
  return USBD_PAT_COMP_CfgDesc;
}
#endif /* USBD_COMPOSITE*/
