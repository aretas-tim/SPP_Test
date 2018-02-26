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
#include "usbd_tunnel_hid_if.h"
#include "usbd_pat_comp.h"
#include "tunnel_shim.h"
#include "led.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

static int8_t TUNNEL_HID_Init     (void);
static int8_t TUNNEL_HID_DeInit   (void);
static int8_t TUNNEL_HID_OutEvent (uint8_t* report);
static int8_t TUNNEL_HID_InEventComplete(bool* wasPacketBufferSet);
//static void TUNNEL_HID_TransmitCompleteCallback(void);

static uint16_t TUNNEL_HID_Transmit(uint8_t* data, uint16_t len);
static void TUNNEL_HID_DisableInterrupt(void);
static void TUNNEL_HID_EnableInterrupt(void);

/* Private variables ---------------------------------------------------------*/

extern USBD_HandleTypeDef hUsbDeviceFS; //so we can send from this file

/**
 * a brief primer on how we got here
 */
uint8_t TUNNEL_HID_Buffer[TUNNEL_HID_BUFFER_LEN];

__ALIGN_BEGIN static uint8_t TUNNEL_HID_ReportDesc[USBD_TUNNEL_HID_REPORT_DESC_SIZE] __ALIGN_END =
{
  /* USER CODE BEGIN 0 */
  0x06, 0x00, 0xFF, /* usage page 0xFF00 3*/
  0x0A, 0x00, 0x03, /* usage 0x0300 6*/
  0xA1, 0x01, /* collection 1 8*/
  0x75, 0x08, /* 8 bit report size 10*/
  0x15, 0x00, /* minimum 0 12*/
  0x26, 0xFF, 0x00, /* maximum 255 15*/
  0x95, 0x40, /* report count 17*/ /* change this to change the amount of data sent */
  0x09, 0x01, /* usage 19*/
  0x81, 0x02, /* input (array) 21*/
  0x95, 0x40, /* report count 23*/ /* change this to change the amount of data sent */
  0x09, 0x02, /* usage 25*/
  0x91, 0x02, /* output (array) 27*/
  /* USER CODE END 0 */
  0xC0    /*     END_COLLECTION                 28*/

};

USBD_TUNNEL_HID_ItfTypeDef USBD_TUNNEL_HID_Callbacks =
{
        TUNNEL_HID_ReportDesc,
        TUNNEL_HID_Init,
        TUNNEL_HID_DeInit,
        TUNNEL_HID_OutEvent,
        TUNNEL_HID_InEventComplete,
        //TUNNEL_HID_TransmitCompleteCallback
};

TunnelShim_PacketFunctions TUNNEL_HID_ShimCallbacks = {
        TUNNEL_HID_Transmit,
        NULL,
        TUNNEL_HID_DisableInterrupt,
        TUNNEL_HID_EnableInterrupt
};

TunnelShim_Context TUNNEL_HID_ShimContext = {.state = TUNNEL_SHIM_UNCONFIGURED,
                                            .funcs = &TUNNEL_HID_ShimCallbacks
                                             //will init the rest later
};
void (*TUNNEL_HID_DisconnectedCallback)(void) = NULL;

/* Private functions ---------------------------------------------------------*/

static int8_t TUNNEL_HID_Init(void) {
    uint8_t initResp = TunnelShim_initContext(&TUNNEL_HID_ShimContext, TUNNEL_HID_Buffer, TUNNEL_HID_BUFFER_LEN, TUNNEL_HID_EPIN_SIZE, &TUNNEL_HID_ShimCallbacks);

    //UartDebug_sendString("Tunnel HID Enable Interrupt at: ");
    //UartDebug_hexprint32((uint32_t) TUNNEL_HID_EnableInterrupt); //yes it'll complain about the cast
    //UartDebug_newline();
    if(initResp == 0) {
        return USBD_OK;
    } else {
        return USBD_FAIL; //uh-oh
    }
}

static int8_t TUNNEL_HID_DeInit(void) {
    TunnelShim_deInit(&TUNNEL_HID_ShimContext);
    if(TUNNEL_HID_DisconnectedCallback != NULL) {
        TUNNEL_HID_DisconnectedCallback();
    }
    return USBD_OK;
}

static int8_t TUNNEL_HID_OutEvent(uint8_t* report) {
    //uint8_t resp =
    TunnelShim_recievePacket(&TUNNEL_HID_ShimContext, report, TUNNEL_HID_EPIN_SIZE);


    return USBD_OK;
}

static int8_t TUNNEL_HID_InEventComplete(bool* packetReady) {
    USBD_TUNNEL_HID_HandleTypeDef* hTunnelHID = &(((PAT_COMP_Data*) hUsbDeviceFS.pClassData)->tunnelHIDData);
    //TunnelShim_Packet* pkt = &(hTunnelHID->inPacketBuffer);
    *packetReady = TunnelShim_getNextPacket(&TUNNEL_HID_ShimContext, &(hTunnelHID->inPacketBuffer));
    /*if(*packetReady) {
        hTunnelHID->transmitState = HID_BUSY;
        USBD_LL_Transmit(&hUsbDeviceFS, TUNNEL_HID_EPIN_ADDR, hTunnelHID->inByteBuffer, TUNNEL_HID_EPIN_SIZE);
        LED_UpdateUSBActivity();
    }*/
    return USBD_OK;

}

static uint16_t TUNNEL_HID_Transmit(uint8_t* data, uint16_t len) {
    USBD_TUNNEL_HID_HandleTypeDef* hTunnelHID = &(((PAT_COMP_Data*) hUsbDeviceFS.pClassData)->tunnelHIDData);
    if((hTunnelHID->transmitState == HID_BUSY) || (data == NULL) || (len == 0)) {
#if defined DEBUG && (TUNNEL_SHIM_VERBOSE > 1)
        UartDebug_sendline("Tunnel HID Transmit called but cannot transmit.\n");
#endif
        return 0; //already transmitting or no point to transmit
    } else {
        if(len >= TUNNEL_HID_EPIN_SIZE) {
            len = TUNNEL_HID_EPIN_SIZE;
        } else {
            //copy to our internal buffer and pad
            memcpy(hTunnelHID->inByteBuffer, data, len);
            uint16_t remainingLen = TUNNEL_HID_EPIN_SIZE - len;
            memset(hTunnelHID->inByteBuffer + len, 0, remainingLen);
            data = hTunnelHID->inByteBuffer; //update this pointer to the internal buffer
        }
        //send away
        hTunnelHID->transmitState = HID_BUSY;
        USBD_LL_Transmit(&hUsbDeviceFS, TUNNEL_HID_EPIN_ADDR, data, TUNNEL_HID_EPIN_SIZE);
        LED_UpdateUSBActivity();
#if defined DEBUG && (TUNNEL_HID_VERBOSE > 1)
        UartDebug_sendline("Tunnel HID Transmit packet sent from IF Transmit:\n");
        UartDebug_hexdump(data, TUNNEL_HID_EPIN_SIZE);
#endif
        return len;
    }
}
static void TUNNEL_HID_DisableInterrupt(void) {
    HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
}
static void TUNNEL_HID_EnableInterrupt(void) {
    HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
}

TunnelShim_Context* TUNNEL_HID_GetShimContext(void) {
    return &TUNNEL_HID_ShimContext;
}

void TUNNEL_HID_SetDisconnectedCallback(void (*callback)(void)) {
    TUNNEL_HID_DisconnectedCallback = callback;
}
/************************ portions (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
