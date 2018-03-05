/*
 * u2f_impl.c
 *
 *  Created on: Feb 14, 2017
 *      Author: Andrew
 *
 *  U2F Implementation file. Where all the actual magic happens
 */





#include "u2f.h"
#include <stdint.h>
#include <stdbool.h>
#include "usbd_u2f_hid_if.h" /* USB HID interface */

uint8_t U2f_implResponseBuffer[sizeof(U2F_REGISTER_RESP)]; //biggest structure we can return


/**
 * U2F Register
 * takes a register request and register response pointer
 * verifies the request, then performs the registration and returns the response
 *
 *
 */
uint32_t U2f_register(U2F_REGISTER_REQ* request, uint16_t (*responseSendFunc) (uint16_t, uint8_t*)) {



    return 0;
}



/**
 * parses the APDU format in to usable data
 * param msgLen the length of msg
 * param msg the message in APDU format
 * param responseSendFunc the function to send the response via
 */
uint32_t U2f_parseMessage(uint16_t msgLen, uint8_t* msg, uint16_t (*responseSendFunc) (uint16_t, uint8_t*)) {
    if(msgLen < 4) {
        return -1; //@TODO something better. basically message was too short to parse
    }
    //CLA is reserved, but is at *(msg)
    uint8_t instruction = *(msg + 1);
    uint8_t param1 = *(msg + 2);
    uint8_t param2 = *(msg + 3);
    uint16_t requestLen = 0;
    uint16_t responseLen = 0;
    uint8_t* requestContentPtr = NULL; //point to the first byte of request content
    //this is kinda terrible but it can at least be followed
    bool isLongFormat = false;
    if((msgLen >= 7) && (*(msg + 4) == 0)) {
        isLongFormat = true;
    }
    if(isLongFormat) {
        if(*(msg + 4) != 0) {
            return -2; //parse error @TODO
        }
        if(msgLen == 7) {
            //no Lc, only a long-format Le
            responseLen = ((*(msg + 5)) << 8) + *(msg + 6);
            if(responseLen == 0) {
                responseLen = -1; //actually this should be 65536 but we can't handle that
            }
        } else {
            requestLen = ((*(msg + 5)) << 8) + *(msg + 6);
            uint16_t responsePos = 7 + requestLen;
            responseLen = ((*(msg + responsePos)) << 8) + (*(msg + responsePos + 1));
            if(responseLen == 0) {
                responseLen = -1; //actually this should be 65536 but we can't handle that
            }
            requestContentPtr = msg + 7;
        }
    } else {
        if(msgLen == 5) {
            //no Lc, only a short-format Le
            responseLen = *(msg + 4);
            if(responseLen == 0) {
                responseLen = 256; //special case when Le is 0
            }
        } else {
            requestLen = *(msg + 4);
            requestContentPtr = msg + 5;
            if(msgLen < (requestLen + 5)) {
                //there's still length remaining after the request content
                //by spec responseLen cannot be long format if the requestLen is short format
                responseLen = *(msg + 5 + requestLen);
                if(responseLen == 0) {
                    responseLen = 256; //special case when Le is 0
                }
            }
        }
    }
    switch (instruction) {
        case U2F_REGISTER:
            //verify parameters, which is pretty easy here
            if((requestContentPtr == NULL) || (requestLen != sizeof(U2F_REGISTER_REQ))) {
                U2F_IMPL_SendError(U2F_SW_CONDITIONS_NOT_SATISFIED, responseSendFunc);
            } else {

                U2F_REGISTER_REQ* regReqPtr = (U2F_REGISTER_REQ*) requestContentPtr;

            }
            break;
        case U2F_AUTHENTICATE:
            //verify parameters
            break;
        default:
            break;

    }
}
