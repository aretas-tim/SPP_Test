/*
 * usb_hid_scancodes.c
 *
 *  Created on: Jan 23, 2017
 *      Author: me
 */

#include "usb_hid.h"

#define MAX_ASCII_LUT_VALUE 127
KeyWithModifiers asciiToScanCodeLUT[] = {
		//0x00
		{0, 0}, //NUL
		{0, 0}, //SOH
		{0, 0}, //STX
		{0, 0}, //ETX
		{0, 0}, //EOT
		{0, 0}, //ENQ
		{0, 0}, //ACK
		{0, 0}, //BEL
		{0, 0x2A}, //Backspace
		{0, 0x2B}, //Tab
		{0, 0x28}, //New Line (Enter)
		{0, 0}, //Vertical Tab
		{0, 0}, //New Page
		{0, 0x28}, //Carriage Return (Enter)
		{0, 0}, //Shift In
		{0, 0}, //Shift Out
		//0x10
		{0, 0}, //Data Link Escape
		{0, 0}, //Device Control 1
		{0, 0}, //Device Control 2
		{0, 0}, //Device Control 3
		{0, 0}, //Device Control 4
		{0, 0}, //NAK
		{0, 0}, //SYN
		{0, 0}, //ETB
		{0, 0}, //CAN
		{0, 0}, //EM
		{0, 0}, //SUB
		{0, 0x29}, //Escape
		{0, 0}, //FS
		{0, 0}, //GS
		{0, 0}, //RS
		{0, 0}, //US
		//0x20
		{0, 0x2C}, //Space
		{MOD_SHIFT, 0x1E}, //!
		{MOD_SHIFT, 0x34}, //"
		{MOD_SHIFT, 0x20}, //#
		{MOD_SHIFT, 0x21}, //$
		{MOD_SHIFT, 0x22}, //%
		{MOD_SHIFT, 0x24}, //&
		{0, 0x34}, //'
		{MOD_SHIFT, 0x26}, //(
		{MOD_SHIFT, 0x27}, //)
		{MOD_SHIFT, 0x25}, //*
		{MOD_SHIFT, 0x2E}, //+
		{0, 0x36}, //,
		{0, 0x2D}, //-
		{0, 0x37}, //.
		{0, 0x38}, ///
		//0x30
		{0, 0x27}, //0
		{0, 0x1E}, //1
		{0, 0x1F}, //2
		{0, 0x20}, //3
		{0, 0x21}, //4
		{0, 0x22}, //5
		{0, 0x23}, //6
		{0, 0x24}, //7
		{0, 0x25}, //8
		{0, 0x26}, //9
		{MOD_SHIFT, 0x33}, //:
		{0, 0x33}, //;
		{MOD_SHIFT, 0x36}, //<
		{0, 0x2E}, //=
		{MOD_SHIFT, 0x37}, //>
		{MOD_SHIFT, 0x38}, //?
		//0x40
		{MOD_SHIFT, 0x1F}, //@
		{MOD_SHIFT, 0x04}, //A
		{MOD_SHIFT, 0x05}, //B
		{MOD_SHIFT, 0x06}, //C
		{MOD_SHIFT, 0x07}, //D
		{MOD_SHIFT, 0x08}, //E
		{MOD_SHIFT, 0x09}, //F
		{MOD_SHIFT, 0x0A}, //G
		{MOD_SHIFT, 0x0B}, //H
		{MOD_SHIFT, 0x0C}, //I
		{MOD_SHIFT, 0x0D}, //J
		{MOD_SHIFT, 0x0E}, //K
		{MOD_SHIFT, 0x0F}, //L
		{MOD_SHIFT, 0x10}, //M
		{MOD_SHIFT, 0x11}, //N
		{MOD_SHIFT, 0x12}, //O
		//0x50
		{MOD_SHIFT, 0x13}, //P
		{MOD_SHIFT, 0x14}, //Q
		{MOD_SHIFT, 0x15}, //R
		{MOD_SHIFT, 0x16}, //S
		{MOD_SHIFT, 0x17}, //T
		{MOD_SHIFT, 0x18}, //U
		{MOD_SHIFT, 0x19}, //V
		{MOD_SHIFT, 0x1A}, //W
		{MOD_SHIFT, 0x1B}, //X
		{MOD_SHIFT, 0x1C}, //Y
		{MOD_SHIFT, 0x1D}, //Z
		{0, 0x2F}, //[
		{0, 0x31}, // \ (backslash text here so it doesn't do a line-continuation)
		{0, 0x30}, //]
		{MOD_SHIFT, 0x23}, //^
		{MOD_SHIFT, 0x2D}, //_
		//0x60
		{0, 0x35}, //`
		{0, 0x04}, //a
		{0, 0x05}, //b
		{0, 0x06}, //c
		{0, 0x07}, //d
		{0, 0x08}, //e
		{0, 0x09}, //f
		{0, 0x0A}, //g
		{0, 0x0B}, //h
		{0, 0x0C}, //i
		{0, 0x0D}, //j
		{0, 0x0E}, //k
		{0, 0x0F}, //l
		{0, 0x10}, //m
		{0, 0x11}, //n
		{0, 0x12}, //o
		//0x70
		{0, 0x13}, //p
		{0, 0x14}, //q
		{0, 0x15}, //r
		{0, 0x16}, //s
		{0, 0x17}, //t
		{0, 0x18}, //u
		{0, 0x19}, //v
		{0, 0x1A}, //w
		{0, 0x1B}, //x
		{0, 0x1C}, //y
		{0, 0x1D}, //z
		{MOD_SHIFT, 0x2F}, //{
		{MOD_SHIFT, 0x31}, //|
		{MOD_SHIFT, 0x30}, //}
		{MOD_SHIFT, 0x35}, //~
		{MOD_SHIFT, 0x99} //DEL, currently using "alternative erase", which may be wrong. @TODO
		//end at 0x7F, HID KB does not support direct entry of extended ASCII set
};

void getKeyFromASCII(char in, KeyWithModifiers* out) {
	if((in < 0) || (in > MAX_ASCII_LUT_VALUE)) {
		out->modifiers = 0;
		out->scancode = 0;
	} else {
		out->modifiers = asciiToScanCodeLUT[in].modifiers;
		out->scancode = asciiToScanCodeLUT[in].scancode;
	}
}
