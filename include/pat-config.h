/*
 * pat-config.h
 *
 *  Created on: May 9, 2016
 *      Author: me
 */

#ifndef PAT_CONFIG_H_
#define PAT_CONFIG_H_

/* One of these MUST be defined or hilarious things will happen, probably including:
 * the clock not being inialized properly
 * peripherals not working
 * general hilarity
 */
#define DEVICE_0_A_1_1_U /* device revision 0.A.1.1, USB micro */
//#define DEVICE_0_A_1_1_S /* device revision, secure micro */
//#define DEVICE_0_A_0_1 /* device revision*/
//#define DEVBOARD /* devboard version*/



#define NUM_KEYS 10
#ifdef DEBUG
/*#define DEBUG_BYPASS_SECURITY*/
#endif /* DEBUG_BYPASS_SECURITY */


/* one of these must be defined for the USB to work (or likely even compile) */
#define USBD_COMPOSITE
/*#define USBD_STANDALONE_HOTKEY*/

#endif /* PAT_CONFIG_H_ */
