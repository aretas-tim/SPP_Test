/*
 * led.h
 *
 *  Created on: Mar 23, 2016
 *      Author: me
 */

#ifndef LED_H_
#define LED_H_

#include "stm32l476xx.h"
#include "stm32l4xx_hal.h"
#include "uart_debug.h"
#include "status.h"
#include "pat-config.h"
#include <stdbool.h>

#define INIT_THIRD_CYCLE_TIME 20

/* these are for the devboard version*/
#ifdef DEVBOARD
#define PORT_LED_1_R GPIOB
#define PORT_LED_1_G GPIOB
#define PORT_LED_1_B GPIOB

#define PIN_LED_1_R GPIO_PIN_13
#define PIN_LED_1_G GPIO_PIN_14
#define PIN_LED_1_B GPIO_PIN_15

#define PORT_LED_2_R GPIOB
#define PORT_LED_2_G GPIOB
#define PORT_LED_2_B GPIOB

#define PIN_LED_2_R GPIO_PIN_1
#define PIN_LED_2_G GPIO_PIN_2
#define PIN_LED_2_B GPIO_PIN_11
#elif defined DEVICE_0_A_0_1
/* these are for the device ver 0.A.0.1
 *
 */
#define PORT_LED_2_R GPIOB
#define PORT_LED_2_G GPIOB
#define PORT_LED_2_B GPIOC

#define PIN_LED_2_R GPIO_PIN_12
#define PIN_LED_2_G GPIO_PIN_13
#define PIN_LED_2_B GPIO_PIN_6

#define PORT_LED_1_R GPIOC
#define PORT_LED_1_G GPIOC
#define PORT_LED_1_B GPIOC

#define PIN_LED_1_R GPIO_PIN_8
#define PIN_LED_1_G GPIO_PIN_9
#define PIN_LED_1_B GPIO_PIN_7
#elif defined DEVICE_0_A_1_1_S
/* there is nothing here for pin definitions, as its all done over I2C */
/* constants in to the buffer */
#define LED_BATTERY_1_B 0
#define LED_BATTERY_1_G 1
#define LED_BATTERY_1_R 2
#define LED_BATTERY_2_B 3
#define LED_BATTERY_2_G 4
#define LED_BATTERY_2_R 5
#define LED_BATTERY_3_B 6
#define LED_BATTERY_3_G 7
#define LED_BATTERY_3_R 8
#define LED_BATTERY_4_B 9
#define LED_BATTERY_4_G 10
#define LED_BATTERY_4_R 11
#define LED_HEARTBEAT_B 12
#define LED_HEARTBEAT_G 13
#define LED_HEARTBEAT_R 14
#define LED_USB_B 15
#define LED_USB_G 16
#define LED_USB_R 17
#define LED_BT_B 18
#define LED_BT_G 19
#define LED_BT_R 20
#define LED_TUNNEL_B 27
#define LED_TUNNEL_G 28
#define LED_TUNNEL_R 29
#define LED_LOCK_ICON_B 24
#define LED_LOCK_ICON_G 25
#define LED_LOCK_ICON_R 26
#define LED_UNLOCK_ICON_B 21
#define LED_UNLOCK_ICON_G 22
#define LED_UNLOCK_ICON_R 23
#define LED_ALT_KEY_B 30
#define LED_ALT_KEY_G 31
#define LED_ALT_KEY_R 32
#define LED_LOCK_KEY_B 33
#define LED_LOCK_KEY_G 34
#define LED_LOCK_KEY_R 35



#endif /*DEVICE_0_A_1_1_S*/

typedef enum tdLED_BatteryState {
    LED_BAT_FULL,
    LED_BAT_GOOD,
    LED_BAT_OKAY,
    LED_BAT_LOW,
    LED_BAT_WARN,
    LED_BAT_CRITICAL,
    LED_BAT_SHUTDOWN,
    LED_BAT_ERROR
} LED_BatteryState;

/*these can be fed directly in to the BSRR for the port, assuming all LEDs are on the same port */
/* these are also broken and not worth fixing due to having casts inside the GPIO_PIN_NN macros >_<
#define LED_1_K ( ( PIN_LED_1_R ) | ( PIN_LED_1_G) | ( PIN_LED_1_B ) )
#define LED_1_R ( ( PIN_LED_1_R << 16 ) | ( PIN_LED_1_G ) | ( PIN_LED_1_B ) )
#define LED_1_G ( ( PIN_LED_1_R ) | ( PIN_LED_1_G << 16 ) | ( PIN_LED_1_B ) )
#define LED_1_B ( ( PIN_LED_1_R ) | ( PIN_LED_1_G ) | ( PIN_LED_1_B << 16 ) )
#define LED_1_Y ( ( PIN_LED_1_R << 16 ) | ( PIN_LED_1_G << 16 ) | ( PIN_LED_1_B ) )
#define LED_1_M ( ( PIN_LED_1_R << 16 ) | ( PIN_LED_1_G ) | ( PIN_LED_1_B << 16 ) )
#define LED_1_C ( ( PIN_LED_1_R ) | ( PIN_LED_1_G << 16 ) | ( PIN_LED_1_B << 16 ) )
#define LED_1_W ( ( PIN_LED_1_R  << 16) | ( PIN_LED_1_G << 16 ) | ( PIN_LED_1_B << 16 ) )*/
#define LED_1_K 0x0000E000
#define LED_1_R 0x2000C000
#define LED_1_G 0x4000A000
#define LED_1_B 0x80006000
#define LED_1_Y 0x60008000
#define LED_1_M 0xA0004000
#define LED_1_C 0xC0002000
#define LED_1_W 0xE0000000

/* LED2 is working with individual colours, not mixing. no need for fanciness*/

/* end devboard port/pin assignments */

/* these are for the prototype 0.A.0.1 version*/
/*#define PORT_LED_1_R GPIOB
#define PORT_LED_1_G GPIOB
#define PORT_LED_1_B GPIOC

#define PIN_LED_1_R GPIO_PIN_12
#define PIN_LED_1_G GPIO_PIN_13
#define PIN_LED_1_B GPIO_PIN_6

#define PORT_LED_2_R GPIOC
#define PORT_LED_2_G GPIOC
#define PORT_LED_2_B GPIOC

#define PIN_LED_2_R GPIO_PIN_8
#define PIN_LED_2_G GPIO_PIN_9
#define PIN_LED_2_B GPIO_PIN_7*/
/* end 0.A.0.1 port/pin assignments */







/*typedef enum TDLedBehavior {
    OFF,
    ON,
    BLINK_SLOW,
    BLINK_FAST,
    BLINK,
    HEARTBEAT_SLOW,
    HEARTBEAT_FAST,
    HEARTBEAT,
    TURNOFF_AT,
    TURNON_AT
} LEDBehavior;*/

/*typedef struct TDLed {
    LEDBehavior behavior;
    uint32_t behaviorChangeAt;
    GPIO_TypeDef* port;
    uint16_t pin;
} led;*/
/* these are an aborted attempt at making it generic. realized this was going to take stupid amounts of work for the limited benefits*/


/* GPIOC in prototype*/

#define LED_TWINKLE_TIME 100
#define LED_ERROR_TIME 1500

#define LED_HEARTBEAT_LEN 120
static uint8_t LED_HEARTBEAT[] = {     0x30, 0x33, 0x35, 0x38, 0x3B, 0x3E,  /* 0..5 */
                                      0x40, 0x43, 0x46, 0x49, 0x4C, 0x4F,  /* 6..11 */
                                     0x52, 0x55, 0x58, 0x5B, 0x5E, 0x61,  /* 12..17 */
                                     0x64, 0x67, 0x6A, 0x6E, 0x71, 0x74,  /* 18..23 */
                                     0x77, 0x7B, 0x7E, 0x81, 0x85, 0x88,  /* 24..29 */
                                     0x8C, 0x8F, 0x93, 0x96, 0x9A, 0x9D,  /* 30..35 */
                                     0xA1, 0xA5, 0xA8, 0xAC, 0xB0, 0xB4,  /* 36..41 */
                                     0xB8, 0xBC, 0xBF, 0xC3, 0xC7, 0xCB,  /* 42..47 */
                                     0xCF, 0xD4, 0xD8, 0xDC, 0xE0, 0xE4,  /* 48..53 */
                                     0xE9, 0xED, 0xF1, 0xF6, 0xFA, 0xFF,  /* 54..59 */
                                     0xFF, 0xFA, 0xF6, 0xF1, 0xED, 0xE9,  /* 60..65 */
                                     0xE4, 0xE0, 0xDC, 0xD8, 0xD4, 0xCF,  /* 66..71 */
                                     0xCB, 0xC7, 0xC3, 0xBF, 0xBC, 0xB8,  /* 72..77 */
                                     0xB4, 0xB0, 0xAC, 0xA8, 0xA5, 0xA1,  /* 78..84 */
                                     0x9D, 0x9A, 0x96, 0x93, 0x8F, 0x8C,  /* 84..89 */
                                     0x88, 0x85, 0x81, 0x85, 0x7E, 0x7B,  /* 90..95 */
                                     0x74, 0x71, 0x6E, 0x6A, 0x67, 0x64,  /* 96..101 */
                                     0x61, 0x5E, 0x5B, 0x58, 0x55, 0x52,  /* 102..107 */
                                     0x4F, 0x4C, 0x49, 0x46, 0x43, 0x40,  /* 108..113 */
                                     0x3E, 0x3B, 0x38, 0x35, 0x33, 0x30   /* 114..119 */
                                 };

#define LED_BLINK_FAST_LEN 12
static uint8_t LED_BLINK_FAST[] = {     0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

#define LED_BLINK_NORMAL_LEN 30
static uint8_t LED_BLINK_NORMAL[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                    0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

typedef struct TDLED_AnimationInfo {
    bool isActive; /* if we're currently active */
    bool continueAnimation;  /*if we need to do another sequence, how this is set is up to the nature of the control of a given LED */
    uint8_t phaseCounter; /* what phase we're at in the sequence */
    uint8_t maxPhaseCounter; /* maximum the curent phaseCounter will reach before wrapping  */
    uint8_t numAnimationsRemaining; /* number of animations remaining, in the case its needed */
} LED_AnimationInfo;


#ifdef DEVICE_0_A_1_1_S

#define NUM_LEDS 36

#define LED_DRIVER_NUM_REGS 0x4B
#define LED_DRIVER_REG_SHUTDOWN 0x00
#define LED_DRIVER_REG_PWM_BASE 0x01
#define LED_DRIVER_REG_UPDATE 0x25
#define LED_DRIVER_REG_CTRL_BASE 0x26
#define LED_DRIVER_REG_CTRL_GLBL 0x4A
#define LED_DRIVER_REG_RESET 0x4F


/* shutdown reg defines */
#define LED_DRIVER_SHUTDOWN 0x00
#define LED_DRIVER_ENABLE 0x01
/* PWM regs are just duty cycle, as expected */
#define LED_DRIVER_UPDATE 0x00 /*update register write value */

/* LED control register defines */
#define LED_DRIVER_CURRENT_MASK 0x06 /* current mask */
#define LED_DRIVER_CURRENT_FULL 0x00 /* current values */
#define LED_DRIVER_CURRENT_HALF 0x02
#define LED_DRIVER_CURRENT_THIRD 0x04
#define LED_DRIVER_CURRENT_QUARTER 0x06
#define LED_DRIVER_LED_ON 0x01
#define LED_DRIVER_LED_OFF 0x00
/* global control register */
#define LED_DRIVER_GLBL_ON 0x00
#define LED_DRIVER_GLBL_OFF 0x01
/* reset register */
#define LED_DRIVER_RESET 0x00 /* write to reset register to reset led driver */
#endif

#ifdef DEVBOARD
#define LED_HEARTBEAT_R_REG TIM1->CCR1
#define LED_HEARTBEAT_G_REG TIM1->CCR2
#define LED_HEARTBEAT_B_REG TIM1->CCR3
#elif defined DEVICE_0_A_0_1
#define LED_HEARTBEAT_R_REG TIM3->CCR3
#define LED_HEARTBEAT_G_REG TIM3->CCR4
#define LED_HEARTBEAT_B_REG TIM3->CCR2
#endif

typedef struct TDLED_Colour {
    uint16_t r;
    uint16_t g;
    uint16_t b;
} LED_Colour;

/* these are 9 because we need 0 through 256, inclusive, so we need a 9th bit*/
#define LED_COLOUR_SHIFT_RED 18
#define LED_COLOUR_SHIFT_GREEN 9
#define LED_COLOUR_SHIFT_BLUE 0

/*okay these are kind of funky as we're packing 9 bits in, not 8*/
#define LED_COLOUR_LOCKED (0x100 << LED_COLOUR_SHIFT_RED)
#define LED_COLOUR_UNLOCKED (0x100 << LED_COLOUR_SHIFT_GREEN)
#define LED_COLOUR_PIN_VERIFY ((0x080 << LED_COLOUR_SHIFT_RED) + (0x080 << LED_COLOUR_SHIFT_GREEN))
#define LED_COLOUR_ERROR ((0x100 << LED_COLOUR_SHIFT_RED) + (0x080 << LED_COLOUR_SHIFT_BLUE))
#define LED_COLOUR_TRANSPORT_CODE_ENTRY ((0x0C0 << LED_COLOUR_SHIFT_GREEN) + (0x040 << LED_COLOUR_SHIFT_BLUE))
#define LED_COLOUR_TRANSPORT_ACTIVE ((0x040 << LED_COLOUR_SHIFT_GREEN) + (0x0C0 << LED_COLOUR_SHIFT_BLUE))
#define LED_COLOUR_WHITE ((0x100 << LED_COLOUR_SHIFT_RED) + (0x100 << LED_COLOUR_SHIFT_GREEN) + (0x100 << LED_COLOUR_SHIFT_BLUE))

#define LED_TIMER_PSC_ACTIVE 0x0014
#define LED_TIMER_PSC_WORKING 0x0005
#define LED_TIMER_PSC_SLEEPING 0x0080
#define LED_TIMER_PSC_OFF 0x0014 /*this is kinda meaningless, as the timer will be stopped*/

#define LED_COLOUR_MAX 256 /*inclusive*/
#define LED_COLOUR_SHIFT 8 /*bits to right-shift*/

void LED_Init(I2C_HandleTypeDef* handle, GPIO_TypeDef* shutdownPort, uint16_t shutdownPin);
void ledSetErrorOn(void);
void ledSetComms(uint8_t);
void ledSetDebug(uint8_t);
void ledCallback(void);
void LED_HeartbeatUpdate(Status_LockStatus s, Status_WorkStatus w);
void LED_SetHeartbeatColour(uint16_t r, uint16_t g, uint16_t b);
void LED_GetColours(LED_Colour* colour, Status_LockStatus status);
void LED_CommSet(void);
void LED_DebugSet(void);
void LED_DoWink(void); //for U2F

void LED_UpdateBatteryState(uint16_t batteryMillivolts, bool isCharging);
void LED_UpdateUSBStatus(Status_CommStatus usb);
void LED_UpdateUSBActivity(void);
void LED_SetKeypadAltIndication(bool isAltActive);
void LED_SetKeypadLocked(bool isKeypadLocked);
void LED_SetKeypadLockedKeypress(void);
void LED_SetTunnelEstablished(bool);
void LED_SetTunnelEstablishing(void); /* cancelled out when an SetTunnelEstablished is called */
void LED_SetBluetooth(void);
void LED_SetErase(void);

void LED_SetLEDs(bool isOn);

#define LED_VBAT_HYSTERESIS_MILLIVOLTS 20 /* millivolts before a change occurs to prevent visual oscillation*/
#define LED_VBAT_OVER_MILLIVOLTS 4300 /* overvoltage error */
#define LED_VBAT_FULL_MILLIVOLTS 4000 /* full green above this */
#define LED_VBAT_GOOD_MILLIVOLTS 3900 /* three green above this */
#define LED_VBAT_OKAY_MILLIVOLTS 3800 /* two yellow above this*/
#define LED_VBAT_LOW_MILLIVOLTS 3700 /* one red above this */
#define LED_VBAT_WARN_MILLIVOLTS 3600 /* one red slow blink above this */
#define LED_VBAT_CRITICAL_MILLIVOLTS 3200 /* one red fast blink above this */
#define LED_VBAT_SHUTDOWN_MILLIVOLTS 0 /* out of juice, go to sleep, wake when vbus detected */


#endif /* LED_H_ */
