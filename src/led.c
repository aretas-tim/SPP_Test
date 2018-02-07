/*
 * led.c
 *
 *  Created on: Mar 23, 2016
 *      Author: me
 */

#include "led.h"
#include <stdbool.h>


#ifdef DEVICE_0_A_1_1_S
I2C_HandleTypeDef* LED_I2CHandle = NULL;
GPIO_TypeDef* LED_ShutdownPort = NULL;
uint16_t LED_ShutdownPin = 0;
#define LED_DRIVER_ADDRESS 0x78 /* seven-bit address, AD on the chip is connected to GND so 00 */
uint8_t LED_BufferRaw[LED_DRIVER_NUM_REGS + 1]; /* local buffer */
uint8_t* LED_Buffer = LED_BufferRaw + 1; //offset this to take in to account the register address on the driver
uint8_t LED_ShortBuffer[2]; //for resetting and stuff
#endif

bool LED_Initialized = false;

/*keep the error LED on until at least this tick, then turn it off*/
volatile uint32_t LEDErrorOnUntil = 0;

/* keep the comm and debug LEDs on until at least this tick, then check the respective on variables*/
volatile uint32_t LEDCommOnUntilAtleast = 0;
volatile uint32_t LEDDebugOnUntilAtleast = 0;
/* keep the comm and debug LEDs off until at least this tick, then turn them on and wait for the On Until to elapse*/
/* this ensures complete blink cycles*/
volatile uint32_t LEDCommOffUntil = 0;
volatile uint32_t LEDDebugOffUntil = 0;
/*are we blinking the comm or debug LEDs*/
volatile uint8_t LEDCommBlinkOn = 0;
volatile uint8_t LEDDebugBlinkOn = 0;

volatile uint32_t LEDCommTransitionTick = 0;
volatile uint32_t LEDDebugTransitionTick = 0;

volatile uint8_t LEDWink = 0;

uint32_t LED_HeartbeatPhase = 0;
uint8_t LED_HeartbeatUpcounting = 1;
LED_Colour LED_StatusColour = { .r = 256, .g = 0, .b = 0 };

volatile LED_BatteryState LED_BatState = LED_BAT_SHUTDOWN; //better to give a misleading empty than a misleading full
volatile bool LED_BatCharging = false;
volatile bool LED_IsOn = true;
volatile bool LED_KeypadLocked = false;
volatile bool LED_KeypadLockedKeypress = false; //if the keypad is locked and a key is being pressed.

volatile bool LED_TunnelEstablished = false;
volatile bool LED_TunnelEstablishing = false;

volatile bool LED_Bluetooth = false;


volatile CommStatus LED_USBStatus = COMM_DISCONNECTED;
volatile LED_AnimationInfo LED_USBInfo = { .isActive = false, .continueAnimation = false, .phaseCounter = 0, .maxPhaseCounter = LED_BLINK_FAST_LEN, .numAnimationsRemaining = 0};
volatile bool LED_KeypadAltActive = false;

volatile bool LED_DoEraseBlink = false; /* set by the erase function to override everything else */


uint16_t LED_WorkingPrescaleLookup[] = { LED_TIMER_PSC_OFF, LED_TIMER_PSC_SLEEPING, LED_TIMER_PSC_ACTIVE, LED_TIMER_PSC_WORKING};

#define LED_UPDATE_TIMER TIM7

void LED_Init(I2C_HandleTypeDef* handle, GPIO_TypeDef* shutdownPort, uint16_t shutdownPin) {
#ifdef DEVICE_0_A_1_1_S
    if(handle == NULL) {
        uart_debug_sendline("I2C Handle null while initializing LED Subsystem. LEDs disabled.\n");
        return;
    }
    if(shutdownPort == NULL) {
        uart_debug_sendline("Shutdown Port null while initializing LED Subsystem. LEDs disabled.\n");
        return;
    }
    LED_I2CHandle = handle;
    LED_ShutdownPort = shutdownPort;
    LED_ShutdownPin = shutdownPin;
    LED_ShutdownPort->BSRR |= LED_ShutdownPin; //enable the LED driver hardware

    *LED_BufferRaw = 0x0; //start at register 0
    LED_Buffer[LED_DRIVER_REG_SHUTDOWN] = LED_DRIVER_ENABLE;
    memset(LED_Buffer + LED_DRIVER_REG_PWM_BASE, 0, NUM_LEDS); /* no PWM to start */
    LED_Buffer[LED_DRIVER_REG_UPDATE] = LED_DRIVER_UPDATE;
    //memset(LED_Buffer + LED_DRIVER_REG_CTRL_BASE, LED_DRIVER_CURRENT_FULL | LED_DRIVER_LED_ON, NUM_LEDS); /* enable all LEDS */
    for(uint8_t i = 0; i < NUM_LEDS; i += 3) {
        LED_Buffer[LED_DRIVER_REG_CTRL_BASE + i] = LED_DRIVER_CURRENT_FULL | LED_DRIVER_LED_ON;
        LED_Buffer[LED_DRIVER_REG_CTRL_BASE + i + 1] = LED_DRIVER_CURRENT_QUARTER | LED_DRIVER_LED_ON;
        LED_Buffer[LED_DRIVER_REG_CTRL_BASE + i + 2] = LED_DRIVER_CURRENT_FULL | LED_DRIVER_LED_ON;
    }
    LED_Buffer[LED_DRIVER_REG_CTRL_GLBL] = LED_DRIVER_GLBL_ON; //global control on

    HAL_StatusTypeDef rc = HAL_I2C_Master_Transmit_IT(LED_I2CHandle, LED_DRIVER_ADDRESS, LED_BufferRaw, LED_DRIVER_NUM_REGS + 1);
    //HAL_StatusTypeDef rc = HAL_I2C_Mem_Write(LED_I2CHandle, LED_DRIVER_ADDRESS, LED_DRIVER_REG_SHUTDOWN, I2C_MEMADD_SIZE_8BIT, LED_BufferRaw, LED_DRIVER_NUM_REGS + 1, 1000);
    if(rc != HAL_OK) {
        uart_debug_sendline("Error Initializing LED Driver.\n");
    }

#endif /* DEVICE_0_A_1_1_S */
    LED_Initialized = true;
    /* no other devices need LED setup as they run the LEDs directly */
}

void ledSetErrorOn(void) { /*any further calls will simply extend the error time*/
    //PORT_LED_2_R->BRR = PIN_LED_2_R;
    LEDErrorOnUntil = HAL_GetTick() + LED_ERROR_TIME;
}

void LED_DebugSet(void) {
    LEDDebugBlinkOn = 1;
    //LEDDebugOnUntilAtleast = HAL_GetTick() + LED_TWINKLE_TIME;
    /* wait for callback to turn on LED */
}

void LED_CommSet(void) {
    LEDCommBlinkOn = 1;
    //LEDCommOnUntilAtleast = HAL_GetTick() + LED_TWINKLE_TIME;
    /* wait for callback to turn on LED */
}

/**
 * LED_DoWink
 *
 * blinks an LED to let the user know which device they're currently communicating with
 * optional part of the U2F spec that we're implementing as it seems like a good idea
 * @TODO tie this in to the callback system and figure out what it looks like.
 */
void LED_DoWink(void) {
    LEDWink = 1;
    /* wait for the callback to do anything */
}

/*void ledSetComms(uint8_t state) {
    if(state) {
        PORT_LED_2_G->BRR = PIN_LED_2_G;
        LEDCommBlinkOn = 1;
        LEDCommOnUntilAtleast = HAL_GetTick() + LED_TWINKLE_TIME;
    } else {
        LEDCommBlinkOn = 0;
        //do not turn off the LED here, it must complete its cycle
    }
}
void ledSetDebug(uint8_t state) {
    if(state) {
        PORT_LED_2_B->BRR = PIN_LED_2_B;
        LEDDebugBlinkOn = 1;
        LEDDebugOnUntilAtleast = HAL_GetTick() + LED_TWINKLE_TIME;
    } else {
        LEDDebugBlinkOn = 0;
        //do not turn off the LED here, it must complete its cycle
    }
}*/

/* call this from the systick callback to do magic
 */
void ledCallback(void) {
    //uint32_t tick = HAL_GetTick();

    /* NEWER CODE. COMMENTED OUT TO GET KEYPAD RUNNING FEB 23rd, 2017
    if(tick > LEDErrorOnUntil) {
        PORT_LED_2_R->BSRR = PIN_LED_2_R;
    }
    if(tick > LEDCommTransitionTick) {
        if(!(PORT_LED_2_G->ODR & PIN_LED_2_G)) { // if the LED is on
            PORT_LED_2_G->BSRR = PIN_LED_2_G; // turn it off, no matter what
            LEDCommTransitionTick += LED_TWINKLE_TIME;
            LEDCommBlinkOn = 0; //turn this off too
        } else { // LED must be off
            if(LEDCommBlinkOn) { // if this got set
                PORT_LED_2_G->BRR = PIN_LED_2_G; // turn on the LED
                LEDCommTransitionTick += LED_TWINKLE_TIME;
            }
        }
    }
    if(tick > LEDDebugTransitionTick) {
        if(!(PORT_LED_2_B->ODR & PIN_LED_2_B)) { // if the LED is on
            PORT_LED_2_B->BSRR = PIN_LED_2_B; // turn it off, no matter what
            LEDDebugTransitionTick += LED_TWINKLE_TIME;
            LEDDebugBlinkOn = 0; //turn this off too
        } else { // LED must be off
            if(LEDDebugBlinkOn) { // if this got set
                PORT_LED_2_B->BRR = PIN_LED_2_B; // turn on the LED
                LEDDebugTransitionTick += LED_TWINKLE_TIME;
            }
        }
    } */

    //OLD CODE
    /*if(tick > LEDCommOnUntilAtleast) {
        PORT_LED_2_G->BSRR = PIN_LED_2_G; // turn it off, no matter what
        if(LEDCommBlinkOn) { //set an off/on cycle
            LEDCommOffUntil = tick + LED_TWINKLE_TIME;
            LEDCommOnUntilAtleast = LEDCommOffUntil + LED_TWINKLE_TIME;
        }
    } else if (tick > LEDCommOffUntil) { //off time has elapsed, turn LED on
        PORT_LED_2_G->BRR = PIN_LED_2_G;
    }
    if(tick > LEDDebugOnUntilAtleast) {
        PORT_LED_2_B->BSRR = PIN_LED_2_B; // turn it off, no matter what
        if(LEDDebugBlinkOn) { //set an off/on cycle
            LEDDebugOffUntil = tick + LED_TWINKLE_TIME;
            LEDDebugOnUntilAtleast = LEDDebugOffUntil + LED_TWINKLE_TIME;
        }
    } else if (tick > LEDDebugOffUntil) {
        PORT_LED_2_B->BRR = PIN_LED_2_B;
    }*/
}

void LED_UpdateLockStatus(LockStatus l) {
    //LED_LockStatus = l;
}

void LED_UpdateTunnelStatus(TunnelStatus t) {
    //LED_TunnelStatus = t;
}

void LED_UpdateWorkStatus(WorkStatus w) {
    //LED_WorkStatus = w;
}

void LED_UpdateUSBStatus(CommStatus usb) {
    LED_USBStatus = usb;
}

void LED_UpdateUSBActivity() {
    LED_USBInfo.continueAnimation = true;
    LED_USBStatus = COMM_CONNECTED;
}

void LED_UpdateBTStatus(CommStatus bt) {
    //LED_BTStatus = bt;
}

/* helper to handle the colour cycle for the LEDs on startup
 *  */
void LED_DoInitCycle(uint8_t cyclePosition, uint8_t* ledBuffer) {
#ifdef DEVICE_0_A_1_1_S
    switch(cyclePosition) {
        case 0 ... 39:
            for(uint8_t i = 0; i < NUM_LEDS; i += 3) { //since there are 3 colours
                ledBuffer[LED_DRIVER_REG_PWM_BASE + i] = 0x00;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + i + 1] = 0x00;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + i + 2] = 0xFF;
            }
            break;
        case 40 ... 79:
            for(uint8_t i = 0; i < NUM_LEDS; i += 3) { //since there are 3 colours
                ledBuffer[LED_DRIVER_REG_PWM_BASE + i] = 0x00;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + i + 1] = 0xFF;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + i + 2] = 0x00;
            }
            break;
        case 80 ... 119:
            for(uint8_t i = 0; i < NUM_LEDS; i += 3) { //since there are 3 colours
                ledBuffer[LED_DRIVER_REG_PWM_BASE + i] = 0xFF;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + i + 1] = 0x00;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + i + 2] = 0x00;
            }
            break;
        default:
            //how did we get here. make em white to signal the error
            for(uint8_t i = 0; i < NUM_LEDS; i += 3) { //since there are 3 colours
                ledBuffer[LED_DRIVER_REG_PWM_BASE + i] = 0xFF;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + i + 1] = 0xFF;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + i + 2] = 0xFF;
            }
            break;
    }
#elif defined DEVICE_0_A_1_1_U
    switch(cyclePosition) {
        case 0 ... (INIT_THIRD_CYCLE_TIME - 1):
            TIM3->CCR1 = 0xFF;
            TIM3->CCR2 = 0;
            TIM3->CCR3 = 0;
            //uart_debug_putchar('R');
            break;
        case INIT_THIRD_CYCLE_TIME ... ((INIT_THIRD_CYCLE_TIME * 2) - 1):
            TIM3->CCR1 = 0;
            TIM3->CCR2 = 0xFF;
            TIM3->CCR3 = 0;
            //uart_debug_putchar('G');
            break;
        case (INIT_THIRD_CYCLE_TIME * 2) ... ((INIT_THIRD_CYCLE_TIME * 3) - 1):
            TIM3->CCR1 = 0;
            TIM3->CCR2 = 0;
            TIM3->CCR3 = 0xFF;
            //uart_debug_putchar('B');
            break;
        default:
            TIM3->CCR1 = 0xFF;
            TIM3->CCR2 = 0xFF;
            TIM3->CCR3 = 0xFF;
            //uart_debug_putchar('X');
            break;
    }
#endif /* DEVICE_0_A_1_1_U */
}

/* decluttering helper to handle the lock status LEDs (lock, unlock)
 *
 */
void LED_DoLockStatus(LockStatus s, uint8_t* ledBuffer) {
#ifdef DEVICE_0_A_1_1_S
    static uint8_t unlockingPhase = 0;
    if(LED_KeypadLocked) {
        if(LED_KeypadLockedKeypress) {
            ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_LOCK_ICON_B] = 0;
            ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_LOCK_ICON_G] = 0x00;
            ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_LOCK_ICON_R] = 0xFF;
            ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_UNLOCK_ICON_B] = 0;
            ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_UNLOCK_ICON_G] = 0;
            ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_UNLOCK_ICON_R] = 0;
            LED_KeypadLockedKeypress = false; //this gets set on every pass through the keypad scan routine
        } else {
            ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_LOCK_ICON_B] = 0;
            ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_LOCK_ICON_G] = 0x3F;
            ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_LOCK_ICON_R] = 0xFF;
            ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_UNLOCK_ICON_B] = 0;
            ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_UNLOCK_ICON_G] = 0;
            ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_UNLOCK_ICON_R] = 0;
        }
    } else {
        switch(s) {
            case LOCK_LOCKED:
            case LOCK_UNOWNED:
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_LOCK_ICON_B] = 0;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_LOCK_ICON_G] = 0x3F;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_LOCK_ICON_R] = 0xFF;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_UNLOCK_ICON_B] = 0;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_UNLOCK_ICON_G] = 0;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_UNLOCK_ICON_R] = 0;
                break;
            case LOCK_UNLOCKING:
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_LOCK_ICON_B] = 0;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_LOCK_ICON_G] = LED_HEARTBEAT[unlockingPhase] >> 2;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_LOCK_ICON_R] = LED_HEARTBEAT[unlockingPhase];
                if(LED_HEARTBEAT_LEN <= ++unlockingPhase) {
                    unlockingPhase = 0; //reset to zero
                }

                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_UNLOCK_ICON_B] = 0;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_UNLOCK_ICON_G] = 0;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_UNLOCK_ICON_R] = 0;
                break;
            case LOCK_UNLOCKED:
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_LOCK_ICON_B] = 0;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_LOCK_ICON_G] = 0;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_LOCK_ICON_R] = 0;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_UNLOCK_ICON_B] = 0;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_UNLOCK_ICON_G] = 0xFF;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_UNLOCK_ICON_R] = 0;
                break;
            case LOCK_PIN_VERIFY:
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_LOCK_ICON_B] = 0;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_LOCK_ICON_G] = 0x3F;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_LOCK_ICON_R] = 0xFF;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_UNLOCK_ICON_B] = 0;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_UNLOCK_ICON_G] = 0x3F;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_UNLOCK_ICON_R] = 0xFF;
                break;
            case LOCK_PIN_CHANGE_VERIFY_CURRENT:
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_LOCK_ICON_B] = 0;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_LOCK_ICON_G] = 0x00;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_LOCK_ICON_R] = 0xFF;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_UNLOCK_ICON_B] = 0;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_UNLOCK_ICON_G] = 0xFF;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_UNLOCK_ICON_R] = 0x0;
                break;
            case LOCK_PIN_CHANGE_ENTER_NEW:
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_LOCK_ICON_B] = 0;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_LOCK_ICON_G] = 0x3F;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_LOCK_ICON_R] = 0xFF;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_UNLOCK_ICON_B] = 0;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_UNLOCK_ICON_G] = 0xFF;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_UNLOCK_ICON_R] = 0x00;
                break;
            case LOCK_PIN_CHANGE_VERIFY_NEW:
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_LOCK_ICON_B] = 0;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_LOCK_ICON_G] = 0x3F;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_LOCK_ICON_R] = 0xFF;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_UNLOCK_ICON_B] = 0;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_UNLOCK_ICON_G] = 0x3F;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_UNLOCK_ICON_R] = 0xFF;
                break;

            case LOCK_ERROR:
            default:
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_LOCK_ICON_B] = 0;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_LOCK_ICON_G] = 0;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_LOCK_ICON_R] = 0xFF;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_UNLOCK_ICON_B] = 0;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_UNLOCK_ICON_G] = 0;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_UNLOCK_ICON_R] = 0xFF;
                break;
        }
    }
#endif /* DEVICE_0_A_1_1_S */
}

void LED_DoUSBStatus(uint8_t* ledBuffer) {
#ifdef DEVICE_0_A_1_1_S
    ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_USB_G] = 0;
    ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_USB_R] = 0;
    switch(LED_USBStatus) {
        case COMM_DISCONNECTED:
            ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_USB_B] = 0;
            break;
        case COMM_CONNECTING:
        case COMM_CONNECTED:
            if(LED_USBInfo.isActive) {
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_USB_B] = LED_BLINK_FAST[LED_USBInfo.phaseCounter++];
                if(LED_USBInfo.phaseCounter == LED_USBInfo.maxPhaseCounter) {
                    LED_USBInfo.phaseCounter = 0;
                    if(!LED_USBInfo.continueAnimation) {
                        LED_USBInfo.isActive = false;
                        LED_USBInfo.maxPhaseCounter = 0;
                    }
                }
            } else {
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_USB_B] = 0xFF;
            }
            break;
        case COMM_ERROR:
        default:
            ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_USB_B] = 0;
            ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_USB_R] = 0xFF;
            break;
    }
#endif /*DEVICE_0_A_1_1_S */
}

void LED_DoHeartbeat(WorkStatus w, uint8_t* ledBuffer) {
#ifdef DEVICE_0_A_1_1_S
    static uint8_t workPhase = 0;
    static uint8_t sleepCount = 0;
    if(w != WORK_ERROR) {
        ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_HEARTBEAT_B] = 0;
        ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_HEARTBEAT_G] = LED_HEARTBEAT[workPhase];
        ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_HEARTBEAT_R] = 0;
    } else {
        ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_HEARTBEAT_B] = 0;
        ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_HEARTBEAT_G] = 0;
        ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_HEARTBEAT_R] = LED_HEARTBEAT[workPhase];
    }
    switch(w) {
        case WORK_OFF:
            //how the heck did we get here?
            //@TODO investigate removing this case
            break;
        case WORK_SLEEP:
            if(++sleepCount == 4) {
                workPhase++;
                sleepCount = 0;
            }
            break;
        case WORK_AWAKE:
            workPhase++;
            break;
        case WORK_WORKING:
        case WORK_ERROR:
            workPhase += 3;
            break;
        default:
            //how did we get here?
            workPhase++;
            break;
    }
    if(LED_HEARTBEAT_LEN <= workPhase) {
        workPhase = 0; //reset to zero
    }
#endif /* DEVICE_0_A_1_1_S */
}

void LED_DoBatteryStatus(uint8_t* ledBuffer) {
#ifdef DEVICE_0_A_1_1_S
    ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_2_R] = 0x00;
    ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_3_R] = 0x00;
    ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_4_R] = 0x00;
    if(LED_KeypadAltActive) {
        ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_1_R] = 0x00;
        ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_1_G] = 0x00;
        ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_1_B] = 0xFF;
        ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_2_G] = 0x00;
        ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_2_B] = 0xFF;
        ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_3_G] = 0x00;
        ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_3_B] = 0xFF;
        ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_4_G] = 0x00;
        ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_4_B] = 0xFF;
    /*} else if (GPIOA->IDR & GPIO_PIN_9) {*/
        //pin 9 is high, battery is (for now) charging
        //@TODO fix this up once we have an actual way to tell if the battery is charging (i.e. 2nd micro is in)
    } else {
        //battery discharging
        static uint8_t batteryPhase = 0; //statics to control the blinking when necessary
        static bool batteryPhaseUpcounting = true;
        ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_1_B] = 0x00;
        ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_2_B] = 0x00;
        ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_3_B] = 0x00;
        ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_4_B] = 0x00;
        switch(LED_BatState) {
            case LED_BAT_FULL:
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_1_R] = 0x00;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_1_G] = 0xFF;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_2_G] = 0xFF;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_3_G] = 0xFF;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_4_G] = 0xFF;
                break;
            case LED_BAT_GOOD:
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_1_R] = 0x00;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_1_G] = 0xFF;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_2_G] = 0xFF;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_3_G] = 0xFF;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_4_G] = 0x00;
                break;
            case LED_BAT_OKAY:
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_1_R] = 0x00;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_1_G] = 0xFF;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_2_G] = 0xFF;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_3_G] = 0x00;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_4_G] = 0x00;
                break;
            case LED_BAT_LOW:
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_1_R] = 0xFF;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_1_G] = 0x7F;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_2_G] = 0x00;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_3_G] = 0x00;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_4_G] = 0x00;
                break;
            case LED_BAT_WARN:
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_1_R] = 0xFF;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_1_G] = 0x00;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_2_G] = 0x00;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_3_G] = 0x00;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_4_G] = 0x00;
                break;
            case LED_BAT_CRITICAL:
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_1_R] = LED_HEARTBEAT[batteryPhase];
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_1_G] = 0x00;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_2_G] = 0x00;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_3_G] = 0x00;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_4_G] = 0x00;
                if(batteryPhaseUpcounting) {
                    if(!(LED_HEARTBEAT_LEN > ++batteryPhase)) {
                        batteryPhaseUpcounting = false; //at top, downcount
                        --batteryPhase; //decrement to avoid overflow on next count
                    }
                } else {
                    if(0 == --batteryPhase) {
                        batteryPhaseUpcounting = 1; //at bottom, upcount
                    }
                }
                break;
            case LED_BAT_SHUTDOWN:
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_1_R] = 0x00;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_1_G] = 0x00;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_2_G] = 0x00;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_3_G] = 0x00;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_4_G] = 0x00;
                break;
            case LED_BAT_ERROR:
            default:
                //special case, less optimized. should never fire
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_1_R] = 0xFF;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_1_G] = 0x00;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_2_R] = 0xFF;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_2_G] = 0x00;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_3_R] = 0xFF;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_3_G] = 0x00;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_4_R] = 0xFF;
                ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BATTERY_4_G] = 0x00;
                break;

        }
    }
#endif /*DEVICE_0_A_1_1_S */
}

void LED_DoTunnelStatus(uint8_t* ledBuffer) {
#ifdef DEVICE_0_A_1_1_S
    static uint8_t tunnelPhase = 0;
    ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_TUNNEL_G] = 0;
    ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_TUNNEL_R] = 0;
    if(LED_TunnelEstablishing) {
        ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_TUNNEL_B] = LED_HEARTBEAT[tunnelPhase];
        if(LED_HEARTBEAT_LEN <= ++tunnelPhase) {
            tunnelPhase = 0; //reset to zero
        }
    } else if(LED_TunnelEstablished) {
        ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_TUNNEL_B] = 0xFF;
    } else {
        ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_TUNNEL_B] = 0;
    }
#endif /*DEVICE_0_A_1_1_S */
}

void LED_DoBluetoothStatus(uint8_t* ledBuffer) {
#ifdef DEVICE_0_A_1_1_S
    ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BT_G] = 0;
    ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BT_R] = 0;
    if(LED_Bluetooth) {
        ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BT_B] = 0xFF;
    } else {
        ledBuffer[LED_DRIVER_REG_PWM_BASE + LED_BT_B] = 0;
    }
#endif /*DEVICE_0_A_1_1_S */
}

void LED_RunEraseBlink(uint8_t* ledBuffer) {

    static uint8_t erasePhase = 0;
#ifdef DEVICE_0_A_1_1_S
    ledBuffer[LED_BATTERY_1_B] = 0;
    ledBuffer[LED_BATTERY_1_G] = LED_HEARTBEAT[erasePhase];
    ledBuffer[LED_BATTERY_1_R] = 0;
    ledBuffer[LED_BATTERY_2_B] = 0;
    ledBuffer[LED_BATTERY_2_G] = LED_HEARTBEAT[erasePhase];
    ledBuffer[LED_BATTERY_2_R] = 0;
    ledBuffer[LED_BATTERY_3_B] = 0;
    ledBuffer[LED_BATTERY_3_G] = LED_HEARTBEAT[erasePhase];
    ledBuffer[LED_BATTERY_3_R] = 0;
    ledBuffer[LED_BATTERY_4_B] = 0;
    ledBuffer[LED_BATTERY_4_G] = LED_HEARTBEAT[erasePhase];
    ledBuffer[LED_BATTERY_4_R] = 0;
    ledBuffer[LED_HEARTBEAT_B] = 0;
    ledBuffer[LED_HEARTBEAT_G] = 0;
    ledBuffer[LED_HEARTBEAT_R] = LED_HEARTBEAT[erasePhase];
    ledBuffer[LED_USB_B] = LED_HEARTBEAT[erasePhase];
    ledBuffer[LED_USB_G] = 0;
    ledBuffer[LED_USB_R] = 0;
    ledBuffer[LED_BT_B] = LED_HEARTBEAT[erasePhase];
    ledBuffer[LED_BT_G] = 0;
    ledBuffer[LED_BT_R] = 0;
    ledBuffer[LED_TUNNEL_B] = LED_HEARTBEAT[erasePhase];
    ledBuffer[LED_TUNNEL_G] = 0;
    ledBuffer[LED_TUNNEL_R] = 0;
    ledBuffer[LED_LOCK_ICON_B] = 0;
    ledBuffer[LED_LOCK_ICON_G] = LED_HEARTBEAT[erasePhase] >> 2;
    ledBuffer[LED_LOCK_ICON_R] = LED_HEARTBEAT[erasePhase];
    ledBuffer[LED_UNLOCK_ICON_B] = 0;
    ledBuffer[LED_UNLOCK_ICON_G] = LED_HEARTBEAT[erasePhase] >> 2;
    ledBuffer[LED_UNLOCK_ICON_R] = LED_HEARTBEAT[erasePhase];
    ledBuffer[LED_ALT_KEY_B] = 0;
    ledBuffer[LED_ALT_KEY_G] = 0;
    ledBuffer[LED_ALT_KEY_R] = 0;
    ledBuffer[LED_LOCK_KEY_B] = 0;
    ledBuffer[LED_LOCK_KEY_G] = 0;
    ledBuffer[LED_LOCK_KEY_R] = 0;
#endif /*DEVICE_0_A_1_1_S */

    if(LED_HEARTBEAT_LEN <= ++erasePhase) {
        erasePhase = 0; //reset to zero
    }


}

void LED_HeartbeatUpdate(LockStatus s, WorkStatus w) {
    if(!LED_Initialized || !LED_IsOn) {
        return; //prevents the timer callback from running this before we're ready.
    }
    static uint8_t initDelay = 0;
#ifdef DEVICE_0_A_1_1_S

    if(LED_I2CHandle == NULL) {
        return; //just don't do anything if its null
    }
    if(LED_DoEraseBlink) { //has priority over everything
        LED_RunEraseBlink(LED_Buffer);
    } else if(initDelay < 120) { /* this runs our bootup routine that cycled the LED colours before settling down. only happens after a reset */
        LED_DoInitCycle(initDelay, LED_Buffer);
        initDelay++; //increment this
        /*if(initDelay >= 120) {
            initDelay = 0; //for testing. @TODO remove this
        }*/
    } else {
        LED_Buffer[LED_ALT_KEY_B] = 0;
        LED_Buffer[LED_ALT_KEY_G] = 0;
        LED_Buffer[LED_ALT_KEY_R] = 0;
        LED_Buffer[LED_LOCK_KEY_B] = 0;
        LED_Buffer[LED_LOCK_KEY_G] = 0;
        LED_Buffer[LED_LOCK_KEY_R] = 0;
        //memset(LED_Buffer + LED_DRIVER_REG_PWM_BASE, 0, NUM_LEDS); //clear everything, this is a waste of time as everything should get updated
        //lock/unlock LEDS
        LED_DoLockStatus(s, LED_Buffer);

        //@TODO add a flag to prevent re-doing this?
        LED_DoBatteryStatus(LED_Buffer);
        //USB
        LED_DoUSBStatus(LED_Buffer);
        //heartbeat / work
        LED_DoHeartbeat(w, LED_Buffer);

        LED_DoTunnelStatus(LED_Buffer);
        LED_DoBluetoothStatus(LED_Buffer);


    }

#ifdef DEBUG_LEDS
    static uint8_t dumpCtr = 0;
    if((++dumpCtr) == 60) {
        dumpCtr = 0;
        uart_debug_sendline("LED Data Dump:\n");
        uart_debug_hexdump(LED_Buffer + LED_DRIVER_REG_PWM_BASE, NUM_LEDS);
    }


#endif /*DEBUG */
    LED_I2CHandle->ErrorCode = HAL_I2C_ERROR_NONE;
    LED_Buffer[LED_DRIVER_REG_PWM_BASE - 1] = LED_DRIVER_REG_PWM_BASE; //start register address
    LED_Buffer[LED_DRIVER_REG_UPDATE] = LED_DRIVER_UPDATE; //make sure to actually trigger the update
    /* send +1 bytes as we need to get the update reg or nothing will happen */
    HAL_StatusTypeDef rc = HAL_I2C_Master_Transmit_IT(LED_I2CHandle, LED_DRIVER_ADDRESS, LED_Buffer + LED_DRIVER_REG_PWM_BASE - 1, NUM_LEDS + 2);
    //HAL_StatusTypeDef rc = HAL_I2C_Mem_Write(LED_I2CHandle, LED_DRIVER_ADDRESS, LED_DRIVER_REG_PWM_BASE, I2C_MEMADD_SIZE_8BIT, LED_Buffer, NUM_LEDS + 1, 1000);
    if(rc) {
        //do something?
    }

#elif defined DEVICE_0_A_1_1_U
    if(initDelay < (INIT_THIRD_CYCLE_TIME * 3)) { /* this runs our bootup routine that cycled the LED colours before settling down. only happens after a reset */
        LED_DoInitCycle(initDelay, NULL);
        initDelay++;
    } else {
        //all we have here is the USB Status to need to deal with
        //TIM8->CCR1 -> RED //or should be anyway
        //TIM8->CCR2 -> GREEN
        //TIM8->CCR3 -> BLUE
        //remember its inverted to active low so it works "as expected" (higher numbers mean more brightness)
        TIM3->CCR3 = 0;
        switch(LED_USBStatus) {
            case COMM_DISCONNECTED:
                TIM3->CCR1 = 0;
                TIM3->CCR2 = 0;
                break;
            case COMM_CONNECTING:
            case COMM_CONNECTED:
                TIM3->CCR1 = 0;
                if(LED_USBInfo.isActive) {
                    TIM3->CCR2 = LED_BLINK_FAST[LED_USBInfo.phaseCounter++];
                    if(LED_USBInfo.phaseCounter == LED_USBInfo.maxPhaseCounter) {
                        LED_USBInfo.phaseCounter = 0;
                        if(!LED_USBInfo.continueAnimation) {
                            LED_USBInfo.isActive = false;
                            LED_USBInfo.maxPhaseCounter = 0;
                        }
                    }
                } else {
                    TIM3->CCR2 = 0xFF;
                    break;
                }
                break;
            case COMM_ERROR:
            default:
                TIM3->CCR1 = 0xFF;
                TIM3->CCR2 = 0;
                break;
        }
    }
#else
    static LockStatus oldLockStatus = LOCK_LOCKED;
    if(oldLockStatus != s) { //make things run a bit faster by caching to avoid unpacking the colour
        LED_GetColours(&LED_StatusColour, s);
        oldLockStatus = s;
    }
    LED_UPDATE_TIMER->PSC = LED_WorkingPrescaleLookup[w];
    //LED_HEARTBEAT_R_REG = (LED_HEARTBEAT[LED_HeartbeatPhase] * LED_StatusColour.r) >> LED_COLOUR_SHIFT;
    //LED_HEARTBEAT_G_REG = (LED_HEARTBEAT[LED_HeartbeatPhase] * LED_StatusColour.g) >> LED_COLOUR_SHIFT;
    //LED_HEARTBEAT_B_REG = (LED_HEARTBEAT[LED_HeartbeatPhase] * LED_StatusColour.b) >> LED_COLOUR_SHIFT;
    if(LED_HeartbeatUpcounting) {
        if(!(LED_HEARTBEAT_LEN > ++LED_HeartbeatPhase)) {
            LED_HeartbeatUpcounting = 0; //at top, downcount
            --LED_HeartbeatPhase; //decrement to avoid overflow on next count
        } else {

        }
    } else {
        if(0 == --LED_HeartbeatPhase) {
            LED_HeartbeatUpcounting = 1; //at bottom, upcount
        }
    }
#endif /*DEVICE_0_A_1_1_S */
}

void LED_GetColours(LED_Colour* colour, LockStatus status) {
    uint32_t colourValue = LED_COLOUR_WHITE;
    switch (status) {
        case LOCK_LOCKED:
            colourValue = LED_COLOUR_LOCKED;
            break;
        case LOCK_UNLOCKED:
            colourValue = LED_COLOUR_UNLOCKED;
            break;
        case LOCK_UNOWNED:
            colourValue = LED_COLOUR_LOCKED;
            break;
        case LOCK_PIN_VERIFY:
            colourValue = LED_COLOUR_PIN_VERIFY;
            break;
        /*case LOCK_TRANSPORT_CODE_ENTRY:
            colourValue = LED_COLOUR_TRANSPORT_CODE_ENTRY;
            break;
        case LOCK_TRANSPORT_ACTIVE:
            colourValue = LED_COLOUR_TRANSPORT_ACTIVE;
            break; */
        case LOCK_ERROR:
        default:
            colourValue = LED_COLOUR_ERROR;
    }
    colour->r = (colourValue >> LED_COLOUR_SHIFT_RED) & 0x1FF;
    colour->g = (colourValue >> LED_COLOUR_SHIFT_GREEN) & 0x1FF;
    colour->b = (colourValue >> LED_COLOUR_SHIFT_BLUE) & 0x1FF;
}

void LED_UpdateBatteryState(uint16_t batteryMillivolts, bool isCharging) {
    static uint16_t lastBatteryMillivolts = 0;
    uint16_t difference = 0;
    if(batteryMillivolts > lastBatteryMillivolts) {
        difference = batteryMillivolts - lastBatteryMillivolts;
    } else {
        difference = lastBatteryMillivolts - batteryMillivolts;
    }
    if(difference <= LED_VBAT_HYSTERESIS_MILLIVOLTS) {
        return; //don't update anything, not enough has changed. this prevents oscillating battery state changes
    }
    switch(batteryMillivolts) {
        case LED_VBAT_SHUTDOWN_MILLIVOLTS ... (LED_VBAT_CRITICAL_MILLIVOLTS - 1):
            LED_BatState = LED_BAT_SHUTDOWN;
            break;
        case LED_VBAT_CRITICAL_MILLIVOLTS ... (LED_VBAT_WARN_MILLIVOLTS - 1):
            LED_BatState = LED_BAT_CRITICAL;
            break;
        case LED_VBAT_WARN_MILLIVOLTS ... (LED_VBAT_LOW_MILLIVOLTS - 1):
            LED_BatState = LED_BAT_WARN;
            break;
        case LED_VBAT_LOW_MILLIVOLTS ... (LED_VBAT_OKAY_MILLIVOLTS - 1):
            LED_BatState = LED_BAT_LOW;
            break;
        case LED_VBAT_OKAY_MILLIVOLTS ... (LED_VBAT_GOOD_MILLIVOLTS - 1):
            LED_BatState = LED_BAT_OKAY;
            break;
        case LED_VBAT_GOOD_MILLIVOLTS ... (LED_VBAT_FULL_MILLIVOLTS - 1):
            LED_BatState = LED_BAT_GOOD;
            break;
        case LED_VBAT_FULL_MILLIVOLTS ... (LED_VBAT_OVER_MILLIVOLTS - 1):
            LED_BatState = LED_BAT_FULL;
            break;
        default:
            LED_BatState = LED_BAT_ERROR;
            break;
    }
    LED_BatCharging = isCharging;
}

void LED_SetKeypadAltIndication(bool isAltActive) {
    LED_KeypadAltActive = isAltActive;
}

void LED_SetLEDs(bool isOn) {
    LED_IsOn = isOn;
    /*if(LED_IsOn) {
        LED_ShutdownPort->BSRR |= LED_ShutdownPin; //enable the LED driver hardware
    } else { //must be off
        LED_ShutdownPort->BRR |= LED_ShutdownPin; //disable the LED driver
    }*/
}

void LED_SetKeypadLocked(bool isKeypadLocked) {
    LED_KeypadLocked = isKeypadLocked;
}

void LED_SetKeypadLockedKeypress(void) {
    LED_KeypadLockedKeypress = true;
}

void LED_SetIsOn(bool isOn) {
    LED_IsOn = isOn;
}

void LED_SetTunnelEstablished(bool isTunnelEstablished) {
    LED_TunnelEstablished = isTunnelEstablished;
    LED_TunnelEstablishing = false;
}
void LED_SetTunnelEstablishing(void) {
    LED_TunnelEstablishing = true;
}

void LED_SetBluetooth(void) {
    //just toggle for now as a dummy
    //@TODO something else
    LED_Bluetooth = !LED_Bluetooth;
}

void LED_SetErase(void) {
    LED_DoEraseBlink = true;
}

