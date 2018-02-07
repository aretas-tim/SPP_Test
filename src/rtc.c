/*
 * rtc.c
 *
 *  Created on: Apr 22, 2016
 *      Author: me
 */

#include "rtc.h"

/* assumptions:
 * - halarm is points to a valid alarm register and only needs its time updated to point (parameters) in the future
 * - clock is set for 24 hour notation
 * - the various parameters plus the current time will not cause them to overflow mathematically (i.e. don't pass in 240 days on the 20th of the month, as your alarm may end up in the past)
 *   the function will check for "time" overflows (i.e. at 0:45 seconds, passing in an alarm for 20 seconds in the future will correctly make it 1:05)
 *   does not deal with subseconds. if you want subseconds, use a timer and an interrupt or tack on to Systick.
 *   this will also probably screw up royally if days puts you ahead of today's date in the next month. You have been warned.
 *   also do not use this with RTC_ALARM_DATEWEEKDAYSEL_WEEKDAY.
 */
uint8_t RTC_SetAlarm(volatile uint32_t* alarm, uint8_t offsetDays, uint8_t offsetHours, uint8_t offsetMinutes, uint8_t offsetSeconds) {
    if(&RTC->ALRMAR != alarm && &RTC->ALRMBR != alarm) {
        return -1; //what are you doing?
    }
    uint32_t date = RTC->DR;
    uint32_t time = RTC->TR;

    uint32_t seconds =  (((time & 0x00000070) * 10) >> 4) + (time & 0x0000000F);
    uint32_t minutes = ((((time & 0x00007000) * 10) >> 4) + (time & 0x00000F00)) >> 8;
    uint32_t hours   = ((((time & 0x00300000) * 10) >> 4) + (time & 0x000F0000)) >> 16;
    uint32_t dayNum   =  (((date & 0x00000030) * 10) >> 4) + (date & 0x0000000F);
    uint32_t monthNum = ((((date & 0x00001000) * 10) >> 4) + (date & 0x00000F00)) >> 8; //as month is stored 1-based :/

    uint8_t yearNum = ((((date & 0x00F00000) >> 4) + (date & 0x000F0000)) >> 16);

    seconds += offsetSeconds;
    uint8_t overflow = 0;
    while(RTC_MAX_SECOND < seconds) {
        seconds -= (RTC_MAX_SECOND + 1);
        overflow += 1;
    }
    minutes += offsetMinutes + overflow;
    overflow = 0;
    while(RTC_MAX_MINUTE < minutes) {
        minutes -= (RTC_MAX_MINUTE + 1);
        overflow += 1;
    }
    hours += offsetHours + overflow;
    overflow = 0;
    while(RTC_MAX_HOUR < hours) {
        hours -= (RTC_MAX_HOUR + 1);
        overflow += 1;
    }

    /*uint8_t daysInMonth = RTC_DAYS_IN_MONTH[monthNum];
    dayNum += offsetDays + overflow;
    overflow = 0;
    do {
        if((monthNum == RTC_MONTH_FEBRUARY) && (!(yearNum & 0x3) && (yearNum != 0))) {
            //if its February AND the year is divisible by 4 AND the year is NOT 0
            //leap year!
            daysInMonth++;
        }
        dayNum -= (daysInMonth + 1);
        overflow += 1;
        monthNum++;
        if(RTC_MAX_MONTH < monthNum) {
            monthNum = RTC_MIN_MONTH;
        }
        daysInMonth = RTC_DAYS_IN_MONTH[monthNum];
    } while(daysInMonth < dayNum);*/

    /* so we can do this in one operation*/
    uint32_t alarmReg = 0;
    alarmReg |= (dayNum / 10)  << 28;
    alarmReg |= (dayNum % 10)  << 24;
    alarmReg |= (hours / 10)   << 20;
    alarmReg |= (hours % 10)   << 16;
    alarmReg |= (minutes / 10) << 12;
    alarmReg |= (minutes % 10) << 8;
    alarmReg |= (seconds / 10) << 4;
    alarmReg |= (seconds % 10);
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;
    RTC->CR &= ~(RTC_CR_ALRAE);
    while(!(RTC->ISR & RTC_ISR_ALRAWF)); //spin until we can update the alarm
    *alarm = alarmReg;
    RTC->CR |= RTC_CR_ALRAE | RTC_CR_ALRAIE; //enable alarm and interrupt
    RTC->WPR = 0x00; //reactivate write protection

    return overflow;
}

void dumpRTC(void) {
    uart_debug_sendline("RTC Dump:\n");
    uart_debug_addToBuffer("  Date: ", 8);
    uart_debug_hexprint32(RTC->DR);
    uart_debug_newline();
    uart_debug_addToBuffer("  Time: ", 8);
    uart_debug_hexprint32(RTC->TR);
    uart_debug_newline();
    uart_debug_addToBuffer("  Alarm A: ", 11);
    uart_debug_hexprint32(RTC->ALRMAR);
    uart_debug_newline();
    uart_debug_addToBuffer("  Alarm B: ", 11);
    uart_debug_hexprint32(RTC->ALRMBR);
    uart_debug_newline();
}


