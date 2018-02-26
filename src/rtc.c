/*
 * rtc.c
 *
 *  Created on: Apr 22, 2016
 *      Author: me
 */

#include "rtc.h"
#include "ownership.h"
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

/*
 * compares the parameters with the current time and date from the RTC.
 * returns 0 if equal
 * negative if the current time and date is before the parameter time and date
 * positive if the current time and date is after the parameter time and date
 *
 * actual return value is not guaranteed to have any specific meaning beyond positive, negative or 0
 */
int32_t compareTimeAndDateWithCurrent(uint32_t DRExt, uint32_t TRExt) {
    //grab copies
    uint32_t DRCur = RTC->DR;
    uint32_t TRCur = RTC->TR;

    int32_t result = 0;

    //mask off WDU in the date register, its not relevant to us
    DRCur &= (~RTC_DR_WDU);
    DRExt &= (~RTC_DR_WDU);

    //so we're doing this the hard way... (@&@# and massively overthinking it
    uint32_t DRResidual = DRCur ^ DRExt;
    if(DRResidual) {
        //dates differ
        /*uint32_t shiftAmount = -1;
        while(DRResidual) {
            DRResidual >>= 1;
            shiftAmount++;
        }
        DRCur >>= shiftAmount;
        DRExt >>= shiftAmount;*/
        if(DRCur > DRExt) {
            result = 1;
        } else {
            result = -1;
        }
    } else {
        uint32_t TRResidual = TRCur ^ TRExt;
        if(TRResidual) {
            /*uint32_t shiftAmount = -1;
            while(TRResidual) {
                TRResidual >>= 1;
                shiftAmount++;
            }
            TRCur >>= shiftAmount;
            TRExt >>= shiftAmount;*/
            if(TRCur > TRExt) {
                result = 1;
            } else {
                result = -1;
            }
        } else {
            //exactly the same
            result = 0;
        }
    }


    return result;
}

/*
 * converts the time and date from the RTC in to seconds since epoch as a 32 bit integer
 * 32 bit integer will overflow in ~136 years, but that's not an issue since we can only count 100 of em.
 *
 * Epoch is year 2000, since this only supports 2 digit years.
 * if this code is still active in 2100, you're going to be having a bad time anyway. sorry, 2016.
 */
uint32_t getSecondsSinceEpoch() {
    uint32_t epoch = 0;
    //grab copies
    uint32_t DRCur = RTC->DR;
    uint32_t TRCur = RTC->TR;

    epoch += TRCur & RTC_TR_SU;
    epoch += 10 * (TRCur & RTC_TR_ST) >> 4;
    epoch += SECONDS_PER_MINUTE * (TRCur & RTC_TR_MNU) >> 8;
    epoch += 10 * SECONDS_PER_MINUTE * (TRCur & RTC_TR_MNT) >> 12;
    epoch += SECONDS_PER_HOUR * (TRCur & RTC_TR_HU) >> 16;
    epoch += 10 * SECONDS_PER_HOUR * (TRCur & RTC_TR_HT) >> 20;
    epoch += 12 * SECONDS_PER_HOUR * (TRCur & RTC_TR_PM) >> 22; //if this is set (and it shouldn't be, we should only be operating on a 24 hour clock), just add 12 hours

    epoch += SECONDS_PER_DAY * ((DRCur & RTC_DR_DU) - 1); //days are one-based
    epoch += 10 * SECONDS_PER_DAY * (DRCur & RTC_DR_DT) >> 4;
    uint8_t months = ((DRCur & RTC_DR_MU) >> 8) + ((DRCur & RTC_DR_MT) >> 12) - 1; //months are one-based too
    switch(months) {
        case 0:
            epoch += MONTH_1_SECONDS_AFTER_YEAR;
            break;
        case 1:
            epoch += MONTH_2_SECONDS_AFTER_YEAR;
            //leap years get handled in years section
            break;
        case 2:
            epoch += MONTH_3_SECONDS_AFTER_YEAR;
            break;
        case 3:
            epoch += MONTH_4_SECONDS_AFTER_YEAR;
            break;
        case 4:
            epoch += MONTH_5_SECONDS_AFTER_YEAR;
            break;
        case 5:
            epoch += MONTH_6_SECONDS_AFTER_YEAR;
            break;
        case 6:
            epoch += MONTH_7_SECONDS_AFTER_YEAR;
            break;
        case 7:
            epoch += MONTH_8_SECONDS_AFTER_YEAR;
            //leap years get handled in years section
            break;
        case 8:
            epoch += MONTH_9_SECONDS_AFTER_YEAR;
            break;
        case 9:
            epoch += MONTH_10_SECONDS_AFTER_YEAR;
            break;
        case 10:
            epoch += MONTH_11_SECONDS_AFTER_YEAR;
            break;
        case 11:
            epoch += MONTH_12_SECONDS_AFTER_YEAR;
            break;
    }

    uint8_t years = (10 * ((DRCur & RTC_DR_YT) >> 4)) + (DRCur & RTC_DR_YU);
    epoch += years * SECONDS_PER_365_DAY_YEAR;

    uint8_t leapYears = ((years + 3) >> 2); //this is the number of leap years that have passed.
    //technically this is going to be incorrect for 2100, 2200, 2300 as they are not leap years, but i have no idea how the RTC handles it yet (2016-12-05)
    epoch += leapYears * SECONDS_PER_DAY; //correct for already-passed leap years

    //if THIS is a leap year, check month and correct.
    if(!(years & 0x3) && (months > 2) && (years != 0)) { //this should work through 2400.
        epoch += SECONDS_PER_DAY; //add leap day
        //february is self-correcting, but we need to adjust march onwards.
    }
    return epoch;

}


