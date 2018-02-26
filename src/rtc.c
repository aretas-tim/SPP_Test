/*
 * rtc.c
 *
 *  Created on: Apr 22, 2016
 *      Author: me
 */

#include "rtc.h"
#include "ownership.h"


void Rtc_dumpRTC(void) {
    UartDebug_sendline("RTC Dump:\n");
    UartDebug_addToBuffer("  Date: ", 8);
    UartDebug_hexprint32(RTC->DR);
    UartDebug_newline();
    UartDebug_addToBuffer("  Time: ", 8);
    UartDebug_hexprint32(RTC->TR);
    UartDebug_newline();
    UartDebug_addToBuffer("  Alarm A: ", 11);
    UartDebug_hexprint32(RTC->ALRMAR);
    UartDebug_newline();
    UartDebug_addToBuffer("  Alarm B: ", 11);
    UartDebug_hexprint32(RTC->ALRMBR);
    UartDebug_newline();
}

/*
 * compares the parameters with the current time and date from the RTC.
 * returns 0 if equal
 * negative if the current time and date is before the parameter time and date
 * positive if the current time and date is after the parameter time and date
 *
 * actual return value is not guaranteed to have any specific meaning beyond positive, negative or 0
 */
int32_t Rtc_compareTimeAndDateWithCurrent(uint32_t DRExt, uint32_t TRExt) {
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

        if(DRCur > DRExt) {
            result = 1;
        } else {
            result = -1;
        }
    } else {
        uint32_t TRResidual = TRCur ^ TRExt;
        if(TRResidual) {

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
uint32_t Rtc_getSecondsSinceEpoch() {
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


