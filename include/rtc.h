/*
 * rtc.h
 *
 *  Created on: Apr 22, 2016
 *      Author: me
 */

#ifndef RTC_H_
#define RTC_H_

#include <stdlib.h>
#include <stdint.h>
#include "stm32l4xx_hal_rtc.h"
#include "uart_debug.h"

#define RTC_MAX_SECOND 59
#define RTC_MAX_MINUTE 59
#define RTC_MAX_HOUR 23
#define RTC_MIN_MONTH 1
#define RTC_MAX_MONTH 12
#define RTC_MAX_YEAR 99


/* sets the RTC_AlarmTypeDef* to the point in the future specified by the parameters*/
uint8_t RTC_SetAlarm(volatile uint32_t* alarm, uint8_t offsetDays, uint8_t offsetHours, uint8_t offsetMinutes, uint8_t offsetSeconds);
void Rtc_dumpRTC(void);


#endif /* RTC_H_ */
