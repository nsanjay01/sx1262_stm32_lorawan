/*
 * systime.c - System time management for LoRaMac-node on STM32F4RE
 * Adapted for STM32CubeMX HAL with RTC, date support, and error handling
 */

 #include "systime.h"
 #include "stm32f4xx_hal.h"
 
 // External RTC handle from CubeMX-generated main.c
 extern RTC_HandleTypeDef hrtc;
 
 // Days per month (non-leap year, adjusted in code for leap years)
 static const uint8_t DaysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
 
 /*!
  * \brief Calculates Unix timestamp from RTC date and time
  */
 static uint32_t RtcToUnixTimestamp(RTC_DateTypeDef *date, RTC_TimeTypeDef *time)
 {
     uint32_t days = 0;
     uint32_t seconds = 0;
     uint16_t year = date->Year + 2000; // RTC year is offset from 2000
 
     // Calculate days since 1970-01-01
     for (uint16_t y = 1970; y < year; y++)
     {
         days += (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0)) ? 366 : 365;
     }
 
     for (uint8_t m = 1; m < date->Month; m++)
     {
         if (m == 2 && (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)))
         {
             days += 29; // Leap year February
         }
         else
         {
             days += DaysInMonth[m - 1];
         }
     }
 
     days += date->Date - 1; // Days in current month (1-based)
 
     // Convert to seconds
     seconds = days * 86400; // 86400 seconds per day
     seconds += (time->Hours * 3600) + (time->Minutes * 60) + time->Seconds;
 
     return seconds;
 }
 
 /*!
  * \brief Converts Unix timestamp to RTC date and time
  */
 static void UnixTimestampToRtc(uint32_t timestamp, RTC_DateTypeDef *date, RTC_TimeTypeDef *time)
 {
     uint32_t days = timestamp / 86400;
     uint32_t seconds = timestamp % 86400;
 
     time->Hours = seconds / 3600;
     seconds %= 3600;
     time->Minutes = seconds / 60;
     time->Seconds = seconds % 60;
 
     uint16_t year = 1970;
     while (days >= (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0) ? 366 : 365))
     {
         days -= (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)) ? 366 : 365;
         year++;
     }
 
     date->Year = year - 2000; // RTC year offset from 2000
 
     uint8_t month = 1;
     while (days >= (month == 2 && (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)) ? 29 : DaysInMonth[month - 1]))
     {
         days -= (month == 2 && (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)) ? 29 : DaysInMonth[month - 1]);
         month++;
     }
 
     date->Month = month;
     date->Date = days + 1; // 1-based
     date->WeekDay = (days + 4) % 7 + 1; // 1970-01-01 was Thursday (4), RTC weekday 1-7
 }
 
 /*!
  * \brief Gets the current system time from RTC
  */
 SysTime_t SysTimeGet(void)
 {
     SysTime_t sysTime = {0};
     RTC_TimeTypeDef time;
     RTC_DateTypeDef date;
 
     if (HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN) != HAL_OK)
     {
         // Error handling: return zero time
         return sysTime;
     }
     if (HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN) != HAL_OK)
     {
         // Error handling: return zero time
         return sysTime;
     }
 
     sysTime.Seconds = RtcToUnixTimestamp(&date, &time);
     sysTime.SubSeconds = (1000 * (RTC_SMOOTHCALIB_PLUSPULSES_RESET - time.SubSeconds)) / RTC_SMOOTHCALIB_PLUSPULSES_RESET;
 
     return sysTime;
 }
 
 /*!
  * \brief Sets the system time using RTC
  */
 void SysTimeSet(SysTime_t sysTime)
 {
     RTC_TimeTypeDef time = {0};
     RTC_DateTypeDef date = {0};
 
     UnixTimestampToRtc(sysTime.Seconds, &date, &time);
     time.SubSeconds = (RTC_SMOOTHCALIB_PLUSPULSES_RESET * (1000 - sysTime.SubSeconds)) / 1000;
 
     if (HAL_RTC_SetTime(&hrtc, &time, RTC_FORMAT_BIN) != HAL_OK)
     {
         // Error handling: could log or assert here
         return;
     }
     if (HAL_RTC_SetDate(&hrtc, &date, RTC_FORMAT_BIN) != HAL_OK)
     {
         // Error handling: could log or assert here
         return;
     }
 }
 
 /*!
  * \brief Adds two SysTime_t values
  */
 SysTime_t SysTimeAdd(SysTime_t a, SysTime_t b)
 {
     SysTime_t result = {0};
 
     result.Seconds = a.Seconds + b.Seconds;
     result.SubSeconds = a.SubSeconds + b.SubSeconds;
 
     if (result.SubSeconds >= 1000)
     {
         result.Seconds += 1;
         result.SubSeconds -= 1000;
     }
 
     return result;
 }
 
 /*!
  * \brief Subtracts two SysTime_t values
  */
 SysTime_t SysTimeSub(SysTime_t a, SysTime_t b)
 {
     SysTime_t result = {0};
     int32_t subSeconds;
 
     result.Seconds = a.Seconds - b.Seconds;
     subSeconds = (int32_t)a.SubSeconds - (int32_t)b.SubSeconds;
 
     if (subSeconds < 0)
     {
         result.Seconds -= 1;
         result.SubSeconds = subSeconds + 1000;
     }
     else
     {
         result.SubSeconds = subSeconds;
     }
 
     return result;
 }