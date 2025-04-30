/*
 * systime.c - System time management for LoRaMac-node on STM32F4RE
 * Adapted for STM32CubeMX HAL with RTC, timer-based MCU time, and logging
 */

 #include "systime.h"
 #include "stm32f4xx_hal.h"
 #include <stdio.h>
 
 // External RTC and TIM handles from CubeMX-generated main.c
 extern RTC_HandleTypeDef hrtc;
 extern TIM_HandleTypeDef htim2; // Using TIM2 for SysTimeGetMcuTime
 
 // Logging macro (replace with your logging system if available)
 #define SYS_LOG_ERROR(...) printf("[SYSTIME ERROR] " __VA_ARGS__)
 
 // Days per month (non-leap year, adjusted for leap years)
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
    while (days >= ((year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)) ? 366 : 365))
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
        SYS_LOG_ERROR("Failed to get RTC time\n");
        return sysTime;
    }
    if (HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN) != HAL_OK)
    {
        SYS_LOG_ERROR("Failed to get RTC date\n");
        return sysTime;
    }

    sysTime.Seconds = RtcToUnixTimestamp(&date, &time);
    // Fix division-by-zero
    if (RTC_SMOOTHCALIB_PLUSPULSES_RESET != 0)
    {
        sysTime.SubSeconds = (1000 * (RTC_SMOOTHCALIB_PLUSPULSES_RESET - time.SubSeconds)) / RTC_SMOOTHCALIB_PLUSPULSES_RESET;
    }
    else
    {
        sysTime.SubSeconds = (1000 * (hrtc.Init.SynchPrediv + 1 - time.SubSeconds)) / (hrtc.Init.SynchPrediv + 1);  // e.g., 256
    }

    return sysTime;
}
 
 /*!
  * \brief Gets the current MCU time using TIM2
  */
 SysTime_t SysTimeGetMcuTime(void)
 {
     SysTime_t mcuTime = {0};
     static uint32_t lastRtcSeconds = 0;
     static uint32_t timerBaseSeconds = 0;
 
     // Get current RTC time as reference
     SysTime_t rtcTime = SysTimeGet();
     if (rtcTime.Seconds == 0)
     {
         SYS_LOG_ERROR("RTC time invalid, returning zero MCU time\n");
         return mcuTime;
     }
 
     // Update base seconds if RTC has advanced
     if (rtcTime.Seconds > lastRtcSeconds)
     {
         timerBaseSeconds += (rtcTime.Seconds - lastRtcSeconds);
         lastRtcSeconds = rtcTime.Seconds;
     }
 
     // Get TIM2 counter (assumes 1ms tick)
     uint32_t timerTicks = __HAL_TIM_GET_COUNTER(&htim2);
     mcuTime.Seconds = timerBaseSeconds;
     mcuTime.SubSeconds = timerTicks % 1000; // Milliseconds
 
     // Adjust seconds if timer has rolled over
     if (timerTicks >= 1000)
     {
         mcuTime.Seconds += timerTicks / 1000;
         mcuTime.SubSeconds = timerTicks % 1000;
     }
 
     return mcuTime;
 }
 
 /*!
  * \brief Sets the system time using RTC
  */
 void SysTimeSet(SysTime_t sysTime)
 {
     RTC_TimeTypeDef time = {0};
     RTC_DateTypeDef date = {0};
 
     UnixTimestampToRtc(sysTime.Seconds, &date, &time);
     // Convert milliseconds to RTC subseconds (down-counting)
     time.SubSeconds = (RTC_SMOOTHCALIB_PLUSPULSES_RESET * (1000 - sysTime.SubSeconds)) / 1000;
 
     if (HAL_RTC_SetTime(&hrtc, &time, RTC_FORMAT_BIN) != HAL_OK)
     {
         SYS_LOG_ERROR("Failed to set RTC time\n");
         return;
     }
     if (HAL_RTC_SetDate(&hrtc, &date, RTC_FORMAT_BIN) != HAL_OK)
     {
         SYS_LOG_ERROR("Failed to set RTC date\n");
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
 
 /*!
  * \brief Gets the seconds component of a SysTime_t
  */
 uint32_t SysTimeGetSeconds(SysTime_t sysTime)
 {
     return sysTime.Seconds;
 }
 
 /*!
  * \brief Gets the subseconds component of a SysTime_t
  */
 uint16_t SysTimeGetSubSeconds(SysTime_t sysTime)
 {
     return sysTime.SubSeconds;
 }
 
 /*!
  * \brief Converts a Unix timestamp to broken-down local time
  */
 void SysTimeLocalTime(const uint32_t timestamp, struct tm *localtime)
 {
     if (localtime == NULL)
     {
         SYS_LOG_ERROR("Null localtime pointer in SysTimeLocalTime\n");
         return;
     }
 
     RTC_TimeTypeDef time = {0};
     RTC_DateTypeDef date = {0};
 
     UnixTimestampToRtc(timestamp, &date, &time);
 
     localtime->tm_year = (date.Year + 2000) - 1900; // tm_year is years since 1900
     localtime->tm_mon = date.Month - 1;            // tm_mon is 0-based (0-11)
     localtime->tm_mday = date.Date;                // tm_mday is 1-based
     localtime->tm_hour = time.Hours;
     localtime->tm_min = time.Minutes;
     localtime->tm_sec = time.Seconds;
     localtime->tm_wday = date.WeekDay - 1;         // tm_wday is 0-based (0=Sunday)
     localtime->tm_yday = 0;                        // Not computed here
     localtime->tm_isdst = -1;                      // No DST info
 }
 
 /*!
  * \brief Converts a SysTime_t to a Unix timestamp
  */
 uint32_t SysTimeMktime(SysTime_t sysTime)
 {
     return sysTime.Seconds;
 }