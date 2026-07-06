/**
  ******************************************************************************
  * @file           : rtc_manager.c
  * @brief          : Software real-time clock for the DWIN display.
  *                   Maintains time in software (no hardware RTC chip) and
  *                   re-writes DWIN system VP 0x0010 once per second so the
  *                   on-screen clock advances.
  ******************************************************************************
  */

#include "rtc_manager.h"
#include <stdlib.h>

/* ======================================================================
 * State
 * ====================================================================== */
static UART_HandleTypeDef *rtc_uart = NULL;
static RTC_Time_t rtc = { 25, 1, 1, 3, 0, 0, 0 };  /* 2025-01-01 00:00:00 */
static uint32_t   rtc_lastTick = 0;

static const uint8_t daysInMonth[13] =
    { 0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

/* ======================================================================
 * Calendar helpers
 * ====================================================================== */
static uint8_t RTC_IsLeap(uint16_t fullYear)
{
    return ((fullYear % 4 == 0 && fullYear % 100 != 0) ||
            (fullYear % 400 == 0)) ? 1u : 0u;
}

static uint8_t RTC_MonthLength(uint8_t year, uint8_t month)
{
    if(month == 2 && RTC_IsLeap((uint16_t)(2000 + year))) return 29;
    return daysInMonth[month];
}

/* Zeller's congruence (Gregorian). Returns 0=Sun .. 6=Sat. */
static uint8_t RTC_ComputeWeekday(uint8_t year, uint8_t month, uint8_t day)
{
    uint16_t y = (uint16_t)(2000 + year);
    uint16_t m = month;
    if(m < 3) { m += 12; y -= 1; }
    uint16_t k = y % 100;
    uint16_t j = y / 100;
    uint16_t h = (day + (13u * (m + 1u)) / 5u + k + k / 4u + j / 4u + 5u * j) % 7u;
    /* Zeller h: 0=Sat..6=Fri  ->  remap to 0=Sun..6=Sat */
    return (uint8_t)((h + 6u) % 7u);
}

/* ======================================================================
 * DWIN write
 * ------------------------------------------------------------------
 *   5A A5 0B 82 00 10 YY MM DD WK HH MM SS 00   (values in plain binary)
 * ====================================================================== */
static void RTC_Send(void)
{
    if(rtc_uart == NULL) return;

    uint8_t pkt[14] =
    {
        0x5A, 0xA5, 0x0B, 0x82,
        (uint8_t)(RTC_VP_ADDR >> 8), (uint8_t)(RTC_VP_ADDR & 0xFF),
        rtc.year, rtc.month, rtc.day, rtc.weekday,
        rtc.hour, rtc.minute, rtc.second,
        0x00
    };
    HAL_UART_Transmit(rtc_uart, pkt, sizeof(pkt), 100);
}

/* ======================================================================
 * Public API
 * ====================================================================== */
void RTC_Init(UART_HandleTypeDef *dwin_uart)
{
    rtc_uart     = dwin_uart;
    rtc_lastTick = HAL_GetTick();
    srand(HAL_GetTick());
    rtc.weekday  = RTC_ComputeWeekday(rtc.year, rtc.month, rtc.day);
    RTC_Send();
}

void RTC_SetDateTime(uint8_t year, uint8_t month, uint8_t day,
                     uint8_t hour, uint8_t minute, uint8_t second)
{
    if(month < 1 || month > 12) month = 1;
    if(day   < 1 || day   > 31) day   = 1;
    rtc.year    = year % 100;
    rtc.month   = month;
    rtc.day     = day;
    rtc.hour    = hour   % 24;
    rtc.minute  = minute % 60;
    rtc.second  = second % 60;
    rtc.weekday = RTC_ComputeWeekday(rtc.year, rtc.month, rtc.day);
    RTC_Send();
}

void RTC_SetRandom(void)
{
    uint8_t y  = (uint8_t)(24 + (rand() % 3));   /* 24-26 */
    uint8_t mo = (uint8_t)(1  + (rand() % 12));  /* 1-12  */
    uint8_t d  = (uint8_t)(1  + (rand() % 28));  /* 1-28  */
    uint8_t h  = (uint8_t)(rand() % 24);
    uint8_t mi = (uint8_t)(rand() % 60);
    uint8_t s  = (uint8_t)(rand() % 60);
    RTC_SetDateTime(y, mo, d, h, mi, s);
}

void RTC_GetTime(RTC_Time_t *out)  { if(out) *out = rtc; }
void RTC_PushToDisplay(void)       { RTC_Send(); }

/* ======================================================================
 * Tick
 * ====================================================================== */
static void RTC_Advance1s(void)
{
    if(++rtc.second < 60) return;
    rtc.second = 0;

    if(++rtc.minute < 60) return;
    rtc.minute = 0;

    if(++rtc.hour < 24) return;
    rtc.hour = 0;

    rtc.weekday = (uint8_t)((rtc.weekday + 1) % 7);
    if(++rtc.day <= RTC_MonthLength(rtc.year, rtc.month)) return;
    rtc.day = 1;

    if(++rtc.month <= 12) return;
    rtc.month = 1;

    rtc.year = (uint8_t)((rtc.year + 1) % 100);
}

void RTC_Tick(void)
{
    uint32_t now = HAL_GetTick();
    if((now - rtc_lastTick) < RTC_UPDATE_MS) return;
    rtc_lastTick += RTC_UPDATE_MS;   /* drift-free cadence */

    RTC_Advance1s();
    RTC_Send();

    /* For a pure "new random value every second" demo instead of a
     * ticking clock, replace the two lines above with: RTC_SetRandom(); */
}

/* ======================================================================
 * RTC_RunDisplayTest
 * ------------------------------------------------------------------
 * Blocking visual self-test for the DWIN RTC control (VP 0x0010).
 * Watch the LCD while this runs — each step should show the new time.
 * Step through it line-by-line and add g_ctest/RTC state to Live
 * Expressions if you also want to watch the values in the debugger.
 *
 * Call it once from main() (e.g. right after RTC_Init) to verify the
 * link, then remove the call for normal operation.
 * ====================================================================== */
void RTC_RunDisplayTest(void)
{
    /* Step 1 — a low reference time */
    RTC_SetDateTime(25, 1, 1, 0, 0, 0);      /* 2025-01-01 00:00:00 Wed */
    HAL_Delay(1500);

    /* Step 2 — a clearly different time (all fields change) */
    RTC_SetDateTime(25, 6, 15, 12, 30, 45);  /* 2025-06-15 12:30:45     */
    HAL_Delay(1500);

    /* Step 3 — set just before a year rollover */
    RTC_SetDateTime(25, 12, 31, 23, 59, 55); /* 2025-12-31 23:59:55     */
    HAL_Delay(1500);

    /* Step 4 — advance a live 10-second window so you can SEE the
     * seconds tick and the 2025-12-31 -> 2026-01-01 rollover on screen. */
    for(uint8_t i = 0; i < 10; i++)
    {
        RTC_Advance1s();       /* +1 second in software */
        RTC_PushToDisplay();   /* re-write VP 0x0010     */
        HAL_Delay(1000);
    }
}
