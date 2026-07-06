#ifndef RTC_MANAGER_H
#define RTC_MANAGER_H

#include "main.h"
#include <stdint.h>

/* ======================================================================
 * DWIN RTC integration
 * ------------------------------------------------------------------
 * DWIN system VP 0x0010 is the RTC register. Writing to it updates the
 * on-screen clock. Write format (values in PLAIN BINARY, not BCD):
 *
 *   5A A5 0B 82 00 10 YY MM DD WK HH MM SS 00
 *
 *   YY = year  (last two digits, e.g. 25 -> 0x19)
 *   MM = month (1-12)          DD = day (1-31)
 *   WK = weekday (0=Sun..6=Sat, DWIN converts to English name)
 *   HH = hour (0-23)  MM = minute (0-59)  SS = second (0-59)
 *
 * No hardware RTC chip is fitted, so the time is held in software and
 * re-written to VP 0x0010 once per second by RTC_Tick().
 * ====================================================================== */

/* DWIN system VP for the RTC register */
#define RTC_VP_ADDR    0x0010

/* Display refresh cadence (software clock re-write interval) */
#define RTC_UPDATE_MS  1000u

typedef struct
{
    uint8_t year;     /* 0-99  (last two digits: 25 -> 2025) */
    uint8_t month;    /* 1-12  */
    uint8_t day;      /* 1-31  */
    uint8_t weekday;  /* 0=Sun .. 6=Sat */
    uint8_t hour;     /* 0-23  */
    uint8_t minute;   /* 0-59  */
    uint8_t second;   /* 0-59  */
} RTC_Time_t;

/* Pass the DWIN UART (huart2). Seeds a default time and pushes it once. */
void RTC_Init(UART_HandleTypeDef *dwin_uart);

/* Set an absolute time (weekday auto-computed) and refresh the display. */
void RTC_SetDateTime(uint8_t year, uint8_t month, uint8_t day,
                     uint8_t hour, uint8_t minute, uint8_t second);

/* Seed with a pseudo-random-but-valid time (bring-up / demo). */
void RTC_SetRandom(void);

/* Read the current software time. */
void RTC_GetTime(RTC_Time_t *out);

/* Force an immediate re-write of VP 0x0010. */
void RTC_PushToDisplay(void);

/* Call every main-loop iteration: advances the clock + refreshes 1x/sec. */
void RTC_Tick(void);

/* Blocking visual self-test: writes several known times to VP 0x0010 with
 * pauses, then advances a live 10-second rollover, so you can confirm on
 * the LCD that the RTC control is bound and updating. Debug use only. */
void RTC_RunDisplayTest(void);

#endif /* RTC_MANAGER_H */
