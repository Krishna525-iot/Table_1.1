#include "button_matrix.h"
#include "lcd_hmi.h"
#include "action_comm.h"
#include "main.h"
#include "sensor_manager.h"

/* ======================================================================
 * Timing constants
 * ====================================================================== */
#define LONG_PRESS_TIME   600   /* ms before long-press animation starts */
#define DEBOUNCE_MAX      5
#define DEBOUNCE_TRIGGER  4

/* ======================================================================
 * State arrays
 * ====================================================================== */
static uint8_t  key_matrix[7][5]  = {0};
static uint8_t  stableCount[7][5] = {0};
static uint32_t pressStart[7][5]  = {0};
static uint8_t  longActive[7][5]  = {0};

/* Reverse-orientation state (0 = normal, 1 = reversed).
 * Exposed volatile so it can be watched in STM32CubeIDE Live Expressions.
 * Stays in lockstep with lcd_hmi's reverseMode (both toggle together). */
static volatile uint8_t g_reverseState = 0;

/* ======================================================================
 * Combo latches
 * ------------------------------------------------------------------
 * One latch per combo key pair. Set on first detection, cleared when
 * either key releases. Prevents re-triggering on every ~8ms scan tick.
 * ====================================================================== */
static uint8_t comboLatch_Flex   = 0;   /* GoBack + Accept   → GOBACK_FLEX   */
static uint8_t comboLatch_Height = 0;   /* GoBack + HeightDn → GOBACK_HEIGHT */
static uint8_t comboLatch_Accept = 0;   /* GoBack + FlexUp   → GOBACK_ACCEPT */
static uint8_t comboLatch_Slide  = 0;   /* GoBack + Slide    → GOBACK_HEIGHT */
static uint8_t comboLatch_Level  = 0;   /* GoBack + Level    → Memory SET    */

/* ======================================================================
 * Motor BLE command code table
 * ------------------------------------------------------------------
 * keyCmdCode[col][row] = BLE command byte sent for that key.
 * 0 = not a motor key (handled separately in the single-key block).
 *
 * Spec table mapping:
 *   col 2: FL Unlock(r0)=0x14, HeightDn(r1)=0x02, HeightUp(r2)=0x01, FLLock(r3)=0x13
 *   col 3: FlexDn(r0)=0x12,    TrendRev(r1)=0x04, Trend(r2)=0x03,    FlexUp(r3)=0x11
 *   col 4: (r0)=0,             BackDn(r1)=0x06,   BackUp(r2)=0x05,   (r3)=0
 *   col 5: Level(r0)=0x19,     TiltRt(r1)=0x08,   TiltLt(r2)=0x07,   (r3)=0
 *   col 6: (r0)=0,             SlideRv(r1)=0x10,  Slide(r2)=0x09,    (r3)=0
 * ====================================================================== */
static const uint8_t keyCmdCode[7][5] =
{
    {0,    0,    0,    0,    0},   /* col 0: cursor keys */
    {0,    0,    0,    0,    0},   /* col 1: special keys */
    {0x14, 0x02, 0x01, 0x13, 0},  /* col 2 */
    {0x12, 0x04, 0x03, 0x11, 0},  /* col 3 */
    {0,    0x06, 0x05, 0,    0},  /* col 4 */
    {0x19, 0x08, 0x07, 0,    0},  /* col 5 */
    {0,    0x10, 0x09, 0,    0}   /* col 6 */
};

/* ======================================================================
 * port/pin lookup tables — STATIC CONST
 * ------------------------------------------------------------------
 * File-scope static const eliminates the per-call stack hit and the
 * repeated initialisation that a local declaration would incur.
 * ====================================================================== */
static GPIO_TypeDef* const col_port[7] =
{
    C1_GPIO_Port, C2_GPIO_Port, C3_GPIO_Port,
    C4_GPIO_Port, C5_GPIO_Port, C6_GPIO_Port, C7_GPIO_Port
};
static const uint16_t col_pin[7] =
    {C1_Pin, C2_Pin, C3_Pin, C4_Pin, C5_Pin, C6_Pin, C7_Pin};

static GPIO_TypeDef* const row_port[5] =
    {R1_GPIO_Port, R2_GPIO_Port, R3_GPIO_Port, R4_GPIO_Port, R5_GPIO_Port};
static const uint16_t row_pin[5] =
    {R1_Pin, R2_Pin, R3_Pin, R4_Pin, R5_Pin};

/* ======================================================================
 * BUTTON_MATRIX_Scan
 * ------------------------------------------------------------------
 * Called from the main loop every iteration.
 *
 * Scan sequence:
 *   1. Assert each column HIGH in turn, read all rows.
 *   2. Debounce each key with a counter (DEBOUNCE_TRIGGER/DEBOUNCE_MAX).
 *   3. Evaluate combo keys FIRST so held pairs don't also fire
 *      as individual presses.
 *   4. Evaluate individual press/hold/release for remaining keys.
 * ====================================================================== */
void BUTTON_MATRIX_Scan(void)
{
    /* ================================================================
     * PHASE 1 — MATRIX SCAN + DEBOUNCE
     * ================================================================ */
    for(int c = 0; c < 7; c++)
    {
        /* Assert all columns LOW first, then drive column c HIGH */
        for(int i = 0; i < 7; i++)
            HAL_GPIO_WritePin(col_port[i], col_pin[i], GPIO_PIN_RESET);

        HAL_GPIO_WritePin(col_port[c], col_pin[c], GPIO_PIN_SET);
        HAL_Delay(1);   /* settling time for RC on row lines */

        for(int r = 0; r < 5; r++)
        {
            uint8_t raw =
                (HAL_GPIO_ReadPin(row_port[r], row_pin[r]) == GPIO_PIN_SET) ? 1 : 0;
            if(raw) { if(stableCount[c][r] < DEBOUNCE_MAX) stableCount[c][r]++; }
            else    { if(stableCount[c][r] > 0)            stableCount[c][r]--; }
        }
    }

    /* ================================================================
     * PHASE 2 — COMBO KEYS
     * Evaluated before the single-key loop. Each active combo returns
     * immediately so its component keys don't also fire individually.
     * ================================================================ */
    if(!LCD_IsLocked() && LCD_IsPowerOn())
    {
        uint8_t goBack   = (stableCount[1][3] >= DEBOUNCE_TRIGGER);
        uint8_t accept   = (stableCount[1][0] >= DEBOUNCE_TRIGGER);
        uint8_t heightDn = (stableCount[2][1] >= DEBOUNCE_TRIGGER);
        uint8_t flexUp   = (stableCount[3][3] >= DEBOUNCE_TRIGGER);
        uint8_t slide    = (stableCount[6][2] >= DEBOUNCE_TRIGGER);
        uint8_t level    = (stableCount[5][0] >= DEBOUNCE_TRIGGER);

        /* GoBack + Accept → GOBACK_FLEX (PA=0x0D) */
        if(goBack && accept)
        {
            if(!comboLatch_Flex)
            {
                comboLatch_Flex = 1;
                ACTION_SendCommand(0x00);
                LCD_StopCircularAnim();
                LCD_ShowSensorPage(GOBACK_FLEX);
            }
            return;
        }
        else comboLatch_Flex = 0;

        /* GoBack + HeightDn → GOBACK_HEIGHT (PA=0x0E) */
        if(goBack && heightDn)
        {
            if(!comboLatch_Height)
            {
                comboLatch_Height = 1;
                ACTION_SendCommand(0x00);
                LCD_StopCircularAnim();
                LCD_ShowSensorPage(GOBACK_HEIGHT);
            }
            return;
        }
        else comboLatch_Height = 0;

        /* GoBack + FlexUp → GOBACK_ACCEPT (PA=0x0F) */
        if(goBack && flexUp)
        {
            if(!comboLatch_Accept)
            {
                comboLatch_Accept = 1;
                ACTION_SendCommand(0x00);
                LCD_StopCircularAnim();
                LCD_ShowSensorPage(GOBACK_ACCEPT);
            }
            return;
        }
        else comboLatch_Accept = 0;

        /* GoBack + Slide → GOBACK_HEIGHT (PA=0x0E) */
        if(goBack && slide)
        {
            if(!comboLatch_Slide)
            {
                comboLatch_Slide = 1;
                ACTION_SendCommand(0x00);
                LCD_StopCircularAnim();
                LCD_ShowSensorPage(GOBACK_HEIGHT);
            }
            return;
        }
        else comboLatch_Slide = 0;

        /* GoBack + Level → GoBackLevel BLE packet + Memory SET page */
        if(goBack && level)
        {
            if(!comboLatch_Level)
            {
                comboLatch_Level = 1;
                ACTION_SendCommand(0x00);
                LCD_StopCircularAnim();
                ACTION_SendGoBackLevel();
                LCD_ShowMemoryPage(MEM_MODE_SET);
            }
            return;
        }
        else comboLatch_Level = 0;
    }

    /* ================================================================
     * PHASE 3 — SINGLE KEY PRESS / HOLD / RELEASE
     * ================================================================ */
    for(int c = 0; c < 7; c++)
    {
        for(int r = 0; r < 5; r++)
        {
            uint8_t pressed = (stableCount[c][r] >= DEBOUNCE_TRIGGER);

            /* --------------------------------------------------------
             * PRESS EDGE
             * -------------------------------------------------------- */
            if(pressed && !key_matrix[c][r])
            {
                key_matrix[c][r] = 1;
                pressStart[c][r] = HAL_GetTick();
                longActive[c][r] = 0;

                /* Lock / Unlock — always respond, bypass power/lock */
                if(c == 4 && r == 3) { LCD_SetLock(1); continue; }
                if(c == 4 && r == 0) { LCD_SetLock(0); continue; }

                /* --------------------------------------------------------
                 * Reverse orientation — 2-state toggle (lock + power gated)
                 *   press 1 -> reverse ENGAGED : icon 6A = 1, LCD reversed
                 *   press 2 -> reverse NORMAL  : icon 6A = 0, LCD normal
                 *
                 * LCD_ToggleReverse() flips reverseMode AND pushes the
                 * indicator icon to DWIN IA 0x6A internally, using the same
                 * lcd_set_icon primitive as the proven test packet. The
                 * local mirror g_reverseState is only for Live Expressions.
                 * -------------------------------------------------------- */
                if(c == 6 && r == 3)
                {
                    if(!LCD_IsLocked() && LCD_IsPowerOn())
                    {
                        g_reverseState ^= 1;   /* mirror for Live Expressions */
                        LCD_ToggleReverse();   /* flips reverseMode + icon 0x6A */
                    }
                    continue;
                }

                /* Power — always respond */
                if(c == 6 && r == 0) { LCD_TogglePower(); continue; }

                /* All keys below gated by lock and power state */
                if(LCD_IsLocked() || !LCD_IsPowerOn()) continue;

                /* ---- Special / navigation keys (col 1) ---- */
                if(c == 1 && r == 3) { LCD_HandleKey(CMD_GOBACK);  continue; }
                if(c == 1 && r == 0) { LCD_HandleKey(CMD_ACCEPT);  continue; }
                if(c == 1 && r == 2) { LCD_HandleKey(CMD_DETAILS); continue; }
                if(c == 1 && r == 1) { LCD_ShowSensorPage(PAGE_SETTING); continue; }

                /* ---- Memory single press → RECALL mode ---- */
                if(c == 5 && r == 3) { LCD_ShowMemoryPage(MEM_MODE_RECALL); continue; }

                /* ---- Cursor keys (col 0) ---- */
                if(c == 0)
                {
                    if(r == 3) LCD_HandleKey(CMD_CURSOR_UP);
                    if(r == 0) LCD_HandleKey(CMD_CURSOR_DOWN);
                    if(r == 1) LCD_HandleKey(CMD_CURSOR_LEFT);
                    if(r == 2) LCD_HandleKey(CMD_CURSOR_RIGHT);
                    continue;
                }

                /* ---- Motor keys ---- */
                uint8_t cmd = keyCmdCode[c][r];
                if(cmd)
                {
                    ACTION_SendCommand(cmd);
                    LCD_HandleKey(cmd);
                    if(Sensor_IsActive())
                        Sensor_ProcessKeys(cmd);
                }
            }

            /* --------------------------------------------------------
             * HELD → start long-press animation for motor keys
             * -------------------------------------------------------- */
            else if(pressed && key_matrix[c][r] && !longActive[c][r])
            {
                uint8_t cmd = keyCmdCode[c][r];
                if(cmd && !LCD_IsLocked() && LCD_IsPowerOn() &&
                   (HAL_GetTick() - pressStart[c][r]) >= LONG_PRESS_TIME)
                {
                    longActive[c][r] = 1;
                    LCD_StartCircularAnim(cmd);
                }
            }

            /* --------------------------------------------------------
             * RELEASE EDGE
             * -------------------------------------------------------- */
            if(!pressed && key_matrix[c][r])
            {
                key_matrix[c][r] = 0;
                longActive[c][r] = 0;
                if(!LCD_IsLocked() && LCD_IsPowerOn())
                {
                    ACTION_SendCommand(0x00);   /* stop motor */
                    LCD_StopCircularAnim();
                }
            }
        }
    }
}
