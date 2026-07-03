#include "lcd_hmi.h"
#include "main.h"
#include "sensor_manager.h"
#include "action_comm.h"
#include <stdint.h>

/* ======================================================================
 * Internal defines
 * ====================================================================== */
#define HOME_RETURN_TIME  4000

/*
 * GoBack+Accept feature access flags.
 * These indexes match featureEnable[] / featureVP[] order below.
 * When a flag is OFF the related page/function is blocked from the UI.
 */
#define LCD_FEATURE_BATTERY     0u
#define LCD_FEATURE_GYRO        1u
#define LCD_FEATURE_REALTIME    2u
#define LCD_FEATURE_COLLISION   3u
#define LCD_FEATURE_SLIDE       4u
#define LCD_FEATURE_FLEXFLOOR   5u
#define LCD_FEATURE_RESERVED    6u

#define CIRC_INC(val, max) do { (val)++; if((val) > (max)) (val) = 0; } while(0)
#define CIRC_DEC(val, max) do { if((val) == 0) (val) = (max); else (val)--; } while(0)

#define STEP_INC(val, max)  do {                                                        \
        if(animStepping) { (val) = ((val) >= (max)) ? 0 : (val)+1; valueChanged = 1; } \
        else if((val) < (max)) { (val)++; valueChanged = 1; }                           \
    } while(0)

#define STEP_DEC(val, max)  do {                                                        \
        if(animStepping) { (val) = ((val) == 0) ? (max) : (val)-1; valueChanged = 1; } \
        else if((val) > 0) { (val)--; valueChanged = 1; }                               \
    } while(0)

/* ======================================================================
 * Static state
 * ====================================================================== */
static UART_HandleTypeDef *lcd_uart    = NULL;
static uint8_t             currentPage = 0xFF;

static UI_State_t  uiState     = UI_HOME;
static UI_State_t  parentState = UI_HOME;

static uint8_t details_cursor = 0;

static uint8_t selRTP1 = 0;
static uint8_t selRTP2 = 0;
static uint8_t rtp1Val[RTP1_FIELD_COUNT];
static uint8_t rtp2Val[RTP2_FIELD_COUNT];

static uint8_t batteryLevel = 0;
static uint8_t selSetting   = 0;

static uint8_t clockCursor = 0;
static uint8_t clockHour   = 0;
static uint8_t clockMin    = 0;

static uint8_t heightState = 0;
static uint8_t trendState  = 0;
static uint8_t backState   = 0;
static uint8_t tiltState   = 2;
static uint8_t slideState  = 0;
static uint8_t flexState   = 0;

static uint8_t valueChanged = 0;
static uint8_t animStepping = 0;

static uint8_t  animActive = 0;
static uint8_t  animCmd    = 0;
static uint32_t animTick   = 0;

static uint8_t reverseMode = 0;
static uint8_t lcdPowerOn  = 1;
static uint8_t lcdLocked   = 0;

static uint8_t selFeature     = 0;
static uint8_t selComboHeight = 0;
static uint8_t selComboAccept = 0;

/* ======================================================================
 * Memory state
 * ====================================================================== */
static uint8_t selMemory  = 0;
static uint8_t memoryMode = MEM_MODE_RECALL;
static uint8_t memSlotSaved[MEMORY_SLOT_COUNT] = {0};

static const uint8_t memSlotVP[MEMORY_SLOT_COUNT] =
{
    VP_MEM_SLOT1, VP_MEM_SLOT2, VP_MEM_SLOT3, VP_MEM_SLOT4, VP_MEM_SLOT5
};

/* ======================================================================
 * Combo / RTP tables
 * ====================================================================== */
static uint16_t      comboHeightVal[7]  = {0};
static const uint8_t comboHeightVP[7]   = {0x6A,0x6B,0x6C,0x6D,0x6E,0x6F,0x70};
static const uint8_t comboHeightEdit[7] = {0,   1,   0,   0,   1,   0,   1   };

/*
 * FIX: All feature flags initialised to 1 (ALL PAGES ENABLED AT STARTUP).
 * The user can disable individual pages via the GoBack+Accept combo page.
 */
static uint8_t       featureEnable[ACCEPT_FIELD_COUNT] = {1, 1, 1, 1, 1, 1, 1};

static const uint8_t featureVP[ACCEPT_FIELD_COUNT] =
{
    VP_ACC_BATTERY, VP_ACC_GYRO, VP_ACC_REALTIME, VP_ACC_COLLISION,
    VP_ACC_SLIDE,   VP_ACC_FLEXFLOOR, VP_ACC_RESERVED
};

static uint16_t      editVal[ACCEPT_EDIT_COUNT] = {0};
static const uint8_t editVP[ACCEPT_EDIT_COUNT]  =
    {0x71,0x72,0x73,0x74,0x75,0x76,0x77,0x78};

static const uint8_t rtp1VP[RTP1_FIELD_COUNT] =
{
    VP_RTP_HEIGHT, VP_RTP_SLIDE,  VP_RTP_BACK_UP, VP_RTP_BACK_DN,
    VP_RTP_TILT_L, VP_RTP_TILT_R, VP_RTP_TREND,   VP_RTP_REV
};
static const uint8_t rtp2VP[RTP2_FIELD_COUNT] =
    {VP_RTP2_LEG1_UP, VP_RTP2_LEG1_DN, VP_RTP2_LEG2_UP, VP_RTP2_LEG2_DN};

static uint32_t homeTimer  = 0;
static uint8_t  homeActive = 0;

/* ======================================================================
 * Forward declarations
 * ====================================================================== */
static void    LCD_RTP1_Seed(void);
static void    LCD_RTP2_Seed(void);
static void    LCD_Clock_Refresh(void);
static void    LCD_PushFeatureState(uint8_t index);
static void    LCD_PushEditValue(uint8_t index);
static void    LCD_Memory_SeedPage(void);
static void    LCD_Memory_UpdateCursor(void);
static void    LCD_HandleCircularAnim(void);
static uint8_t MapReverse(uint8_t cmd);
static void    LCD_ResetHomeTimer(void);
static uint8_t LCD_FeatureEnabled(uint8_t index);
static uint8_t LCD_DetailsItemAllowed(uint8_t item);
static uint8_t LCD_DetailsHasAnyAllowedItem(void);
static void    LCD_DetailsSelectFirstAllowedItem(void);
static void    LCD_DetailsMoveCursor(void);
static uint8_t LCD_PageAllowed(uint8_t page);
static uint8_t LCD_SetPageIfAllowed(uint8_t page);
static void    LCD_CloseCurrentPageIfDisabled(void);

/* ======================================================================
 * Init
 * ====================================================================== */
void LCD_HMI_Init(UART_HandleTypeDef *huart)
{
    lcd_uart    = huart;
    currentPage = 0xFF;
    uiState     = UI_HOME;
    parentState = UI_HOME;
    lcdLocked   = 0;
    lcdPowerOn  = 1;
    homeActive  = 0;

    /* All feature pages are enabled by default at init.
     * The featureEnable[] array is already initialised to all-1 above,
     * but reset explicitly here in case Init() is called more than once. */
    for(uint8_t i = 0; i < ACCEPT_FIELD_COUNT; i++)
        featureEnable[i] = 1;
}

/* ======================================================================
 * Low-level DWIN primitives
 * ====================================================================== */
void lcd_set_page(uint8_t page)
{
    if(lcd_uart == NULL)    return;
    if(page == currentPage) return;
    uint8_t pkt[10] = {0x5A,0xA5,0x07,0x82,0x00,0x84,0x5A,0x01,0x00,page};
    HAL_UART_Transmit(lcd_uart, pkt, 10, 100);
    currentPage = page;
}

/* Force a page switch even if currentPage already matches.
 * Used when the DWIN may have been reset (power cycle, startup).        */
static void lcd_force_page(uint8_t page)
{
    currentPage = 0xFF;
    lcd_set_page(page);
}

void lcd_set_icon(uint8_t vp, uint8_t value)
{
    if(lcd_uart == NULL) return;
    uint8_t pkt[8] = {0x5A,0xA5,0x05,0x82,0x00,vp,0x00,value};
    HAL_UART_Transmit(lcd_uart, pkt, 8, 100);
}

void lcd_set_state_value(uint8_t vp, uint8_t value)
{
    lcd_set_icon(vp, value);
}

void lcd_set_state_value16(uint8_t vp, uint16_t value)
{
    if(lcd_uart == NULL) return;
    uint8_t pkt[8] =
    {
        0x5A, 0xA5, 0x05, 0x82, 0x00, vp,
        (uint8_t)(value >> 8), (uint8_t)(value & 0xFF)
    };
    HAL_UART_Transmit(lcd_uart, pkt, 8, 100);
}

/* ======================================================================
 * Lock / Unlock
 * ====================================================================== */
void LCD_SetLock(uint8_t state)
{
    if(state)
    {
        /* ----- LOCK ----- */
        lcdLocked  = 1;
        homeActive = 0;
        animActive = 0;

        lcd_set_page(PAGE_LOCK);
        lcd_set_icon(IA_LOCK, IS_LOCKED);
    }
    else
    {
        /* ----- UNLOCK ----- */
        lcd_set_icon(IA_LOCK, IS_UNLOCKED);

        lcdLocked  = 0;
        uiState    = UI_HOME;

        /* FIX: reset home timer on unlock so a stale pre-lock
         * motor countdown does not fire the moment the screen unlocks. */
        homeActive = 0;
        homeTimer  = HAL_GetTick();

        lcd_set_page(PAGE_HOME);
    }
}

uint8_t LCD_IsLocked(void)  { return lcdLocked;  }
uint8_t LCD_IsPowerOn(void) { return lcdPowerOn; }

/* ======================================================================
 * Power
 * ------------------------------------------------------------------
 * Power ON sequence:
 *   1. Energise relay FIRST.
 *   2. Wait 100 ms for DWIN rail / UART receiver to stabilise.
 *   3. Run full startup logo sequence (pages 01 → 02 → 03).
 *   4. Land on HOME page.
 *
 * Power OFF sequence:
 *   1. Stop all pending activity and motor.
 *   2. Show Thank-You page (PAGE_POWER_OFF) for a visible moment.
 *   3. Wait 800 ms so the user actually sees it.
 *   4. Cut relay — DWIN loses power.
 * ====================================================================== */
void LCD_TogglePower(void)
{
    lcdPowerOn ^= 1;

    if(lcdPowerOn)
    {
        /* ---- POWER ON ---- */
        RELAY_SET(1);
        HAL_Delay(1000);          /* let DWIN power rail and UART settle    */
        LCD_ShowStartupSequence();
    }
    else
    {
        /* ---- POWER OFF ---- */
        homeActive = 0;
        animActive = 0;
        ACTION_SendCommand(0x00);       /* stop any active motor            */

        /* Show Thank-You page so user sees a clean shutdown               */
        lcd_force_page(PAGE_POWER_OFF);
        HAL_Delay(800);                 /* hold long enough to be readable  */

        RELAY_SET(0);                   /* cut power AFTER page is visible  */
    }
}

void LCD_ToggleReverse(void)
{
    reverseMode ^= 1;
}

/* ======================================================================
 * Startup
 * ------------------------------------------------------------------
 * Plays logo pages 01 → 02 → 03, then lands on HOME (or LOCK).
 * lcd_force_page() is used throughout so the stale currentPage cache
 * never silently suppresses a UART transmission.
 * ====================================================================== */
void LCD_ShowStartupSequence(void)
{
    homeActive = 0;
    animActive = 0;

    /* Logo / splash sequence */
    lcd_force_page(0x01); HAL_Delay(300);
    lcd_force_page(0x02); HAL_Delay(300);
    lcd_force_page(0x03); HAL_Delay(300);

    /* Land on the correct operational page */
    if(lcdLocked)
    {
        uiState = UI_HOME;
        lcd_force_page(PAGE_LOCK);
        lcd_set_icon(IA_LOCK, IS_LOCKED);
    }
    else
    {
        uiState = UI_HOME;
        lcd_force_page(PAGE_HOME);
    }
}

/* ======================================================================
 * Command reverse-mapping
 * ====================================================================== */
static uint8_t MapReverse(uint8_t cmd)
{
    if(!reverseMode) return cmd;
    switch(cmd)
    {
    case 0x01: return 0x02;  case 0x02: return 0x01;
    case 0x03: return 0x04;  case 0x04: return 0x03;
    case 0x05: return 0x06;  case 0x06: return 0x05;
    case 0x07: return 0x08;  case 0x08: return 0x07;
    case 0x09: return 0x10;  case 0x10: return 0x09;
    default: return cmd;
    }
}

/* ======================================================================
 * Private helpers
 * ====================================================================== */
static void LCD_ResetHomeTimer(void)
{
    homeActive = 1;
    homeTimer  = HAL_GetTick();
}

/* ======================================================================
 * Feature / page access helpers
 * ------------------------------------------------------------------
 * The GoBack+Accept page (UI_COMBO_FLEX) lets the user toggle each
 * feature flag ON or OFF at runtime. When a flag is OFF the
 * corresponding page is silently blocked (returns to HOME instead).
 *
 * Flag index → feature:
 *   0  Battery status page
 *   1  Gyro / tilt page
 *   2  Real-time positioning (RTP1 / RTP2 / GoBack+Height)
 *   3  Anti-collision setting page
 *   4  Slide position page
 *   5  Flex-floor page
 *   6  Reserved (blocked when off, no additional action)
 * ====================================================================== */
static uint8_t LCD_FeatureEnabled(uint8_t index)
{
    if(index >= ACCEPT_FIELD_COUNT) return 0;
    return featureEnable[index] ? 1u : 0u;
}

/* Maps a details-page cursor row index to its required feature flag. */
static uint8_t LCD_DetailsItemAllowed(uint8_t item)
{
    switch(item)
    {
        case 0: return LCD_FeatureEnabled(LCD_FEATURE_REALTIME);
        case 1: return LCD_FeatureEnabled(LCD_FEATURE_BATTERY);
        default: return 0;
    }
}

static uint8_t LCD_DetailsHasAnyAllowedItem(void)
{
    return (LCD_DetailsItemAllowed(0) || LCD_DetailsItemAllowed(1)) ? 1u : 0u;
}

static void LCD_DetailsSelectFirstAllowedItem(void)
{
    if     (LCD_DetailsItemAllowed(0)) details_cursor = 0;
    else if(LCD_DetailsItemAllowed(1)) details_cursor = 1;
    else                               details_cursor = 0;   /* fallback */
}

/* Move cursor up/down on the Details page, skipping disabled items. */
static void LCD_DetailsMoveCursor(void)
{
    if(LCD_DetailsItemAllowed(0) && LCD_DetailsItemAllowed(1))
    {
        details_cursor ^= 1;
    }
    else
    {
        /* Only one item available — keep cursor there. */
        LCD_DetailsSelectFirstAllowedItem();
    }
    lcd_set_icon(IA_DETAILS, details_cursor);
}

/* Returns 1 if the given page is currently allowed by the feature flags. */
static uint8_t LCD_PageAllowed(uint8_t page)
{
    switch(page)
    {
        case PAGE_BATTERY:
            return LCD_FeatureEnabled(LCD_FEATURE_BATTERY);

        case PAGE_RTP1:
        case PAGE_RTP2:
        case GOBACK_HEIGHT:
            return LCD_FeatureEnabled(LCD_FEATURE_REALTIME);

        case PAGE_TILT:
            return LCD_FeatureEnabled(LCD_FEATURE_GYRO);

        case PAGE_SETTING_ANTI:
            return LCD_FeatureEnabled(LCD_FEATURE_COLLISION);

        case PAGE_SLIDE:
            return LCD_FeatureEnabled(LCD_FEATURE_SLIDE);

        case PAGE_FLEX:
            return LCD_FeatureEnabled(LCD_FEATURE_FLEXFLOOR);

        default:
            return 1;   /* all other pages are always allowed */
    }
}

/* Attempt to switch to a page; if blocked, return to HOME instead.
 * Returns 1 on success, 0 if the page was blocked.                      */
static uint8_t LCD_SetPageIfAllowed(uint8_t page)
{
    if(!LCD_PageAllowed(page))
    {
        homeActive = 0;
        animActive = 0;
        uiState    = UI_HOME;
        lcd_set_page(PAGE_HOME);
        return 0;
    }
    lcd_set_page(page);
    return 1;
}

/* If the currently displayed page has just been disabled (e.g. user
 * toggled its feature flag off via the GoBack+Accept menu), kick the
 * UI back to HOME so the user is never stranded on a blocked page.      */
static void LCD_CloseCurrentPageIfDisabled(void)
{
    if(LCD_PageAllowed(currentPage)) return;

    homeActive = 0;
    animActive = 0;
    uiState    = UI_HOME;
    lcd_set_page(PAGE_HOME);
}

/* ======================================================================
 * Seed helpers
 * ====================================================================== */
static void LCD_RTP1_Seed(void)
{
    for(uint8_t i = 0; i < RTP1_FIELD_COUNT; i++)
        lcd_set_state_value16(rtp1VP[i], rtp1Val[i]);
}

static void LCD_RTP2_Seed(void)
{
    for(uint8_t i = 0; i < RTP2_FIELD_COUNT; i++)
        lcd_set_state_value16(rtp2VP[i], rtp2Val[i]);
}

static void LCD_Clock_Refresh(void)
{
    lcd_set_state_value16(VP_CLOCK_HOUR, clockHour);
    lcd_set_state_value16(VP_CLOCK_MIN,  clockMin);
}

static void LCD_PushFeatureState(uint8_t index)
{
    if(index >= ACCEPT_FIELD_COUNT) return;
    lcd_set_icon(featureVP[index],
                 featureEnable[index] ? ACCEPT_EN_ON : ACCEPT_EN_OFF);
}

static void LCD_PushEditValue(uint8_t index)
{
    if(index >= ACCEPT_EDIT_COUNT) return;
    lcd_set_state_value16(editVP[index], editVal[index]);
}

/* ======================================================================
 * LCD_Memory_UpdateCursor
 * ------------------------------------------------------------------
 * Writes BOTH IA_SEL_MEMORY and IA_MEM_MODE on every call.
 * Uses lcd_force_page() to survive DWIN silent resets.
 * ====================================================================== */
static void LCD_Memory_UpdateCursor(void)
{
    lcd_force_page(MOMORY);
    lcd_set_icon(IA_SEL_MEMORY, selMemory);
    lcd_set_icon(IA_MEM_MODE,
                 (memoryMode == MEM_MODE_SET) ? IS_MEM_SET : IS_MEM_RECALL);
}

/* ======================================================================
 * LCD_Memory_SeedPage
 * ------------------------------------------------------------------
 * Full seed — called ONLY on page open.
 * Also writes IA_MEM_MODE so the mode indicator is correct on open.
 * Uses lcd_force_page() to guarantee the page switch fires.
 * ====================================================================== */
static void LCD_Memory_SeedPage(void)
{
    lcd_force_page(MOMORY);
    lcd_set_icon(IA_SEL_MEMORY, selMemory);
    lcd_set_icon(IA_MEM_MODE,
                 (memoryMode == MEM_MODE_SET) ? IS_MEM_SET : IS_MEM_RECALL);

    for(uint8_t i = 0; i < MEMORY_SLOT_COUNT; i++)
        lcd_set_icon(memSlotVP[i], memSlotSaved[i]);
}

/* ======================================================================
 * Main key handler
 * ====================================================================== */
void LCD_HandleKey(uint8_t cmd)
{
    if(!lcdPowerOn || lcdLocked) return;

    cmd = MapReverse(cmd);
    valueChanged = 0;

    /* ==================================================================
     * GLOBAL: GO BACK
     * ================================================================== */
    if(cmd == CMD_GOBACK)
    {
        homeActive = 0;
        switch(uiState)
        {
            case UI_DETAILS:
                uiState = UI_HOME;
                lcd_set_page(PAGE_HOME);
                break;

            case UI_RTP1:
            case UI_RTP2:
            case UI_BATTERY:
                if(parentState == UI_SETTING)
                {
                    uiState = UI_SETTING;
                    lcd_set_page(PAGE_SETTING);
                    lcd_set_icon(IA_SETTING, selSetting);
                }
                else
                {
                    uiState = UI_DETAILS;
                    lcd_set_page(DETAILS);
                    lcd_set_icon(IA_DETAILS, details_cursor);
                }
                break;

            case UI_SETTING_CLOCK:
            case UI_SETTING_ANTI:
                uiState = UI_SETTING;
                lcd_set_page(PAGE_SETTING);
                lcd_set_icon(IA_SETTING, selSetting);
                break;

            case UI_SETTING:
                uiState = UI_HOME;
                lcd_set_page(PAGE_HOME);
                break;

            case UI_MEMORY:
            case UI_COMBO_FLEX:
            case UI_COMBO_HEIGHT:
            case UI_COMBO_ACCEPT:
                uiState = UI_HOME;
                lcd_set_page(PAGE_HOME);
                break;

            default:
                uiState = UI_HOME;
                lcd_set_page(PAGE_HOME);
                break;
        }
        return;
    }

    /* ==================================================================
     * HOME
     * ================================================================== */
    if(uiState == UI_HOME)
    {
        if(cmd == CMD_DETAILS)
        {
            /* Details page only opens if at least one item is enabled. */
            if(!LCD_DetailsHasAnyAllowedItem()) return;

            uiState = UI_DETAILS;
            LCD_DetailsSelectFirstAllowedItem();
            lcd_set_page(DETAILS);
            lcd_set_icon(IA_DETAILS, details_cursor);
            return;
        }
    }

    /* ==================================================================
     * DETAILS — IA=0x52, IS 0=RTP  1=Battery
     * ================================================================== */
    if(uiState == UI_DETAILS)
    {
        if(cmd == CMD_CURSOR_UP || cmd == CMD_CURSOR_DOWN)
        {
            LCD_DetailsMoveCursor();
        }
        else if(cmd == CMD_ACCEPT)
        {
            if(!LCD_DetailsItemAllowed(details_cursor)) return;

            if(details_cursor == 0)
            {
                uiState     = UI_RTP1;
                parentState = UI_DETAILS;
                selRTP1     = 0;
                if(!LCD_SetPageIfAllowed(PAGE_RTP1)) return;
                lcd_set_icon(IA_RTP1_CURSOR, selRTP1);
                LCD_RTP1_Seed();
            }
            else
            {
                uiState     = UI_BATTERY;
                parentState = UI_DETAILS;
                if(!LCD_SetPageIfAllowed(PAGE_BATTERY)) return;
                lcd_set_icon(IA_BATTERY, batteryLevel);
            }
        }
        return;
    }

    /* ==================================================================
     * RTP PAGE 1 — IA=0x53, IS 0-7
     * ================================================================== */
    if(uiState == UI_RTP1)
    {
        if(cmd == CMD_CURSOR_UP)
        {
            if(selRTP1 > 0) { selRTP1--; lcd_set_icon(IA_RTP1_CURSOR, selRTP1); }
        }
        else if(cmd == CMD_CURSOR_DOWN)
        {
            if(selRTP1 < (RTP1_FIELD_COUNT - 1))
            {
                selRTP1++;
                lcd_set_icon(IA_RTP1_CURSOR, selRTP1);
            }
            else
            {
                uiState = UI_RTP2; selRTP2 = 0;
                if(!LCD_SetPageIfAllowed(PAGE_RTP2)) return;
                lcd_set_icon(IA_RTP2_CURSOR, selRTP2);
            }
        }
        return;
    }

    /* ==================================================================
     * RTP PAGE 2 — IA=0x54, IS 0-3
     * ================================================================== */
    if(uiState == UI_RTP2)
    {
        if(cmd == CMD_CURSOR_UP)
        {
            if(selRTP2 > 0) { selRTP2--; lcd_set_icon(IA_RTP2_CURSOR, selRTP2); }
            else
            {
                uiState = UI_RTP1; selRTP1 = RTP1_FIELD_COUNT - 1;
                if(!LCD_SetPageIfAllowed(PAGE_RTP1)) return;
                lcd_set_icon(IA_RTP1_CURSOR, selRTP1);
                LCD_RTP1_Seed();
            }
        }
        else if(cmd == CMD_CURSOR_DOWN)
        {
            if(selRTP2 < (RTP2_FIELD_COUNT - 1))
            {
                selRTP2++;
                lcd_set_icon(IA_RTP2_CURSOR, selRTP2);
            }
        }
        return;
    }

    /* ==================================================================
     * BATTERY — read-only
     * ================================================================== */
    if(uiState == UI_BATTERY) { return; }

    /* ==================================================================
     * SETTING SELECTOR — IA=0x56, IS 0-3
     * ================================================================== */
    if(uiState == UI_SETTING)
    {
        if(cmd == CMD_CURSOR_UP)
        {
            if(selSetting > 0) selSetting--;
            lcd_set_icon(IA_SETTING, selSetting);
        }
        else if(cmd == CMD_CURSOR_DOWN)
        {
            if(selSetting < (SETTING_ITEM_COUNT - 1)) selSetting++;
            lcd_set_icon(IA_SETTING, selSetting);
        }
        else if(cmd == CMD_ACCEPT)
        {
            switch(selSetting)
            {
                case 0:
                    uiState = UI_SETTING_CLOCK; parentState = UI_SETTING;
                    clockCursor = 0;
                    lcd_set_page(PAGE_SETTING_CLOCK);
                    lcd_set_icon(IA_CLOCK_CURSOR, clockCursor);
                    LCD_Clock_Refresh();
                    break;
                case 1:
                    /* Battery page gated by BATTERY feature flag */
                    if(!LCD_FeatureEnabled(LCD_FEATURE_BATTERY)) break;
                    uiState = UI_BATTERY; parentState = UI_SETTING;
                    if(!LCD_SetPageIfAllowed(PAGE_BATTERY)) return;
                    lcd_set_icon(IA_BATTERY, batteryLevel);
                    break;
                case 2:
                    /* Anti-collision page gated by COLLISION feature flag */
                    if(!LCD_FeatureEnabled(LCD_FEATURE_COLLISION)) break;
                    uiState = UI_SETTING_ANTI; parentState = UI_SETTING;
                    if(!LCD_SetPageIfAllowed(PAGE_SETTING_ANTI)) return;
                    break;
                default: break;
            }
        }
        return;
    }

    /* ==================================================================
     * SETTING → CLOCK — PA=22, IA=0x57
     * ================================================================== */
    if(uiState == UI_SETTING_CLOCK)
    {
        if(cmd == CMD_CURSOR_UP || cmd == CMD_CURSOR_DOWN)
        {
            clockCursor ^= 1;
            lcd_set_icon(IA_CLOCK_CURSOR, clockCursor);
        }
        else if(cmd == CMD_CURSOR_RIGHT)
        {
            if(clockCursor == 0) { clockHour++; if(clockHour > 23) clockHour = 0; }
            else                 { clockMin++;  if(clockMin  > 59) clockMin  = 0; }
            LCD_Clock_Refresh();
        }
        else if(cmd == CMD_CURSOR_LEFT)
        {
            if(clockCursor == 0) { if(clockHour == 0) clockHour = 23; else clockHour--; }
            else                 { if(clockMin  == 0) clockMin  = 59; else clockMin--;  }
            LCD_Clock_Refresh();
        }
        else if(cmd == CMD_ACCEPT)
        {
            /* TODO: ACTION_SendClock(clockHour, clockMin); */
        }
        return;
    }

    /* ==================================================================
     * SETTING → ANTI — PA=23, IA=0x58
     * ================================================================== */
    if(uiState == UI_SETTING_ANTI) { (void)cmd; return; }

    /* ==================================================================
     * COMBO: GoBack+Accept — PA=13, IA=0x48, IS 0-6
     * User can toggle any feature ON (right/accept) or OFF (left).
     * After each toggle LCD_CloseCurrentPageIfDisabled() is called so
     * if the user disables the page they are currently on, the HMI
     * immediately returns to HOME.
     * ================================================================== */
    if(uiState == UI_COMBO_FLEX)
    {
        if(cmd == CMD_CURSOR_UP)
        {
            CIRC_DEC(selFeature, ACCEPT_FIELD_COUNT - 1);
            lcd_set_icon(IA_SEL_COMBO_FLEX, selFeature);
        }
        else if(cmd == CMD_CURSOR_DOWN)
        {
            CIRC_INC(selFeature, ACCEPT_FIELD_COUNT - 1);
            lcd_set_icon(IA_SEL_COMBO_FLEX, selFeature);
        }
        else if(cmd == CMD_CURSOR_RIGHT || cmd == CMD_ACCEPT)
        {
            featureEnable[selFeature] ^= 1;
            LCD_PushFeatureState(selFeature);
            /* If the toggled flag disabled the currently active page, go HOME */
            LCD_CloseCurrentPageIfDisabled();
        }
        else if(cmd == CMD_CURSOR_LEFT)
        {
            featureEnable[selFeature] = 0;
            LCD_PushFeatureState(selFeature);
            LCD_CloseCurrentPageIfDisabled();
        }
        return;
    }

    /* ==================================================================
     * COMBO: GoBack+HeightDn — PA=14, IA=0x49, IS 0-6
     * ================================================================== */
    if(uiState == UI_COMBO_HEIGHT)
    {
        if(cmd == CMD_CURSOR_UP)
        {
            if(selComboHeight > 0) selComboHeight--;
            lcd_set_icon(IA_SEL_COMBO_HEIGHT, selComboHeight);
        }
        else if(cmd == CMD_CURSOR_DOWN)
        {
            if(selComboHeight < 6) selComboHeight++;
            lcd_set_icon(IA_SEL_COMBO_HEIGHT, selComboHeight);
        }
        else if((cmd == CMD_CURSOR_RIGHT || cmd == CMD_CURSOR_LEFT)
                 && comboHeightEdit[selComboHeight])
        {
            if(cmd == CMD_CURSOR_RIGHT)
                { if(comboHeightVal[selComboHeight] < 2500) comboHeightVal[selComboHeight]++; }
            else
                { if(comboHeightVal[selComboHeight] > 0)    comboHeightVal[selComboHeight]--; }
            lcd_set_state_value16(comboHeightVP[selComboHeight],
                                  comboHeightVal[selComboHeight]);
            if(selComboHeight == 1)
            {
                comboHeightVal[2] = comboHeightVal[0] + comboHeightVal[1];
                lcd_set_state_value16(comboHeightVP[2], comboHeightVal[2]);
            }
            else if(selComboHeight == 4)
            {
                comboHeightVal[5] = comboHeightVal[3] + comboHeightVal[4];
                lcd_set_state_value16(comboHeightVP[5], comboHeightVal[5]);
            }
        }
        return;
    }

    /* ==================================================================
     * COMBO: GoBack+FlexUp — PA=15, IA=0x4A, IS 0-7
     * ================================================================== */
    if(uiState == UI_COMBO_ACCEPT)
    {
        if(cmd == CMD_CURSOR_UP)
        {
            if(selComboAccept > 0) selComboAccept--;
            lcd_set_icon(IA_SEL_COMBO_ACCEPT, selComboAccept);
        }
        else if(cmd == CMD_CURSOR_DOWN)
        {
            if(selComboAccept < (ACCEPT_EDIT_COUNT - 1)) selComboAccept++;
            lcd_set_icon(IA_SEL_COMBO_ACCEPT, selComboAccept);
        }
        else if(cmd == CMD_CURSOR_RIGHT)
        {
            if(editVal[selComboAccept] < ACCEPT_EDIT_MAX)
                editVal[selComboAccept]++;
            LCD_PushEditValue(selComboAccept);
        }
        else if(cmd == CMD_CURSOR_LEFT)
        {
            if(editVal[selComboAccept] > 0)
                editVal[selComboAccept]--;
            LCD_PushEditValue(selComboAccept);
        }
        return;
    }

    /* ==================================================================
     * MEMORY PAGE — PA=16 (0x10), IA=0x30, IS 0-4
     * ================================================================== */
    if(uiState == UI_MEMORY)
    {
        if(cmd == CMD_CURSOR_UP)
        {
            CIRC_DEC(selMemory, MEMORY_SLOT_COUNT - 1);
            LCD_Memory_UpdateCursor();
        }
        else if(cmd == CMD_CURSOR_DOWN)
        {
            CIRC_INC(selMemory, MEMORY_SLOT_COUNT - 1);
            LCD_Memory_UpdateCursor();
        }
        else if(cmd == CMD_CURSOR_RIGHT)
        {
            memoryMode = MEM_MODE_SET;
            LCD_Memory_UpdateCursor();
        }
        else if(cmd == CMD_CURSOR_LEFT)
        {
            memoryMode = MEM_MODE_RECALL;
            LCD_Memory_UpdateCursor();
        }
        else if(cmd == CMD_ACCEPT)
        {
            memSlotSaved[selMemory] ^= 1;
            lcd_set_icon(memSlotVP[selMemory], memSlotSaved[selMemory]);
            ACTION_SendMemory(selMemory + 1, memoryMode);
        }
        return;
    }

    /* ==================================================================
     * MOTOR COMMANDS
     * ================================================================== */
    switch(cmd)
    {
        case 0x01:
            STEP_INC(heightState, 4); lcd_set_page(PAGE_HEIGHT);
            if(valueChanged) lcd_set_icon(IA_HEIGHT, heightState);
            LCD_ResetHomeTimer(); break;
        case 0x02:
            STEP_DEC(heightState, 4); lcd_set_page(PAGE_HEIGHT);
            if(valueChanged) lcd_set_icon(IA_HEIGHT, heightState);
            LCD_ResetHomeTimer(); break;
        case 0x03:
            STEP_INC(trendState, 4); lcd_set_page(PAGE_TREND);
            if(valueChanged) lcd_set_icon(IA_TREND, trendState);
            LCD_ResetHomeTimer(); break;
        case 0x04:
            STEP_DEC(trendState, 4); lcd_set_page(PAGE_TREND);
            if(valueChanged) lcd_set_icon(IA_TREND, trendState);
            LCD_ResetHomeTimer(); break;
        case 0x05:
            STEP_INC(backState, 4); lcd_set_page(PAGE_BACK);
            if(valueChanged) lcd_set_icon(IA_BACK, backState);
            LCD_ResetHomeTimer(); break;
        case 0x06:
            STEP_DEC(backState, 4); lcd_set_page(PAGE_BACK);
            if(valueChanged) lcd_set_icon(IA_BACK, backState);
            LCD_ResetHomeTimer(); break;
        case 0x07:
            STEP_DEC(tiltState, 4);
            if(!LCD_SetPageIfAllowed(PAGE_TILT)) break;
            if(valueChanged) lcd_set_icon(IA_TILT, tiltState);
            LCD_ResetHomeTimer(); break;
        case 0x08:
            STEP_INC(tiltState, 4);
            if(!LCD_SetPageIfAllowed(PAGE_TILT)) break;
            if(valueChanged) lcd_set_icon(IA_TILT, tiltState);
            LCD_ResetHomeTimer(); break;
        case 0x09:
            STEP_INC(slideState, 2);
            if(!LCD_SetPageIfAllowed(PAGE_SLIDE)) break;
            if(valueChanged) lcd_set_icon(IA_SLIDE, slideState);
            LCD_ResetHomeTimer(); break;
        case 0x10:
            STEP_DEC(slideState, 2);
            if(!LCD_SetPageIfAllowed(PAGE_SLIDE)) break;
            if(valueChanged) lcd_set_icon(IA_SLIDE, slideState);
            LCD_ResetHomeTimer(); break;
        case 0x11:
            STEP_DEC(flexState, 1);
            if(!LCD_SetPageIfAllowed(PAGE_FLEX)) break;
            if(valueChanged) lcd_set_icon(IA_FLEX, flexState);
            LCD_ResetHomeTimer(); break;
        case 0x12:
            STEP_INC(flexState, 1);
            if(!LCD_SetPageIfAllowed(PAGE_FLEX)) break;
            if(valueChanged) lcd_set_icon(IA_FLEX, flexState);
            LCD_ResetHomeTimer(); break;
        default: break;
    }
}

/* ======================================================================
 * Animation
 * ====================================================================== */
void LCD_StartCircularAnim(uint8_t cmd)
{
    if(!lcdPowerOn || lcdLocked) return;
    animActive = 1; animCmd = cmd; animTick = HAL_GetTick();
}

void LCD_StopCircularAnim(void) { animActive = 0; }

static void LCD_HandleCircularAnim(void)
{
    if(!animActive) return;
    if((HAL_GetTick() - animTick) < 250) return;
    animTick     = HAL_GetTick();
    animStepping = 1;
    LCD_HandleKey(animCmd);
    animStepping = 0;
}

/* ======================================================================
 * Task  (call from main loop)
 * ====================================================================== */
void LCD_Task(void)
{
    LCD_HandleCircularAnim();

    if(homeActive && !Sensor_IsActive() && lcdPowerOn && !lcdLocked)
    {
        if((HAL_GetTick() - homeTimer) >= HOME_RETURN_TIME)
        {
            uiState    = UI_HOME;
            homeActive = 0;
            lcd_set_page(PAGE_HOME);
        }
    }
}

/* ======================================================================
 * Sensor icon page
 * ====================================================================== */
void LCD_UpdateSensorIcon(uint8_t iconAddr, uint8_t state)
{
    if(!lcdPowerOn) return;
    lcd_set_icon(iconAddr, state);
}

/* ======================================================================
 * LCD_ShowSensorPage / LCD_ShowMemoryPage
 * ====================================================================== */
void LCD_ShowSensorPage(uint8_t page)
{
    if(!lcdPowerOn || lcdLocked) return;

    switch(page)
    {
        case GOBACK_FLEX:
            uiState    = UI_COMBO_FLEX;
            selFeature = 0;
            lcd_set_page(page);
            lcd_set_icon(IA_SEL_COMBO_FLEX, 0);
            for(uint8_t i = 0; i < ACCEPT_FIELD_COUNT; i++)
                LCD_PushFeatureState(i);
            break;

        case GOBACK_HEIGHT:
            if(!LCD_PageAllowed(GOBACK_HEIGHT)) return;
            uiState        = UI_COMBO_HEIGHT;
            selComboHeight = 0;
            comboHeightVal[0] = Sensor_GetActualHeight();
            comboHeightVal[3] = Sensor_GetSlide();
            comboHeightVal[2] = comboHeightVal[0] + comboHeightVal[1];
            comboHeightVal[5] = comboHeightVal[3] + comboHeightVal[4];
            if(!LCD_SetPageIfAllowed(page)) return;
            lcd_set_icon(IA_SEL_COMBO_HEIGHT, 0);
            for(uint8_t i = 0; i < 7; i++)
                lcd_set_state_value16(comboHeightVP[i], comboHeightVal[i]);
            break;

        case GOBACK_ACCEPT:
            uiState        = UI_COMBO_ACCEPT;
            selComboAccept = 0;
            lcd_set_page(page);
            lcd_set_icon(IA_SEL_COMBO_ACCEPT, 0);
            for(uint8_t i = 0; i < ACCEPT_EDIT_COUNT; i++)
                LCD_PushEditValue(i);
            break;

        case MOMORY:
            uiState    = UI_MEMORY;
            selMemory  = 0;
            memoryMode = MEM_MODE_RECALL;
            LCD_Memory_SeedPage();
            break;

        case PAGE_SETTING:
            uiState    = UI_SETTING;
            selSetting = 0;
            lcd_set_page(page);
            lcd_set_icon(IA_SETTING, 0);
            break;

        default:
            (void)LCD_SetPageIfAllowed(page);
            break;
    }
}

void LCD_ShowMemoryPage(uint8_t mode)
{
    if(!lcdPowerOn || lcdLocked) return;
    uiState    = UI_MEMORY;
    selMemory  = 0;
    memoryMode = mode;
    LCD_Memory_SeedPage();
}

/* ======================================================================
 * GoBack+HeightDn live sensor update
 * ====================================================================== */
void LCD_RefreshHeightPage(uint8_t actual_height, uint8_t slide)
{
    comboHeightVal[0] = actual_height;
    comboHeightVal[3] = slide;
    comboHeightVal[2] = comboHeightVal[0] + comboHeightVal[1];
    comboHeightVal[5] = comboHeightVal[3] + comboHeightVal[4];
    if(uiState != UI_COMBO_HEIGHT) return;
    lcd_set_state_value16(comboHeightVP[0], comboHeightVal[0]);
    lcd_set_state_value16(comboHeightVP[2], comboHeightVal[2]);
    lcd_set_state_value16(comboHeightVP[3], comboHeightVal[3]);
    lcd_set_state_value16(comboHeightVP[5], comboHeightVal[5]);
}

/* ======================================================================
 * GoBack+Accept API
 * External callers (e.g. action_comm) can push individual flag changes
 * or bulk-refresh all flags.  After any change the HMI closes any page
 * that just became disabled.
 * ====================================================================== */
void LCD_UpdateAcceptEnable(uint8_t index, uint8_t enabled)
{
    if(index >= ACCEPT_FIELD_COUNT) return;
    featureEnable[index] = enabled ? 1 : 0;
    if(uiState == UI_COMBO_FLEX) LCD_PushFeatureState(index);
    LCD_CloseCurrentPageIfDisabled();
}

void LCD_RefreshAcceptPage(const uint8_t *states)
{
    if(!states) return;
    for(uint8_t i = 0; i < ACCEPT_FIELD_COUNT; i++)
        featureEnable[i] = states[i] ? 1 : 0;
    if(uiState == UI_COMBO_FLEX)
        for(uint8_t i = 0; i < ACCEPT_FIELD_COUNT; i++)
            LCD_PushFeatureState(i);
    LCD_CloseCurrentPageIfDisabled();
}

/* ======================================================================
 * GoBack+FlexUp API
 * ====================================================================== */
void LCD_UpdateEditValue(uint8_t index, uint16_t value)
{
    if(index >= ACCEPT_EDIT_COUNT) return;
    if(value > ACCEPT_EDIT_MAX) value = ACCEPT_EDIT_MAX;
    editVal[index] = value;
    if(uiState == UI_COMBO_ACCEPT) LCD_PushEditValue(index);
}

void LCD_RefreshEditPage(const uint16_t *vals)
{
    if(!vals) return;
    for(uint8_t i = 0; i < ACCEPT_EDIT_COUNT; i++)
        editVal[i] = (vals[i] > ACCEPT_EDIT_MAX) ? ACCEPT_EDIT_MAX : vals[i];
    if(uiState != UI_COMBO_ACCEPT) return;
    for(uint8_t i = 0; i < ACCEPT_EDIT_COUNT; i++) LCD_PushEditValue(i);
}

/* ======================================================================
 * RTP live-update API
 * ====================================================================== */
void LCD_PushRTP1Values(uint8_t height, uint8_t slide,
                        uint8_t back_up, uint8_t back_dn,
                        uint8_t tilt_l,  uint8_t tilt_r,
                        uint8_t trend,   uint8_t rev_trend)
{
    rtp1Val[0]=height;   rtp1Val[1]=slide;    rtp1Val[2]=back_up;
    rtp1Val[3]=back_dn;  rtp1Val[4]=tilt_l;   rtp1Val[5]=tilt_r;
    rtp1Val[6]=trend;    rtp1Val[7]=rev_trend;
    if(uiState == UI_RTP1) LCD_RTP1_Seed();
}

void LCD_PushRTP2Values(uint8_t leg1_up, uint8_t leg1_dn,
                        uint8_t leg2_up, uint8_t leg2_dn)
{
    rtp2Val[0]=leg1_up; rtp2Val[1]=leg1_dn;
    rtp2Val[2]=leg2_up; rtp2Val[3]=leg2_dn;
    if(uiState == UI_RTP2) LCD_RTP2_Seed();
}

/* ======================================================================
 * Battery API
 * ====================================================================== */
void LCD_UpdateBattery(uint8_t level)
{
    if(level > 4) level = 4;
    batteryLevel = level;
    if(uiState == UI_BATTERY) lcd_set_icon(IA_BATTERY, batteryLevel);
}

/* ======================================================================
 * Memory slot saved-state API
 * ====================================================================== */
void LCD_UpdateMemorySlotState(uint8_t slot, uint8_t saved)
{
    if(slot < 1 || slot > MEMORY_SLOT_COUNT) return;
    memSlotSaved[slot - 1] = saved ? 1 : 0;
    if(uiState == UI_MEMORY)
        lcd_set_icon(memSlotVP[slot - 1], memSlotSaved[slot - 1]);
}
