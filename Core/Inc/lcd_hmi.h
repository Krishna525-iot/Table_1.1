#ifndef LCD_HMI_H
#define LCD_HMI_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* ======================================================================
 * DWIN page addresses  (PA)  — from spec table
 * ====================================================================== */
#define PAGE_HOME           0x04
#define PAGE_LOCK           0x0C   /* PA 12 — Lock/Unlock page                */
#define PAGE_POWER_OFF      0x00   /* Adjust if HMI has a dedicated power page */
#define DETAILS             0x11   /* PA 17 — Details selector                 */
#define PAGE_HEIGHT         0x07   /* PA  7 — Height motor page                */
#define PAGE_FLEX           0x05   /* PA  5 — Flex motor page                  */
#define PAGE_BACK           0x09   /* PA  9 — Back motor page                  */
#define PAGE_TILT           0x0B   /* PA 11 — Tilt motor page                  */
#define PAGE_SLIDE          0x0A   /* PA 10 — Slide motor page                 */
#define PAGE_TREND          0x08   /* PA  8 — Trend motor page                 */
#define PAGE_RTP1           0x12   /* PA 18 — Real-Time Positioning page 1     */
#define PAGE_RTP2           0x13   /* PA 19 — Real-Time Positioning page 2     */
#define PAGE_BATTERY        0x14   /* PA 20 — Battery status page              */
#define PAGE_SETTING        0x15   /* PA 21 — Setting selector page            */
#define PAGE_SETTING_CLOCK  0x16   /* PA 22 — Setting → Clock                  */
#define PAGE_SETTING_ANTI   0x17   /* PA 23 — Setting → Anti-collision         */
#define GOBACK_FLEX         0x0D   /* PA 13 — GoBack+Accept  (feature toggles) */
#define GOBACK_HEIGHT       0x0E   /* PA 14 — GoBack+HeightDn                  */
#define GOBACK_ACCEPT       0x0F   /* PA 15 — GoBack+FlexUp  (8 edit values)   */
#define MOMORY              0x10   /* PA 16 — Memory page                      */

/* FIX: PAGE_SENSOR_1 was a private local define (0x0D) inside sensor_manager.c.
 * It is identical to GOBACK_FLEX (both PA=0x0D) — Sensor_SetActive() calls
 * LCD_ShowSensorPage(PAGE_SENSOR_1) which intentionally hits the GOBACK_FLEX
 * case in the switch.  Centralised here as an alias so sensor_manager.c can
 * #include "lcd_hmi.h" and use this name instead of a magic literal.         */
#define PAGE_SENSOR_1       GOBACK_FLEX   /* PA 13 — sensor debug page = GOBACK_FLEX */

/* ======================================================================
 * Icon addresses  (IA) and Icon State values  (IS) — from spec table
 * ====================================================================== */

/* Motor pages */
#define IA_HEIGHT           0x42   /* IS 0-4  */
#define IA_FLEX             0x40   /* IS 0-1  */
#define IA_BACK             0x44   /* IS 0-4  */
#define IA_TILT             0x46   /* IS 0-4  */
#define IA_SLIDE            0x45   /* IS 0-2  */
#define IA_TREND            0x43   /* IS 0-4  */

/* Lock page — PA=12, IA=0x47, IS 0-1
 *   IS_UNLOCKED (0): written briefly on PAGE_LOCK before switching to HOME
 *   IS_LOCKED   (1): written when the display is locked                    */
#define IA_LOCK             0x47
#define IS_UNLOCKED         0x00
#define IS_LOCKED           0x01

/* Navigation cursors */
/* WARNING: IA_DETAILS (0x52) shares the same VP address as IA_SENSOR_BACK_UP
 * (also 0x52, defined locally in sensor_manager.c).  Both live on different
 * DWIN pages (DETAILS=0x11 vs PAGE_SENSOR_1=0x0D) so no visual conflict
 * exists today, but the DGUS project should assign distinct VPs in the next
 * hardware revision to eliminate the ambiguity.                             */
#define IA_DETAILS          0x52   /* PA 17  IS 0=RTP  1=Battery               */
#define IA_RTP1_CURSOR      0x53   /* PA 18  IS 0-7                            */
#define IA_RTP2_CURSOR      0x54   /* PA 19  IS 0-3                            */
#define IA_BATTERY          0x55   /* PA 20  IS 0-4                            */
#define IA_SETTING          0x56   /* PA 21  IS 0-3                            */
#define IA_CLOCK_CURSOR     0x57   /* PA 22  IS 0-1 (0=hour, 1=minute)         */
#define IA_ANTI_CURSOR      0x58   /* PA 23  anti-collision page               */

/* Combo page cursors */
#define IA_SEL_COMBO_FLEX   0x48   /* PA 13  IS 0-6                            */
#define IA_SEL_COMBO_HEIGHT 0x49   /* PA 14  IS 0-6                            */
#define IA_SEL_COMBO_ACCEPT 0x4A   /* PA 15  IS 0-7                            */

/* Memory page — PA=16 (0x10)
 *
 * IA_SEL_MEMORY (0x30): cursor icon.  A single lcd_set_icon(0x30, N) moves
 * the DWIN highlight rectangle to slot N+1.  IS=0→Slot1 … IS=4→Slot5.
 *
 * IA_MEM_MODE   (0x31): SET/RECALL mode indicator icon.
 *   IS_MEM_RECALL (0) — display shows RECALL mode
 *   IS_MEM_SET    (1) — display shows SET mode
 *
 * FIX: IA_MEM_MODE was missing from this header entirely.  lcd_hmi.c was
 * never sending the mode icon write, so LEFT/RIGHT key presses on the memory
 * page had no visible effect on the DWIN.  Added here and wired into both
 * LCD_Memory_UpdateCursor() and LCD_Memory_SeedPage() in lcd_hmi.c.
 *
 * Verify IA_MEM_MODE=0x31 against your DGUS project .bin — change if the
 * designer used a different VP address for the SET/RECALL indicator widget. */
#define IA_SEL_MEMORY       0x4B   /* IS 0-4 — slot cursor                     */
#define IA_MEM_MODE         0x31   /* IS 0-1 — SET/RECALL mode indicator        */
#define IS_MEM_RECALL       0x00   /* IS value → RECALL mode displayed          */
#define IS_MEM_SET          0x01   /* IS value → SET mode displayed             */

/* ======================================================================
 * Memory slot data VPs (16-bit) — from spec table value_id column
 * Updated from 0x81-0x85 → 0x9A-0x9E per hardware spec revision.
 *   0x00 = slot empty (no position saved)
 *   0x01 = slot has a saved position
 * ====================================================================== */
#define VP_MEM_SLOT1        0x9A
#define VP_MEM_SLOT2        0x9B
#define VP_MEM_SLOT3        0x9C
#define VP_MEM_SLOT4        0x9D
#define VP_MEM_SLOT5        0x9E
/* NOTE: VP_MEM_MODE (was 0x9F) removed — it was dead code.  The mode state
 * is communicated to the DWIN exclusively via the icon write to IA_MEM_MODE
 * (0x31), not via a 16-bit VP register.  Keeping a VP define for it created
 * a false impression that a separate VP write was needed.                   */

/* ======================================================================
 * VP (variable pointer) — 16-bit data registers
 * ====================================================================== */

/* RTP page 1 — value_id from spec table */
#define VP_RTP_HEIGHT       0x8A
#define VP_RTP_SLIDE        0x8B
#define VP_RTP_BACK_UP      0x8C
#define VP_RTP_BACK_DN      0x8D
#define VP_RTP_TILT_L       0x8E
#define VP_RTP_TILT_R       0x8F
#define VP_RTP_TREND        0x90
#define VP_RTP_REV          0x91

/* RTP page 2 — value_id from spec table */
#define VP_RTP2_LEG1_UP     0x92
#define VP_RTP2_LEG1_DN     0x93
#define VP_RTP2_LEG2_UP     0x94
#define VP_RTP2_LEG2_DN     0x95

/* GoBack+Accept feature toggle VPs */
#define VP_ACC_BATTERY      0x7A
#define VP_ACC_GYRO         0x7B
#define VP_ACC_REALTIME     0x7C
#define VP_ACC_COLLISION    0x7D
#define VP_ACC_SLIDE        0x7E
#define VP_ACC_FLEXFLOOR    0x7F
#define VP_ACC_RESERVED     0x80

/* Clock setting VPs */
#define VP_CLOCK_HOUR       0x60
#define VP_CLOCK_MIN        0x61

/* ======================================================================
 * Counts / limits
 * ====================================================================== */
#define RTP1_FIELD_COUNT    8
#define RTP2_FIELD_COUNT    4
#define ACCEPT_FIELD_COUNT  7
#define ACCEPT_EDIT_COUNT   8
#define ACCEPT_EDIT_MAX     2500
#define ACCEPT_EN_ON        0x01
#define ACCEPT_EN_OFF       0x00
#define SETTING_ITEM_COUNT  3
#define MEMORY_SLOT_COUNT   5

/* ======================================================================
 * Memory mode
 * Matches ACTION_SendMemory() BLE packet encoding:
 *   MEM_MODE_SET    (1) → byte[4] = (slot<<4)|0x01  e.g. slot1 = 0x11
 *   MEM_MODE_RECALL (2) → byte[4] = (slot<<4)|0x02  e.g. slot1 = 0x12
 * Also maps to IS_MEM_SET / IS_MEM_RECALL for the IA_MEM_MODE icon:
 *   MEM_MODE_RECALL (2) → lcd_set_icon(IA_MEM_MODE, IS_MEM_RECALL=0)
 *   MEM_MODE_SET    (1) → lcd_set_icon(IA_MEM_MODE, IS_MEM_SET=1)
 * ====================================================================== */
#define MEM_MODE_SET        1
#define MEM_MODE_RECALL     2

/* ======================================================================
 * Internal software key command codes (not BLE codes)
 * ====================================================================== */
#define CMD_GOBACK          0xF0
#define CMD_ACCEPT          0xF1
#define CMD_DETAILS         0xF2
#define CMD_CURSOR_UP       0xF3
#define CMD_CURSOR_DOWN     0xF4
#define CMD_CURSOR_LEFT     0xF5
#define CMD_CURSOR_RIGHT    0xF6

/* ======================================================================
 * UI state enum
 * ====================================================================== */
typedef enum
{
    UI_HOME = 0,
    UI_DETAILS,
    UI_RTP1,
    UI_RTP2,
    UI_BATTERY,
    UI_SETTING,
    UI_SETTING_CLOCK,
    UI_SETTING_ANTI,
    UI_COMBO_FLEX,
    UI_COMBO_HEIGHT,
    UI_COMBO_ACCEPT,
    UI_MEMORY
} UI_State_t;
uint8_t LCD_IsAntiCollisionEnabled(void);
uint8_t LCD_IsCollisionAlertActive(void);
void LCD_ShowCollisionAlert(uint8_t collision);
/* ======================================================================
 * Public API
 * ====================================================================== */
void    LCD_HMI_Init(UART_HandleTypeDef *huart);

/* Startup */
void    LCD_ShowStartupSequence(void);

/* Lock / power / orientation
 *   LCD_SetLock(1) — locks display, shows PAGE_LOCK with IS_LOCKED icon
 *   LCD_SetLock(0) — unlocks, clears icon, returns to PAGE_HOME
 *   LCD_TogglePower — power-off → PAGE_POWER_OFF; power-on → startup
 *                     sequence; landing page respects current lock state */
void    LCD_SetLock(uint8_t state);
void    LCD_TogglePower(void);
void    LCD_ToggleReverse(void);
uint8_t LCD_IsLocked(void);
uint8_t LCD_IsPowerOn(void);

/* Page navigation */
void    LCD_ShowSensorPage(uint8_t page);
void    LCD_ShowMemoryPage(uint8_t mode);

/* Key / animation */
void    LCD_HandleKey(uint8_t cmd);
void    LCD_StartCircularAnim(uint8_t cmd);
void    LCD_StopCircularAnim(void);

/* Main-loop task */
void    LCD_Task(void);

/* Sensor icon passthrough */
void    LCD_UpdateSensorIcon(uint8_t iconAddr, uint8_t state);

/* Live sensor push (called from Sensor_ParsePacket — ISR context) */
void    LCD_RefreshHeightPage(uint8_t actual_height, uint8_t slide);
void    LCD_PushRTP1Values(uint8_t height,  uint8_t slide,
                           uint8_t back_up, uint8_t back_dn,
                           uint8_t tilt_l,  uint8_t tilt_r,
                           uint8_t trend,   uint8_t rev_trend);
void    LCD_PushRTP2Values(uint8_t leg1_up, uint8_t leg1_dn,
                           uint8_t leg2_up, uint8_t leg2_dn);
void    LCD_UpdateBattery(uint8_t level);

/* GoBack+Accept feature API */
void    LCD_UpdateAcceptEnable(uint8_t index, uint8_t enabled);
void    LCD_RefreshAcceptPage(const uint8_t *states);

/* GoBack+FlexUp edit value API */
void    LCD_UpdateEditValue(uint8_t index, uint16_t value);
void    LCD_RefreshEditPage(const uint16_t *vals);

/* Memory slot saved-state API */
void    LCD_UpdateMemorySlotState(uint8_t slot, uint8_t saved);

/* Low-level DWIN primitives (also called by sensor_manager.c) */
void    lcd_set_page(uint8_t page);
void    lcd_set_icon(uint8_t vp, uint8_t value);
void    lcd_set_state_value(uint8_t vp, uint8_t value);
void    lcd_set_state_value16(uint8_t vp, uint16_t value);
uint8_t LCD_IsAntiCollisionEnabled(void);
void LCD_ShowCollisionAlert(uint8_t collision);
#endif /* LCD_HMI_H */
