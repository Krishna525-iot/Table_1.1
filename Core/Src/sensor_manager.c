#include "sensor_manager.h"
#include "lcd_hmi.h"

/* ======================================================================
 * Sensor debug icon VPs (sensor live-test page)
 * NOTE: IA_SENSOR_TILT_L shares address 0x50 with IA_DETAILS_CURSOR in
 * lcd_hmi.c.  Both are on different display pages so no visual conflict
 * exists today, but the HMI designer should assign distinct VPs in the
 * next DGUS revision.
 * ====================================================================== */
#define PAGE_SENSOR_1     0x0D

#define IA_SENSOR_TILT_L  0x50
#define IA_SENSOR_TILT_R  0x51
#define IA_SENSOR_BACK_UP 0x52
#define IA_SENSOR_BACK_DN 0x53
#define IA_SENSOR_TREND   0x54
#define IA_SENSOR_REV     0x55

/* ======================================================================
 * DISPLAY MAPPING FIX
 * ----------------------------------------------------------------------
 * Required screen mapping:
 *
 *   LEG1_UP  -> HEIGHT value on screen
 *   LEG2_UP  -> SLIDE value on screen
 *
 * Back and trend values remain unchanged.
 * ====================================================================== */
#define SENSOR_DISPLAY_HEIGHT()   (sensorState.leg1_up)
#define SENSOR_DISPLAY_SLIDE()    (sensorState.leg2_up)

/* ======================================================================
 * Module state
 * ====================================================================== */
static SensorState_t sensorState;
static uint8_t       selected_sensor  = 0;
static uint8_t       edit_mode        = 0;
static uint8_t       sensorPageActive = 0;

/* sensorDirty — set whenever sensorState is updated, cleared after the
 * DWIN icons are pushed.  Prevents the main-loop Sensor_UpdateDisplay()
 * call from flooding the DWIN with redundant VP writes every ~8ms.       */
static uint8_t sensorDirty = 0;

/* Test-mode state */
static uint8_t testModeActive = 0;
static uint8_t testSensorType = 0;
static uint8_t testValue      = 0;

/* ======================================================================
 * Private helpers
 * ====================================================================== */
static void increaseValue(void);
static void decreaseValue(void);

/* ======================================================================
 * Init
 * ====================================================================== */

void Sensor_Init(void)
{
    selected_sensor  = 0;
    edit_mode        = 0;
    sensorPageActive = 0;
    sensorDirty      = 0;

    testModeActive = 0;
    testSensorType = 0;
    testValue      = 0;

    /* Zero-fill all fields including leg / battery */
    SensorState_t empty = {0};
    sensorState = empty;
}

/* ======================================================================
 * Active-page control
 * ====================================================================== */

void Sensor_SetActive(uint8_t state)
{
    sensorPageActive = state;

    if(sensorPageActive)
    {
        sensorDirty = 1;           /* force a full refresh on page open */
        LCD_ShowSensorPage(PAGE_SENSOR_1);
    }
}

uint8_t Sensor_IsActive(void)
{
    return sensorPageActive;
}

/* ======================================================================
 * Getters  (used by lcd_hmi on GOBACK_HEIGHT entry)
 * ----------------------------------------------------------------------
 * FIX:
 *   Actual height now comes from LEG1_UP.
 *   Slide now comes from LEG2_UP.
 * ====================================================================== */
uint8_t Sensor_GetActualHeight(void)
{
    return sensorState.leg1_up;     /* Height screen value from LEG1_UP */
}

uint8_t Sensor_GetSlide(void)
{
    return sensorState.leg2_up;     /* Slide screen value from LEG2_UP */
}
/* ======================================================================
 * Sensor_ParsePacket
 * ------------------
 * Called from HAL_UART_RxCpltCallback.
 *
 * Byte layout (data[] starts after the 0x43 0x47 header):
 *   [0]  tilt_left      [1]  tilt_right
 *   [2]  reserved
 *   [3]  back_up        [4]  back_down
 *   [5]  trend          [6]  reverse_trend
 *   [7]  leg1_up        [8]  leg1_down
 *   [9]  leg2_up        [10] leg2_down
 *   [11] battery_level  (0-4, clamped)
 *
 * Screen mapping:
 *   HEIGHT = leg1_up
 *   SLIDE  = leg2_up
 *
 * Back and trend remain as original:
 *   BACK UP = back_up
 *   BACK DN = back_down
 *   TREND   = trend
 *   REV     = reverse_trend
 * ====================================================================== */
void Sensor_ParsePacket(uint8_t *data)
{
    /* Core motion sensors */
    sensorState.tilt_left     = data[0];
    sensorState.tilt_right    = data[1];
    /* data[2] reserved */
    sensorState.back_up       = data[3];
    sensorState.back_down     = data[4];
    sensorState.trend         = data[5];
    sensorState.reverse_trend = data[6];

    /* Leg actuator sensors */
    sensorState.leg1_up    = data[7];
    sensorState.leg1_down  = data[8];
    sensorState.leg2_up    = data[9];
    sensorState.leg2_down  = data[10];

    /* Battery (clamp to valid range) */
    sensorState.battery_level = (data[11] > 4) ? 4 : data[11];

    /* Mark debug icon page dirty for the main-loop update */
    sensorDirty = 1;

    /*
     * FIXED SCREEN VALUES:
     *   Height screen value should follow LEG1_UP.
     *   Slide screen value should follow LEG2_UP.
     */
    uint8_t displayHeight = SENSOR_DISPLAY_HEIGHT();
    uint8_t displaySlide  = SENSOR_DISPLAY_SLIDE();

    /* GoBack + HeightDn read-only fields */
    /* Only screen Height and Slide mapping changed.
     * Actual leg variables are not modified.
     */
    LCD_RefreshHeightPage(sensorState.leg1_up, sensorState.leg2_up);

    LCD_PushRTP1Values(
        sensorState.leg1_up,            /* height screen value */
        sensorState.leg2_up,            /* slide screen value */
        sensorState.back_up,            /* back up unchanged */
        sensorState.back_down,          /* back down unchanged */
        sensorState.tilt_left,          /* tilt left unchanged */
        sensorState.tilt_right,         /* tilt right unchanged */
        sensorState.trend,              /* trend unchanged */
        sensorState.reverse_trend       /* reverse trend unchanged */
    );

    /* RTP Page 2 live leg values — unchanged */
//    LCD_PushRTP2Values(
//        sensorState.leg1_up,
//        sensorState.leg1_down,
//        sensorState.leg2_up,
//        sensorState.leg2_down
//    );

    /* Battery level */
    LCD_UpdateBattery(sensorState.battery_level);
}

/* ======================================================================
 * Sensor_UpdateDisplay
 * --------------------
 * Called from the main loop every iteration.
 * Pushes the 6 debug icon VPs to the DWIN only when:
 *   - the sensor debug page is active, AND
 *   - new data has arrived since the last push (sensorDirty == 1)
 * This prevents flooding the DWIN with 6 VP writes every ~8ms.
 * ====================================================================== */
void Sensor_UpdateDisplay(void)
{
    if(!sensorPageActive) return;
    if(!sensorDirty)      return;   /* nothing changed — skip */

    LCD_UpdateSensorIcon(IA_SENSOR_TILT_L,  sensorState.tilt_left);
    LCD_UpdateSensorIcon(IA_SENSOR_TILT_R,  sensorState.tilt_right);
    LCD_UpdateSensorIcon(IA_SENSOR_BACK_UP, sensorState.back_up);
    LCD_UpdateSensorIcon(IA_SENSOR_BACK_DN, sensorState.back_down);
    LCD_UpdateSensorIcon(IA_SENSOR_TREND,   sensorState.trend);
    LCD_UpdateSensorIcon(IA_SENSOR_REV,     sensorState.reverse_trend);

    sensorDirty = 0;
}

/* ======================================================================
 * Key processing for the sensor debug / test page
 * ====================================================================== */
void Sensor_ProcessKeys(uint8_t cmd)
{
    if(!sensorPageActive) return;

    edit_mode = 1;

    switch(cmd)
    {
        case 0x01:
            increaseValue();
            sensorDirty = 1;
            break;

        case 0x02:
            decreaseValue();
            sensorDirty = 1;
            break;

        case 0x03:
            if(selected_sensor < 5) selected_sensor++;
            break;

        case 0x04:
            if(selected_sensor > 0) selected_sensor--;
            break;

        default:
            break;
    }
}

/* ======================================================================
 * Test page
 * ====================================================================== */
void Sensor_LoadTestPage(uint8_t sensorType)
{
    testModeActive = 1;
    testSensorType = sensorType;

    switch(sensorType)
    {
        case SENSOR_TILT:
            testValue = sensorState.tilt_left;
            break;

        case SENSOR_SLIDE:
            /*
             * FIX:
             * Slide test value should also come from LEG2_UP.
             */
            testValue = SENSOR_DISPLAY_SLIDE();
            break;

        case SENSOR_BACK:
            testValue = sensorState.back_up;
            break;

        default:
            testModeActive = 0;
            return;
    }

    sensorDirty = 1;
    Sensor_UpdateDisplay();
}

/* ======================================================================
 * Value helpers
 * ====================================================================== */

static void increaseValue(void)
{
    switch(selected_sensor)
    {
        case 0: sensorState.tilt_left++;     break;
        case 1: sensorState.tilt_right++;    break;
        case 2: sensorState.back_up++;       break;
        case 3: sensorState.back_down++;     break;
        case 4: sensorState.trend++;         break;
        case 5: sensorState.reverse_trend++; break;
        default: break;
    }
}

static void decreaseValue(void)
{
    switch(selected_sensor)
    {
        case 0:
            if(sensorState.tilt_left > 0) sensorState.tilt_left--;
            break;

        case 1:
            if(sensorState.tilt_right > 0) sensorState.tilt_right--;
            break;

        case 2:
            if(sensorState.back_up > 0) sensorState.back_up--;
            break;

        case 3:
            if(sensorState.back_down > 0) sensorState.back_down--;
            break;

        case 4:
            if(sensorState.trend > 0) sensorState.trend--;
            break;

        case 5:
            if(sensorState.reverse_trend > 0) sensorState.reverse_trend--;
            break;

        default:
            break;
    }
}
