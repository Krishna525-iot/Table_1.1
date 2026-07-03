#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <stdint.h>

/* ======================================================================
 * Sensor state struct
 *
 * Packet byte assignments (data[] offset after the 0x43 0x47 header):
 *   [0]  tilt_left       — side tilt left angle
 *   [1]  tilt_right      — side tilt right angle
 *   [2]  (reserved)
 *   [3]  back_up         — back-up actuator sensor (= actual height)
 *   [4]  back_down       — back-down actuator sensor
 *   [5]  trend           — Trendelenburg sensor
 *   [6]  reverse_trend   — Reverse Trendelenburg sensor
 *   [7]  leg1_up         — Leg 1 up sensor  (RTP2)
 *   [8]  leg1_down       — Leg 1 down sensor
 *   [9]  leg2_up         — Leg 2 up sensor  (RTP2)
 *   [10] leg2_down       — Leg 2 down sensor
 *   [11] battery_level   — System battery level 0-4
 * ====================================================================== */
typedef struct
{
    uint8_t tilt_left;
    uint8_t tilt_right;
    uint8_t back_up;
    uint8_t back_down;
    uint8_t trend;
    uint8_t reverse_trend;

    /* RTP2 leg actuator sensors */
    uint8_t leg1_up;
    uint8_t leg1_down;
    uint8_t leg2_up;
    uint8_t leg2_down;

    /* Battery (from sensor packet or separate source) */
    uint8_t battery_level;  /* 0 = empty … 4 = full */

} SensorState_t;

/* ======================================================================
 * Sensor-type identifiers  (used by Sensor_LoadTestPage)
 * ====================================================================== */
#define SENSOR_TILT   0
#define SENSOR_SLIDE  1
#define SENSOR_BACK   2

/* ======================================================================
 * Public API
 * ====================================================================== */

void              Sensor_Init(void);

/* Sensor debug / icon page control (separate from RTP live display) */
void              Sensor_SetActive(uint8_t state);
uint8_t           Sensor_IsActive(void);

/* Getters used by lcd_hmi.c when entering the GOBACK_HEIGHT page */
uint8_t           Sensor_GetActualHeight(void);
uint8_t           Sensor_GetSlide(void);

/* Read pointer — allows lcd_hmi.c to access full state if needed */
const SensorState_t *Sensor_GetState(void);

/* Called from HAL_UART_RxCpltCallback in action_comm.c */
void              Sensor_ParsePacket(uint8_t *data);

/* Called from the main loop; updates sensor icon page if active */
void              Sensor_UpdateDisplay(void);

/* Called by button_matrix.c when a motor key is pressed on the sensor page */
void              Sensor_ProcessKeys(uint8_t cmd);

/* Internal test-page helper */
void              Sensor_LoadTestPage(uint8_t sensorType);

#endif /* SENSOR_MANAGER_H */
