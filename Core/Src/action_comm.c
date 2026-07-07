#include "action_comm.h"
#include "sensor_manager.h"
#include "lcd_hmi.h"          /* LCD_ShowCollisionAlert / LCD_IsAntiCollisionEnabled */
#include <string.h>

static UART_HandleTypeDef *uart_bt;
static UART_HandleTypeDef *uart_act;
static UART_HandleTypeDef *uart_screen = NULL;

#define PACKET_SIZE            16
#define SENSOR_PACKET_SIZE     25
#define COLLISION_PACKET_SIZE  21     /* 43 47 .. 4E = 21 bytes */

#define HEADER_BYTE1     0x43
#define HEADER_SENSOR    0x47         /* sensor AND collision share this header */

#define COLLISION_ALERT_TAG  0x16     /* byte[4] = 22 -> COLLISION */
#define COLLISION_CLEAR_TAG  0x15     /* byte[4] = 21 -> NO collision */
#define COLLISION_MARKER     0x0F     /* byte[17] structural marker */
#define COLLISION_TAIL_A     0x4E     /* byte[20] tail (both frames) */
#define COLLISION_TAIL_B     0x4F     /* tolerated for older revisions */

static const uint8_t basePacket[PACKET_SIZE] =
{
    0x43, 0x47, 0xFE,
    0x00,
    0x00, 0x00, 0x00, 0x01, 0x00,
    0x95, 0xA5, 0xB5, 0xC5, 0xD5,
    0x00,
    0x4E
};

static uint8_t txPacket[PACKET_SIZE];

/* ---- BLE / primary link parser (sensor + collision share 43 47) ---- */
static uint8_t rxByte;
static uint8_t packetBuffer[SENSOR_PACKET_SIZE];
static uint8_t packetIndex  = 0;
static uint8_t headerState  = 0;

/* ---- wired action-controller parser (collision only) ---- */
static uint8_t rxByteAct;
static uint8_t pktBufAct[SENSOR_PACKET_SIZE];
static uint8_t pktIdxAct   = 0;
static uint8_t hdrStateAct = 0;

/* ------------------------------------------------------------------
 * Collision flag: written ONLY in the RX ISR, consumed in the main
 * loop by ACTION_COMM_Task(). No UART / page work runs in the ISR.
 *   1 = collision (0x16 / 22)   0 = clear (0x15 / 21)
 * ------------------------------------------------------------------ */
static volatile uint8_t g_collisionActive = 0;
uint8_t ACTION_GetCollisionActive(void) { return g_collisionActive; }

/* ===== DEBUG (declared extern in action_comm.h) — Live Expressions ===== */
volatile uint8_t dbg_ac_packet_ok       = 0;  /* ++ per VALID collision frame   */
volatile uint8_t dbg_ac_task_hit        = 0;  /* ++ per ACTION_COMM_Task() call */
volatile uint8_t dbg_ac_lcd_call        = 0;  /* ++ per LCD alert call          */
volatile uint8_t dbg_ac_state           = 0;  /* current collision flag 0/1     */
volatile uint8_t dbg_ac_feature_enabled = 0;  /* anti-collision feature ON/OFF  */
volatile uint8_t dbg_ac_last_packet[21] = {0};/* last VALID collision frame     */

/* optional RX counters */
volatile uint32_t g_dbg_rx_bt    = 0, g_dbg_rx_act    = 0;
volatile uint32_t g_dbg_sensor_ok = 0;         /* sensor frames parsed          */
volatile uint32_t g_dbg_coll_bad = 0;          /* collision-shaped but invalid  */

/* ------------------------------------------------------------------
 * Returns 1 if the 21-byte window is a fully-valid collision frame.
 * This is the ONLY thing that separates a 21-byte collision frame
 * from a 25-byte sensor frame (both start 43 47, both can carry 0x15
 * at byte[4]). All three fields must match.
 * ------------------------------------------------------------------ */
static uint8_t IsCollisionSignature(const uint8_t *buf)
{
    if(buf[4] != COLLISION_ALERT_TAG && buf[4] != COLLISION_CLEAR_TAG) return 0;
    if(buf[17] != COLLISION_MARKER) return 0;
    if(buf[20] != COLLISION_TAIL_A && buf[20] != COLLISION_TAIL_B) return 0;
    return 1;
}

/* Apply a confirmed collision frame (signature already checked). */
static void ApplyCollisionFrame(const uint8_t *buf)
{
    if(buf[4] == COLLISION_ALERT_TAG)      g_collisionActive = 1;   /* 0x16 */
    else if(buf[4] == COLLISION_CLEAR_TAG) g_collisionActive = 0;   /* 0x15 */

    for(uint8_t i = 0; i < COLLISION_PACKET_SIZE; i++)
        dbg_ac_last_packet[i] = buf[i];
    dbg_ac_packet_ok++;
}

/* ================= INIT ================= */

void ACTION_COMM_Init(UART_HandleTypeDef *huart_bt,
                      UART_HandleTypeDef *huart_act)
{
    uart_bt  = huart_bt;
    uart_act = huart_act;
    packetIndex = 0; headerState = 0;
    pktIdxAct   = 0; hdrStateAct = 0;

    if(uart_bt  != NULL) HAL_UART_Receive_IT(uart_bt,  &rxByte,    1);
    if(uart_act != NULL) HAL_UART_Receive_IT(uart_act, &rxByteAct, 1);
}

void ACTION_SetScreenUart(UART_HandleTypeDef *huart_screen_in)
{
    uart_screen = huart_screen_in;
}

/* ================= SEND HELPERS ================= */

static void SendPacket(void)
{
    if (uart_bt != NULL)
        HAL_UART_Transmit(uart_bt, txPacket, PACKET_SIZE, HAL_MAX_DELAY);
    if (uart_act != NULL)
        HAL_UART_Transmit(uart_act, txPacket, PACKET_SIZE, HAL_MAX_DELAY);
    if (uart_screen != NULL)
        HAL_UART_Transmit(uart_screen, txPacket, PACKET_SIZE, HAL_MAX_DELAY);
}

void ACTION_SendCommand(uint8_t cmd)
{
    memcpy(txPacket, basePacket, PACKET_SIZE);
    txPacket[3] = cmd;
    SendPacket();
}

void ACTION_SendMemory(uint8_t slot, uint8_t action)
{
    if (slot < 1 || slot > 5) return;
    if (action != 1 && action != 2) return;
    memcpy(txPacket, basePacket, PACKET_SIZE);
    txPacket[3]  = 0x26;
    txPacket[4]  = (slot << 4) | action;
    txPacket[14] = 0x12;
    SendPacket();
}

void ACTION_SendGoBackLevel(void)
{
    memcpy(txPacket, basePacket, PACKET_SIZE);
    txPacket[3]  = 0x00;
    txPacket[4]  = 0x01;
    txPacket[14] = 0x13;
    SendPacket();
}

/* ================= RX CALLBACK ================= */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* -------- BLE / primary link: 43 47 = sensor(25) OR collision(21) -------- */
    if (huart == uart_bt)
    {
        g_dbg_rx_bt++;

        switch(headerState)
        {
            case 0:
                if(rxByte == HEADER_BYTE1)
                { packetBuffer[0]=rxByte; packetIndex=1; headerState=1; }
                break;

            case 1:
                if(rxByte == HEADER_SENSOR)           /* 43 47 */
                { packetBuffer[1]=rxByte; packetIndex=2; headerState=2; }
                else if(rxByte == HEADER_BYTE1)       /* stray 0x43: resync */
                { packetBuffer[0]=rxByte; packetIndex=1; }
                else
                { headerState=0; packetIndex=0; }
                break;

            case 2:
                if(packetIndex < SENSOR_PACKET_SIZE) packetBuffer[packetIndex] = rxByte;
                packetIndex++;

                /* At 21 bytes, test the FULL collision signature. This is the
                 * only reliable way to tell a 21-byte collision frame from a
                 * 25-byte sensor frame — byte[4] alone is ambiguous because
                 * sensor payload also carries 0x15 there. */
                if(packetIndex == COLLISION_PACKET_SIZE)
                {
                    if(IsCollisionSignature(packetBuffer))
                    {
                        ApplyCollisionFrame(packetBuffer);   /* sets flag */
                        packetIndex = 0; headerState = 0;    /* consume, re-hunt */
                    }
                    /* else: NOT collision -> keep reading to 25 as sensor */
                }
                else if(packetIndex >= SENSOR_PACKET_SIZE)   /* 25 bytes */
                {
                    Sensor_ParsePacket(&packetBuffer[2]);
                    g_dbg_sensor_ok++;
                    packetIndex = 0; headerState = 0;
                }
                break;
        }
        HAL_UART_Receive_IT(uart_bt, &rxByte, 1);
    }
    /* -------- wired action controller: 43 47 collision-only (21 bytes) -------- */
    else if (huart == uart_act)
    {
        g_dbg_rx_act++;

        switch(hdrStateAct)
        {
            case 0:
                if(rxByteAct == HEADER_BYTE1)
                { pktBufAct[0]=rxByteAct; pktIdxAct=1; hdrStateAct=1; }
                break;

            case 1:
                if(rxByteAct == HEADER_SENSOR)
                { pktBufAct[1]=rxByteAct; pktIdxAct=2; hdrStateAct=2; }
                else if(rxByteAct == HEADER_BYTE1)
                { pktBufAct[0]=rxByteAct; pktIdxAct=1; }
                else
                { hdrStateAct=0; pktIdxAct=0; }
                break;

            case 2:
                if(pktIdxAct < SENSOR_PACKET_SIZE) pktBufAct[pktIdxAct] = rxByteAct;
                pktIdxAct++;

                if(pktIdxAct == COLLISION_PACKET_SIZE)
                {
                    if(IsCollisionSignature(pktBufAct))
                        ApplyCollisionFrame(pktBufAct);
                    else
                        g_dbg_coll_bad++;
                    /* wired link is collision-only: reset either way */
                    pktIdxAct = 0; hdrStateAct = 0;
                }
                break;
        }
        HAL_UART_Receive_IT(uart_act, &rxByteAct, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == uart_bt)
    { packetIndex=0; headerState=0; HAL_UART_Receive_IT(uart_bt, &rxByte, 1); }
    else if (huart == uart_act)
    { pktIdxAct=0; hdrStateAct=0; HAL_UART_Receive_IT(uart_act, &rxByteAct, 1); }
}

/* ================= MAIN-LOOP TASK ================= */
/*
 * ACTION_COMM_Task — called from while(1).
 * Edge-triggered: calls the LCD ONCE per 0->1 (collision) and 1->0 (clear)
 * transition, so it never spams the DWIN or fights manual navigation.
 *   flag 0 -> 1 (0x16) : LCD_ShowCollisionAlert(1) -> opens Anti page (PA 23)
 *   flag 1 -> 0 (0x15) : LCD_ShowCollisionAlert(0) -> clears alert, HOME
 * If anti-collision is OFF (GoBack+Accept #4) the LCD layer ignores it and
 * the bed keeps working normally.
 */
void ACTION_COMM_Task(void)
{
    static uint8_t lastCollision = 0;

    dbg_ac_task_hit++;

    uint8_t now = g_collisionActive;
    dbg_ac_state           = now;
    dbg_ac_feature_enabled = LCD_IsAntiCollisionEnabled();

    if(now != lastCollision)
    {
        lastCollision = now;
        dbg_ac_lcd_call++;
        LCD_ShowCollisionAlert(now);
    }
}
