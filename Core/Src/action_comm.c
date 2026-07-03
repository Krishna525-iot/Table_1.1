#include "action_comm.h"
#include "sensor_manager.h"
#include <string.h>

static UART_HandleTypeDef *uart_bt;
static UART_HandleTypeDef *uart_act;
static UART_HandleTypeDef *uart_screen = NULL;   /* optional DWIN/screen mirror */

#define PACKET_SIZE         16
#define SENSOR_PACKET_SIZE  25
#define HEADER_BYTE1  0x43
#define HEADER_BYTE2  0x47

static const uint8_t basePacket[PACKET_SIZE] =
{
    0x43, 0x47, 0xFE,
    0x00,                               /* [3]  cmd / type       */
    0x00, 0x00, 0x00, 0x01, 0x00,       /* [4]-[8]               */
    0x95, 0xA5, 0xB5, 0xC5, 0xD5,       /* [9]-[13]              */
    0x00,                               /* [14] packet-type tag  */
    0x4E                                /* [15] tail             */
};

static uint8_t txPacket[PACKET_SIZE];
static uint8_t rxByte;
static uint8_t packetBuffer[SENSOR_PACKET_SIZE];
static uint8_t packetIndex  = 0;
static uint8_t headerState  = 0;

/* ================= INIT ================= */

void ACTION_COMM_Init(UART_HandleTypeDef *huart_bt,
                      UART_HandleTypeDef *huart_act)
{
    uart_bt  = huart_bt;
    uart_act = huart_act;
    packetIndex = 0;
    headerState = 0;
    HAL_UART_Receive_IT(uart_bt, &rxByte, 1);
}

/*
 * ACTION_SetScreenUart  (optional)
 * --------------------------------
 * Register a third UART so every outgoing packet is ALSO mirrored to the
 * screen / HMI link. If you never call this, behaviour is unchanged
 * (packet goes to BLE + action controller only).
 *
 * Call once in main() after ACTION_COMM_Init(), e.g.:
 *     ACTION_SetScreenUart(&huart3);
 *
 * Prototype to add in action_comm.h:
 *     void ACTION_SetScreenUart(UART_HandleTypeDef *huart_screen);
 */
void ACTION_SetScreenUart(UART_HandleTypeDef *huart_screen)
{
    uart_screen = huart_screen;
}

/* ================= SEND HELPERS ================= */

/* Transmit txPacket on Bluetooth, the action controller, and (if set)
 * the screen UART — so the same command reaches screen and bluetooth both. */
static void SendPacket(void)
{
    /* Bluetooth — always */
    if (uart_bt != NULL)
        HAL_UART_Transmit(uart_bt, txPacket, PACKET_SIZE, HAL_MAX_DELAY);

    /* Wired action / controller board */
    if (uart_act != NULL)
        HAL_UART_Transmit(uart_act, txPacket, PACKET_SIZE, HAL_MAX_DELAY);

    /* Screen UART mirror — only if registered via ACTION_SetScreenUart() */
    if (uart_screen != NULL)
        HAL_UART_Transmit(uart_screen, txPacket, PACKET_SIZE, HAL_MAX_DELAY);
}

/* Normal motor/stop command — byte[3]=cmd, rest default */
void ACTION_SendCommand(uint8_t cmd)
{
    memcpy(txPacket, basePacket, PACKET_SIZE);
    txPacket[3]  = cmd;
    /* txPacket[14] stays 0x00 (normal) */
    SendPacket();
}

/*
 * Memory set / recall
 *   slot   : 1-5
 *   action : 1 = set,  2 = recall
 *
 * byte[3]  = 0x26
 * byte[4]  = (slot << 4) | action
 *            slot 1 set  → 0x11   slot 1 recall → 0x12
 *            slot 2 set  → 0x21   slot 2 recall → 0x22  …
 * byte[14] = 0x12
 *
 * Sent to BLE + controller (+ screen if registered) via SendPacket().
 */
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

/*
 * GoBack + Level combo
 * 43 47 FE 00 01 00 00 01 00 95 A5 B5 C5 D5 13 4E
 */
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
    if (huart == uart_bt)
    {
        switch(headerState)
        {
            case 0:
                if(rxByte == HEADER_BYTE1)
                {
                    packetBuffer[0] = rxByte;
                    packetIndex = 1;
                    headerState = 1;
                }
                break;
            case 1:
                if(rxByte == HEADER_BYTE2)
                {
                    packetBuffer[1] = rxByte;
                    packetIndex = 2;
                    headerState = 2;
                }
                else
                {
                    headerState = 0;
                    packetIndex = 0;
                }
                break;
            case 2:
                packetBuffer[packetIndex++] = rxByte;
                if(packetIndex >= SENSOR_PACKET_SIZE)
                {
                    Sensor_ParsePacket(&packetBuffer[2]);
                    packetIndex = 0;
                    headerState = 0;
                }
                break;
        }
        HAL_UART_Receive_IT(uart_bt, &rxByte, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == uart_bt)
    {
        packetIndex = 0;
        headerState = 0;
        HAL_UART_Receive_IT(uart_bt, &rxByte, 1);
    }
}
