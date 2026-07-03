#ifndef __ACTION_COMM_H
#define __ACTION_COMM_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

void ACTION_COMM_Init(UART_HandleTypeDef *huart_bt,
                      UART_HandleTypeDef *huart_act);

/* Normal motor command — byte[3] = cmd, byte[14] = 0x00 */
void ACTION_SendCommand(uint8_t cmd);

/*
 * Memory set / recall
 *   slot   : 1-5
 *   action : 1 = set,  2 = recall
 *
 * Packet: 43 47 FE 26 <slot<<4|action> 00 00 01 00 95 A5 B5 C5 D5 12 4E
 */
void ACTION_SendMemory(uint8_t slot, uint8_t action);

/*
 * GoBack + Level combo packet
 * Packet: 43 47 FE 00 01 00 00 01 00 95 A5 B5 C5 D5 13 4E
 */
void ACTION_SendGoBackLevel(void);

#endif
