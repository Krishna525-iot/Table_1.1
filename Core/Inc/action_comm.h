#ifndef ACTION_COMM_H
#define ACTION_COMM_H

#include "main.h"
#include <stdint.h>

/* ======================================================================
 * Public API
 * ====================================================================== */
void ACTION_COMM_Init(UART_HandleTypeDef *huart_bt,
                      UART_HandleTypeDef *huart_act);

void ACTION_SetScreenUart(UART_HandleTypeDef *huart_screen);

void ACTION_SendCommand(uint8_t cmd);
void ACTION_SendMemory(uint8_t slot, uint8_t action);
void ACTION_SendGoBackLevel(void);

/*
 * Handles UART packet events outside ISR.
 * Must be called from while(1).
 */
void ACTION_COMM_Task(void);

/* ======================================================================
 * Debug variables for STM32CubeIDE Live Expressions
 * ====================================================================== */
extern volatile uint8_t dbg_ac_packet_ok;
extern volatile uint8_t dbg_ac_task_hit;
extern volatile uint8_t dbg_ac_lcd_call;
extern volatile uint8_t dbg_ac_state;
extern volatile uint8_t dbg_ac_feature_enabled;
extern volatile uint8_t dbg_ac_last_packet[21];

#endif /* ACTION_COMM_H */
