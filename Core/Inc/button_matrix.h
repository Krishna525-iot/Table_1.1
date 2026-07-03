#ifndef __BUTTON_MATRIX_H
#define __BUTTON_MATRIX_H

#include "stm32f1xx_hal.h"

void BUTTON_MATRIX_Init(void);
void BUTTON_MATRIX_Scan(void);
#define KEY_NONE    0xFF
#define KEY_UP      1
#define KEY_DOWN    2
#define KEY_LEFT    3
#define KEY_RIGHT   4
uint8_t BUTTON_MATRIX_GetKey(void);

#endif
