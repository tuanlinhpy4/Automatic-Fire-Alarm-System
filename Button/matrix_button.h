#ifndef MATRIX_BUTTON_H
#define MATRIX_BUTTON_H
#include "stm32f1xx.h"

#define KEYPAD_ROW 4
#define KEYPAD_COL 4

void Keypad_Filter(void);
void Keypad_Handle(void);

#endif
