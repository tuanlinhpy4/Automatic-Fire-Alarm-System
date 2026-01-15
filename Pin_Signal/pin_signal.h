#ifndef PIN_SIGNAL_H
#define PIN_SIGNAL_H

#include <stdint.h>

#define PIN_LOCATION 0
#define PIN_ADDRESS_COL 19
#define PIN_ADDRESS_ROW 0
#define PIN_PERSENT_ADDRESS_COL 15
#define PIN_PERSENT_ADDRESS_ROW 0
#define SIGNAL_LOCATION 1
#define SIGNAL_ADDRESS_COL 14
#define SIGNAL_ADDRESS_ROW 0

void Display_Pin(uint16_t pin_adc_value);
void Display_Signal(uint8_t signal);

#endif
