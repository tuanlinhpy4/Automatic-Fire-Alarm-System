#ifndef DISPLAY_H
#define DISPLAY_H

#include "zigbee.h"

void Display_line1(uint8_t signal, uint16_t pin_adc_value);
void Display_Home(uint8_t warning_flag);
void Display_Menu(void);
void Display_List_Detector(void);
void Display_Detector(Detector_Typedef* detector);
void Display_List_Phone_Number(void);
void Display_Phone_Number(char phone[][13], uint8_t order);
void Display_Set_Phone_Number(void);

#endif
