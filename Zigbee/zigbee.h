#ifndef ZIGBEE_H
#define ZIGBEE_H

#include "stm32f1xx_hal.h"

#define TEMP_MAX 50.0f

typedef struct{
	float temperature;
	float last_temperature;
	uint8_t smoke;
	uint8_t flag_warning;
}Detector_Typedef;

void zigbee_receive_data(uint8_t data_rx);
void zigbee_handle(void);

void detect_init(Detector_Typedef* detect);

#endif
