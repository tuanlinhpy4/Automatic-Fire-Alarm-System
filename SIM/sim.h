#ifndef SIM_H
#define SIM_H

#include "stm32f1xx_hal.h"

#define RX_BUFFER_SIZE 255

typedef enum{
	SIM_OK,
	SIM_ERROR,
	SIM_TIMEOUT,
	SIM_BUSY
}SIM_Status_t;

typedef struct {
    char phone_number[20];
    char message[160];
    char timestamp[30];
} SIM_SMS_t;

SIM_Status_t SIM_Init(void);
SIM_Status_t SIM_SendATCommand(char *command, char *expected_response, uint32_t timeout);
SIM_Status_t SIM_SendSMS(const char *phone_number, const char *message);
SIM_Status_t SIM_ReadSMS(uint8_t index, SIM_SMS_t *sms);
SIM_Status_t SIM_CheckSignalQuality(void);
void SIM_receive_data(void);
void SIM_PowerOn(void);
void SIM_PowerOff(void);
void SIM_Sleep(void);
void SIM_Wakeup(void);

#endif
