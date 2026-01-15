#ifndef BUTTON_H
#define BUTTON_H

#include "stm32f1xx.h"

typedef enum{
	MODE1,
	MODE2,
	MODE3
}Button_Option;
Button_Option Button_Pressing(void);

#endif
