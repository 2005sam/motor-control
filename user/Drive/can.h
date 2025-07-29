#ifndef CAN_H
#define CAN_H

#include "stm32f4xx_hal.h"
#include "freertos.h"
#include "cmsis_os.h"

// External variables

// Function declarations
char CanInit(void);
char CanSend(uint8_t *data[8]);

#endif