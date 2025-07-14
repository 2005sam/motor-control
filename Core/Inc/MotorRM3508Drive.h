#ifndef MOTORRM3508DRIVE_H
#define MOTORRM3508DRIVE_H
#include <stm32f4xx_hal.h>
#include "stdint.h"
#include "FreeRTOS.h"
#include "semphr.h"

struct usr_tx_data{
    char motor;
    uint16_t rx_data;
};

struct tx_data_motor_rm3508_struct{
    int16_t motor1;
    int16_t motor2;
    int16_t motor3;
    int16_t motor4;
};

struct rx_date_motor_rm3508_struct{
    float angle;
    int16_t rpm;
    uint16_t current;
    uint8_t temperture;
    char motor_number;
};
void motor_rm3508_rx_massage(void *argument);
void motor_RM3508_Init(CAN_HandleTypeDef* hcan1,char fifo_number);
void moter_rm3508_tx_massage(void *argument);

#endif

