#ifndef MOTOR_RM3508_H
#define MOTOR_RM3508_H
#include <stm32f4xx_hal.h>
#include "stdint.h"
#include "freertos.h"
#include "cmsis_os.h"


struct MotorRm3508ReturnData
{
    float angle;
    int16_t rpm;
    uint16_t current;
    uint8_t temperture;
};

char MotorRm3508Init(CAN_HandleTypeDef *hcan1);
char MotorRm3508Set(char motor_number, int16_t motor_current);
char MotorRm3508Get(char motor_number, struct MotorRm3508ReturnData *kpdata);

#endif
