
#ifndef RM3508_MOTOR_CONTRAL_H
#define RM3508_MOTOR_CONTRAL_H
#include "stm32f4xx_hal.h"
#include "MotorRM3508Drive.h"
#include "pid.h"

void RM3508_Motor_SetSpeed(void *argument);
void RM3508_Motor_SetAngle(void *argument);
void receive_date(float date,char flag);
void RM3508_PID_Motor_Init(void);


#endif