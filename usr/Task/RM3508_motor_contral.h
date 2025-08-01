
#ifndef RM3508_MOTOR_CONTRAL_H
#define RM3508_MOTOR_CONTRAL_H
#include "stm32f4xx_hal.h"
#include "contral_DR16.h"
#include "motor_rm3508.h"
#include "pid.h"
#include "freertos.h"
#include "cmsis_os.h"

struct StructChassisSpeedSet
{
  float vx;
  float vy;
  float angle;
};

void RM3508MotorSetSpeed(void *argument);
void RM3508MotorSetAngle(void *argument);
void receive_date(float date, char flag);
void RM3508PIDMotorInit(UART_HandleTypeDef *UARTx, CAN_HandleTypeDef *hcan);

#endif