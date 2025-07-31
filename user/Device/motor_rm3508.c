#include "motor_rm3508.h"
#include "queue.h"
#include "semphr.h"
#include "can.h"

struct MotorRm3508ReturnData received_data[4];
uint8_t send_date[8];

char MotorRm3508GetOriginalData(uint8_t data[8], char motor_number)
{
  received_data[motor_number].rpm = data[0] << 8 | data[1];
  received_data[motor_number].angle = data[2] << 8 | data[3];
  received_data[motor_number].current = data[4] << 8 | data[5];
  received_data[motor_number].temperture = data[6];
  return 0;
}

char MotorRm3508Get(char motor_number, struct MotorRm3508ReturnData *kpdata)
{
  *kpdata = received_data[motor_number];
  return 0;
}

char MotorRm3508Set(char motor_number, int16_t motor_current)
{
  send_date[motor_number * 2] = motor_current >> 8;
  send_date[motor_number * 2 + 1] = motor_current & 0xFF;
  CanSend(send_date);
  return 0;
}
