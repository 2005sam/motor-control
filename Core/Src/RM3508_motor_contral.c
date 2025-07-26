#include "RM3508_motor_contral.h"
#include "queue.h"
#include "contral_DR16.h"
#include "motor_rm3508.h"
// #define speed_kp 1
// #define speed_ki 1
// #define speed_kd 1
// #define angle_kp 0.002f // Proportional gain for angle control
// #define angle_ki 0.0005f // Integral gain for angle control
// #define angle_kd 0.0f // Derivative gain for angle control

int16_t motor_target_speed[4] = {0, 0, 0, 0};
struct StructChassisSpeedSet chassis_target_speed;
PIDController pidcontraller;
PIDController PID_speed_contraller[4];
PIDController angle_pid_contraller;
QueueHandle_t chassis_set_queue;
int16_t pre_motor_speed = 0; // Previous motor speed, used to avoid oscillation
// warring:this function is only used to regulating PID,plase delete it in the final version
/***********************************************************************************************/
float speed_kp = 100.0f;                       // Proportional gain
float speed_ki = 1.0f;                         // Integral gain
float speed_kd = 0.0f;                         // Derivative gain
float angle_kp = 2000.0f;                      // Proportional gain for angle control
float angle_ki = 0.5f;                         // Integral gain for angle control
float angle_kd = 0.0f;                         // Derivative gain for angle control
struct MotorRm3508ReturnData motor_rx_data[4]; // Array to hold data for 4 motors
static QueueHandle_t motor_rm3508_tx_queue;
static QueueHandle_t motor_rm3508_rx_queue;
static QueueHandle_t speed_queue;
static QueueHandle_t angle_queue;
void ChassisSpeedSet(float vx, float vy, float angle);
void ChassisTargetSpeedUpdate(void *argument);
void ComputeSpeed(void *argument);
float sp;

void receive_date(float date, char flag)
{
  if (flag == 0xA1)
  {
    speed_kp = date;
  }
  else if (flag == 0xA2)
  {
    speed_ki = date;
  }
  else if (flag == 0xA3)
  {
    speed_kd = date;
  }
}

/************************************************************************************************/
// init motor control
void RM3508PIDMotorInit(UART_HandleTypeDef *UARTx, CAN_HandleTypeDef *hcan)
{
  ControlDR16Init(UARTx);
  MotorRm3508Init(hcan);
  //PID_init(&pidcontraller, speed_kp, speed_ki, speed_kd, 0.0f, 0.0f, 1000.0f, 1.0f, 0.0f, 0, 0); // Set max_output to 100.0f as an example
  // PID_init(&angle_pid_contraller, angle_kp, angle_ki, angle_kd, 0.0f, 0.0f, 10000.0f, 0.3, 0.01, 1, 1.0f); // Set max_output to 100.0f as an example
  for (int i = 0; i < 4; i++)
  {
    PID_init(&PID_speed_contraller[i], speed_kp, speed_ki, speed_kd, 0.0f, 0.0f, 1000.0f, 1.0f, 0.0f, 0, 0); // Set max_output to 100.0f as an example
  }
  pre_motor_speed = 0;
  chassis_set_queue = xQueueCreate(4, sizeof(struct StructChassisSpeedSet));
  xTaskCreate(RM3508MotorSetSpeed, "RM3508_Motor_SetSpeed", 256, NULL, 1, NULL);
  xTaskCreate(ChassisTargetSpeedUpdate, "Chassis_TargetSpeed_Update", 128, NULL, 1, NULL);
  xTaskCreate(ComputeSpeed, "Conpute_Speed", 128, NULL, 1, NULL);
}

void ChassisSpeedSet(float vx, float vy, float angle)
{
  struct StructChassisSpeedSet set_data;
  set_data.vx = vx;
  set_data.vy = vy;
  set_data.angle = angle;
  xQueueSend(chassis_set_queue, &set_data, 0);
}

void ChassisTargetSpeedUpdate(void *argument)
{
  struct StructChassisSpeedSet set_data;
  while (1)
  {
    xQueueReceive(chassis_set_queue, &set_data, portMAX_DELAY);
    chassis_target_speed = set_data;
  }
}

int16_t tmeptemp[4];

void RM3508MotorSetSpeed(void *argument)
{
  struct ControlDR16Data control_data;
  while (1)
  {
    for (int i = 0; i < 4; i++)
    {
      struct MotorRm3508ReturnData motor_data;
      MotorRm3508Get(i, &motor_data);
      float fb = motor_data.rpm;
      float co = PID_compute(&PID_speed_contraller[i], &fb);
			tmeptemp[i]=co;
      MotorRm3508Set(i, (int16_t)co);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void ComputeSpeed(void *argument)
{
  while (1)
  {
    struct ControlDR16Data contral_data;
    ControlDR16GetValue(&contral_data);
    chassis_target_speed.vy = (contral_data.ch0 - 1024)*10; // Adjusting the range from 0-2048 to -1024 to 1024
    chassis_target_speed.vx = (contral_data.ch1 - 1024)*10;
    motor_target_speed[0] = -chassis_target_speed.vx - chassis_target_speed.vy;
    motor_target_speed[1] = chassis_target_speed.vx - chassis_target_speed.vy;
    motor_target_speed[2] = -chassis_target_speed.vx + chassis_target_speed.vy;
    motor_target_speed[3] = chassis_target_speed.vx + chassis_target_speed.vy;

    for (int i = 0; i < 4; i++)
    {
      pid_sp_set(&PID_speed_contraller[i], motor_target_speed[i]);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
// used to set control the angle of the motor
// return the speed should be set to the motor
/*
void RM3508MotorSetAngle(void *argument)
{
  // float Kp = angle_kp;
  // float Ki = angle_ki;
  // float Kd = angle_kd;
  while (1)
  {
    float sp;
    xQueuePeek(angle_queue, &sp, 0); // Peek the desired angle from the queue
    int16_t co;
    float fb = 0;

    pid_sp_set(&angle_pid_contraller, sp); // Set the desired value (setpoint) for the PID controller
    struct rx_date_motor_rm3508_struct motor_rx_data;
    xQueuePeek(motor_rm3508_rx_queue, &motor_rx_data, 0);  // Peek the motor data from the queue
    fb = motor_rx_data.angle;                              // Get the feedback value from the motor
    co = (int16_t)PID_compute(&angle_pid_contraller, &fb); // Compute the control output using the PID controller
    xQueueOverwrite(speed_queue, &co);                     // Overwrite the speed queue with the computed control output
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
  */