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

PIDController pidcontraller;
PIDController angle_pid_contraller;
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
void RM3508_Motor_get(void *argument);

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
  PID_init(&pidcontraller, speed_kp, speed_ki, speed_kd, 0.0f, 0.0f, 1000.0f, 1.0f, 0.0f, 0, 0); // Set max_output to 100.0f as an example
  // PID_init(&angle_pid_contraller, angle_kp, angle_ki, angle_kd, 0.0f, 0.0f, 10000.0f, 0.3, 0.01, 1, 1.0f); // Set max_output to 100.0f as an example

  pre_motor_speed = 0;
  xTaskCreate(RM3508MotorSetSpeed, "RM3508_Motor_SetSpeed", 1024, NULL, 1, NULL);
}

void RM3508MotorSetSpeed(void *argument)
{
  struct ControlDR16Data control_data;
  while (1)
  {
    ControlDR16GetValue(&control_data);
    float sp = (float)control_data.ch0 - 1024.0f;
    float fb = 0;

    // update setspeed if speed changed
    pid_sp_set(&pidcontraller, (float)sp);

    // comput co and tx to mot
    struct MotorRm3508ReturnData motor_data;
    MotorRm3508Get(0, &motor_data);
    fb = motor_data.rpm;
    float co = PID_compute(&pidcontraller, &fb);
    MotorRm3508Set(0, co);
    vTaskDelay(2); // Delay for a period to control the loop frequency
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