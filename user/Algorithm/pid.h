#ifndef PID_H
#define PID_H
#include <stm32f4xx_hal.h>
#include "stdint.h"

typedef struct 
{
    float Kp;  // Proportional gain
    float Ki;  // Integral gain
    float Kd;  // Derivative gaind
    float sp;  // esired value
    float fd;  //derived feedforward value
    float fp;  // Feedforward proportional gain
    float fb;  // Feedback value
    uint64_t pre_time;//last pid calculation time
    float pre_err;// Previous error value
    float pre_err_integral;// Previous integral of error
    float pre_target; // Previous target value
    float max_output; // Maximum output value (optional, can be used for limiting)
    float ki_start_err; // Start error for Ki, used to avoid integral windup at the beginning
    float deadband;
    char flag_circle; // Flag to indicate if the PID controller is in circular mode
    float maxnumber; // Maximum number for the PID controller, used to limit the output
} PIDController;

void pid_sp_set(PIDController *pid, float sp);
void PID_init(PIDController *pid, float Kp, float Ki, float Kd,float fd, float fp, float max_output,float ki_start_err,float deadband,char flag_circle,float maxnumber);
float PID_compute(PIDController *pid, float *fd);
#endif