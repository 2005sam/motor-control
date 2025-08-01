// Copyright (C) 2025 b2(shengpengxiang1@outlook.com)
// This This program is free software:
// you can redistribute it and/or modify it under the terms of the
// GNU General Public License as published by the Free Software Foundation,
// either version 3 of the License, or (at your option) any later version.
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
// See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with this program.
// If not, see <http://www.gnu.org/licenses/>.
#include "pid.h"
#include "FreeRTOS.h"
#include "semphr.h"

struct PID_local_information
{
    char ki_flag;
    float err;
    float co;
    float cof;
    float dt;
    float err_integral;
    float fd;
};

struct PID_local_information local_info;
float PID_back = 10;
float PID_back2 = 10;
float PID_back3 = 10;
float PID_back4 = 10;
float PID_back5 = 10;

SemaphoreHandle_t pid_semaphore;
StaticSemaphore_t pid_semaphore_computer;

extern void PID_pre_process(PIDController *pid);
extern float PID_compute_co(PIDController *pid);
extern float fedforward_compute(PIDController *pid);
extern void PID_after_process(PIDController *pid, float *result);

// initializes the PID controller with specified gains and setpoint
void PID_init(PIDController *pid, float Kp, float Ki, float Kd, float fd, float fp, float max_output, float ki_start_err, float deadband, char flag_circle, float maxnumber)
{

    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->fd = fd;
    pid->fp = fp;
    pid->fb = 0;
    pid->pre_time = 0;
    pid->pre_err = 0;
    pid->pre_err_integral = 0;
    pid->max_output = max_output;
    pid->ki_start_err = ki_start_err;
    pid->deadband = deadband;
    pid->flag_circle = flag_circle;
    pid->maxnumber = maxnumber;
}

// update setpoint for the PID controller
void pid_sp_set(PIDController *pid, float sp)
{
    pid->sp = sp;
}

// Computes the PID control output based on the current feedback value
float PID_compute(PIDController *pid, float *feedback)
{
    // xSemaphoreTake(pid_semaphore, portMAX_DELAY); // Take the semaphore to ensure thread safety
    //  define a local information structure to hold intermediate values
    local_info.ki_flag = 1;         // Initialize ki_flag to 0
    local_info.err = 0.0f;          // Initialize error to 0
    local_info.co = 0.0f;           // Initialize control output to 0
    local_info.cof = 0.0f;          // Initialize feedforward term to 0
    local_info.dt = 0.0f;           // Initialize time difference to 0
    local_info.err_integral = 0.0f; // Initialize integral of error to 0
    local_info.fd = *feedback;      // Initialize feedback to feedback
    PID_back2 = local_info.fd;
    // pre-process the PID controller
    PID_pre_process(pid);
    PID_back = local_info.err;

    // compute co in PID
    float co = PID_compute_co(pid);

    // compute feedforward term
    float cof = fedforward_compute(pid);
    cof = 0;
    // Calculate the total control output including feedforward
    float result = co + cof;
    // update the previous values and check the limits
    PID_after_process(pid, &result);
    // xSemaphoreGive(pid_semaphore); // Release the semaphore after processing

    return result;
}

extern void PID_pre_process(PIDController *pid)
{
    // Check if the previous time is zero, indicating this is the first call,and set current time
    if (pid->pre_time == 0)
    {
        pid->pre_time = HAL_GetTick();
    }
    uint64_t current_time = HAL_GetTick();
    local_info.dt = (current_time - pid->pre_time) * 1.0e-6f;

    // Calculate the error between the setpoint and feedback value
    local_info.err = pid->sp - local_info.fd;

    // if in the circular mode, adjust the error to the closest pass

    if (pid->flag_circle == 1 && (2 * local_info.err > pid->maxnumber || 2 * local_info.err < -pid->maxnumber))
    {
        if (local_info.err > 0)
        {
            local_info.err -= pid->maxnumber; // Adjust error for circular mode
        }
        else
        {
            local_info.err += pid->maxnumber; // Adjust error for circular mode
        }
    }

    // Check if the error is within the deadband range
    if ((local_info.err < pid->deadband) && (local_info.err > -pid->deadband))
    {
        local_info.err = 0;
    }

    // check if the error is greater than the start error for Ki
    if (local_info.err > pid->ki_start_err || local_info.err < -pid->ki_start_err)
    {
        local_info.ki_flag = 0;
    }
}

extern float PID_compute_co(PIDController *pid)
{
    // compute the integral and derivative errors
    float d_err = (local_info.err - pid->pre_err) / local_info.dt;
    float integral_err = pid->pre_err_integral + local_info.ki_flag * (local_info.err + pid->pre_err) * local_info.dt / 2.0f;
    local_info.err_integral = integral_err;

    // compute the cop, coi, and cod values
    //  cop: proportional term, coi: integral term, cod: derivative term
    PID_back4 = local_info.err;
    float cop = pid->Kp * (local_info.err);
    float coi = pid->Ki * integral_err;
    float cod = pid->Kd * d_err;
    PID_back5 = cop;
    float co = cop + coi + cod;
    return co;
}

extern float fedforward_compute(PIDController *pid)
{
    // compute the feedforward term
    if (pid->sp == 0)
    {
        pid->sp = pid->pre_target;
    }
    float cof = pid->fd * (pid->sp - pid->pre_target) / local_info.dt + (pid->sp - pid->pre_target) * pid->fp;
    return cof;
}

extern void PID_after_process(PIDController *pid, float *result)
{
    // update previous values
    pid->pre_time = HAL_GetTick();
    pid->pre_err = local_info.err;
    pid->pre_err_integral = local_info.err_integral;
    pid->pre_target = pid->sp;

    // Limit the control output to the maximum and minimum values
    if (*result > pid->max_output)
    {
        *result = pid->max_output; // Limit the output to the maximum value
    }
    else if (*result < -pid->max_output)
    {
        *result = -pid->max_output; // Limit the output to the minimum value
    }
}