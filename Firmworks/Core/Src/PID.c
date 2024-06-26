#include "pid.h"
#include "stm32f4xx_hal.h"

#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

// 实例化PID(速度，角度)
PIDController PID_Vel, PID_Angle,PID_Current;

void PID_Init(PIDController *pid, float P, float I, float D, float ramp, float limit)
{
  pid->Kp = P;
  pid->Ki = I;
  pid->Kd = D;
  pid->output_ramp = ramp;
  pid->limit = limit;
  pid->prev_error = 0.0f;
  pid->prev_output = 0.0f;
  pid->prev_integral = 0.0f;
  pid->prev_timestamp = TIM2->CNT;
}

float PID_Output(PIDController *pid, float error)
{
  uint32_t timestamp_now = TIM2->CNT;
  float Ts = (timestamp_now - pid->prev_timestamp) * 1e-6f; //转化为秒
  if (Ts <= 0 || Ts > 0.5f)
    Ts = 1e-3f;

  float proportional = pid->Kp * error;

  float integral = pid->prev_integral + pid->Ki * Ts * 0.5f * (error + pid->prev_error);
  integral = _constrain(integral, -pid->limit, pid->limit);

  float derivative = pid->Kd * (error - pid->prev_error) / Ts;

  float output = proportional + integral + derivative;
  output = _constrain(output, -pid->limit, pid->limit);

  if (pid->output_ramp > 0)
  {
    float output_rate = (output - pid->prev_output) / Ts;
    if (output_rate > pid->output_ramp)
      output = pid->prev_output + pid->output_ramp * Ts;
    else if (output_rate < -pid->output_ramp)
      output = pid->prev_output - pid->output_ramp * Ts;
  }

  pid->prev_integral = integral;
  pid->prev_output = output;
  pid->prev_error = error;
  pid->prev_timestamp = timestamp_now;
  return output;
}

void PID_Set(PIDController *pid, float P, float I, float D, float ramp)
{
  pid->Kp = P;
  pid->Ki = I;
  pid->Kd = D;
  pid->output_ramp = ramp;
}
