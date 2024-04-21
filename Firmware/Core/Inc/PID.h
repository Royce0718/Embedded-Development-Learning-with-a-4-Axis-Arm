#ifndef __PID_H__
#define __PID_H__

#include "stm32f4xx_hal.h"

typedef struct {
  float Kp;  				             // 比例增益
  float Ki;  				             // 积分增益
  float Kd;  				             // 微分增益
  float output_ramp;             // PID控制器加速度限幅
  float limit;                   // PID控制器输出限幅

  float prev_error;              // 最后跟踪误差
  float prev_output;             // 最后一个pid输出值
  float prev_integral;           // 最后一个积分分量
  uint32_t prev_timestamp;  // 上次执行的时间戳
} PIDController;

void PID_Init(PIDController *pid, float P, float I, float D, float ramp, float limit);
void PID_Set(PIDController *pid, float P, float I, float D, float ramp);
float PID_Output(PIDController *pid, float error);

#endif

