#ifndef __PID_H__
#define __PID_H__

#include "stm32f4xx_hal.h"

typedef struct
{
  float Kp;          // ��������
  float Ki;          // ��������
  float Kd;          // ΢������
  float output_ramp; // PID���������ٶ��޷�
  float limit;       // PID����������޷�

  float prev_error;        // ���������
  float prev_output;       // ���һ��pid���ֵ
  float prev_integral;     // ���һ�����ַ���
  uint32_t prev_timestamp; // �ϴ�ִ�е�ʱ���
} PIDController;

void PID_Init(PIDController *pid, float P, float I, float D, float ramp, float limit);
void PID_Set(PIDController *pid, float P, float I, float D, float ramp);
float PID_Output(PIDController *pid, float error);

#endif
