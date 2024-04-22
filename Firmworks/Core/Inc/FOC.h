#ifndef __FOC_H__
#define __FOC_H__

#include "stm32f4xx_hal.h"

float _normalizeAngle(float angle);

void setPwm(float Ua, float Ub, float Uc);

void setTorque(float Uq, float angle_el);

float _electricalAngle(void);

void DFOC_alignSensor(int _PP, int _DIR);

void DFOC_Vbus(float power_suply);

float DFOC_M0_Angle(void);

float DFOC_M0_Velocity(void);

void DFOC_M0_set_Velocity_Angle(float Target);

#endif
