#ifndef __INCURRENT_H__
#define __INCURRENT_H__

#include "stm32f4xx_hal.h"

typedef struct
{
	float current_a,current_b;
	
  float offset_ia;
  float offset_ib;

  float gain_a; //电流检测运算放大器增益
  float gain_b;
	
}CurrentSensor;

void Current_GetCurrent(CurrentSensor* f);

void Current_GetOffset(CurrentSensor* instance);

#endif
