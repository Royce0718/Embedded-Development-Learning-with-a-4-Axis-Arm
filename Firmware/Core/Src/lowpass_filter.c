#include "stm32f4xx_hal.h"
#include "lowpass_filter.h"

LOWPASS_FILTER_T LowFilter;

void LOWPASS_FILTER_Init(LOWPASS_FILTER_T *f, float time_const) {
  f->tf = time_const;
  f->prev_y = 0.0f;
  f->prev_timestamp = TIM2->CNT;
}

float LOWPASS_FILTER_Calc(LOWPASS_FILTER_T *f, float x)
{
  uint32_t timestamp = TIM2->CNT;
  float delta = (timestamp - f->prev_timestamp) * 1e-6f;
	
  if (delta < 0.0f) delta = 1e-3f;
  else if (delta > 0.3f)
	{
    f->prev_y = x;
    f->prev_timestamp = timestamp;
    return x;
  }

  float alpha = f->tf / (f->tf + delta);
  float y = alpha * f->prev_y + (1.0f - alpha) * x;
  f->prev_y = y;
  f->prev_timestamp = timestamp;
  return y;
}

