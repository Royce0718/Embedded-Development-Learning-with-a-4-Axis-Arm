#include "stm32f4xx_hal.h"
#include "arm_math.h"
#include "MT6701.h"
#include "lowpass_filter.h"
#include "PID.h"

extern MT6701 MT6701_Instance;
extern PIDController PID_Vel, PID_Angle;
extern LOWPASS_FILTER_T LowFilter;

#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

#define _3PI_2 4.71238898038f

float voltage_power_supply;
float Ualpha, Ubeta = 0, Ua = 0, Ub = 0, Uc = 0;
float zero_electric_angle = 0;

int8_t PP = 7, DIR = -1;

// 归一化角度到[0,2PI]
float _normalizeAngle(float angle)
{
  float a = fmod(angle, 2 * PI);
  return a >= 0 ? a : (a + 2 * PI);
}

void setPwm(float Ua, float Ub, float Uc)
{
  //占空比：0-1
  float dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
  float dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
  float dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

  TIM1->CCR1 = dc_a * 1400;
  TIM1->CCR2 = dc_b * 1400;
  TIM1->CCR3 = dc_c * 1400;
}

void setTorque(float Uq, float angle_el)
{
  MT6701_Update(&MT6701_Instance);
  Uq = _constrain(Uq, -voltage_power_supply / 2, voltage_power_supply / 2);
  // float Ud=0;
  angle_el = _normalizeAngle(angle_el);

  Ualpha = -Uq * sin(angle_el);
  Ubeta = Uq * cos(angle_el);

  Ua = Ualpha + voltage_power_supply / 2;
  Ub = (sqrt(3) * Ubeta - Ualpha) / 2 + voltage_power_supply / 2;
  Uc = (-Ualpha - sqrt(3) * Ubeta) / 2 + voltage_power_supply / 2;
  setPwm(Ua, Ub, Uc);
}

float _electricalAngle(void)
{
  return _normalizeAngle((float)(DIR * PP) * MT6701_GetMechanicalAngle(&MT6701_Instance) - zero_electric_angle);
}

void DFOC_alignSensor(int _PP, int _DIR)
{
  PP = _PP;
  DIR = _DIR;
  setTorque(3, _3PI_2);
  HAL_Delay(1000);
  MT6701_Update(&MT6701_Instance);
  zero_electric_angle = _electricalAngle();
  setTorque(0, _3PI_2);
}

void DFOC_Vbus(float power_suply)
{
  voltage_power_supply = power_suply;
}

float DFOC_M0_Angle(void)
{
  return DIR * MT6701_GetAngle(&MT6701_Instance);
}

float DFOC_M0_Velocity(void)
{
  //低通滤波
  float vel_M0_ori = MT6701_GetVelocity(&MT6701_Instance);
  float vel_M0_flit = LOWPASS_FILTER_Calc(&LowFilter, DIR * vel_M0_ori);
  return vel_M0_flit; //方向
}

void DFOC_M0_set_Velocity_Angle(float Target)
{
  setTorque(PID_Output(&PID_Vel, PID_Output(&PID_Angle, (Target - DFOC_M0_Angle()) * 180 / PI)), _electricalAngle());
}
