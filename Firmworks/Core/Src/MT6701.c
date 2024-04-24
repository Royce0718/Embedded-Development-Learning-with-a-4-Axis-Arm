#include "MT6701.h"

MT6701 MT6701_Instance;

unsigned char mt6701_write_reg(unsigned char reg, unsigned char value)
{
  return HAL_I2C_Mem_Write(&hi2c3, MT6701_SLAVE_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, MT6701_Timeout);
}

unsigned char mt6701_write_regs(unsigned char reg, unsigned char *value, unsigned char len)
{
  return HAL_I2C_Mem_Write(&hi2c3, MT6701_SLAVE_ADDR, reg, I2C_MEMADD_SIZE_8BIT, value, len, MT6701_Timeout);
}

unsigned char mt6701_read_reg(unsigned char reg, unsigned char *buf, unsigned short len)
{
  return HAL_I2C_Mem_Read(&hi2c3, MT6701_SLAVE_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, len, MT6701_Timeout);
}

// 14bits angle info:0x03[13:6] 0x04[5:0].  Big-endian(高位在前)
float MT6701_GetSensorAngle(void)
{
  uint16_t angle;
  float angle_f;
  uint8_t temp[2];
  mt6701_read_reg(MT6701_REG_ANGLE_14b, temp, 2);

  angle = ((uint16_t)temp[0] << 6) | (temp[1] >> 2);
  angle_f = (float)angle / 16384 * PI * 2; //弧度制
  return angle_f;
}

float MT6701_GetAngle(MT6701 *instance)
{
  return (float)instance->full_rotations * _2PI + instance->angle_prev;
}

float MT6701_GetMechanicalAngle(MT6701 *instance)
{
  return instance->angle_prev;
}

void MT6701_Init(MT6701 *instance)
{
  MT6701_GetSensorAngle();
  HAL_Delay(1);
  instance->vel_angle_prev = MT6701_GetSensorAngle();
  instance->vel_angle_prev_ts = TIM2->CNT;
  HAL_Delay(1);
  MT6701_GetSensorAngle();
  HAL_Delay(1);
  instance->angle_prev = MT6701_GetSensorAngle();
  instance->angle_prev_ts = TIM2->CNT;
}

void MT6701_Update(MT6701 *instance)
{
  float angle_data = MT6701_GetSensorAngle();
  instance->angle_prev_ts = TIM2->CNT;

  float d_angle = angle_data - instance->angle_prev;

  if (fabs(d_angle) > (0.8 * _2PI)) // 圈数判断
  {
    instance->full_rotations += (d_angle > 0) ? -1 : 1;
  }

  instance->angle_prev = angle_data;
}

float MT6701_GetVelocity(MT6701 *instance)
{
  float Ts = (instance->angle_prev_ts - instance->vel_angle_prev_ts) * 1e-6;

  if (Ts <= 0)
    Ts = 1e-3f;

  float vel = ((float)(instance->full_rotations - instance->vel_full_rotations) * _2PI + (instance->angle_prev - instance->vel_angle_prev)) / Ts;

  instance->vel_angle_prev = instance->angle_prev;
  instance->vel_full_rotations = instance->full_rotations;
  instance->vel_angle_prev_ts = instance->angle_prev_ts;
  return vel;
}
