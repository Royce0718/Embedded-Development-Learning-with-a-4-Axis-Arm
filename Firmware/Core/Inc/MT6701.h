#ifndef __MT6701_H__
#define __MT6701_H__

#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "arm_math.h"

#define MT6701_SLAVE_ADDR         0x06 << 1
#define MT6701_Timeout            50

#define MT6701_REG_ANGLE_14b      0x03  // 14bits angle info : 0x03[13:6] 0x04[5:0]

#define _2PI 6.28318530718f

typedef struct
{
	  float angle_prev;        // 最后一次调用 getSensorAngle() 的输出结果，用于得到完整的圈数和速度
    uint32_t angle_prev_ts;     // 上次调用 getAngle 的时间戳
    float vel_angle_prev;    // 最后一次调用 getVelocity 时的角度
    uint32_t vel_angle_prev_ts; // 最后速度计算时间戳
    int32_t full_rotations;    // 总圈数计数
    int32_t vel_full_rotations;//用于速度计算的先前完整旋转圈数
}MT6701;


void MT6701_Init(MT6701* instance);

float MT6701_GetSensorAngle(void);

void MT6701_Update(MT6701* instance);

float MT6701_GetVelocity(MT6701* instance);

float MT6701_GetMechanicalAngle(MT6701* instance);

float MT6701_GetAngle(MT6701* instance);
	
#endif



