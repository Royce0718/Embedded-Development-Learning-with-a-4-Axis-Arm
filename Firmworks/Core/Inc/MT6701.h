#ifndef __MT6701_H__
#define __MT6701_H__

#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "arm_math.h"

#define MT6701_SLAVE_ADDR 0x06 << 1
#define MT6701_Timeout 50

#define MT6701_REG_ANGLE_14b 0x03 // 14bits angle info : 0x03[13:6] 0x04[5:0]

#define _2PI 6.28318530718f

typedef struct
{
    float angle_prev;           // ���һ�ε��� getSensorAngle() �������������ڵõ�������Ȧ�����ٶ�
    uint32_t angle_prev_ts;     // �ϴε��� getAngle ��ʱ���
    float vel_angle_prev;       // ���һ�ε��� getVelocity ʱ�ĽǶ�
    uint32_t vel_angle_prev_ts; // ����ٶȼ���ʱ���
    int32_t full_rotations;     // ��Ȧ������
    int32_t vel_full_rotations; // �����ٶȼ������ǰ������תȦ��
} MT6701;

void MT6701_Init(MT6701 *instance);

float MT6701_GetSensorAngle(void);

void MT6701_Update(MT6701 *instance);

float MT6701_GetVelocity(MT6701 *instance);

float MT6701_GetMechanicalAngle(MT6701 *instance);

float MT6701_GetAngle(MT6701 *instance);

#endif
