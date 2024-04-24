#include "stm32f4xx_hal.h"
#include "Inlinecurrent.h"

extern uint16_t ADC_Value[2];

// 实例化
CurrentSensor Current_Instance;

#define _ADC_VOLTAGE 3.3f            //ADC 电压
#define _ADC_RESOLUTION 4095.0f      //ADC 分辨率

// ADC 计数到电压转换比率求解
#define _ADC_CONV ( (_ADC_VOLTAGE) / (_ADC_RESOLUTION) )

#define _shunt_resistor 0.01 //分流电阻值 10mΩ

#define amp_gain 50 // IN199B1DCRK——增益

// 可由公式推出：I = (ADC*3.3)/(4096*50*0.01)
#define ADC_to_Current (_ADC_CONV/(amp_gain*_shunt_resistor))

// 查找 ADC 零偏移量的函数,初始化时使用
void Current_GetOffset(CurrentSensor* instance)
{
	uint16_t calibration_rounds = 1000;
	
	// 查找0电流时候的电压
	instance->offset_ia = 0;
	instance->offset_ib = 0;
	
	// 读数1000次
	for (int i = 0; i < calibration_rounds; i++) {
			instance->offset_ia += ADC_Value[0];
			instance->offset_ib += ADC_Value[1];
			HAL_Delay(1);
	}
	// 求平均，得到误差
	instance->offset_ia = instance->offset_ia / calibration_rounds;
	instance->offset_ib = instance->offset_ib / calibration_rounds;
}

void Current_GetCurrent(CurrentSensor* f)
{
    f->current_a = (ADC_Value[0] - f->offset_ia)*ADC_to_Current;// amps
    f->current_b = (ADC_Value[1] - f->offset_ib)*ADC_to_Current;// amps 
    // amps
}
















