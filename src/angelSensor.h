#ifndef __ANGEL_SENSOR_H__
#define __ANGEL_SENSOR_H__

#include "MDR1986VE1T.h"


// С датчика угла получаем код угла от 0...0x7FFFF
// Точность определения угла необходима 1/128градуса:
// 1 градус = 128 отсчетов, 360 градусов = 128*360 = 46080 = 0xB400
//
// Цена деления датчика угла             d1 = 1/0x7FFFF 
// Цена деления выдаваемых значений угла d2 = 1/0xB400
// 1 шаг выдаваемых значений в отсчетах датчика угла = d2/d1 = 0x7FFFF/0xB400 = 11


#define MAX_SYSANGLE_CODE          0x7FFFF
#define USYSANGLE_TO_FLOAT(code32) (float)(code32*360.0/MAX_SYSANGLE_CODE)
#define USYSANGLE_TO_CODE(angleF)  (uint32_t)(angleF*MAX_SYSANGLE_CODE/360.0)


#define VEL_FROM_ANGLE_SAMPLES  16
//#define AVERAGE



void angelSensor_init();
uint32_t angle_getNextSampleX32();                          // рассчитать текущее значение угла
uint8_t angle_getAverageVelocity(float *averageVelocity);   // рассчитать среднюю скорость
float angle_getVelocityFromAngle();
int32_t  angle_getMinimalAngleRange(uint32_t startAngle, uint32_t stopAngle, uint32_t code360grad);


uint32_t angle_deductCorrection(uint32_t angle, uint32_t deltaFi, uint32_t code360);
uint32_t angle_addCorrection(uint32_t angle, uint32_t deltaFi, uint32_t code360);

uint32_t angle_convertCmdS2SysU(int32_t sCmdAngle, uint32_t scaleCorrection, uint32_t code360);
int32_t  angle_convertSysU2CmdS(uint32_t uSysAngle, uint32_t scaleCorrection, uint32_t code360);








/// dfdfdf



//int32_t angle_cmdConvertU2S(uint32_t angleU);
//uint32_t angle_cmdConvertS2U(int32_t angleS);

//int32_t angle_sysConvertU2S(uint32_t angleU);
//uint32_t angle_sysConvertS2U(int32_t angleS);

//uint32_t angle_correctSysAngleU(uint32_t sysAngleU, uint32_t correctionAngleU);
//uint32_t angle_correctCorrectPlus(uint32_t angle, uint32_t correction);
/*
//uint16_t angle_convertAbs32BitCodeToAbs16BitCode(uint32_t angle); // преобразовать 32битный угол в 16битный
int16_t  angle_convert360To180(uint16_t angle360);
uint16_t angle_convert180To360(int16_t angle180);

uint16_t angle_convertFromScaleFFFFToScale128(uint16_t angleFFFF);
uint16_t angle_convertFromScale128ToScaleFFFF(uint16_t angle128);

uint16_t angle_convertScale180ToSystem360(int16_t angle180, uint16_t scaleCorrectionAngle);
int16_t angle_convertSystem360ToAnother180(uint16_t sysAngle, uint16_t sysCorrectionAngle);
*/

#endif //__ANGEL_SENSOR_H__