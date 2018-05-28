#ifndef __ANGLE_FILTER_H__
#define __ANGLE_FILTER_H__

#include "MDR1986VE1T.h"




uint32_t _angle_DigitalFilter_100Hz(uint32_t angleCode);
uint32_t _angle_DigitalFilter_200Hz(uint32_t angleCode);
uint32_t _angle_DigitalFilter_300Hz(uint32_t angleCode);
uint32_t _angle_DigitalFilter_500Hz(uint32_t angleCode);

uint32_t angle_Filter(uint32_t rawCode);


#endif //__DIGITAL_FILTER_H__