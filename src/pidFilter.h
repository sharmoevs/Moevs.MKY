#ifndef __PID_FILTER_H__
#define __PID_FILTER_H__

#include "MDR1986VE1T.h"


void pidFilter_Init();
void pidFilter_Reset();

// Частота дискретизации ДУС 2000Гц
float _pid_2kHz_DigitalFilter_200Hz_500Hz_20dB_80dB(float upr);
float _pid_2kHz_DigitalFilter_200Hz_order2(float upr);
float _pid_2kHz_DigitalFilter_200Hz_500Hz_1dB_60dB(float upr);
float _pid_2kHz_DigitalFilter_500Hz_700Hz_1dB_60dB(float upr);
float _pid_2kHz_DigitalFilter_200Hz_700Hz_2dB_26dB(float upr);







#endif //__PID_FILTER_H__