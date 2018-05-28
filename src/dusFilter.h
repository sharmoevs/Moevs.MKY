#ifndef __DUS_FILTER_H__
#define __DUS_FILTER_H__

#include "MDR1986VE1T.h"


void dusFilter_Init();          // Инициализация фильтра
void dusFilter_Reset();         // Сброс коэф.

// Частота дискретизации ДУС 4000Гц
uint16_t _dus_4kHz_DigitalFilter_100Hz(uint16_t dusCode);
uint16_t _dus_4kHz_DigitalFilter_200Hz(uint16_t dusCode);
uint16_t _dus_4kHz_DigitalFilter_300Hz(uint16_t dusCode);
uint16_t _dus_4kHz_DigitalFilter_500Hz(uint16_t dusCode);

// Частота дискретизации ДУС 2000Гц
uint16_t _dus_2kHz_DigitalFilter_200Hz_500Hz_20dB_80dB(uint16_t dusCode);
uint16_t _dus_2kHz_DigitalFilter_200Hz_order2(uint16_t dusCode);
uint16_t _dus_2kHz_DigitalFilter_200Hz_500Hz_1dB_60dB(uint16_t dusCode);
uint16_t _dus_2kHz_DigitalFilter_500Hz_700Hz_1dB_60dB(uint16_t dusCode);
uint16_t _dus_2kHz_200Hz_700Hz_2dB_26dB(uint16_t dusCode);

#endif //__DUS_FILTER_H__
