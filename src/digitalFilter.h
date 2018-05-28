#ifndef __DIGITAL_FILTER_H__
#define __DIGITAL_FILTER_H__

#include "MDR1986VE1T.h"

#define _DIGFILTER_SAMPLES 3

typedef struct
{
  float x[_DIGFILTER_SAMPLES];
  float y1[_DIGFILTER_SAMPLES];
  float y2[_DIGFILTER_SAMPLES];
  float y3[_DIGFILTER_SAMPLES];
  float y4[_DIGFILTER_SAMPLES];
  
  float b11;
  float b12;
  float b13;
  
  float b21;
  float b22;
  float b23;

  float b31;
  float b32;
  float b33;

  float b41;
  float b42;
  float b43;
  
  float a11;
  float a12;
  
  float a21;
  float a22;    
  
  float a31;
  float a32;    
  
  float a41;
  float a42;    
} DigitalFilter4_t;


void digFilter4_init(DigitalFilter4_t *filter,// Инициализация цифрового фильтра 4ого порядка
                     float b11, float b12, float b13,
                     float b21, float b22, float b23,
                     float b31, float b32, float b33,
                     float b41, float b42, float b43,                     
                     float a11, float a12,
                     float a21, float a22,
                     float a31, float a32,
                     float a41, float a42); 
void digFilter4_reset(DigitalFilter4_t *filter);// Сброс цифрового фильтра 4ого порядка
float digFilter4_filter(DigitalFilter4_t *filter, float x);     // отфильтровать


#endif //__DIGITAL_FILTER_H__