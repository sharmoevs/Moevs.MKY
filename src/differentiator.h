#ifndef __DIFFERENTIATOR_H__
#define __DIFFERENTIATOR_H__

#include "MDR1986VE1T.h"


typedef struct
{
  float x[2];   // входные отсчеты 
  float y[2];   // выходные отсчеты
  float D;      // коэф. D
  float N;      // коэф. Т
  float T;      
  //float DN;     // произведение коэф. D*N - для уменьшения вычислений
  //float NT;     // произведение коэф. N*T - для уменьшения вычислений
} Differential_t;


void diff_init(Differential_t *diff, float D, float N, float T, float initialValue);
//void diff_changeKoef(Differential_t *diff, float D, float N, float T);
float diff_takeDifferential(Differential_t *diff, float x);


#endif //__DIFFERENTIATOR_H__