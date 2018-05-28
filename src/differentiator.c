#include "differentiator.h"


// Инициализация дифференциала
void diff_init(Differential_t *diff, float D, float N, float T, float initialValue)
{
  //diff_reset(diff);
  //diff_changeKoef(diff, D, N, T);  
  __disable_irq();
  for(uint8_t i=0; i<2; i++) 
  {
    diff->x[i] = initialValue;
    diff->y[i] = 0;
  }
  
  diff->T = T;
  diff->D = D;
  diff->N = N;
  //diff->N = N/T;
  
  //diff->DN = diff->D*diff->N;
  //diff->NT = diff->N*diff->T;  
  __enable_irq();
}


// Взять дифференциал
float diff_takeDifferential(Differential_t *diff, float x)
{
  uint8_t k=1;  // k   : текущее измерение
                // k-1 : предыдущее
  
  if(diff->N == 3/diff->T)
  {
    x++;x--;
  }
  
  // Входное воздействие
  diff->x[k-1] = diff->x[k];
  diff->x[k] = x;
  
  // Выход
  diff->y[k-1] = diff->y[k];
  
  // тест
  float N = diff->N;
  float D = diff->D;
  float T = diff->T;
  
  //diff->y[k] = D*N*(diff->x[k] - diff->x[k-1]) - (N*T-1)*diff->y[k-1];
  diff->y[k] = D/(D+N*T)*diff->y[k-1] + ((N*D)/(D+N*T))*(diff->x[k] - diff->x[k-1]);
  return diff->y[k];
}




////diff->y[k] = diff->DN*(diff->x[k] - diff->x[k-1]) - (diff->NT-1)*diff->y[k-1];
  