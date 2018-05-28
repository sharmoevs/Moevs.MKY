#include "digitalFilter.h"


// =============================================================================
// ====================== Цифровой фильтр 4ого порядка =========================
// =============================================================================

// Инициализация цифрового фильтра 4ого порядка
void digFilter4_init(DigitalFilter4_t *filter,
                     float b11, float b12, float b13,
                     float b21, float b22, float b23,
                     float b31, float b32, float b33,
                     float b41, float b42, float b43,
                     
                     float a11, float a12,
                     float a21, float a22,
                     float a31, float a32,
                     float a41, float a42)
{
  digFilter4_reset(filter);
  
  filter->b11 = b11;
  filter->b12 = b12;
  filter->b13 = b11;
  
  filter->b21 = b21;
  filter->b22 = b22;
  filter->b23 = b23;
  
  filter->b31 = b31;
  filter->b32 = b32;
  filter->b33 = b33;
  
  filter->b41 = b41;
  filter->b42 = b42;
  filter->b43 = b43;
  
  filter->a11 = a11;
  filter->a12 = a12;
  
  filter->a21 = a21;
  filter->a22 = a22;
  
  filter->a31 = a31;
  filter->a32 = a32;
  
  filter->a41 = a41;
  filter->a42 = a42;
}

// Сброс цифрового фильтра 4ого порядка
void digFilter4_reset(DigitalFilter4_t *filter)
{
  for(int i=0; i<_DIGFILTER_SAMPLES; i++)
  {
    filter->x[i] = 0;
    filter->y1[i] = 0;
    filter->y2[i] = 0;
    filter->y3[i] = 0;
    filter->y4[i] = 0;
  }
}

// Отфильтровать
float digFilter4_filter(DigitalFilter4_t *f, float x)
{
  static uint8_t N = _DIGFILTER_SAMPLES-1; 
  
  for(int i=1; i<=N; i++)
  {
    f->x[i-1] = f->x[i];
    f->y1[i-1] = f->y1[i];
    f->y2[i-1] = f->y2[i];
    f->y3[i-1] = f->y3[i];
    f->y4[i-1] = f->y4[i];
  } 
  
  f->x[N] = x;
  f->y1[N] = (f->b11)*(f->x[N])  + (f->b12)*(f->x[N-1])  + (f->b13)*(f->x[N-2])  - (f->a11)*(f->y1[N-1]) - (f->a12)*(f->y1[N-2]);
  f->y2[N] = (f->b21)*(f->y1[N]) + (f->b22)*(f->y1[N-1]) + (f->b23)*(f->y1[N-2]) - (f->a21)*(f->y2[N-1]) - (f->a22)*(f->y2[N-2]);
  f->y3[N] = (f->b31)*(f->y2[N]) + (f->b32)*(f->y2[N-1]) + (f->b33)*(f->y2[N-2]) - (f->a31)*(f->y3[N-1]) - (f->a32)*(f->y3[N-2]);
  f->y4[N] = (f->b41)*(f->y3[N]) + (f->b42)*(f->y3[N-1]) + (f->b43)*(f->y3[N-2]) - (f->a41)*(f->y4[N-1]) - (f->a42)*(f->y4[N-2]);
  
  return f->y4[N];
}