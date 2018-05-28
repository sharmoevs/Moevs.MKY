#include "pidFilter.h"


#define FILTER_SAMPLES     3    // количество последних отсчетов передаточных функций
// Отсчеты фильтров
double _pidX[FILTER_SAMPLES];
double _pidY1[FILTER_SAMPLES];
double _pidY2[FILTER_SAMPLES];
double _pidY3[FILTER_SAMPLES];
double _pidY4[FILTER_SAMPLES];
double _pidY5[FILTER_SAMPLES];

// Инициализация фильтра
void pidFilter_Init()
{
  pidFilter_Reset();
}

// Обнуление предыдущих отсчетов ДУС
void pidFilter_Reset()
{
   for(int i=0; i<FILTER_SAMPLES; i++)
   {
     _pidX[i] = 0;
     _pidY1[i] = 0;
     _pidY2[i] = 0;
     _pidY3[i] = 0;
     _pidY4[i] = 0;
     _pidY5[i] = 0;
   }
}

// Цифровой фильтр с полосой 200Hz, Fdiskr = 2000Гц
float _pid_2kHz_DigitalFilter_200Hz_500Hz_20dB_80dB(float upr)
{
  double b11 = 0.060408761217579758;
  double b12 = 0.120817522435159516;
  double b13 = 0.060408761217579758;
  
  double b21 = 0.051168723683070641;
  double b22 = 0.102337447366141282;
  double b23 = 0.051168723683070641;
  
  double b31 = 0.046270918609395605;
  double b32 = 0.09254183721879121;
  double b33 = 0.046270918609395605;

  double b41 = 0.21152409866907507;
  double b42 = 0.21152409866907507;
  double b43 = 0;
  
     
  double a11 = -1.5579365058164167;
  double a12 = 0.79957155068673569;
  
  double a21 = -1.3196367708114813;
  double a22 = 0.52431166554376396;
  
  double a31 = -1.1933228195094865;
  double a32 = 0.37840649394706899;
  
  double a41 = -0.57695180266184976;
  double a42 = 0;
       
  uint8_t N = FILTER_SAMPLES-1; 
  
  for(int i=1; i<=N; i++)
  {
    _pidX[i-1] = _pidX[i];
    _pidY1[i-1] = _pidY1[i];
    _pidY2[i-1] = _pidY2[i];
    _pidY3[i-1] = _pidY3[i];
    _pidY4[i-1] = _pidY4[i];
  }   
  
  _pidX[N] = upr;
  _pidY1[N] = b11*_pidX[N]  + b12*_pidX[N-1]  + b13*_pidX[N-2]  - a11*_pidY1[N-1] - a12*_pidY1[N-2];
  _pidY2[N] = b21*_pidY1[N] + b22*_pidY1[N-1] + b23*_pidY1[N-2] - a21*_pidY2[N-1] - a22*_pidY2[N-2];
  _pidY3[N] = b31*_pidY2[N] + b32*_pidY2[N-1] + b33*_pidY2[N-2] - a31*_pidY3[N-1] - a32*_pidY3[N-2];
  _pidY4[N] = b41*_pidY3[N] + b42*_pidY3[N-1] + b43*_pidY3[N-2] - a41*_pidY4[N-1] - a42*_pidY4[N-2];

  return _pidY4[N];
}

// Цифровой фильтр с полосой 200Hz, Fdiskr = 2000Гц
float _pid_2kHz_DigitalFilter_200Hz_order2(float upr)
{
  double b11 = 0.067455273889071896;
  double b12 = 0.134910547778143792;
  double b13 = 0.067455273889071896;
       
  double a11 = -1.1429805025399011;
  double a12 = 0.41280159809618877;
       
  uint8_t N = FILTER_SAMPLES-1; 
  
  for(int i=1; i<=N; i++)
  {
    _pidX[i-1] = _pidX[i];
    _pidY1[i-1] = _pidY1[i];
  }   
  
  _pidX[N] = upr;
  _pidY1[N] = b11*_pidX[N]  + b12*_pidX[N-1]  + b13*_pidX[N-2]  - a11*_pidY1[N-1] - a12*_pidY1[N-2];

  return _pidY1[N];
}

// Цифровой фильтр с полосой 200Hz, Fdiskr = 2000Гц
float _pid_2kHz_DigitalFilter_200Hz_500Hz_1dB_60dB(float upr)
{
  static const double b11 = 0.10648756804426596;
  static const double b12 = 0.21297513608853192;
  static const double b13 = 0.10648756804426596;
  
  static const double b21 = 0.086639178230500075;
  static const double b22 = 0.17327835646100015;
  static const double b23 = 0.086639178230500075;
  
  static const double b31 = 0.076740634071398944;
  static const double b32 = 0.153481268142797888;
  static const double b33 = 0.076740634071398944;

  static const double b41 = 0.27154022694858837;
  static const double b42 = 0.27154022694858837;
  static const double b43 = 0;
  

  
  static const double a11 = -1.3197761864307387;
  static const double a12 = 0.74572645860780251;
  
  static const double a21 = -1.0737809712492496;
  static const double a22 = 0.42033768417124984;
  
  static const double a31 = -0.95110127162380398;
  static const double a32 =  0.25806380790939976;
  
  static const double a41 = -0.4569195461028232;
  static const double a42 = 0;
     
  
  uint8_t N = FILTER_SAMPLES-1; 
  
  for(int i=1; i<=N; i++)
  {
    _pidX[i-1] = _pidX[i];
    _pidY1[i-1] = _pidY1[i];
    _pidY2[i-1] = _pidY2[i];
    _pidY3[i-1] = _pidY3[i];
    _pidY4[i-1] = _pidY4[i];
  }   
  
  _pidX[N] = upr;
  _pidY1[N] = b11*_pidX[N]  + b12*_pidX[N-1]  + b13*_pidX[N-2]  - a11*_pidY1[N-1] - a12*_pidY1[N-2];
  _pidY2[N] = b21*_pidY1[N] + b22*_pidY1[N-1] + b23*_pidY1[N-2] - a21*_pidY2[N-1] - a22*_pidY2[N-2];
  _pidY3[N] = b31*_pidY2[N] + b32*_pidY2[N-1] + b33*_pidY2[N-2] - a31*_pidY3[N-1] - a32*_pidY3[N-2];
  _pidY4[N] = b41*_pidY3[N] + b42*_pidY3[N-1] + b43*_pidY3[N-2] - a41*_pidY4[N-1] - a42*_pidY4[N-2];

  return _pidY4[N];
}

// Цифровой фильтр с полосой 500Hz, Fdiskr = 2000Гц
float _pid_2kHz_DigitalFilter_500Hz_700Hz_1dB_60dB(float upr)
{
  double b11 = 0.38665689084371269;
  double b12 = 0.77331378168742538;
  double b13 = 0.38665689084371269;
  
  double b21 = 0.30277609147405005;
  double b22 = 0.6055521829481001;
  double b23 = 0.30277609147405005;
  
  double b31 = 0.25727671561899057;
  double b32 = 0.51455343123798114;
  double b33 = 0.25727671561899057;

  double b41 = 0.23429594100238713;
  double b42 = 0.46859188200477426;
  double b43 = 0.23429594100238713;
    
  double b51 = 0.47670367201451691;
  double b52 = 0.47670367201451691;
  double b53 = 0;
  
     
  double a11 = -0.15855364234014221;
  double a12 = 0.70518120571499276;
  
  double a21 = -0.12415723928253206;
  double a22 = 0.33526160517873221;
  
  double a31 = -0.10549963369769133;
  double a32 = 0.13460649617365353;
  
  double a41 = -0.096076070829564042;
  double a42 = 0.033259834839112608;
     
  double a51 = -0.046592655970966193;
  double a52 = 0;
  
  uint8_t N = FILTER_SAMPLES-1; 
  
  for(int i=1; i<=N; i++)
  {
    _pidX[i-1] = _pidX[i];
    _pidY1[i-1] = _pidY1[i];
    _pidY2[i-1] = _pidY2[i];
    _pidY3[i-1] = _pidY3[i];
    _pidY4[i-1] = _pidY4[i];
  }   
  
  _pidX[N] = upr;
  _pidY1[N] = b11*_pidX[N]  + b12*_pidX[N-1]  + b13*_pidX[N-2]  - a11*_pidY1[N-1] - a12*_pidY1[N-2];
  _pidY2[N] = b21*_pidY1[N] + b22*_pidY1[N-1] + b23*_pidY1[N-2] - a21*_pidY2[N-1] - a22*_pidY2[N-2];
  _pidY3[N] = b31*_pidY2[N] + b32*_pidY2[N-1] + b33*_pidY2[N-2] - a31*_pidY3[N-1] - a32*_pidY3[N-2];
  _pidY4[N] = b41*_pidY3[N] + b42*_pidY3[N-1] + b43*_pidY3[N-2] - a41*_pidY4[N-1] - a42*_pidY4[N-2];
  _pidY5[N] = b51*_pidY4[N] + b52*_pidY4[N-1] + b53*_pidY4[N-2] - a51*_pidY5[N-1] - a52*_pidY5[N-2];

  return _pidY4[N];
}

// Цифровой фильтр с полосой 200Hz, Fdiskr = 2000Гц
float _pid_2kHz_DigitalFilter_200Hz_700Hz_2dB_26dB(float upr)
{
  double b11 = 0.10649407679530942;
  double b12 = 0.21298815359061884;
  double b13 = 0.10649407679530942;
       
  double a11 = -0.88890903448162961;
  double a12 =  0.3148853416628673;
       
  uint8_t N = FILTER_SAMPLES-1; 
  
  for(int i=1; i<=N; i++)
  {
    _pidX[i-1] = _pidX[i];
    _pidY1[i-1] = _pidY1[i];
  }   
  
  _pidX[N] = upr;
  _pidY1[N] = b11*_pidX[N]  + b12*_pidX[N-1]  + b13*_pidX[N-2]  - a11*_pidY1[N-1] - a12*_pidY1[N-2];

  return _pidY1[N];
}


