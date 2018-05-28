#include "dusFilter.h"


#define FILTER_SAMPLES     3    // количество последних отсчетов передаточных функций
// Отсчеты фильтров
uint16_t _dusX[FILTER_SAMPLES];
double _dusY1[FILTER_SAMPLES];
double _dusY2[FILTER_SAMPLES];
double _dusY3[FILTER_SAMPLES];
double _dusY4[FILTER_SAMPLES];
double _dusY5[FILTER_SAMPLES];

// Инициализация фильтра
void dusFilter_Init()
{
  dusFilter_Reset();
}

// Обнуление предыдущих отсчетов ДУС
void dusFilter_Reset()
{
   for(int i=0; i<FILTER_SAMPLES; i++)
   {
     _dusX[i] = 0;
     _dusY1[i] = 0;
     _dusY2[i] = 0;
     _dusY3[i] = 0;
     _dusY4[i] = 0;
     _dusY5[i] = 0;
   }
}

// Цифровой фильтр с полосой 100Hz, Fdiskr = 4000Гц
uint16_t _dus_4kHz_DigitalFilter_100Hz(uint16_t dusCode)
{
  double b11 = 0.00095627815163022299;
  double b12 = 0.00191255630326044598;
  double b13 = 0.00095627815163022299;
      
  double a11 = -1.9106427906787955;
  double a12 = 0.91446790328531635;
  
  uint8_t N = FILTER_SAMPLES-1; 
  
  for(int i=1; i<=N; i++)
  {
    _dusX[i-1] = _dusX[i];
    _dusY1[i-1] = _dusY1[i];
  }   
  
  _dusX[N] = dusCode;
  _dusY1[N] = b11*_dusX[N]  + b12*_dusX[N-1]  + b13*_dusX[N-2]  - a11*_dusY1[N-1] - a12*_dusY1[N-2];
  
  return (uint16_t)_dusY1[N];
}

// Цифровой фильтр с полосой 200Hz, Fdiskr = 4000Гц
uint16_t _dus_4kHz_DigitalFilter_200Hz(uint16_t dusCode)
{
  double b11 = 0.0090090118496893787;
  double b12 = 0.0180180236993787574;
  double b13 = 0.0090090118496893787;
    
  double b21 = 0.090909104683203429;
  double b22 = 0.090909104683203429;
  double b23 = 0;
  
  double a11 = -1.783783745637507;
  double a12 = 0.8198197930362644;
  
  double a21 = -0.8181817906335932;
  double a22 = 0;
    
  uint8_t N = FILTER_SAMPLES-1; 
  
  for(int i=1; i<=N; i++)
  {
    _dusX[i-1] = _dusX[i];
    _dusY1[i-1] = _dusY1[i];
    _dusY2[i-1] = _dusY2[i];
  }   
  
  _dusX[N] = dusCode;
  _dusY1[N] = b11*_dusX[N]  + b12*_dusX[N-1]  + b13*_dusX[N-2]  - a11*_dusY1[N-1] - a12*_dusY1[N-2];
  _dusY2[N] = b21*_dusY1[N] + b22*_dusY1[N-1] + b23*_dusY1[N-2] - a21*_dusY2[N-1] - a22*_dusY2[N-2];
  
  return (uint16_t)_dusY2[N];
}

// Цифровой фильтр с полосой 300Hz, Fdiskr = 4000Гц
uint16_t _dus_4kHz_DigitalFilter_300Hz(uint16_t dusCode)
{
   double b11 = 0.027080645000476394;
   double b12 = 0.054161290000952788;
   double b13 = 0.027080645000476394;

   double b21 = 0.02324852569762606;
   double b22 = 0.04649705139525212;
   double b23 = 0.02324852569762606;

   double a11 = -1.658568655975434;
   double a12 = 0.76689123597733955;

   double a21 = -1.4238684499222107;
   double a22 = 0.5168625527127152;
    
  uint8_t N = FILTER_SAMPLES-1; 
  
  for(int i=1; i<=N; i++)
  {
    _dusX[i-1] = _dusX[i];
    _dusY1[i-1] = _dusY1[i];
    _dusY2[i-1] = _dusY2[i];
  }   
  
  _dusX[N] = dusCode;
  _dusY1[N] = b11*_dusX[N]  + b12*_dusX[N-1]  + b13*_dusX[N-2]  - a11*_dusY1[N-1] - a12*_dusY1[N-2];
  _dusY2[N] = b21*_dusY1[N] + b22*_dusY1[N-1] + b23*_dusY1[N-2] - a21*_dusY2[N-1] - a22*_dusY2[N-2];
  
  return (uint16_t)_dusY2[N];
}

// Цифровой фильтр с полосой 500Hz, Fdiskr = 4000Гц
uint16_t _dus_4kHz_DigitalFilter_500Hz(uint16_t dusCode)
{
  double b11 = 0.15649903907359147;
  double b12 = 0.31299807814718294;
  double b13 = 0.15649903907359147;
  
  double b21 = 0.12827053156947993;
  double b22 = 0.25654106313895986;
  double b23 = 0.12827053156947993;
  
  double b31 = 0.11182713304698108;
  double b32 = 0.22365426609396216;
  double b33 = 0.11182713304698108;

  double b41 = 0.10319277567378911;
  double b42 = 0.20638555134757822;
  double b43 = 0.10319277567378911;
  
  double b51 = 0.31701402508148857;
  double b52 = 0.31701402508148857;
  double b53 = 0;
  
     
  double a11 = -1.1398101448998159;
  double a12 = 0.7658063011941818;
  
  double a21 = -0.9342169385834691;
  double a22 = 0.44729906486138893;
  
  double a31 = -0.81445676265189992;
  double a32 = 0.26176529483982425;
  
  double a41 = -0.75157121276665817;
  double a42 = 0.16434231546181458;
     
  double a51 = -0.3659719498370228;
  double a52 = 0;


  
  uint8_t N = FILTER_SAMPLES-1; 
  
  for(int i=1; i<=N; i++)
  {
    _dusX[i-1] = _dusX[i];
    _dusY1[i-1] = _dusY1[i];
    _dusY2[i-1] = _dusY2[i];
    _dusY3[i-1] = _dusY3[i];  
    _dusY4[i-1] = _dusY4[i];  
    _dusY5[i-1] = _dusY5[i];  
  }   
  
  _dusX[N] = dusCode;
  _dusY1[N] = b11*_dusX[N]  + b12*_dusX[N-1]  + b13*_dusX[N-2]  - a11*_dusY1[N-1] - a12*_dusY1[N-2];
  _dusY2[N] = b21*_dusY1[N] + b22*_dusY1[N-1] + b23*_dusY1[N-2] - a21*_dusY2[N-1] - a22*_dusY2[N-2];
  _dusY3[N] = b31*_dusY2[N] + b32*_dusY2[N-1] + b33*_dusY2[N-2] - a31*_dusY3[N-1] - a32*_dusY3[N-2];
  _dusY4[N] = b41*_dusY3[N] + b42*_dusY3[N-1] + b43*_dusY3[N-2] - a41*_dusY4[N-1] - a42*_dusY4[N-2];
  _dusY5[N] = b51*_dusY4[N] + b52*_dusY4[N-1] + b53*_dusY4[N-2] - a51*_dusY5[N-1] - a52*_dusY5[N-2];
  
  return (uint16_t)_dusY5[N];
}





// Цифровой фильтр с полосой 200Hz, Fdiskr = 2000Гц
uint16_t _dus_2kHz_DigitalFilter_200Hz_500Hz_20dB_80dB(uint16_t dusCode)
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
    _dusX[i-1] = _dusX[i];
    _dusY1[i-1] = _dusY1[i];
    _dusY2[i-1] = _dusY2[i];
    _dusY3[i-1] = _dusY3[i];
    _dusY4[i-1] = _dusY4[i];
  }   
  
  _dusX[N] = dusCode;
  _dusY1[N] = b11*_dusX[N]  + b12*_dusX[N-1]  + b13*_dusX[N-2]  - a11*_dusY1[N-1] - a12*_dusY1[N-2];
  _dusY2[N] = b21*_dusY1[N] + b22*_dusY1[N-1] + b23*_dusY1[N-2] - a21*_dusY2[N-1] - a22*_dusY2[N-2];
  _dusY3[N] = b31*_dusY2[N] + b32*_dusY2[N-1] + b33*_dusY2[N-2] - a31*_dusY3[N-1] - a32*_dusY3[N-2];
  _dusY4[N] = b41*_dusY3[N] + b42*_dusY3[N-1] + b43*_dusY3[N-2] - a41*_dusY4[N-1] - a42*_dusY4[N-2];

  return (uint16_t)_dusY4[N];
}

// Цифровой фильтр с полосой 200Hz, Fdiskr = 2000Гц
uint16_t _dus_2kHz_DigitalFilter_200Hz_order2(uint16_t dusCode)
{
  double b11 = 0.067455273889071896;
  double b12 = 0.134910547778143792;
  double b13 = 0.067455273889071896;
       
  double a11 = -1.1429805025399011;
  double a12 = 0.41280159809618877;
       
  uint8_t N = FILTER_SAMPLES-1; 
  
  for(int i=1; i<=N; i++)
  {
    _dusX[i-1] = _dusX[i];
    _dusY1[i-1] = _dusY1[i];
  }   
  
  _dusX[N] = dusCode;
  _dusY1[N] = b11*_dusX[N]  + b12*_dusX[N-1]  + b13*_dusX[N-2]  - a11*_dusY1[N-1] - a12*_dusY1[N-2];

  return (uint16_t)_dusY1[N];
}

// Цифровой фильтр с полосой 200Hz, Fdiskr = 2000Гц
uint16_t _dus_2kHz_DigitalFilter_200Hz_500Hz_1dB_60dB(uint16_t dusCode)
{
  double b11 = 0.10648756804426596;
  double b12 = 0.21297513608853192;
  double b13 = 0.10648756804426596;
  
  double b21 = 0.086639178230500075;
  double b22 = 0.17327835646100015;
  double b23 = 0.086639178230500075;
  
  double b31 = 0.076740634071398944;
  double b32 = 0.153481268142797888;
  double b33 = 0.076740634071398944;

  double b41 = 0.27154022694858837;
  double b42 = 0.27154022694858837;
  double b43 = 0;
  

     
  double a11 = -1.3197761864307387;
  double a12 = 0.74572645860780251;
  
  double a21 = -1.0737809712492496;
  double a22 = 0.42033768417124984;
  
  double a31 = -0.95110127162380398;
  double a32 =  0.25806380790939976;
  
  double a41 = -0.4569195461028232;
  double a42 = 0;
     
  
  uint8_t N = FILTER_SAMPLES-1; 
  
  for(int i=1; i<=N; i++)
  {
    _dusX[i-1] = _dusX[i];
    _dusY1[i-1] = _dusY1[i];
    _dusY2[i-1] = _dusY2[i];
    _dusY3[i-1] = _dusY3[i];
    _dusY4[i-1] = _dusY4[i];
  }   
  
  _dusX[N] = dusCode;
  _dusY1[N] = b11*_dusX[N]  + b12*_dusX[N-1]  + b13*_dusX[N-2]  - a11*_dusY1[N-1] - a12*_dusY1[N-2];
  _dusY2[N] = b21*_dusY1[N] + b22*_dusY1[N-1] + b23*_dusY1[N-2] - a21*_dusY2[N-1] - a22*_dusY2[N-2];
  _dusY3[N] = b31*_dusY2[N] + b32*_dusY2[N-1] + b33*_dusY2[N-2] - a31*_dusY3[N-1] - a32*_dusY3[N-2];
  _dusY4[N] = b41*_dusY3[N] + b42*_dusY3[N-1] + b43*_dusY3[N-2] - a41*_dusY4[N-1] - a42*_dusY4[N-2];

  return (uint16_t)_dusY4[N];
}

// Цифровой фильтр с полосой 500Hz, Fdiskr = 2000Гц
uint16_t _dus_2kHz_DigitalFilter_500Hz_700Hz_1dB_60dB(uint16_t dusCode)
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
    _dusX[i-1] = _dusX[i];
    _dusY1[i-1] = _dusY1[i];
    _dusY2[i-1] = _dusY2[i];
    _dusY3[i-1] = _dusY3[i];
    _dusY4[i-1] = _dusY4[i];
    _dusY5[i-1] = _dusY5[i];
  }   
  
  _dusX[N] = dusCode;
  _dusY1[N] = b11*_dusX[N]  + b12*_dusX[N-1]  + b13*_dusX[N-2]  - a11*_dusY1[N-1] - a12*_dusY1[N-2];
  _dusY2[N] = b21*_dusY1[N] + b22*_dusY1[N-1] + b23*_dusY1[N-2] - a21*_dusY2[N-1] - a22*_dusY2[N-2];
  _dusY3[N] = b31*_dusY2[N] + b32*_dusY2[N-1] + b33*_dusY2[N-2] - a31*_dusY3[N-1] - a32*_dusY3[N-2];
  _dusY4[N] = b41*_dusY3[N] + b42*_dusY3[N-1] + b43*_dusY3[N-2] - a41*_dusY4[N-1] - a42*_dusY4[N-2];
  _dusY5[N] = b51*_dusY4[N] + b52*_dusY4[N-1] + b53*_dusY4[N-2] - a51*_dusY5[N-1] - a52*_dusY5[N-2];

  return (uint16_t)_dusY5[N];
}

// Цифровой фильтр с полосой 200Hz, Fdiskr = 2000Гц
uint16_t _dus_2kHz_200Hz_700Hz_2dB_26dB(uint16_t dusCode)
{
  double b11 = 0.10649407679530942;
  double b12 = 0.21298815359061884;
  double b13 = 0.10649407679530942;
       
  double a11 = -0.88890903448162961;
  double a12 =  0.3148853416628673;
  
  uint8_t N = FILTER_SAMPLES-1; 
  
  for(int i=1; i<=N; i++)
  {
    _dusX[i-1] = _dusX[i];
    _dusY1[i-1] = _dusY1[i];
  }   
  
  _dusX[N] = dusCode;
  _dusY1[N] = b11*_dusX[N]  + b12*_dusX[N-1]  + b13*_dusX[N-2]  - a11*_dusY1[N-1] - a12*_dusY1[N-2];
  
  return (uint16_t)_dusY1[N];
}
