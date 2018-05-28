#include "angleFilter.h"

#define FILTER_SAMPLES     3    // количество последних отсчетов передаточных функций

// Отсчеты фильтров
uint32_t _angleX[FILTER_SAMPLES];
double _angleY1[FILTER_SAMPLES];
double _angleY2[FILTER_SAMPLES];
double _angleY3[FILTER_SAMPLES];
double _angleY4[FILTER_SAMPLES];
double _angleY5[FILTER_SAMPLES];




// Цифровой фильтр с полосой 100Hz
uint32_t _angle_DigitalFilter_100Hz(uint32_t angleCode)
{
  double b11 = 0.00095627815163022299;
  double b12 = 0.00191255630326044598;
  double b13 = 0.00095627815163022299;
      
  double a11 = -1.9106427906787955;
  double a12 = 0.91446790328531635;
  
  uint8_t N = FILTER_SAMPLES-1; 
  
  for(int i=1; i<=N; i++)
  {
    _angleX[i-1] = _angleX[i];
    _angleY1[i-1] = _angleY1[i];
  }   
  
  _angleX[N] = angleCode;
  _angleY1[N] = b11*_angleX[N]  + b12*_angleX[N-1]  + b13*_angleX[N-2]  - a11*_angleY1[N-1] - a12*_angleY1[N-2];
  
  return (uint32_t)_angleY1[N];
}

// Цифровой фильтр с полосой 200Hz
uint32_t _angle_DigitalFilter_200Hz(uint32_t angleCode)
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
    _angleX[i-1] = _angleX[i];
    _angleY1[i-1] = _angleY1[i];
    _angleY2[i-1] = _angleY2[i];
  }   
  
  _angleX[N] = angleCode;
  _angleY1[N] = b11*_angleX[N]  + b12*_angleX[N-1]  + b13*_angleX[N-2]  - a11*_angleY1[N-1] - a12*_angleY1[N-2];
  _angleY2[N] = b21*_angleY1[N] + b22*_angleY1[N-1] + b23*_angleY1[N-2] - a21*_angleY2[N-1] - a22*_angleY2[N-2];
  
  return (uint32_t)_angleY2[N];
}

// Цифровой фильтр с полосой 300Hz
uint32_t _angle_DigitalFilter_300Hz(uint32_t angleCode)
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
    _angleX[i-1] = _angleX[i];
    _angleY1[i-1] = _angleY1[i];
    _angleY2[i-1] = _angleY2[i];
  }   
  
  _angleX[N] = angleCode;
  _angleY1[N] = b11*_angleX[N]  + b12*_angleX[N-1]  + b13*_angleX[N-2]  - a11*_angleY1[N-1] - a12*_angleY1[N-2];
  _angleY2[N] = b21*_angleY1[N] + b22*_angleY1[N-1] + b23*_angleY1[N-2] - a21*_angleY2[N-1] - a22*_angleY2[N-2];
  
  return (uint32_t)_angleY2[N];
}

// Цифровой фильтр с полосой 500Hz
uint32_t _angle_DigitalFilter_500Hz(uint32_t angleCode)
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
    _angleX[i-1] = _angleX[i];
    _angleY1[i-1] = _angleY1[i];
    _angleY2[i-1] = _angleY2[i];
    _angleY3[i-1] = _angleY3[i];  
    _angleY4[i-1] = _angleY4[i];  
    _angleY5[i-1] = _angleY5[i];  
  }   
  
  _angleX[N] = angleCode;
  _angleY1[N] = b11*_angleX[N]  + b12*_angleX[N-1]  + b13*_angleX[N-2]  - a11*_angleY1[N-1] - a12*_angleY1[N-2];
  _angleY2[N] = b21*_angleY1[N] + b22*_angleY1[N-1] + b23*_angleY1[N-2] - a21*_angleY2[N-1] - a22*_angleY2[N-2];
  _angleY3[N] = b31*_angleY2[N] + b32*_angleY2[N-1] + b33*_angleY2[N-2] - a31*_angleY3[N-1] - a32*_angleY3[N-2];
  _angleY4[N] = b41*_angleY3[N] + b42*_angleY3[N-1] + b43*_angleY3[N-2] - a41*_angleY4[N-1] - a42*_angleY4[N-2];
  _angleY5[N] = b51*_angleY4[N] + b52*_angleY4[N-1] + b53*_angleY4[N-2] - a51*_angleY5[N-1] - a52*_angleY5[N-2];
  
  return (uint32_t)_angleY5[N];
}




uint32_t angle_Filter(uint32_t rawCode)
{
  uint32_t filteredCode;
  filteredCode = _angle_DigitalFilter_200Hz(rawCode);
  filteredCode = rawCode;
  return filteredCode;
}