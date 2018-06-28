#include "dus.h"
#include "global.h"
#include "settings.h"
#include "adc.h"
#include "timers.h"
#include "canterminal.h"
 
extern int16_t dusCodeAmmend;

double dusCalibrationKoef = 0.72392910;    // коэффициент пропорциональности угловой скорости для кода

uint16_t dusRawCode;            // значение с ДУС после АЦП
uint16_t dusFilteredCode;       // значение с ДУС после цифрового фильтра
uint16_t dusCode;               // значение с ДУС после оцифровки и корректировки
int16_t dusAmplitude;           // амплитуда сигнала относительно уровня 2047
float g_currentSpeedFromDus;    // текущая скорость в градусах/сек


uint16_t _dus_filter(uint16_t code);
uint16_t _dus_correct(uint16_t filteredCode);


// Получить следующую выборку
uint16_t dus_getNextSample()
{
  dusRawCode      = adc_getVelocityCode();         // значение с АЦП
  dusFilteredCode = _dus_filter(dusRawCode);       // отфильтрованное значение АЦП
  dusCode         = _dus_correct(dusFilteredCode); // корректировка  
  dusAmplitude    = 0x7FF - dusCode;               // амплитуда ДУС относительно уровня 2047
  
  g_currentSpeedFromDus = DUS_CONVERT_TO_DEGREES_PER_SEC(dusAmplitude);
  return dusCode;
}


// Оцифровать и отфильтровать следующее значение с ДУС
uint16_t _dus_filter(uint16_t code)
{  
  uint16_t filteredCode;
  
//#ifdef DUS_FILTER_NONE
//  filteredCode = code;                                       // без фильтра
  
#if defined DUS_FILTER_F4kHz_B100Hz
  filteredCode = _dus_4kHz_DigitalFilter_100Hz(code);        // F=4kHz b=100Hz
    
#elif defined DUS_FILTER_F4kHz_B200Hz
  filteredCode = _dus_4kHz_DigitalFilter_200Hz(code);        // F=4kHz b=200Hz
  
#elif defined DUS_FILTER_F4kHz_B300Hz
  filteredCode = _dus_4kHz_DigitalFilter_300Hz(code);        // F=4kHz b=300Hz
  
#elif defined DUS_FILTER_F4kHz_B500Hz
  filteredCode = _dus_4kHz_DigitalFilter_500Hz(code);        // F=4kHz b=500Hz
    
  
  
#elif defined DUS_FILTER_F2kHz_B200Hz_500Hz_20dB_80dB
  filteredCode = _dus_2kHz_DigitalFilter_200Hz_500Hz_20dB_80dB(code);           // F=4kHz b=200Hz
  
#elif defined DUS_FILTER_F2kHz_B200Hz_SPECIFY_ORDER_2
  filteredCode = _dus_2kHz_DigitalFilter_200Hz_order2(code);                    // F=4kHz b=200Hz ver2    P=5!
  
#elif defined DUS_FILTER_F2kHz_B200Hz_500Hz_1dB_60dB
  filteredCode = _dus_2kHz_DigitalFilter_200Hz_500Hz_1dB_60dB(code);            // F=4kHz b=200Hz ver3    P=25
  
#elif defined DUS_FILTER_F2kHz_B500Hz_700Hz_1dB_60dB
  filteredCode = _dus_2kHz_DigitalFilter_500Hz_700Hz_1dB_60dB(code);            // F=4kHz b=500Hz         P=5

#elif defined DUS_PID_FILTER_F2kHz_B200Hz_700Hz_2dB_26dB
  filteredCode = _dus_2kHz_200Hz_700Hz_2dB_26dB(code);                          // F=4kHz b=500Hz         P=5

#else
  filteredCode = code;                                       // без фильтра  
#endif
  
  return filteredCode;
}

// Поправки к отфильтрованным значениям ДУС
uint16_t _dus_correct(uint16_t filteredCode)
{
  uint16_t adjustedCode = filteredCode + dusCodeAmmend;
  int16_t delta = 0x7FF - adjustedCode;
  adjustedCode = 0x7FF - (int16_t)(delta*dusCalibrationKoef);         // при vel>0 code<2047
  return adjustedCode;
}




// Преобразовать код с ДУС в угловую скорость с шагом 1/128.
// Для теста, постобработка сохраненных значений с ДУС
float dus_convertCodeToFloatVelocity(uint16_t code)
{
   uint8_t sign = (code>0x7FF);  
   uint16_t samples = DIFFERENCE_MODULE(code, 0x7FF);    // количество отсчетов от середины
   uint16_t sample128 = samples<<2;                      // переходим к шкале 1/128 градуса
   
   int16_t velocityI = sign ? 
                      -sample128:
                      sample128;
                      
   float velocityF = (float)(velocityI/128.0);            
   return velocityF;
}






static float k = 0.001F; //0.0001F;
static float yavg = 0;

void dusHardFilterReset()
{
  yavg = 0;
}

float dusHardFilter(float in)
{   
  yavg = k * in + (1.0F - k) * yavg;
 
  return in;
  return in - yavg;
}
