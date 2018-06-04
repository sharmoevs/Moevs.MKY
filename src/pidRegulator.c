#include "pidRegulator.h"
#include "timers.h"
#include "settings.h"
#include "canTerminal.h" // для отладки


float pi_x[3];                // рассогласования
float pi_y[3];                  // значения управляющего сигнала на выходе PI-регулятора

float diff_x[3];              // значения на входе дифференциальной части PID-регулятора (с выхода PI-регулятора)
float diff_y[3];                // значения управляющего сигнала на выходе PID-регулятора    

float koef_P = 1.0;
float koef_I = 0.5;
float koef_D = 0;
float koef_N = 0;               // коэф. N в дифференциальной части
float tDiskr = 0.0005;          // период дескритизации, сек

// Константы в уравнении PID-регулятора
float _calcI;                   // рассчетный коэф перед I в PI-регуляторе
float _calcD;                   // D*N в уравнении дифференциальной части
float _calcN;                   // (N*T-1) в уравнении дифференциальной части

float lastPidUprValue;          // последнее значение с выхода ПИД-регулятора (для отладки)
float lastDiffOut;              // последнее значение с выхода дифф. части ПИД-регулятора (для отладки)


void _pid_updateConstantCoefficients();                                 // пересчет константы
float _pid_regulatorPI(float mismatch);           // PI-регулятор
float _pid_differencialPart(float mismatch);      // дифференциальная часть PID-регулятора
float _pid_regulatorPID(float mismatch);


// Инициализация ПИД-регулятора
void pid_init()
{
  pid_reset();  
  _pid_updateConstantCoefficients();
  
  uint32_t mcs = (uint32_t)(tDiskr*1000*1000);
  gk_timer2_init(mcs);
}

// Обнуление предыдущих отсчетов
void pid_reset()
{
  for(int i=0; i<3;i++)
  {
    pi_x[i] = 0;
    pi_y[i] = 0;
    diff_x[i] = 0;
    diff_y[i] = 0;
  }
}

// Обновить константы
void _pid_updateConstantCoefficients()
{
  _calcI = koef_P*(1 - koef_I*tDiskr);
  _calcD = koef_D * koef_N;
  _calcN = (koef_N*tDiskr - 1);
}

// Установить новые коэффициенты ПИД-регулятора
void pid_setNewKoef(float newP, float newI, float newD, float newN, float newT)
{
  __disable_irq();
  koef_P = newP;
  koef_I = newI;
  koef_D = newD;
  koef_N = newN;
  
  if(tDiskr != newT)
  {  
    tDiskr = newT;      // время в секундах
    uint32_t mcs = (uint32_t)(newT*1000*1000);
    gk_timer2_init(mcs);
  }  
  
  pid_reset();  
  _pid_updateConstantCoefficients();
  __enable_irq();
}

// Запуск/Останов ПИД-регулятора
void pid_setEnable(uint8_t enable)
{
  timer_setEnable(MDR_TIMER2, enable);
}


// ПИ-регулятор
float _pid_regulatorPI(float mismatch)
{
  uint8_t k=2;  // k   : текущее измерение
                // k-1 : предыдущее
                // k-2 : предпредыдущее

  //float mismatch = presetCode - realCode;                  // рассогласование
  pi_x[k-2] = pi_x[k-1];
  pi_x[k-1] = pi_x[k];
  pi_x[k] = mismatch;

  pi_y[k-2] = pi_y[k-1];
  pi_y[k-1] = pi_y[k];
  pi_y[k]   = pi_y[k-1] + koef_P*pi_x[k] - _calcI*pi_x[k-1];  
  return pi_y[k];
}

// Дифференциальная часть ПИД-регулятора
float _pid_differencialPart(float mismatch)
{
  uint8_t k=2;  // k   : текущее измерение
                // k-1 : предыдущее
                // k-2 : предпредыдущее
  
  
  // Входное воздействие
  //float mismatch = presetCode - realCode;                  // рассогласование
  diff_x[k-1] = diff_x[k];
  diff_x[k] = mismatch;
  
  // Выход 
  diff_y[k-1] = diff_y[k];
  diff_y[k] = _calcD*diff_x[k] - _calcD*diff_x[k-1] - _calcN*diff_y[k-1];  
  return diff_y[k];
}

// ПИД-регулятор
float _pid_regulatorPID(float mismatch)
{
  float pi = _pid_regulatorPI(mismatch);
  float d = _pid_differencialPart(mismatch);
  lastDiffOut = d;
  float pid = pi+d;
  return pid;
}


// Рассчет следующего значения по кодам АЦП
float pid_nextCode(float mismatch)
{  
  float upr;
  //upr = _pid_regulatorPI(presetCode, dusCode);        // PI-регулятор
  upr = _pid_regulatorPID(mismatch);       // PID-регулятор
  
#ifdef PID_FILTER_NONE

#elif defined PID_FILTER_F2kHz_B200Hz_500Hz_20dB_80dB
  upr = _pid_2kHz_DigitalFilter_200Hz_500Hz_20dB_80dB(upr);           // Fdiskr = 2000Hz, полоса 200Hz

#elif defined PID_FILTER_F2kHz_B200Hz_SPECIFY_ORDER_2
  upr = _pid_2kHz_DigitalFilter_200Hz_order2(upr);                    // Fdiskr = 2000Hz, полоса 200Hz ver2         K=5

#elif defined PID_FILTER_F2kHz_B200Hz_500Hz_1dB_60dB
  upr = _pid_2kHz_DigitalFilter_200Hz_500Hz_1dB_60dB(upr);            // Fdiskr = 2000Hz, полоса 200Hz ver3

#elif defined PID_FILTER_F2kHz_B500Hz_700Hz_1dB_60dB
  upr = _pid_2kHz_DigitalFilter_500Hz_700Hz_1dB_60dB(upr);            // Fdiskr = 2000Hz, полоса 500Hz              K=5
  
#elif defined PID_FILTER_F2kHz_B200Hz_700Hz_2dB_26dB
  upr = _pid_2kHz_DigitalFilter_200Hz_700Hz_2dB_26dB(upr);            // Fdiskr = 2000Hz, полоса 200Hz 
#endif
  
  lastPidUprValue = upr;
  return lastPidUprValue;
}







float _pi_regulatorDeltaAngleMinusDus(float mismatch);
float g_I0 = 1;
float g_Threshold = 2048;
float g_k = 1;

// ПИ-регулятор
float _pi_regulatorDeltaAngleMinusDus(float mismatch)
{
  uint8_t k=2;  // k   : текущее измерение
                // k-1 : предыдущее
                // k-2 : предпредыдущее

  //float mismatch = presetCode - realCode;                  // рассогласование
  pi_x[k-2] = pi_x[k-1];
  pi_x[k-1] = pi_x[k];
  pi_x[k] = mismatch;

  pi_y[k-2] = pi_y[k-1];
  pi_y[k-1] = pi_y[k];
  pi_y[k]   = pi_y[k-1] + koef_P*pi_x[k] - // _calcI*pi_x[k-1];  
              koef_P*pi_x[k-1]*(1-tDiskr*(g_I0 + g_k*(g_Threshold - lastPidUprValue)));
    
  return pi_y[k];
}


//версия пид-регулятора для модели в которой к рассогласовынию углов,
// которое поступает на PI-регулятор подмешивается сигнал ДУС с коэф.
float pid_nextCodeDeltaAngleMinusDus(float mismatch, float dus)
{
  float pi = _pi_regulatorDeltaAngleMinusDus(mismatch);  
  float upr = pi - koef_D * dus;
  lastPidUprValue = upr;
  return lastPidUprValue;  
}


/*
void pid_test()
{  
  koef_P = 1.0;
  koef_I = 1.0;
  koef_D = 2;
  koef_N = 1;
  tDiskr = 0.5;
  pid_init();
    
  float piOut;
  float diffOut;
  for(int i=0; i<10; i++)
  {
    _pid_regulatorPID(1, 0);                   // PID-регулятор
    
    piOut = pi_y[2];
    diffOut = diff_y[2];
    
    canTerminal_printf("PI[%d]=%.4f\n"
                       "D[%d]=%.4f\n\n", i, piOut, i, diffOut);
  }  
}
*/