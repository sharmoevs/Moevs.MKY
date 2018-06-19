#include "coreControl.h"
#include "gkControlMode.h"
#include "timers.h"
#include "settings.h"

/*
TODO:
 1) _getCurrentPosition _getEndPosition должны выдавать значения в одних и тех же единицах в разных режимах
    пока не сделано приведение к общему виду
 2) реализовать _filter_fir_2
 3) реализовать _pid
 4) проверить _setPwm

*/

typedef enum
{
  CTRL_MODE_ARRETIER,   // режим арретир
  CTRL_MODE_VUS,        // режим ВУС
  CTRL_MODE_ENGINE_OFF  // в данном режиме двигатели выключены
} _controlMode_t;


extern GKControlMode_t gk_controlMode;             // режим управления ГК
static float vusIntegral = 0;


_controlMode_t  _getCurrentMode();
float           _getEndPosition(_controlMode_t mode);
float           _getCurrentPosition(_controlMode_t mode);
float           _filter_fir_2(float z);
float           _saturation(float in, float *correction);
float           _pid(float delta);
void            _setPwm(float pidOut);


// Возвращает текущий режим управления (ВУС, АР или двигатели выкл)
_controlMode_t _getCurrentMode()
{
  switch(gk_controlMode)
  {
    case GkMode_EngineDisabled: // Запрет управлением двигателями    
    case GkMode_EngineOffBySpeedProtection: // Двигатели выключены после срабатывания защиты по скорости
    case GkMode_EngineOffByCommand: // Двигатели выключены командой
    case GkMode_HardFault: // Hardfault, Безнадежная ошибка
      return CTRL_MODE_ENGINE_OFF;
      
    case GkMode_AR: // Внешнее управление по углу
    case GkMode_TP: // Транспортное положение
    case GkMode_Initialize: // Инициализация по включению питания
    case GkMode_TudaSudaAR: // Движение туда-сюда в Арретире
      return CTRL_MODE_ARRETIER;
      
    case GkMode_VUS: // Внешнее управление по угловым скоростям (ВУС)
    case GkMode_VelocityCalibration:
    case GkMode_TudaSudaVUS:  // Движение туда-сюда ВУС
      return CTRL_MODE_VUS;// Калибровка по скорости
      
      
    // TODO
    case GkMode_VUO: // Внешнее управление ориентацией
    case GkMode_SelfControl: // Самоконтроль
    case GkMode_DUSTemperatureCalibration: // Калибровка ДУС по температуре
      return CTRL_MODE_ENGINE_OFF;
      
    default: return CTRL_MODE_ENGINE_OFF;
  }
}


// Получить конечное значение
float _getEndPosition(_controlMode_t mode)
{
  if(mode == CTRL_MODE_ARRETIER)
  {
    extern uint32_t arretierRequiredAngleU32;
    return arretierRequiredAngleU32;
  }
  else
  {
    extern int16_t constantSpeedCode;          // код АЦП, соответствующей желаемой постоянной скорости вращения
    return constantSpeedCode;
  }
}

// Получить текущее значение угла или скорости
float _getCurrentPosition(_controlMode_t mode)
{
  if (mode == CTRL_MODE_ARRETIER)
  {
    extern uint32_t g_sysAngle360;
    return g_sysAngle360;
  }
  if(mode == CTRL_MODE_VUS)  // VUS
  {
    //extern int16_t dusAmplitude;
    //extern float arretier_AngleToVelKoef;
    //vusIntegral += GetSpeedFromVUS() * T * K;  // K - коэффиицент пересчёта в те же единицы, что и GetPosition для АРРЕТИР, T - период
    
    return vusIntegral;
  }
  return 0;
}
// Сбросить накопленное значение
void resetVUSIntegral()
{
  vusIntegral = 0;
}

// Фильтр второго порядка
float _filter_fir_2(float z)
{
  return z;  // TODO: filter
}

// Противонакопление
float _saturation(float in, float *correction)
{
#define MAX_VALUE 1000
  
  float out;
  if(in > MAX_VALUE) out = MAX_VALUE;
  else if(in < -MAX_VALUE) out = -MAX_VALUE;
  else out = in;
  
  *correction = (in - out);
  return out;
}


// ПИД-регулятор
float _pid(float delta)
{
  return delta;
}

// Установить заполнение ШИМ
void _setPwm(float pidOut)
{
#define OUT_THRESHOLD   3000
     
   uint16_t ccr3;    
   if(pidOut > OUT_THRESHOLD) ccr3 = TIM_PWM_60_PERCENTAGE_CCR3_REG;
   else
   {
     ccr3 = (uint16_t)pidOut;
   }
      
   MDR_TIMER1->CCR3 = ccr3;
}


// Алгоритм управления
void coreMove()
{
  static float errorCorrection = 0;
  
  
  _controlMode_t mode = _getCurrentMode();
  if(mode == CTRL_MODE_ENGINE_OFF) return;
  
  float endPosition = _getEndPosition(mode); 
  float currentPosition = _getCurrentPosition(mode);
  float delta = endPosition - currentPosition;
  
  delta = _filter_fir_2(delta);
  float regulatorIn = delta - errorCorrection;
  float pwmWithoutSaturation = _pid(delta);
  float out = _saturation(pwmWithoutSaturation, &errorCorrection);
  _setPwm(out);
}

