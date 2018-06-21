#include "coreControl.h"
#include "gkControlMode.h"
#include "timers.h"
#include "settings.h"
#include "angelSensor.h"
#include "dus.h"
#include "engineControl.h"

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
extern uint32_t g_sysAngle360;
extern uint32_t arretierRequiredAngleU32;          // заданный угол в арретире
extern int16_t dusAmplitude;
extern int16_t constantSpeedCode;                  // заданная скорость в режиме ВУС

extern float koef_P;
extern float koef_I;
extern float koef_D;
extern float lastPidUprValue;

static float vusIntegral = 0;


_controlMode_t  _getCurrentMode();
float           _getPosition(_controlMode_t mode, float value);

float           _getCurrentPosition(_controlMode_t mode);
float           _getEndPosition(_controlMode_t mode);

float           _saturation(float in, float *correction);
void            _setPwm(float pidOut);


void F_2_reset();
float F_2(float x);
float PID(float mismatch);
void PID_reset();


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


// Получить текущее значение угла или скорости
float _getPosition(_controlMode_t mode, float value)
{
  float result;
  if (mode == CTRL_MODE_ARRETIER)
  {
    result = USYSANGLE_TO_FLOAT(value);
    return result;
  }
  if(mode == CTRL_MODE_VUS)  // VUS
  {
    float speed = DUS_CONVERT_TO_DEGREES_PER_SEC(value);
    static float T = 0.0005/1000.0;   // 500мкс
    
    vusIntegral += speed * T;    
    return vusIntegral;
  }
  return 0;
}




// Сбросить накопленное значение
void resetVUSIntegral()
{
  vusIntegral = 0;
}


// Противонакопление
float _saturation(float in, float *correction)
{
#define MAX_VALUE 60
  
  float out;
  if(in > MAX_VALUE) out = MAX_VALUE;
  else if(in < -MAX_VALUE) out = -MAX_VALUE;
  else out = in;
  
  *correction = (out - in);
  return out;
}


// Установить заполнение ШИМ
// pid - от -100 до 100 
void _setPwm(float pid)
{
#define PWM_THRESHOLD   60.0
  static float pwmThresholdCode = TIM_PWM_GET_CCR3(PWM_THRESHOLD);
  
  float pidAbs = pid < 0 ? -pid : pid;
  if(pidAbs >= PWM_THRESHOLD)
  {
    MDR_TIMER1->CCR3 = (uint16_t)pwmThresholdCode;
  }
  else
  {
    MDR_TIMER1->CCR3  = TIM_PWM_GET_CCR3(pidAbs);
  }
  
  if(pid > 0) engine_spinCW();
  else engine_spinCCW();
}






// Фильтр второго порядка, переменные

typedef struct
{
  float y_prev;
  float x_prev;
  
  float a0;
  float b0;
  float b1;
} f_2_struct_t;

static f_2_struct_t f_2_struct = {.a0 = 0,
                                  .b0 = 0,
                                  .b1 = 0,
                                  .y_prev = 0,
                                  .x_prev = 0};

// Фильтр 2-го порядка для фильтрации рассогласования на входе регулятора
// Разностная формула: (b1z + b0) / (z + a0)
// Формула: y = -a0 * y_prev + b1 * x_prev + b0 * x
float F_2(float x)
{
  float y = -f_2_struct.a0 * f_2_struct.y_prev + f_2_struct.b1 * f_2_struct.x_prev + f_2_struct.b0 * x;
  
  f_2_struct.x_prev = x;
  f_2_struct.y_prev = y;
  
  return y;
}

void F_2_reset()
{
  f_2_struct.x_prev = 0;
  f_2_struct.y_prev = 0;
}



// PID регулятор
typedef struct
{
  float integral;
  float prev_x;
} pid_struct_t;

pid_struct_t pid_struct = {.integral = 0,
                           .prev_x   = 0};

float PID(float mismatch)
{
  float d = mismatch - pid_struct.prev_x;
  
  float sum = koef_P * mismatch + koef_I * pid_struct.integral + koef_D * d;
  
  pid_struct.integral += mismatch;
  pid_struct.prev_x = mismatch;
  
  return sum;
}

void PID_reset()
{
  pid_struct.integral = 0;
  pid_struct.prev_x = 0;
}








// Инициализация
void coreControlInit()
{
  PID_reset();
  F_2_reset();
}


// Алгоритм управления
void coreMove()
{
  static float errorCorrection = 0;
  _controlMode_t mode = _getCurrentMode();
  
  float startValue = (mode == CTRL_MODE_ARRETIER) ? g_sysAngle360 : dusAmplitude;
  float endValue = (mode == CTRL_MODE_ARRETIER) ? arretierRequiredAngleU32 : constantSpeedCode;

  float currentPosition = _getPosition(mode, startValue);
  float endPosition = _getPosition(mode, endValue); 
  
  float delta = endPosition - currentPosition;  
  delta = F_2(delta);
  
  float regulatorIn = delta - errorCorrection;
  float pwmWithoutSaturation = PID(regulatorIn);
  float out = _saturation(pwmWithoutSaturation, &errorCorrection);
  
  lastPidUprValue = out;
  
  
  if(mode == CTRL_MODE_ENGINE_OFF) return;
  _setPwm(out);
}

