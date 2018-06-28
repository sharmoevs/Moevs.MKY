#include "coreControl.h"
#include "gkControlMode.h"
#include "timers.h"
#include "settings.h"
#include "angelSensor.h"
#include "dus.h"
#include "engineControl.h"
#include <stdlib.h>


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


void F_1_reset();
float F_1(float x);
float F_2(float x);
void F_2_reset();
float PID(float mismatch, float mismatch_for_i);
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
    //case GkMode_Initialize: // Инициализация по включению питания
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
    
    // result = F_2(result);
    
    return result;
  }
  if(mode == CTRL_MODE_VUS)  // VUS
  {
    extern float g_currentSpeedFromDus;
    extern float g_averageSpeedCorrection;
    float curSpeed = g_currentSpeedFromDus + g_averageSpeedCorrection;
    float endSpeed = DUS_CONVERT_TO_DEGREES_PER_SEC(constantSpeedCode);    
    
    if(endSpeed == 0) // Калибровка в режиме ВУС0
    {      
      curSpeed = dusHardFilter(curSpeed);
    }
    else dusHardFilterReset();
    
    float delta = curSpeed - endSpeed;

    vusIntegral += delta * DUS_SAMPLING_PERIOD_sec;   
    return vusIntegral;
    
    /*
    float endspeed = DUS_CONVERT_TO_DEGREES_PER_SEC(constantSpeedCode);    
    float speed = DUS_CONVERT_TO_DEGREES_PER_SEC(value) - endspeed;
       
    vusIntegral += speed * DUS_SAMPLING_PERIOD_sec;    
   
    return vusIntegral; 
    */
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
//#define Ka 0.7192F
  
  static float Ka = 0.7192F;
#if DUS_SAMPLING_FREQUENCY == 8000
  // Lreq = 18.4   Lrho = 1.2  freq = 8000
  //Ka = 1.294208F;
  
// Lreq = 36.8   Lrho = 1.2  freq = 8000 - срывается 
//  Ka = 0.1608F;
  
  // Lreq = 23   Lrho = 1.2  freq = 8000
  Ka = 0.6611F;
  
  // Lreq = 18.4   Lrho = 1.2  freq = 8000, Kmd
  // Ka = 2.1375F;
#endif
  
  float out;
  if(in > MAX_VALUE) out = MAX_VALUE;
  else if(in < -MAX_VALUE) out = -MAX_VALUE;
  else out = in;
  
  *correction = Ka * (out - in);
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






// Фильтр первого порядка, переменные

typedef struct
{
  float y_prev;
  float x_prev;
  
  float a0;
  float b0;
  float b1;
} f_1_struct_t;

// Работает
// static f_1_struct_t f_1_struct = {.a0 = -0.9484F,
//                                   .b0 = 1,
//                                   .b1 = -0.02641F,
//                                   .y_prev = 0,
//                                   .x_prev = 0};


// Эксперимент
// static f_1_struct_t f_1_struct = {.a0 = -0.844128F,
//                                   .b0 = 1,
//                                   .b1 = -0.08053F,
//                                   .y_prev = 0,
//                                   .x_prev = 0};

// эксперимент медленный Treq=1 Lro = 1.8. Работало в ВУС и арретир
// static f_1_struct_t f_1_struct = {.a0 = -0.973452F,
//                                   .b0 = 1,
//                                   .b1 = -0.013727F,
//                                   .y_prev = 0,
//                                   .x_prev = 0};

#if DUS_SAMPLING_FREQUENCY != 8000

// эксперимент более быстрые Treq=0.25, Lro=1.2. Хорошо работает для ВУС, хуже для арретир
static f_1_struct_t f_1_struct = {.a0 = -0.952194F,
                                  .b0 = 1,
                                  .b1 = -0.24488F,
                                  .y_prev = 0,
                                  .x_prev = 0};

#else // Для f = 8000

// Lreq = 18.4   Lrho = 1.2  freq = 8000
static f_1_struct_t f_1_struct = {.a0 = -0.9878F,
                                  .b0 = 1,
                                  .b1 = -0.7516F,
                                  .y_prev = 0,
                                  .x_prev = 0};

// Lreq = 18.4   Lrho = 1.2  freq = 8000, Kmd
// static f_1_struct_t f_1_struct = {.a0 = -0.9924F,
//                                   .b0 = 1,
//                                   .b1 = -0.7557F,
//                                   .y_prev = 0,
//                                   .x_prev = 0};



// Lreq = 36.8   Lrho = 1.2  freq = 8000 - срывается 
//static f_1_struct_t f_1_struct = {.a0 = -0.9957F,
//                                  .b0 = 1,
//                                  .b1 = -0.7531F,
//                                  .y_prev = 0,
//                                  .x_prev = 0};


  // Lreq = 23   Lrho = 1.2  freq = 8000
// static f_1_struct_t f_1_struct = {.a0 = -0.9848F,
//                                   .b0 = 1,
//                                   .b1 = -0.7520F,
//                                   .y_prev = 0,
//                                   .x_prev = 0};
#endif

// Фильтр 1-го порядка для фильтрации рассогласования на входе регулятора
// Разностная формула: (b1z + b0) / (z + a0)
// Формула: y = -a0 * y_prev + b1 * x_prev + b0 * x
float F_1(float x)
{
  float y = -f_1_struct.a0 * f_1_struct.y_prev + f_1_struct.b1 * f_1_struct.x_prev + f_1_struct.b0 * x;
  
  f_1_struct.x_prev = x;
  f_1_struct.y_prev = y;
  
  return y;
}

void F_1_reset()
{
  f_1_struct.x_prev = 0;
  f_1_struct.y_prev = 0;
}



// Фильтр второго порядка, переменные

typedef struct
{
  float y_prev;
  float x_prev;
  float y_prev_2;
  float x_prev_2;
  
  float a1;
  float a2;
  float b0;
  float b1;
  float b2;
  
} f_2_struct_t;


#if DUS_SAMPLING_FREQUENCY != 8000

// Рабочие коэффициенты (de facto фильтр первого порядка )
static f_2_struct_t f_2_struct = {.a1 = -0.350920F,
                                  .a2 = 0.0F,
                                  .b0 = 0.649080F,
                                  .b1 = 0,
                                  .b2 = 0.0F,
                                  .y_prev = 0,
                                  .x_prev = 0,
                                  .x_prev_2 = 0,
                                  .y_prev_2 = 0};

// Не рабочие коэффициенты. Таким фильтр должен быть по модели
// static f_2_struct_t f_2_struct = {.a1 = -0.704020F,
//                                   .a2 = 0.227419F,
//                                   .b0 = 0.326263F,
//                                   .b1 = 0.197136F,
//                                   .b2 = 0.0F,
//                                   .y_prev = 0,
//                                   .x_prev = 0,
//                                   .x_prev_2 = 0,
//                                   .y_prev_2 = 0};

#else // Для f = 8000

 // Lreq = 23   Lrho = 1.2  freq = 8000
// freqСut = 333Гц
// static f_2_struct_t f_2_struct = {.a1 = -1.6336F,
//                                  .a2 = 0.69056F,
//                                  .b0 = 0.030235F,
//                                  .b1 = 0.026721,
//                                  .b2 = 0.0F,
//                                  .y_prev = 0,
//                                  .x_prev = 0,
//                                  .x_prev_2 = 0,
//                                  .y_prev_2 = 0};


// freqСut = 1rГц
//static f_2_struct_t f_2_struct = {.a1 = -0.975239F,
//                                  .a2 = 0.329322F,
//                                  .b0 = 0.209810F,
//                                  .b1 = 0.144272,
//                                  .b2 = 0.0F,
//                                  .y_prev = 0,
//                                  .x_prev = 0,
//                                  .x_prev_2 = 0,
//                                  .y_prev_2 = 0};


//// Фильтр первого порядка freqCut = 1кГц - рабочий
static f_2_struct_t f_2_struct = {.a1 = -0.455938F,
                                  .a2 = 0.0F,
                                  .b0 = 0.544062,
                                  .b1 = 0.0,
                                  .b2 = 0.0F,
                                  .y_prev = 0,
                                  .x_prev = 0,
                                  .x_prev_2 = 0,
                                  .y_prev_2 = 0};

//// freqCut = 500Гц
//static f_2_struct_t f_2_struct = {.a1 = -0.854636F,
//                                  .a2 = -0,
//                                  .b0 =  0.145364,
//                                  .b1 = 0.0,
//                                  .b2 = 0.0F,
//                                  .y_prev = 0,
//                                  .x_prev = 0,
//                                  .x_prev_2 = 0,
//                                  .y_prev_2 = 0};

// задержка по времени
// static f_2_struct_t f_2_struct = {.a1 = -0.0F,
//                                   .a2 = 0.0F,
//                                   .b0 = 1,
//                                   .b1 = 0.0,
//                                   .b2 = 0.0F,
//                                   .y_prev = 0,
//                                   .x_prev = 0,
//                                   .x_prev_2 = 0,
//                                   .y_prev_2 = 0};

#endif



// Фильтр 2-го порядка для фильтрации координаты
float F_2(float x)
{
  float y = -f_2_struct.a1 * f_2_struct.y_prev - 
             f_2_struct.a2 * f_2_struct.y_prev_2 + 
             f_2_struct.b0 * x + 
             f_2_struct.b1 * f_2_struct.x_prev +
             f_2_struct.b2 * f_2_struct.x_prev_2;
  
  f_2_struct.x_prev_2 = f_2_struct.x_prev;
  f_2_struct.y_prev_2 = f_2_struct.y_prev;
  
  f_2_struct.x_prev = x;
  f_2_struct.y_prev = y;
  
  return y;
}

void F_2_reset()
{
  f_2_struct.x_prev = 0;
  f_2_struct.y_prev = 0;
  f_2_struct.x_prev_2 = 0;
  f_2_struct.y_prev_2 = 0;
}





// Префильтр

f_1_struct_t prefilter = {.a0 = -0.9981F,
                          .b0 = 0.0F,
                          .b1 = 0.0019F,
                          .y_prev = 0,
                          .x_prev = 0 };


float filter_1(f_1_struct_t *filter,  float x);
void  filter_1_init(f_1_struct_t *filter,  float startValue);
void  filter_1_reset(f_1_struct_t *filter);

float filter_1(f_1_struct_t *filter,  float x)
{
  float y = -filter->a0 * filter->y_prev + filter->b1 * filter->x_prev + filter->b0 * x;
  
  filter->x_prev = x;
  filter->y_prev = y;
  
  return y;
}

void filter_1_init(f_1_struct_t *filter, float startValue)
{
  filter->x_prev = startValue;
  filter->y_prev = startValue;
}

void filter_1_reset(f_1_struct_t *filter)
{
  filter->x_prev = 0;
  filter->y_prev = 0;
}







// PID регулятор
typedef struct
{
  float integral;
  float prev_x;
} pid_struct_t;

pid_struct_t pid_struct = {.integral = 0,
                           .prev_x   = 0};

float PID(float mismatch, float mismatch_for_i)
{
  float d = mismatch - pid_struct.prev_x;
  
  float sum = koef_P * mismatch + koef_I * pid_struct.integral + koef_D * d;
  
  pid_struct.integral += mismatch_for_i;
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
  F_1_reset();
  
  
  angle_getNextSampleX32();     // отсчет с датчика угла 
  filter_1_init(&prefilter, USYSANGLE_TO_FLOAT(g_sysAngle360));
}

float dbgDusIntegral;

static float runningValue = 0;

void onARStartMoving()
{
  float startValue = g_sysAngle360;
  float currentPosition = _getPosition(CTRL_MODE_ARRETIER, (float)startValue);
  
  runningValue = currentPosition;
}

float g_pidOut;
float g_Udump;
float g_pidDelta;
float g_profileDelta;

// Алгоритм управления
void coreMove()
{
  static float errorCorrection = 0;
  _controlMode_t mode = _getCurrentMode();
  
  float startValue = (mode == CTRL_MODE_ARRETIER) ? g_sysAngle360 : (float)dusAmplitude;
  float endValue = (mode == CTRL_MODE_ARRETIER) ? arretierRequiredAngleU32 : constantSpeedCode;

  float currentPosition = _getPosition(mode, (float)startValue);
  float endPosition = (mode == CTRL_MODE_ARRETIER) ? _getPosition(mode, endValue) : 0; 
  
  
  /*
   * Профилировщик позиции (только для АРРЕТИР!)
   */
  #define sgn(x) ((x) >= 0 ? +1 : -1)
  #define fabs(x) ((x) > 0 ? (x) : -(x))
  #define PROFILE_FREQ (8000) // in Hz
  #define PROFILE_FREQ_COUNT (8000 / PROFILE_FREQ)  // TODO: 8 kHz parameter in firmware
  #define SPEED (20.0F) // in grad/sec
  static uint32_t ProfileCounter = 0;
  
  if (ProfileCounter++ >= PROFILE_FREQ_COUNT)
  {
      ProfileCounter = 1;
      
      // это значение должно быть менее ~ 3 градусов из-за оосбенности контура управления
      float profileDelta = sgn(endPosition - runningValue) * SPEED / ((float)PROFILE_FREQ);
      
      g_profileDelta = runningValue;
      
      if (fabs(runningValue - endPosition) < fabs(profileDelta))
      {
        runningValue = endPosition;
      }
      else
      {
        runningValue += profileDelta;
      }    
  } 
  
  
  // текущая позиция для контура ограничения скорости (Speed Saturation Contour)
  static float currentPositionSSCPrev = 0;
  float currentPositionSSC = F_2(_getPosition(mode, (float)startValue));
  float currentSpeedSSC = (currentPositionSSC - currentPositionSSCPrev) * 8000.0F; // 8 kHz TODO: constant
  currentPositionSSCPrev = currentPositionSSC;
  
    
  dbgDusIntegral = currentPosition;
  
  // профилировщик не имеет смысла для ВУС
  
  // отключить профилировщик для тестов
  // if (mode == CTRL_MODE_VUS)
    runningValue = endPosition;
  
  // префильтр
  if(mode == CTRL_MODE_ARRETIER) endPosition = filter_1(&prefilter, runningValue);
  
  float delta = runningValue - currentPosition;  
  g_pidDelta = delta;
  
  delta = F_1(delta);
  
  
  
  float delta_for_i = delta - errorCorrection;
  float pwmWithoutSaturation = PID(delta, delta_for_i);
  float out = _saturation(pwmWithoutSaturation, &errorCorrection);
  g_pidOut = out;
  
  const float Kmd = 0; //1.6312F;
  
  // ограничение по скорости здесь
  g_Udump = Kmd * currentSpeedSSC;
  out = out - g_Udump;
  
  if(mode == CTRL_MODE_ENGINE_OFF) return;
  _setPwm(out);
}

