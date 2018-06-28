#include "gkControlMode.h"
#include "engineControl.h"
#include "pidRegulator.h"
#include "timers.h"
#include "global.h"
#include "mkpinout.h"
#include "canmonitor.h"
#include "angelSensor.h"
#include "tangageControl.h"
#include "differentiator.h"
#include "digitalFilter.h"

#define DBG_START_IN_ENGINE_OFF_MODE        // ДЛЯ ОТЛАДКИ - запуститься с выключенными двигателями

extern int16_t dusAmplitude;
extern uint32_t courseAngleCodeX32;
extern uint32_t g_sysAngle360;  
extern uint32_t g_hardwareZeroAngleCorrection;
extern uint32_t g_logicalZeroAngleCorrection;
extern uint32_t g_tangageSysAngle360;
extern uint32_t g_tangageHardwareZeroAngleCorrection;
extern uint32_t g_tangageLogicalZeroAngleCorrection;
extern uint32_t tangageArretieredRequiredAngleU32;
extern uint32_t g_prevSysAngle360;
extern uint8_t gs_preparationNow;

GKControlMode_t gk_controlMode = GkMode_Initialize;             // режим управления ГК
GSFailure_t     gs_failure     = GgFailure_None;                // код возникшей ошибки

int16_t constantSpeedCode = 0;          // код АЦП, соответствующей желаемой постоянной скорости вращения

float    arretier_AngleToVelKoef = ((ARRETIR_VELOCITY*32)/AR_THRESHOLD); // коэф. преобразования угла в скорость 640 - код скорости 20гр/сек, 2912 - код 2градусов с датчика угла
uint32_t arretierRequiredAngleU32 = 0;  // арретир для управления по углу
uint8_t  courseArretieredNow = 0;       // курсовой контур находится в арретире
uint8_t  tangageArretieredNow = 0;      // тангажный контур находится в арретире

uint8_t  dusSpeedCalibrationStage;      // стадия калибровки по скорости
double   newDusCalibrationKoef;         // новый коэф. пропорциональности ДУС

uint16_t modeTudaSudaVUSPeriod;         // время вращение в одну сторону в режиме Туда-Сюда
int16_t  modeTudaSudaVUSVelocityCode;   // код скорости вращения в режиме Туда-Сюда. Шаг 1/32 градуса

uint16_t modeTudaSudaARPeriod;
uint32_t modeTudaSudaARAngle1;
uint32_t modeTudaSudaARAngle2;

float    averageVelocityFromAngle;      // средняя сокрость, рассчитанная по углу


VUOStage_t vuoStage;                    // стадия ВУО
FuncTestStage_t functionalTestStage;    // стадия функционального теста
FuncTestErrorCode_t funcTestErrorCode;  // код ошибки функционального теста, для определения какой тест выдал ошибку
uint8_t functionalTestDone = 1;         // завершен функциональный тест
uint8_t funcTest_arNumOfTestAngles;     // перебол углов при арретировании
int16_t funcTestArAngle128;             // угол в котором происходит арретирование в фунециональном тесте
uint32_t dbg_endOfFuncTestTime;         // время завершения функционального теста (для отладки)
uint8_t dbg_infiniteFuncTest = 0;

uint8_t dbg_useDusWhileArretiered = 1;  // ОТЛАДКА - режим арретирования - с использованием ДУС или без него (угловую скорость брать как изменение угла)
int32_t g_dbg_distanceTraveledCode;     // на сколько повернулись за определенное время. Для рассчета скорости по углу
float   g_dbg_velocityFromAngleF = 0;



// Управление
void _gk_nextInitialize();

void _gk_nextAR();
void _gk_nextAR_withoutDUS();
void _gk_nextAR_withDiffAndFilter();

void _gk_nextVUS();
void _gk_nextVelocityCalibration();
void _gk_nextTudaSudaVUS();
void _gk_nextTudaSudaAR();
void _gk_nextVUO();
void _gk_nextFuncTest();

void    _funcTest_copmleteSuccess();    // функциональный тест завершен успешно
void    _funcTest_errorOccured(FuncTestErrorCode_t errorCode); // функиональный тест завершен с ошибкой
uint8_t _funcTest_setNextArretiredAngle(FuncTestArrieterMode_t arMode, uint8_t *numOfAngle, uint32_t *maxTransferTime);

void     gk_setModeDisableEngineAtStartup();
uint8_t  _gk_canChangeMode();           // возможность сменить режим работы
void     _gk_spinVus(int16_t desiredDusCode);
void     gk_setConstVelocityCode(int16_t velocityCode32);
void     gk_PidRegulatorSpin(float upr);
void     setPwmFromUprCode(float uprCode);


// Инициализация
void gk_init()
{  
#ifdef DBG_START_IN_ENGINE_OFF_MODE
  gs_preparationNow = 0;
  gk_controlMode = GkMode_EngineOffByCommand;
#else
  gs_preparationNow = 1;
  gk_controlMode = GkMode_Initialize;
#endif
}

// Установить код ошибки
void gk_setFailure(GSFailure_t error)
{
  gs_failure |= error;
}
// Снять код ошибки
void gk_clearFailure(GSFailure_t error)
{
  gs_failure &= ~error;
}

// Проверить режим работы
void gk_checkNSetupPin()
{
  if(IS_ENGINE_ENABLE) return;
  
  gk_setModeDisableEngineAtStartup();
}


void gk_moveNext()
{
  switch(gk_controlMode)
  {
    case GkMode_Initialize:             _gk_nextInitialize(); break;            // Инициализация
    case GkMode_VUS:                    _gk_nextVUS(); break;                   // ВУС                        
    //case GkMode_AR:                     _gk_nextAR(); break;                    // арретир    
    case GkMode_AR:// арретир БЕЗ ДУС
      if(dbg_useDusWhileArretiered) _gk_nextAR();
      else 
      {
        _gk_nextAR_withoutDUS();
        //_gk_nextAR_withDiffAndFilter();
      }
      break;
    case GkMode_VelocityCalibration:    _gk_nextVelocityCalibration(); break;   // калибровка по скорости
    case GkMode_TudaSudaVUS:            _gk_nextTudaSudaVUS(); break;           // туда-сюда ВУС
    case GkMode_TudaSudaAR:             _gk_nextTudaSudaAR(); break;            // туда-сюда АР
    case GkMode_TP:                     _gk_nextAR(); break;                    // транспортное положение                            
    case GkMode_VUO:                    _gk_nextVUO(); break;                   // ВУО
    case GkMode_SelfControl:            _gk_nextFuncTest(); break;              // самоконтроль
    case GkMode_DUSTemperatureCalibration: _gk_nextAR(); break;                 // калибровка по температуре
    
    case GkMode_EngineDisabled:
    case GkMode_EngineOffBySpeedProtection:
    case GkMode_EngineOffByCommand:
      break;
  }
}


// Защита по скорости
void gk_checkSpeedProtection()
{
#ifndef ENABLE_VELOCITY_PROTECTION
  return;
#endif
  
  static uint8_t protectionWorked = 0;
  static uint8_t numOfThresholdExceeded = 0;    // количество превышений порога
  if(!angle_getAverageVelocity(&averageVelocityFromAngle)) return;
  if(protectionWorked) return;
  
  // Получено новое значение средней скорости по углу
  if(ABS(averageVelocityFromAngle) >= VELOCITY_PROTECTION_THRESHOLD)
  {
     numOfThresholdExceeded++;
     if(numOfThresholdExceeded >= 5)
     {
       protectionWorked = 1;
       gk_setModeEngineOffBySpeedProtection();       
     }
  }
  else 
  {
    if(numOfThresholdExceeded != 0) numOfThresholdExceeded--;
  }
}



// =============================================================================
// =============================================================================
// ============================ Установка режима работы ========================
// =============================================================================
// =============================================================================

// Возвращает 1, если можно сменить режим работы, в противном случае 0
uint8_t _gk_canChangeMode()
{
  if(gk_controlMode == GkMode_EngineDisabled || 
     gk_controlMode == GkMode_EngineOffBySpeedProtection)
  {
    return 0;
  }
  
  return 1;
}

// Установить запрет управления двигателями по включению питания
void gk_setModeDisableEngineAtStartup()
{
  gk_controlMode = GkMode_EngineDisabled;
  tangageCtrl_setModeDisableEngine();
}

// Выключить двигатели после срабатывания защиты по скорости
void gk_setModeEngineOffBySpeedProtection()
{
  extern int8_t prevSectorCW;
  extern int8_t prevSectorCCW;
  
  gk_setFailure(GgFailure_CourseSpeedProtect);
 
  gk_controlMode = GkMode_EngineOffBySpeedProtection;
  pid_setEnable(0);
  engine_disableSpin();
  prevSectorCW = -1;
  prevSectorCCW = -1;
}

// Арретир
void gk_setModeAR(uint32_t sysAngleU)                                          //*****
{
  extern void onARStartMoving();
  
  if(!_gk_canChangeMode()) return;
  
   __disable_irq();
  gk_controlMode = GkMode_AR;
  arretierRequiredAngleU32 = sysAngleU;
  __enable_irq();
  
  onARStartMoving();
  pid_setEnable(1);
}

// ВУС
// velocityCode32 - угловая скорость с шагом 1/32 градуса
void gk_setModeVUS(int16_t velocityCode32)
{
  if(!_gk_canChangeMode()) return;
  
  __disable_irq();
  gk_controlMode = GkMode_VUS;
  gk_setConstVelocityCode(velocityCode32);
  __enable_irq();
  pid_setEnable(1);
}

// Установить режим калибровки по угловым скоростям
void gk_setModeVelocityCalibration()
{
  if(!_gk_canChangeMode()) return;
  
  __disable_irq();
  gk_controlMode = GkMode_VelocityCalibration;
  dusSpeedCalibrationStage = 0;
  __enable_irq();
  pid_setEnable(1);
}

// Движение туда-сюда ВУС
// velocityCode32 - угловая скорость с шагом 1/32 градуса
void gk_setModeTudaSudaVUS(uint16_t period, int16_t velocityCode32)
{
  if(!_gk_canChangeMode()) return;
  
  __disable_irq();
  gk_controlMode = GkMode_TudaSudaVUS;  
  modeTudaSudaVUSPeriod = period;
  modeTudaSudaVUSVelocityCode = velocityCode32;
  pid_setEnable(1);
  __enable_irq();
}

// Движение туда-сюда АРРЕТИР
// angle1, angle2 - углы арретирования
void gk_setModeTudaSudaAR(uint16_t period, uint32_t angle1, uint32_t angle2)
{
  if(!_gk_canChangeMode()) return;
  
  __disable_irq();
  gk_controlMode = GkMode_TudaSudaAR;
  modeTudaSudaARPeriod = period;
  modeTudaSudaARAngle1 = angle1;
  modeTudaSudaARAngle2 = angle2;
  pid_setEnable(1);
  __enable_irq();
}

// Выключение двигателей
void gk_setModeEngineOff()
{
  if(!_gk_canChangeMode()) return; 
  
  extern int8_t prevSectorCW;
  extern int8_t prevSectorCCW;
  
  gk_controlMode = GkMode_EngineOffByCommand;
  pid_setEnable(0);
  engine_disableSpin();
  prevSectorCW = -1;
  prevSectorCCW = -1;
}

// Транспортное положение в АППАРАТНОЙ ШКАЛЕ
void gk_setModeTP(uint32_t angleU)
{
  if(!_gk_canChangeMode()) return;
  
  __disable_irq();
  gk_controlMode = GkMode_TP;
  arretierRequiredAngleU32 = angleU;
  __enable_irq();
  pid_setEnable(1);
}

// ВУО - внешнее управление ориентацией
// velocityCode32 - угловая скорость с шагом 1/32 градуса
void gk_setModeVOU(uint32_t sysAngleU, int16_t velocityCode32)
{
  if(!_gk_canChangeMode()) return;
  
  __disable_irq();
  vuoStage = VUO_INIT;    // Инициализация режима ВУО
  gk_controlMode = GkMode_VUO;
  
  arretierRequiredAngleU32 = sysAngleU;
  gk_setConstVelocityCode(velocityCode32);
  
  __enable_irq();
  pid_setEnable(1);
}


// Самоконтроль
uint8_t gk_setModeSelfControl()
{
  if(!_gk_canChangeMode()) return 0;
  
  __disable_irq();
  functionalTestStage = FUNCTEST_INIT;    // Инициализация режима Самоконтроль
  gk_controlMode = GkMode_SelfControl;  
  __enable_irq();
  pid_setEnable(1);

  return 1;
}

// Калибровка ДУС по температуре
void gk_setModeDusTemperatureCalibration()
{
  if(!_gk_canChangeMode()) return;
    
  __disable_irq();
  gk_controlMode = GkMode_DUSTemperatureCalibration;
  arretierRequiredAngleU32 = g_sysAngle360;
  __enable_irq();
  pid_setEnable(1);
}



// =============================================================================
// =============================================================================
// ================================= Режимы работы =============================
// =============================================================================
// =============================================================================

// Инициализация
void _gk_nextInitialize()
{
  static uint8_t stage = 0;  
  switch(stage)
  {
    case 0:
      {
        if(system_time < 1000) return;
        static uint8_t cnt = 0;
        if(cnt++ < 3)
        {
          //uint32_t rangageAngle = angle_convertCmdS2SysU(0*128, g_tangageHardwareZeroAngleCorrection, MAX_TANGAGE_SYSANGLE_CODE);
          //tangageCtrl_setStartupCalibrationAngle(rangageAngle);
          tangageCtrl_sendAngleCorretion(g_tangageHardwareZeroAngleCorrection, g_tangageLogicalZeroAngleCorrection);
        }
        else stage = 1;
      }
      break;
      
    case 1:
      {
        if(system_time < ENGINE_STARTUP_DELAY) return;
        arretierRequiredAngleU32 = angle_convertCmdS2SysU(0*128, g_hardwareZeroAngleCorrection, MAX_SYSANGLE_CODE);
        stage = 2;
      }
      break;
      
    case 2:
        _gk_nextAR();
      break;
  }  
}

// Арретир
void _gk_nextAR()                                                               // ***********
{  
  uint32_t startAngle = g_sysAngle360;
  uint32_t stopAngle = arretierRequiredAngleU32;
  int32_t deltaFi = angle_getMinimalAngleRange(startAngle, stopAngle, USYSANGLE_TO_CODE(360));
  courseArretieredNow = (ABS(deltaFi) <= COURSE_ARRETIERED_DELTA_FI); // флаг нахождения в арретире КУРСОВОГО контура
  
  if((deltaFi>=0) && (deltaFi>AR_THRESHOLD)) deltaFi = AR_THRESHOLD;
  if((deltaFi<0) && (deltaFi<-AR_THRESHOLD)) deltaFi = -AR_THRESHOLD;
    
  float x = (float)deltaFi * arretier_AngleToVelKoef;
  float mismatch = x - dusAmplitude;
  float upr = pid_nextCode(mismatch);
  gk_PidRegulatorSpin(upr);  // управление движением
    
    
  // Проверить находится ли ТАНГАЖНЫЙ контур в Арретире
  startAngle = g_tangageSysAngle360;
  stopAngle = tangageArretieredRequiredAngleU32;
  deltaFi = angle_getMinimalAngleRange(startAngle, stopAngle, TANGAGE_USYSANGLE_TO_CODE(360));
  tangageArretieredNow = (ABS(deltaFi) <= COURSE_ARRETIERED_DELTA_FI); // флаг нахождения в арретире ТАНГАЖНОГО контура
}

float dbg_dissOufVel;
// Арретир2 - БЕЗ ДУС
void _gk_nextAR_withoutDUS()                                                               // ***********
{  
  uint32_t startAngle = g_sysAngle360;
  uint32_t stopAngle = arretierRequiredAngleU32;
  int32_t deltaFi = angle_getMinimalAngleRange(startAngle, stopAngle, USYSANGLE_TO_CODE(360));
  courseArretieredNow = (ABS(deltaFi) <= COURSE_ARRETIERED_DELTA_FI); // флаг нахождения в арретире КУРСОВОГО контура
  
  if((deltaFi>=0) && (deltaFi>AR_THRESHOLD)) deltaFi = AR_THRESHOLD;
  if((deltaFi<0) && (deltaFi<-AR_THRESHOLD)) deltaFi = -AR_THRESHOLD;
  
  float x = (float)deltaFi * arretier_AngleToVelKoef;
  
  //рассчет угловой скорости - простой дифференциал
  
  int16_t velocity = (int16_t)(g_dbg_velocityFromAngleF*32);
      
  /*
  // дифф. Дениса
  float velocity = diff_takeDifferential(&arDifferential, USYSANGLE_TO_FLOAT(startAngle));
  dbg_dissOufVel = velocity;
  velocity = velocity*32;
  */
  
  float mismatch = x - velocity;
  float upr = pid_nextCode(mismatch);
  gk_PidRegulatorSpin(upr);  // управление движением
    
  // Проверить находится ли ТАНГАЖНЫЙ контур в Арретире
  startAngle = g_tangageSysAngle360;
  stopAngle = tangageArretieredRequiredAngleU32;
  deltaFi = angle_getMinimalAngleRange(startAngle, stopAngle, TANGAGE_USYSANGLE_TO_CODE(360));
  tangageArretieredNow = (ABS(deltaFi) <= COURSE_ARRETIERED_DELTA_FI); // флаг нахождения в арретире ТАНГАЖНОГО контура
}

float lastUprWithoutDUS;
// Арретир без ДУС, с дифференцированием и фильтрацией
void _gk_nextAR_withDiffAndFilter()                                             // ***********
{  
  uint32_t startAngle = g_sysAngle360;
  uint32_t stopAngle = arretierRequiredAngleU32;
  int32_t deltaFi = angle_getMinimalAngleRange(startAngle, stopAngle, USYSANGLE_TO_CODE(360));
  courseArretieredNow = (ABS(deltaFi) <= COURSE_ARRETIERED_DELTA_FI); // флаг нахождения в арретире КУРСОВОГО контура
  
  float mismatch = deltaFi;    
  /*
  // ПИД регулятор -> Дифференц. с фильтром
  float upr = pid_nextCode(mismatch);
  lastUprWithoutDUS = diff_takeDifferential(&arDifferential, upr);
  */
  /*
  // Дифференц. с фильтром -> ПИД регулятор
  float diff = diff_takeDifferential(&arDifferential, mismatch);
  lastUprWithoutDUS = pid_nextCode(diff);
  */

  //ПИД регулятор
  lastUprWithoutDUS = pid_nextCode(mismatch);

  
  
  /*
  // Простой дифф.
  static float prevValue = 0;
  static float currentValue = 0;
  prevValue = currentValue;
  currentValue = mismatch;
  float diff = (currentValue - prevValue)/0.0005;
  lastUprWithoutDUS = pid_nextCode(diff);
  */
  
  gk_PidRegulatorSpin(lastUprWithoutDUS);  // управление движением
  
  // Проверить находится ли ТАНГАЖНЫЙ контур в Арретире
  startAngle = g_tangageSysAngle360;
  stopAngle = tangageArretieredRequiredAngleU32;
  deltaFi = angle_getMinimalAngleRange(startAngle, stopAngle, TANGAGE_USYSANGLE_TO_CODE(360));
  tangageArretieredNow = (ABS(deltaFi) <= COURSE_ARRETIERED_DELTA_FI); // флаг нахождения в арретире ТАНГАЖНОГО контура
}


// ВУС
void _gk_nextVUS()
{
  _gk_spinVus(constantSpeedCode);
  courseArretieredNow = 0;      // Курсовой контур не находится в арретире
}

// Калибровка скорости
void _gk_nextVelocityCalibration()
{
  static uint32_t startCalibrationTime; // время начала калибровки
  static uint32_t startTurnTime;        // время начала круга
  static uint8_t numOfCircle = 0;
  static uint32_t startAngle;
    
  switch(dusSpeedCalibrationStage)
  {
    case 0:     // Начало калибровки
      {
        gk_setConstVelocityCode(CONVERT_VELOCITY_TO_CODE32(GK_VEL_CALIBRATION_SPEED));
        startCalibrationTime = system_time;
        numOfCircle = 1;
        dusSpeedCalibrationStage = 1;
      }
      break;
            
    case 1:     // первые несколько секунд вращения, что бы набрать скорость
      {
        if(elapsed(&startCalibrationTime, 3000))
        {
          dusSpeedCalibrationStage = 2;
        }
        else
        {
           _gk_spinVus(constantSpeedCode);
        }
      }
      break;
      
    case 2:     // засечка угла, начало нового круга
      {
        if(numOfCircle == 1) // первый круг
        {
          startCalibrationTime = system_time;
          startAngle = g_sysAngle360;
        } 
        
        startTurnTime =  system_time;
        dusSpeedCalibrationStage = 3;
        //canTerminal_printf("Start turn #%d time=%d Ang=%f", numOfCircle, system_time, USYSANGLE_TO_FLOAT(g_sysAngle360));
        canMonitor_printf("Turn #%d started", numOfCircle);
      }
      break;
         
    case 3:     // первые несколько секунд КАЛИБРОВКИ, что бы шар ушел с начального угла
      {
        if((system_time - startTurnTime) >= 2000)
        {
          dusSpeedCalibrationStage = 4;          
        }
        _gk_spinVus(constantSpeedCode);
      }
      break;
            
    case 4:     // ожидание завершения оборота
      {
        uint32_t delta = (startAngle>g_sysAngle360) ? (startAngle-g_sysAngle360) : (g_sysAngle360-startAngle);
        if(delta <= USYSANGLE_TO_CODE(0.3))
        {
          //canTerminal_printf("End turn %d Ang=%f Delta A=%f. Time=%d", numOfCircle, USYSANGLE_TO_FLOAT(g_sysAngle360), USYSANGLE_TO_FLOAT(delta),  (system_time - startTurnTime));
          canMonitor_printf("Turn #%d completed. Time=%dms", numOfCircle, (system_time - startTurnTime));
          
          if(numOfCircle == GK_CALIB_NUM_OF_CIRCLES)
          {
            dusSpeedCalibrationStage = 5;
          }
          else 
          {
            dusSpeedCalibrationStage = 2;       // ожидание круга         
            numOfCircle++;
          }
        }
        _gk_spinVus(constantSpeedCode);
      }
      break;
      
    case 5:     // завершение калибровки, подсчет всего
      {
         extern double dusCalibrationKoef;
         uint32_t endCalibrationTime = system_time;
         uint32_t elapsedTime = endCalibrationTime - startCalibrationTime;
       
         float t1_ms = GK_TOTAL_CALIB_TIME*1000;
         float t2_ms = elapsedTime;
         float k = (t1_ms/t2_ms);
         newDusCalibrationKoef = dusCalibrationKoef*k;
         canMonitor_printf("Calibration done. Elapsed time - %dms. K=%f\n", elapsedTime, k);
         
         dusSpeedCalibrationStage = 6;
         gk_setConstVelocityCode(CONVERT_VELOCITY_TO_CODE32(0));
         
         // Сохранить новый коэффициент
         dusCalibrationKoef = newDusCalibrationKoef;
         extern void saveUserDataInFlash();
         saveUserDataInFlash();
         
         endCalibrationTime+=0;
         t1_ms+=0;
         t2_ms+=0;
      }
      break;
            
    case 6:     // удерживать нулевую скорость после калибровки
      {
        _gk_spinVus(dusSpeedCalibrationStage);
      }
      break;
  }
}

// Движение туда-сюда ВУС
void _gk_nextTudaSudaVUS()
{
  static uint32_t startTime = 0;
  if(elapsed(&startTime, modeTudaSudaVUSPeriod))
  {
    modeTudaSudaVUSVelocityCode = -modeTudaSudaVUSVelocityCode;
  }
  _gk_spinVus(modeTudaSudaVUSVelocityCode);      
}

// Движение туда-сюда АР
void _gk_nextTudaSudaAR()
{
  static uint32_t startTime = 0;
  static uint8_t firstAngle=0; 
  if(elapsed(&startTime, modeTudaSudaARPeriod))
  {
    arretierRequiredAngleU32 = firstAngle ? modeTudaSudaARAngle1 : modeTudaSudaARAngle2;
    firstAngle = !firstAngle;
  }
  //_gk_nextAR();
  _gk_nextAR_withoutDUS();
  //_gk_nextAR_withDiffAndFilter();
}


// ВУО
void _gk_nextVUO()
{
  static uint32_t arTime;       // время вхождение в арретир
  static uint8_t arretered;     // вошел в арретир

#define ARRETIRED_TIME  100
  
  switch(vuoStage)
  {
    case VUO_INIT:
      {
        arretered = 0;
        vuoStage = VUO_AR;
      }
      break;
      
    case VUO_AR:        // Арретирование
      {
        _gk_nextAR();
        
        if(courseArretieredNow)
        {
          if(!arretered)
          {
            arretered = 1;
            arTime = system_time;
          }
          if(elapsed(&arTime, ARRETIRED_TIME))
          {
            vuoStage = VUO_VUS;
          }
        }
        else arretered = 0;
      }
      break;
              
    case VUO_VUS:       // ВУС
      {
        _gk_nextVUS();
      }
      break;
  }
}



// Самокнотроль!
// В ходе самоконтроля:
// - ГС встает в начальную позицию (транспортное положение)
// - Каждый контур делает полный оборот с заданной скоростью, рассчитывается средняя скорость по углу и контролируется, что она отличается от заданной не больше, чем на Х
// - ГК должен встать в арретир на 4 угла с заданной точностью после чего встать в юстировочное положение
void _gk_nextFuncTest()
{
  static uint32_t funcTestStartTime;
  static uint32_t startCourseAngle;
  static uint32_t startTangageAngle;
  static uint32_t startTime;

  static FuncTestArrieterMode_t arretierMode;
  
  // Выполнение оборота
  static uint8_t vusTestStage = 0;              // стадия выполнения оборота
  static uint8_t courseTurnCompleted = 0;       // завершен оборот курсового контура
  static uint8_t tangageTurnCompleted = 0;      // завершен оборот тангажного контура
  static uint8_t courseTurnTestFailured = 0;    // скорость курсового контура не соответствует заданной 
  static uint8_t tangageTurnTestFailured = 0;   // скорость тангажного контура не соотвествует заданной
  static uint32_t endOfTurnTime = 0;            // время завершения оборота
  // Позиционирование в нужных углах
  static uint8_t arretieredNow;         // флаг для арретирования в течении какого-то времени
  static uint32_t arretieredTime;       // время в арретировании (для проверки, что находимся в арретире Х мс)
  static uint32_t maxTransferTime;      // максимальное время переброски в заданную точку
  
  uint32_t currentCourseAngle = g_sysAngle360;
  uint32_t currentTagageAngle = g_tangageSysAngle360;
     
  switch(functionalTestStage)
  {
    case FUNCTEST_INIT:      // инициализация самоконтроля
      {
        functionalTestStage = FUNCTEST_AR_TEST;
        functionalTestDone = 0;
        vusTestStage = 0;
        funcTest_arNumOfTestAngles = 0;
        funcTestErrorCode = FuncTestErrorCode_None;
        arretierMode = FuncTestArMode_MoveToStart;
        
        funcTestStartTime = system_time;
        startTime = system_time;
        
        _funcTest_setNextArretiredAngle(arretierMode, &funcTest_arNumOfTestAngles, &maxTransferTime);
      }
      break;
           
    case FUNCTEST_VUS_TEST:  // тест ВУС
      {
        _gk_nextVUS();
        
        switch(vusTestStage)
        {
          case 0: // инициализация
            {
              vusTestStage = 1;
              courseTurnCompleted = 0;
              tangageTurnCompleted = 0;
              courseTurnTestFailured = 0;
              tangageTurnTestFailured = 0;
             
              startCourseAngle = currentCourseAngle;
              startTangageAngle = currentTagageAngle;
              startTime = system_time;
            }
            break;
          case 1: // ожидание завершения оборота
            {
              if(system_time - startTime < 3000) return;
              uint32_t elapsedTurnTime; // время за которое был сделан оборот
              float averageVelocity;    // средняя скорость движения
                
              // проверяем сделал ли оборот курсовой контур
              if(!courseTurnCompleted)
              {                
                int32_t courseDeltaFi = angle_getMinimalAngleRange(startCourseAngle, currentCourseAngle, USYSANGLE_TO_CODE(360));
                if(ABS(courseDeltaFi) <= USYSANGLE_TO_CODE(0.5))
                {
                    courseTurnCompleted = 1;
                    if(tangageTurnCompleted) endOfTurnTime = system_time;
                    gk_setConstVelocityCode(0);
                    
                    elapsedTurnTime = system_time - startTime;
                    averageVelocity = ((360.0/elapsedTurnTime)*1000);
                    courseTurnTestFailured = (averageVelocity < (SELFCONTROL_COURSE_CIRCLE_VELOCITY - SELFCONTROL_MAX_VELOCITY_ERROR)) ||
                                             (averageVelocity > (SELFCONTROL_COURSE_CIRCLE_VELOCITY + SELFCONTROL_MAX_VELOCITY_ERROR));
                }
              }

              // проверяем сделал ли оборот тангажный контур              
              if(!tangageTurnCompleted)
              {                
                int32_t tangageDeltaFi = angle_getMinimalAngleRange(startTangageAngle, currentTagageAngle, TANGAGE_USYSANGLE_TO_CODE(360));
                if(ABS(tangageDeltaFi) <= USYSANGLE_TO_CODE(0.5))
                {
                   tangageTurnCompleted = 1;
                   if(courseTurnCompleted) endOfTurnTime = system_time;
                   tangageCtrl_setSelfControlSubmodeVUS(0);
                   
                   elapsedTurnTime = system_time - startTime;
                   averageVelocity = ((360.0/elapsedTurnTime)*1000);
                   tangageTurnTestFailured = (averageVelocity < (SELFCONTROL_TANGAGE_CIRCLE_VELOCITY - SELFCONTROL_MAX_VELOCITY_ERROR)) ||
                                             (averageVelocity > (SELFCONTROL_TANGAGE_CIRCLE_VELOCITY + SELFCONTROL_MAX_VELOCITY_ERROR));
                }
              }
              
              // Оба контура завершили оборот
              if(courseTurnCompleted && tangageTurnCompleted)
              {
                 if(courseTurnTestFailured || tangageTurnTestFailured) // тест завершен с ошибкой (скорости контуров вышли за допустимые пределы)
                 {
                    FuncTestErrorCode_t errCode = FuncTestErrorCode_None;
                    if(courseTurnTestFailured)  errCode |= FuncTestErrorCode_CourseVusVelocityIncorrect;
                    if(tangageTurnTestFailured) errCode |= FuncTestErrorCode_TangageVusVelocityIncorrect;
                    _funcTest_errorOccured(errCode);
                 }              
                 else // тест завершен успешно - встать в арретир
                 {
                    if(system_time - endOfTurnTime > 1500)
                    {                       
                      functionalTestStage = FUNCTEST_AR_TEST;
                      startTime = system_time;
                      _funcTest_setNextArretiredAngle(FuncTestArMode_GetNextTestAngle, &funcTest_arNumOfTestAngles, &maxTransferTime);
                    }
                 }
              }
              else if(system_time - startTime > SELFCONTROL_MAX_CIRCLE_TIME) // Если оба контура не выполнили оборот за допустимое время - завершить тест с ошибкой
              {
                 FuncTestErrorCode_t errCode = FuncTestErrorCode_None;
                 if(!courseTurnCompleted)  errCode |= FuncTestErrorCode_CourseRotationTimeout;
                 if(!tangageTurnCompleted) errCode |= FuncTestErrorCode_TangageRotationTimeout;
                 _funcTest_errorOccured(errCode);
              }
            }
            break;
        }
      }
      break;
           
    case FUNCTEST_AR_TEST: // ожидание выхода в заданную точку
      {
        _gk_nextAR();
        
        if(courseArretieredNow && tangageArretieredNow) // вышли в арретир
        {
            if(!arretieredNow)
            {
                arretieredTime = system_time;
                arretieredNow = 1; 
            }
            else 
            {
               if(system_time - arretieredTime > 500) // если продолжительно находимся в арретире - встать на другой угол
               {
                  // Проверяем в какой стадии самоконтроля мы встали в арретир
                  if(arretierMode == FuncTestArMode_MoveToStart) // встали в арретир в самом начале ф. теста - сделать оборот
                  {
                      functionalTestStage = FUNCTEST_VUS_TEST;
                      arretierMode = FuncTestArMode_GetNextTestAngle;
   
                      int16_t courseVelocity32 = CONVERT_VELOCITY_TO_CODE32(SELFCONTROL_COURSE_CIRCLE_VELOCITY);
                      int16_t tangVelocity32 = CONVERT_VELOCITY_TO_CODE32(SELFCONTROL_TANGAGE_CIRCLE_VELOCITY);
                      gk_setConstVelocityCode(courseVelocity32);
                      tangageCtrl_setSelfControlSubmodeVUS(tangVelocity32);
                      return;
                  }
                  
                  startTime = system_time;
                  uint8_t done = _funcTest_setNextArretiredAngle(arretierMode, &funcTest_arNumOfTestAngles, &maxTransferTime);
                  if(done)
                  {
                     _funcTest_copmleteSuccess();
                  }
               }
            }
        }
        else // находимся не в арретире
        { 
            arretieredNow = 0;
            if(system_time - startTime > maxTransferTime) // не вышли в заданную точку за отведенное время
            {
              FuncTestErrorCode_t errCode = FuncTestErrorCode_None;
              if(arretierMode == FuncTestArMode_MoveToStart) // выходим на стартовую позицию
              {
                if(!courseArretieredNow) errCode |= FuncTestErrorCode_CourseMoveToStartTimeout;
                if(!tangageArretieredNow) errCode |= FuncTestErrorCode_TangageMoveToStartTimeout;
              }
              else // тест АР
              {
                if(!courseArretieredNow) errCode |= FuncTestErrorCode_CourseArrieterTimeout;
                if(!tangageArretieredNow) errCode |= FuncTestErrorCode_TangageArrieterTimeout;
              }
              _funcTest_errorOccured(errCode);
              return;
            }
        }
      }
      break;
            
    case FUNCTEST_COMPLETE_SUCCESS:    // самоконтроль завершен успешно
      {
        _gk_nextAR();
        if(dbg_infiniteFuncTest && (system_time - dbg_endOfFuncTestTime > 5000)) // отладка - гонять функ. тест постоянно
        {
          functionalTestStage = FUNCTEST_INIT;
          tangageCtrl_setModeSelfControl();
        }
      }
      break;
      
    case FUNCTEST_COMPLETE_WITH_ERROR: // самоконтроль завершен с ошибкой
      {
        _gk_nextVUS();
      }
      break;
  }
  
  
  uint8_t checkTestTimeout = (functionalTestStage != FUNCTEST_COMPLETE_SUCCESS) &&      // если тест не закончен и выполняется дольше минуты - остановить его
                             (functionalTestStage != FUNCTEST_COMPLETE_WITH_ERROR) &&
                             ((system_time - funcTestStartTime) > 60000);
  if(checkTestTimeout)
  {
     _funcTest_errorOccured(FuncTestErrorCode_TestTimeout);
  }
}

// Функциональный тест завершен успешно
void _funcTest_copmleteSuccess()
{
  dbg_endOfFuncTestTime = system_time;
  
  functionalTestDone = 1;
  functionalTestStage = FUNCTEST_COMPLETE_SUCCESS;
  tangageCtrl_setSelfControlSubmodeEndOfTest(FUNCTEST_COMPLETE_SUCCESS, FuncTestErrorCode_None);
}

// Самоконтроль завершился с ошибкой
void _funcTest_errorOccured(FuncTestErrorCode_t errorCode)
{
  functionalTestDone = 1;
  functionalTestStage = FUNCTEST_COMPLETE_WITH_ERROR;
  funcTestErrorCode = errorCode;
  gk_setFailure(GgFailure_FuncTestError);
  gk_setConstVelocityCode(0);
  tangageCtrl_setSelfControlSubmodeEndOfTest(FUNCTEST_COMPLETE_WITH_ERROR, errorCode);
}


// Задать следующий угол арретирования для самоконтроля
// Возвращает 1, если все углы проработаны
// numOfAngle - номер угла 
// maxTravelTime - максимальное время передвижения в заданные углы (выбирается тот контур, которому дальше ехать)
uint8_t _funcTest_setNextArretiredAngle(FuncTestArrieterMode_t arMode, uint8_t *numOfAngle, uint32_t *maxTransferTime)
{
  int16_t courseArAngle128 = 0;
  int16_t tangageArAngle128 = 0;  
  
  if(arMode == FuncTestArMode_MoveToStart) // перейти в начальную позицию
  {
    courseArAngle128 = SELFCONTROL_COURSE_START_POSITION;
    tangageArAngle128 = SELFCONTROL_TANGAGE_START_POSITION;
  }
  else if(arMode == FuncTestArMode_GetNextTestAngle) // выставить следующий угол
  {  
    switch(*numOfAngle)
    {
      case 0:
          courseArAngle128  = SELFCONTROL_COURSE_ANGLE_1;
          tangageArAngle128 = SELFCONTROL_TANGAGE_ANGLE_1;
          *numOfAngle = (*numOfAngle)+1;
        break;
        
      case 1:
          courseArAngle128  = SELFCONTROL_COURSE_ANGLE_2;
          tangageArAngle128 = SELFCONTROL_TANGAGE_ANGLE_2;
          *numOfAngle = (*numOfAngle)+1;
        break;
        
      case 2:
          courseArAngle128  = SELFCONTROL_COURSE_ANGLE_3;
          tangageArAngle128 = SELFCONTROL_TANGAGE_ANGLE_3;
          *numOfAngle = (*numOfAngle)+1;
        break;
        
      case 3:
          courseArAngle128  = SELFCONTROL_COURSE_ANGLE_4;
          tangageArAngle128 = SELFCONTROL_TANGAGE_ANGLE_4;
          *numOfAngle = (*numOfAngle)+1;
        break;
        
      case 4: return 1; // ПЕРЕБРАНЫ ВСЕ УГЛЫ
    }
  }
  funcTestArAngle128 = courseArAngle128;
  
  // установить настройки для арретирования
  uint32_t courseArAngleU = angle_convertCmdS2SysU(courseArAngle128, g_logicalZeroAngleCorrection, MAX_SYSANGLE_CODE);
  uint32_t tangageArAngleU = angle_convertCmdS2SysU(tangageArAngle128, g_tangageLogicalZeroAngleCorrection, MAX_TANGAGE_SYSANGLE_CODE); 
  arretierRequiredAngleU32 = courseArAngleU;
  tangageCtrl_setSelfControlSubmodeAR(tangageArAngleU, tangageArAngle128);
    
  // Рассчет углов арретирования для позиционирования в логической системе координат. 
  // courseArAngleU, tangageArAngleU - углы в СИСТЕМНОЙ системе координат для арретирования
  // Рассчет времени выхода в заданный угол в СИСТЕМНОЙ ШКАЛЕ (заданный угол в команде АР приводится к системной шкале)
  uint32_t currentCourseAngle = g_sysAngle360;
  uint32_t currentTagageAngle = g_tangageSysAngle360;
  uint32_t courseTravelTime_ms, tangageTravelTime_ms; // сколько времени понадобиться, что бы приехать в заданный угол каждому контуру
  int32_t courseDeltaFi, tangageDeltaFi;
  float courseAnglePath, tangageAnglePath;
  
  // курс
  courseDeltaFi = angle_getMinimalAngleRange(currentCourseAngle, courseArAngleU, USYSANGLE_TO_CODE(360));
  courseAnglePath = USYSANGLE_TO_FLOAT(ABS(courseDeltaFi));
  courseTravelTime_ms = (uint32_t)((courseAnglePath/(ARRETIR_VELOCITY - SELFCONTROL_MAX_VELOCITY_ERROR))*1000);  
  // тангаж  
  tangageDeltaFi = angle_getMinimalAngleRange(currentTagageAngle, tangageArAngleU, TANGAGE_USYSANGLE_TO_CODE(360));
  tangageAnglePath = TANGAGE_SYSANGLE_TO_FLOAT(ABS(tangageDeltaFi));
  tangageTravelTime_ms = (uint32_t)((tangageAnglePath/(ARRETIR_VELOCITY - SELFCONTROL_MAX_VELOCITY_ERROR))*1000);
  
  *maxTransferTime = MAX(courseTravelTime_ms, tangageTravelTime_ms);  
  *maxTransferTime = (*maxTransferTime) + 3000;   // защита от перемещений на короткие расстояния. Без нее были проблемы
  
  ////canTerminal_printf("C: cur=%.4f  cmd=%.4f, path=%.4f, time=%d\n"
  ////                   "T: cur=%.4f  cmd=%.4f, path=%.4f, time=%d",
  ////                   USYSANGLE_TO_FLOAT(currentCourseAngle), USYSANGLE_TO_FLOAT(courseArAngleU), courseAnglePath, courseTravelTime_ms,
  ////                   TANGAGE_SYSANGLE_TO_FLOAT(currentTagageAngle), TANGAGE_SYSANGLE_TO_FLOAT(tangageArAngleU), tangageAnglePath, tangageTravelTime_ms);
  return 0;
}




// =============================================================================
// =============================================================================
// =============================== PID-регулятор ===============================
// =============================================================================
// =============================================================================

// Вращение в заданной постоянной скоростью скоростью
void _gk_spinVus(int16_t desiredDusCode)
{
  int16_t mismatch = desiredDusCode - dusAmplitude;
  float upr = pid_nextCode(mismatch);
  gk_PidRegulatorSpin(upr);
}

// Задать постоянную скорость вращения ГК
void gk_setConstVelocityCode(int16_t velocityCode32)
{ 
  constantSpeedCode = velocityCode32;
}

// Управление движением
void gk_PidRegulatorSpin(float upr)
{
  uint8_t clockwise = (upr>0) ? 1 : 0; // 1-вращение по часовой стрелке, 0-вращение против часовой стрелке 
  setPwmFromUprCode(upr);
  
  if(clockwise) engine_spinCW();
  else engine_spinCCW();
}

// Установить ШИМ
void setPwmFromUprCode(float uprCode)
{
  float absUpr = uprCode<0 ? -uprCode : uprCode;
  
  static const uint32_t threshold60percentage = 2797;
  if(absUpr > threshold60percentage) absUpr = threshold60percentage;
  float pwm = (absUpr*60)/threshold60percentage;
  setPwmFillingF(pwm);
  
  //setPwmFillingF_2((uint16_t)absUpr);
}
























// Режим работы ГК
void gkModeControl()
{
  switch(gk_controlMode)
  {
    case GkMode_Initialize: // Инициализация
      {
         static uint8_t initDone = 0;
         if(initDone) return;
        
         if(system_time < 1000) return;
         arretierRequiredAngleU32 = g_sysAngle360; //angle_convertCmdS2SysU(0*128, g_hardwareZeroAngleCorrection, MAX_SYSANGLE_CODE);
         gk_controlMode = GkMode_AR;
         initDone = 1;
      }
      break;
    default : return;    
  }
      
}




