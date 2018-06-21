#include "debugingInfo.h"
#include "timers.h"
#include "canmonitor.h"
#include "canMonitorText.h"
#include "dataQueue.h"
#include "canFrameQueue.h"

#define DEBUGINFO_SEND_PERIOD                         3
#define DEBUGINFO_SEND_CURRENT_STATE_PERIOD           500


uint8_t debugInfo_globalEnable = 0;     // общее разрешение отправки отладочной информации
uint8_t debugInfo_velocityEnable = 0;   // передача скорости
uint8_t debugInfo_uprEnable = 0;        // передача управляющего сигнала
uint8_t debugInfo_angleEnable = 0;      // передача угла
uint8_t debugInfo_testValue1Enable = 0; // передача тестового значения 1




void dbg_currentParametersService();
void dbg_sendCurrentGsState();



// Отправка состояния ГС и отладочной инфы
void debugingInfoService()
{
  dbg_currentParametersService();       // текущий угол, скорость, упр. сигнал и т.д.
  dbg_sendCurrentGsState();             // текущее состояния ГС
  //canMonitorText_server();              // сервер тектовых сообщений
}

// Отправить текущие параметры скорости, угла, управляющего сигнала
void dbg_currentParametersService()
{  
  static uint32_t timer = 0;
  
  if(!debugInfo_globalEnable) return;
  if(!elapsed(&timer, DEBUGINFO_SEND_PERIOD)) return;
  
  /*
  if(debugInfo_velocityEnable)
  {
    canMonitor_sendCourseVelocity();    // угловая скорость    
  }
  */
  if(debugInfo_uprEnable)
  {
    canMonitor_sendCourseUpr();         // рассогласование    
  }
  /*
  if(debugInfo_angleEnable)
  {
    canMonitor_sendAngle();             // текущий угол    
  }
  */
  if(debugInfo_testValue1Enable)        // тестовое значение 1
  {
    extern float lastDiffOut;
    extern int16_t dusAmplitude;    
    extern int32_t g_dbg_distanceTraveledCode;
    extern float g_dbg_velocityFromAngleF;
    extern float lastUprWithoutDUS;
    extern float dbg_dissOufVel;
    extern DataQueue_t canTextQueue;
    extern CanSoftwareBuffer_t *canmonitorSoftBuffer;
    extern CanSoftwareBuffer_t *tangageControlSoftBuffer;
    extern float dbgDusIntegral;
    
    canMonitor_sendTestValue1(dbgDusIntegral);
  }
}

// Отправить текущее состояние ГС (тангажного и курсового контура)
void dbg_sendCurrentGsState()
{
  static uint32_t timer = 0;
  if(!elapsed(&timer, DEBUGINFO_SEND_CURRENT_STATE_PERIOD)) return;
 
  canMonitor_GsSendState();
}