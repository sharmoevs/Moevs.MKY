#include "momentInertiaTest.h"
#include "timers.h"
#include "settings.h"
#include "angelSensor.h"
#include "dus.h"
#include "engineControl.h"
#include "global.h"
#include <stdlib.h>
#include "canMonitor.h"



#define MAX_SPEED       250
#define OFFSET_ANGLE    60
//#define PWM_PERSENTAGES 60

#define SPEED_CALC_PERIOD   10

extern uint32_t g_sysAngle360;
extern void _setPwm(float pid);
float _calculateOffsetAngle(float startAngle, float offset);
void setEngineEnable(uint8_t enable, uint8_t protectionWorked);
uint8_t _calculateAverageSpeed(float *averSpeed, uint32_t interval);
void _setTimerPwm(float percentages);

float mitest_currentAngle;
float mitest_averageSpeed;

float mitest_numOfRevolutions = 0;      // количество оборотов

uint8_t mitest_testRunning = 0;
uint8_t mitest_engineEnable = 0;
uint32_t mitest_startTime;
uint32_t mitest_endTime;

int mitest_pwm;

float mitest_startAngle;        // начальный угол
float mitest_calculatedEndAngle;// расчитанный конечный угол
float mitest_realEndAngle;      // на самом деле какой угол был выбран


float mitest_acceleration = 0;      // ускорение

uint8_t mitest_sendDebugValueEnable = 1;


// Рассчитать конечный угол
float _calculateOffsetAngle(float startAngle, float offset)
{
  float tmp = startAngle + offset;
  if(tmp > 360) return tmp - 360;
  return tmp;
}

// Начало теста
void momentInertiaTest_start(int pwm)
{
  if(pwm == 0) 
  {
    momentInertiaTest_stop();
    return;
  }
  
  mitest_pwm = pwm;
  mitest_startTime = system_time;
  mitest_startAngle = USYSANGLE_TO_FLOAT(g_sysAngle360);
  mitest_calculatedEndAngle = _calculateOffsetAngle(mitest_startAngle, OFFSET_ANGLE);
  mitest_numOfRevolutions = 0;
  
  
  canMonitor_printf("Начальный угол %.3f. Конечный угол %.3f (смещение %d град)",
                    mitest_startAngle, mitest_calculatedEndAngle, OFFSET_ANGLE);
  
  //mitest_sendDebugValueEnable = 1; // отладка - отправить значения
  mitest_testRunning = 1;
  setEngineEnable(1, 0);  
}

// останов
void momentInertiaTest_stop()
{
  canMonitor_printf("Тест остановлен");
  mitest_engineEnable = 0;
  mitest_testRunning = 0;
  _setTimerPwm(0);
}
  


// Выключены двигатели
void momentInertiaTest_omEngineDisabled(uint8_t speedProtected)
{
  mitest_testRunning = 0;
  mitest_endTime = system_time;
  mitest_realEndAngle = mitest_currentAngle;
  
   
  uint32_t elapsedTime = mitest_endTime - mitest_startTime;
  canMonitor_printf("Двигатели выключены на %.3f через %dмс", mitest_realEndAngle, elapsedTime);
  if(speedProtected) canMonitor_printf("Сработала защита %dград/сек", MAX_SPEED);
}

// Включить/выключить двигатели
void setEngineEnable(uint8_t enable, uint8_t protectionWorked)
{
  mitest_engineEnable = enable;
  _setTimerPwm(enable ? mitest_pwm : 0);
  if(!enable)
  {
     engine_disableSpin();
     momentInertiaTest_omEngineDisabled(protectionWorked);
  }
}

void _setTimerPwm(float percentages)
{
#define PWM_THRESHOLD   100.0 //60.0
  static float pwmThresholdCode = TIM_PWM_GET_CCR3(PWM_THRESHOLD);
  
  if(percentages >= PWM_THRESHOLD)
  {
    MDR_TIMER1->CCR3 = (uint16_t)pwmThresholdCode;
  }
  else
  {
    MDR_TIMER1->CCR3  = TIM_PWM_GET_CCR3(percentages);
  }
}


// сам тест
void momentInertia_test()
{
  //static float prevAngle;
  //prevAngle = mitest_currentAngle;
  mitest_currentAngle = USYSANGLE_TO_FLOAT(g_sysAngle360);             // текущий угол
  _calculateAverageSpeed(&mitest_averageSpeed, SPEED_CALC_PERIOD);     // средняя скорость
 
  
  
      
  if(!mitest_engineEnable) return;
  //if(system_time - mitest_startTime < 300) return;

  
  // защита по скорости
  if(mitest_averageSpeed > MAX_SPEED)
  {
    uint8_t protectionWorked = 1;
    setEngineEnable(0, protectionWorked);
  }
  /* // отключение двигателей при повороте на заданный угол
  // Проеряем, что повернулись на заданный угол  
  float maxAngle = MAX(mitest_currentAngle, mitest_calculatedEndAngle);
  float minAngle = MIN(mitest_currentAngle, mitest_calculatedEndAngle);
        
  float r1 = maxAngle - minAngle;
  float r2 = 360 - r1;
        
  float delta = MIN(r1, r2);
  if(delta<0.01)
  {
    uint8_t protectionWorked = 0;
    setEngineEnable(0, protectionWorked);
  }
  */
  
  
  if(mitest_engineEnable)
  {
    engine_spinCW();
  }
}




// Рассчет средней скорости
// Возвращает 1, если была рассчитана средняя скорость, в противном случае 0
uint8_t _calculateAverageSpeed(float *averSpeed, uint32_t intervalMs)
{
  static uint32_t lastSampleTime = 0;     // время последнего отсчета
  static float prevAngle = 0;
  static float currAngle = 0; 
  static uint8_t  stage = 0;
    
  if(!elapsed(&lastSampleTime, intervalMs)) return 0;  
  switch(stage)
  {
    case 0:     // инициализация
      stage = 1;
      prevAngle = USYSANGLE_TO_FLOAT(g_sysAngle360);
      return 0;
      
    case 1:
      {
        currAngle = USYSANGLE_TO_FLOAT(g_sysAngle360);
                      
        float maxAngle = MAX(currAngle, prevAngle);
        float minAngle = MIN(currAngle, prevAngle);
        
        float r1 = maxAngle - minAngle;
        float r2 = 360 - r1;
        
        
        float deltaFi = MIN(r1, r2);       
        *averSpeed = deltaFi/((float)intervalMs/1000.0);
        prevAngle = currAngle;
        
        
        
        // рассчет ускорения
        static float prevSpeed = 0;
        float currestSpeed =  *averSpeed;        
        mitest_acceleration = (currestSpeed - prevSpeed)/((float)intervalMs/1000.0); 
        prevSpeed = currestSpeed;
        
        
        averSpeed++;
        deltaFi++;
        r1++;
        r2++;
        
        return 1;
      }
      
    default: return 0;
  }
}
