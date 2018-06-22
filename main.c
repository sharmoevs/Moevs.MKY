#include "stdlib.h"
#include "stdio.h"
#include "MDR1986VE1T.h"
#include "loaders.h"
#include "arincProtocol.h"
#include "canterminal.h"
#include "canmonitor.h"
#include "timers.h"
#include "adc.h"
#include "angelSensor.h"
#include "global.h"
#include "flashuserdata.h"
#include "engineControl.h"
#include "dac.h"
#include "pidRegulator.h"
#include "dus.h"
#include "dusCalibration.h"
#include "angleFilter.h"
#include "debugingInfo.h"
#include "gkControlMode.h"
#include "tangageControl.h"
#include "differentiator.h"
#include "dataQueue.h"
#include "canMonitorText.h"
#include "canFrameQueue.h"
#include "coreControl.h"



void stopwatchTestFunc(int us);
void testfunc();
extern CanSoftwareBuffer_t *canmonitorSoftBuffer;
extern CanSoftwareBuffer_t *tangageControlSoftBuffer;




int main()
{
  timer_setEnable(MDR_TIMER4, 1);  // системный таймер и таймер ДУС
  loaders_init();               // инициализация загрузчиков и включение CAN-прерываний
  readUserDataFromFlash();      // чтение настроек из флешь-памяти
  angelSensor_init();           // инициализация датчика угла
  pid_init();                   // инициализация ПИД-регулатора
  dusFilter_Init();             // инициализация фильтра ДУС
  pidFilter_Init();             // инициализация фильтра PID-регулятора
#ifdef USE_DYNAMIC_CAN_QUEUE
  canText_init(MDR_CAN1);       // инициализация отладки
  canMonitor_init();            // инициализация
  tangageCtrl_init();
#endif  
  gk_init();                    // инициализация режима ГК
  coreControlInit();
  
  //timer_setEnable(MDR_TIMER2, 1); // управление перенесено в таймер 4
  NVIC_EnableIRQ(CAN1_IRQn);    // разрешение прерываний can. Чтобы после перепрограммирования загрузчики были инициализированы

  
  ////while(1) stopwatchTestFunc(100);
  
    
  uint32_t debugTimer = 0;
  while(1)
  {
    arincService();                       // поддержка протокола
    debugingInfoService();                // вывод отладочной инфы
       
#ifdef USE_DYNAMIC_CAN_QUEUE
    canMonitorText_server();              // сервер тектовых сообщений
    canSwBuffer_service(canmonitorSoftBuffer);
    canSwBuffer_service(tangageControlSoftBuffer);
#endif    
    
    if(elapsed(&debugTimer, 10000))
    {
      
    }
  }
}

// Обработчик прерывания Timer4 - Фильтр ДУС
void TIMER4_IRQHandler()
{
  MDR_TIMER4->STATUS &= ~TIMER_STATUS_CNT_ARR_EVENT;                            // сбросить флаг
  static uint8_t numOfIrq = 0;
  if(++numOfIrq == DUS_TIMER_COUNT_PER_ms)
  {
    numOfIrq = 0;
    system_time++;
  }
   
  
  dus_getNextSample();          // отсчет с ДУС
  angle_getNextSampleX32();     // отсчет с датчика угла 
  gk_checkSpeedProtection();    // средняя скорость 
  //dus_calibrate();              // калибровка ДУС    
  coreMove();
  
  
  
  // Отправка угла и проч
  static uint32_t timer = 0;
  if(elapsed(&timer, 1))
  {
    canMonitor_sendAngle();             // текущий угол
    canMonitor_sendCourseVelocity();    // угловая скорость
  }  
}

// Обработчик прерывания Timer2 - Управление ГК
void TIMER2_IRQHandler()
{
  MDR_TIMER2->STATUS &= ~TIMER_STATUS_CNT_ARR_EVENT; // сбросить флаг

  //gk_moveNext();
  //coreMove();
}






// Тест - измерить время выполнения функции
void stopwatchTestFunc(int delayMs)
{  
   timer_setEnable(MDR_TIMER4, 0); // отключаем прерывания упрвления ГС
  
    __START_STOPWATCH
      TIMER4_IRQHandler();
      //uint16_t code = adc_getVelocityCode();
      //dus_filter(0);  
    __STOP_STOPWATCH      
   canMonitor_printf("elapsed = %d",  __STOPWATCH_ELAPSED);
   
   delay_us(delayMs*1000);
}

void testfunc()
{  
  /*
  extern float _pid_regulatorPID(float mismatch);
  Differential_t   differential;  
  diff_init(&differential, 1, 2, 0.0005, 0);
     
  pid_setNewKoef(1, 1, 0, 0, 0.0005);
  
  for(int i=0; i<1000; i++)
  {
    float upr = _pid_regulatorPID(1.0);
    float value = diff_takeDifferential(&differential, upr);
    canMonitor_sendDbgSavedValue(value);
    delay_us(500);    
    
    value++;
    upr++;
  } 
  */
}

                  
                  

  /* // проверка частоты работы таймера.
  static int i=0; i++;
  if(i%2) AH_HI();
  else AH_LO();
  return;

  */
  
