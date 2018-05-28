#include "angelSensor.h"
#include "mkpinout.h"
#include "global.h"
#include "timers.h"
#include "angle_reductosin.h"
#include "canterminal.h"
#include "differentiator.h"

uint16_t as_readData();
uint16_t as_readRough();
uint16_t as_readAccurate();
uint32_t _angle_shiftScaleToZero(uint32_t angle);
uint32_t angle_convertSystemToEngine(uint32_t absAngle);  // поправки к фазовому нулю двигателей


// new
int32_t  _angle_convertU2S(uint32_t angleU, uint32_t code360);           // Преобразовать беззнаковый угол 0..360 в знаковый -180..180
uint32_t _angle_convertS2U(int32_t angleS,  uint32_t code360);           // Преобразовать знаковый -180..180 в беззнаковый 0..360 


// Сырые данные
uint16_t roughData;                     // код грубого угла от 0..120
uint16_t accurateData;                  // код точного угла от 0..11.25

uint32_t g_engineZeroAngleCorrection;   // поправка к датчику угла для определения фазового нуля двигателей
uint32_t g_hardwareZeroAngleCorrection; // ноль аппаратной шкалы
uint32_t g_logicalZeroAngleCorrection;  // ноль программной шкалы

uint32_t g_sysAngle360;                 // угол с датчика угла (0..360)
uint32_t g_engineControlAngle360;       // угол от 0..360 градусов для управления двигателями (с поправкой на ноль фазового двигателя)
uint32_t g_prevSysAngle360;             // предпоследний отсчет угла

// Скорость как дифференциал угла
Differential_t   angleDifferential;

// Инициализация датчика угла
void angelSensor_init()
{
  angle_getNextSampleX32();
  diff_init(&angleDifferential, 1, 1, 0.0005, USYSANGLE_TO_FLOAT(g_sysAngle360));  
}


// Чтение данных из выбранной микросхемы
uint16_t as_readData()
{
  uint8_t dataH, dataL;
  uint16_t result;

  delay_tics(20);   // задержка после выбора микросхемы
  AS_INHIBIT_LO;  
  delay_tics(20);
  
  AS_BYTE_SELECT_LO;
  delay_tics(20);
  
  dataL = AS_DATA;
  AS_BYTE_SELECT_HI;
  delay_tics(20);
  
  dataH = AS_DATA;  
  AS_INHIBIT_HI;
  
  result = dataH<<8 | dataL;
  return result; 
}
// Грубо
uint16_t as_readRough()
{  
  AS_SELECT_ROUGH;
  return as_readData();
}
// Точно
uint16_t as_readAccurate()
{
  AS_SELECT_ACCURATE;
  return as_readData();
}


// 32х разрядный код курсового угла
uint32_t angle_getNextSampleX32()
{
  roughData = as_readRough();
  accurateData = as_readAccurate();
  g_prevSysAngle360 = g_sysAngle360;
  
  uint32_t absAngle = 0 - angle_Reductosin(roughData, accurateData);         // угол от 0 до 360
  g_sysAngle360           = absAngle>>13;
  g_engineControlAngle360 = angle_deductCorrection(g_sysAngle360, g_engineZeroAngleCorrection, MAX_SYSANGLE_CODE);  
      
  ////static uint32_t time = 0;
  ////if(elapsed(&time, 10)) canTerminal_printf("%.8x",g_sysAngle360);
  
  return g_sysAngle360;
}




// Рассчет средней скорости
// Возвращает 1, если была рассчитана средняя скорость, в противном случае 0
uint8_t angle_getAverageVelocity(float *averageVelocity)
{
  static uint32_t lastSampleTime = 0;     // время последнего отсчета
  static uint32_t angle1 = 0;
  static uint32_t angle2 = 0; 
  static uint8_t  stage = 0;
    
  if(!elapsed(&lastSampleTime, AVERAGE_VELOCITY_INTERVAL)) return 0;  
  switch(stage)
  {
    case 0:     // инициализация
      stage = 1;
      angle1 = g_sysAngle360;
      return 0;
      
    case 1:
      {
        angle2 = g_sysAngle360;
        float deltaFi = USYSANGLE_TO_FLOAT(angle2) - USYSANGLE_TO_FLOAT(angle1);      
        *averageVelocity = deltaFi/((float)AVERAGE_VELOCITY_INTERVAL/1000.0);
        angle1 = angle2;
        return 1;
      }
      
    default: return 0;
  }
}

// Рассчет скорости по углу
float angle_getVelocityFromAngle()
{
  extern float g_dbg_velocityFromAngleF;
  
  extern int32_t g_dbg_distanceTraveledCode;
  
  static uint32_t i = 0;
  static uint32_t prevAngle = 0;
  static float velocity = 0;
  
#ifndef AVERAGE
  i++;
  if(i%VEL_FROM_ANGLE_SAMPLES == 0)
  {
    i = 0;
    g_dbg_distanceTraveledCode = g_sysAngle360 - prevAngle;    
    float deltaFiF = USYSANGLE_TO_FLOAT(g_sysAngle360) - USYSANGLE_TO_FLOAT(prevAngle);    
    velocity = deltaFiF / (0.0005 * VEL_FROM_ANGLE_SAMPLES);
    
    prevAngle = g_sysAngle360;
  }
  g_dbg_velocityFromAngleF = velocity;
  return velocity;
#else 
  static int32_t deltaSum;  
  int delta = g_sysAngle360 - prevAngle;
  prevAngle = g_sysAngle360;
  if(i != VEL_FROM_ANGLE_SAMPLES)
  {
    deltaSum += delta;
    i++;
  }
  else
  {
    i = 0;
    g_dbg_distanceTraveledCode = deltaSum/VEL_FROM_ANGLE_SAMPLES;
    float deltaFiF = USYSANGLE_TO_FLOAT(g_dbg_distanceTraveledCode);    
    velocity = deltaFiF / (0.0005);
    deltaSum = 0;
  }
  g_dbg_velocityFromAngleF = velocity;
  return velocity;
#endif
  
  /*
  g_dbg_velocityFromAngleF = diff_takeDifferential(&angleDifferential, USYSANGLE_TO_FLOAT(g_sysAngle360));
  return g_dbg_velocityFromAngleF;
  */
}


// Получает минимальное расстояние между двумя углами.
// code360grad - код угла, соответствующий 360градусам. Необходим при рассчете расстояний для разных углов (тангажный/курсовой контура)
// Пример:
// startAngle = 110, stopAngle = 210
// Расстояние по часовой стрелке: 100 градусов
// Расстояние против часово стрелке: -260 градусов
// Возвращаемое значение 100
// Углы в аппаратной системе координат!
int32_t angle_getMinimalAngleRange(uint32_t startAngle, uint32_t stopAngle, uint32_t code360grad)
{
  int32_t rangeCW, rangeCCW;
  if(stopAngle>startAngle) 
  {
    rangeCW = stopAngle - startAngle;
    rangeCCW = rangeCW - code360grad;       // со знаком минус (против часовой стрелки)
  }
  else 
  {
    rangeCCW = stopAngle - startAngle;  // со знаком минус
    rangeCW = code360grad + rangeCCW;
  }
  int32_t absCW = ABS(rangeCW);
  int32_t absCCW = ABS(rangeCCW);  
  int32_t deltaFi = (absCW<absCCW) ? rangeCW : rangeCCW;    // выбор кратчайшего расстояния
  return deltaFi;
}













/*
Системный угол - беззнаковый (0..360) угол с датчика угла с ценой деления 360/0x7FFFF
Командный угол - знаковый (-180..180) угол с ценой деления 1/128 градуса
*/
 


// Преобразовать беззнаковый угол 0..360 в знаковый -180..180
// angleU - код угла
// code360 - код угла, соответствующий 360 градусам (он будет отличаться в зависимости от шкалы)
int32_t _angle_convertU2S(uint32_t angleU, uint32_t code360)
{
  uint32_t code180 = code360>>1;
  int32_t res = (angleU>=code180) ? ((int32_t)angleU - code360) : angleU;
  return res;
}
// Преобразовать знаковый -180..180 в беззнаковый 0..360 
// angleS - код угла
// code360 - код угла, соответствующий 360 градусам (он будет отличаться в зависимости от шкалы)
uint32_t _angle_convertS2U(int32_t angleS, uint32_t code360)
{
  uint32_t res = (angleS<0) ? (code360 + angleS) : angleS;
  return res;
}

// Преобразовать угол с ценой деления 1/128 градуса в угол с ценой деления 360/MAX_SYSANGLE_CODE
uint32_t angle_convertCmdU2SysU(uint32_t cmdAngle, uint32_t code360)
{
  float ANGLE_STEP = (float)code360/(float)0xB400;
  uint32_t res = (uint32_t)((float)cmdAngle*ANGLE_STEP); // угол от 0..360
  return res;
}
// Преобразовать угол с ценой деления 360/MAX_SYSANGLE_CODE в угол с ценой деления 1/128 градуса 
uint32_t angle_convertSysU2CmdU(uint32_t sysAngle, uint32_t code360)
{
  float ANGLE_STEP = (float)0xB400/(float)code360;
  uint32_t res = (uint32_t)(sysAngle*ANGLE_STEP); // угол от 0..360
  return res;
}

// Вычесть из угла поправку deltaFi
uint32_t angle_deductCorrection(uint32_t angle, uint32_t deltaFi, uint32_t code360)
{
  uint32_t res = (angle >= deltaFi) ? 
                 (angle - deltaFi) :
                 (code360 - (deltaFi - angle));
  return res;
}
// Прибавить к углу поправку deltaFi
uint32_t angle_addCorrection(uint32_t angle, uint32_t deltaFi, uint32_t code360)
{
  uint32_t res = (angle + deltaFi<=code360) ? 
                 (angle + deltaFi) :
                 (deltaFi + angle - code360);
  return res;
}

// Преобразовать знаковый командный угол с ценой деления 1/128 (в аппаратной или логической шкале)
// в системный угол с учетом поправки на шкалу
uint32_t angle_convertCmdS2SysU(int32_t sCmdAngle, uint32_t scaleCorrection, uint32_t code360)
{
  uint32_t uCmdAngle = _angle_convertS2U(sCmdAngle, 360*128);                   // беззнаковый 1/128 апаратн/лог
  uint32_t uExtAngle = angle_convertCmdU2SysU(uCmdAngle, code360);              // беззнаковый 360/7ffff апаратн/лог
  uint32_t uSysAngle = angle_addCorrection(uExtAngle, scaleCorrection, code360);

  uCmdAngle++;  uExtAngle++;  uSysAngle++; uSysAngle--;
  return uSysAngle;
}

// Для отправки по ARINC-каналу
// В функцию передается параметр code360 для определения 
// Преобразовать системный угол c ценой деления 360/7ffff в
// знаковый командный с ценой деления 1/128 с учетом поправки на шкалу (аппар/лог)
int32_t angle_convertSysU2CmdS(uint32_t uSysAngle, uint32_t scaleCorrection, uint32_t code360)
{
  uint32_t uCorrAngle = angle_deductCorrection(uSysAngle, scaleCorrection, code360);     // беззнак. угол в лог/аппар. шкале
  uint32_t uCmdAngle = angle_convertSysU2CmdU(uCorrAngle, code360);                      // беззнак. угол с ценой деления 1/128
  int32_t sCmdAngle = _angle_convertU2S(uCmdAngle, 360*128);
  
  uCorrAngle++;  uCmdAngle++;  sCmdAngle++; sCmdAngle--;  scaleCorrection++;
  return sCmdAngle;
}









































/*
// Преобразовать угол 0..360 c ценой деления 1/128 в -180..180
int32_t angle_cmdConvertU2S(uint32_t angleU)
{
  int32_t res = (angleU>=180*128) ? ((int32_t)angleU - 360*128) : angleU;
  return res;
}
// Преобразовать угол -180..180 c ценой деления 1/128 в 0..360 
uint32_t angle_cmdConvertS2U(int32_t angleS)
{
  uint32_t res = (angleS<0) ? (360*128 + angleS) : angleS;
  return res;
}
*/

/*
// Преобразовать угол 0..360 в -180..180
int32_t angle_sysConvertU2S(uint32_t angleU)
{
  int32_t res = (angleU>=USYSANGLE_TO_CODE(180)) ? (angleU - USYSANGLE_TO_CODE(360)) : angleU;
  return res;
}

// Преобразовать угол -180..180 в 0..360 
uint32_t angle_sysConvertS2U(int32_t angleS)
{
  uint32_t res = (angleS<0) ? (USYSANGLE_TO_CODE(360) + angleS) : angleS;
  return res;
}
*/
/*

// Внести поправку к системной шкале
uint32_t angle_correctSysAngleU(uint32_t sysAngleU, uint32_t correctionAngleU)
{
  uint32_t res = (sysAngleU >= correctionAngleU) ? 
                 (sysAngleU - correctionAngleU) :
                 (MAX_SYSANGLE_CODE - (correctionAngleU - sysAngleU));
  return res;
}


// ОФОРМИРО ФУНКЦИЮ КРАСИВО, ОНА ПЕРЕВОДИТ УГОЛ В АППАРАТНОЙ ИЛИ ЛОГИЧЕСКОЙ ШКАЛЕ В СИСТЕМНЫЙ (ДЛЯ КОМАНД)
uint32_t angle_correctCorrectPlus(uint32_t angle, uint32_t correction)
{
  uint32_t uSysAngle = (angle + correction<=MAX_SYSANGLE_CODE) ? 
                       (angle + correction) :
                       (correction + angle - MAX_SYSANGLE_CODE);
  return uSysAngle;
}

*/