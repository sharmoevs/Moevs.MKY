#ifndef __GK_CONTROL_MODE_H__
#define __GK_CONTROL_MODE_H__

#include "MDR1986VE1T.h"


#define CONVERT_VELOCITY_TO_CODE32(velocityFloat)  ((int32_t)(velocityFloat*32))         // кол-во отсчетов с шагом 1/32 градуса


#define GK_VEL_CALIBRATION_SPEED        20              // угловая скорость при калибровке
#define GK_CALIB_NUM_OF_CIRCLES         2               // количество оборотов при калибровке
#define GK_CALIB_TIME_OF_ONE_TURN       ((360)/GK_VEL_CALIBRATION_SPEED)
#define GK_TOTAL_CALIB_TIME             (GK_CALIB_NUM_OF_CIRCLES*GK_CALIB_TIME_OF_ONE_TURN)


// Причины неисправности ГС
typedef enum
{
  GgFailure_None                = 0x00, // нет ошибок
  GgFailure_FuncTestError       = 0x01, // провален функциональный тест
  GgFailure_CourseSpeedProtect  = 0x02, // сработала защита по скорости курсового контура
  GgFailure_TangageSpeedProtect = 0x04, // сработала защита по скорости тангажного контура
  GgFailure_StartUpCalibration  = 0x08, // калибровка по включению питания не завершена за отведенное время
} GSFailure_t;

// Режимы управления ГК
typedef enum
{
  GkMode_EngineDisabled                 = 0x00, // Запрет управлением двигателями    
  GkMode_EngineOffBySpeedProtection     = 0x01, // Двигатели выключены после срабатывания защиты по скорости
  GkMode_EngineOffByCommand             = 0x02, // Двигатели выключены командой
  GkMode_VUS                            = 0x03, // Внешнее управление по угловым скоростям (ВУС)
  GkMode_AR                             = 0x04, // Внешнее управление по углу
  GkMode_VelocityCalibration            = 0x05, // Калибровка по скорости
  GkMode_TudaSudaVUS                    = 0x06, // Движение туда-сюда ВУС
  GkMode_TP                             = 0x07, // Транспортное положение
  GkMode_VUO                            = 0x08, // Внешнее управление ориентацией
  GkMode_SelfControl                    = 0x09, // Самоконтроль
  GkMode_Initialize                     = 0x0A, // Инициализация по включению питания
  GkMode_TudaSudaAR                     = 0x0B, // Движение туда-сюда в Арретире
  GkMode_DUSTemperatureCalibration      = 0x0C, // Калибровка ДУС по температуре
  GkMode_HardFault                      = 0x0D, // Hardfault, Безнадежная ошибка
} GKControlMode_t;

// Стадии режима ВУО
typedef enum 
{ 
  VUO_INIT,
  VUO_AR, 
  VUO_VUS 
} VUOStage_t;

// Стадии режима Самоконтроль
typedef enum 
{ 
  FUNCTEST_INIT = 0,                 // инициализация режима самоконтроля
  FUNCTEST_VUS_TEST,                 // тест ВУС (сделать полный оборот)  
  FUNCTEST_AR_TEST,                  // тест АР (встать на заданные углы)
  FUNCTEST_COMPLETE_SUCCESS,         // самоконтроль завершен успешно
  FUNCTEST_COMPLETE_WITH_ERROR       // самоконтроль завершен с ошибкой
} FuncTestStage_t;

// Коды ошибок функционального теста
typedef enum 
{ 
  FuncTestErrorCode_None = 0,
  FuncTestErrorCode_TestTimeout                 = 0x01,         // функциональный тест не завершился в течении 60с
  
  FuncTestErrorCode_CourseMoveToStartTimeout    = 0x02,         // не успел выйти на стартовую позицию
  FuncTestErrorCode_CourseArrieterTimeout       = 0x04,         // не успел выйти в арретир за отведенное время
  FuncTestErrorCode_CourseRotationTimeout       = 0x08,         // не успел сделать оборот за заданное время
  FuncTestErrorCode_CourseVusVelocityIncorrect  = 0x10,         // сделал оборот, но скорость не соответствует заданной (40 градусов вместо 20, например)
  
  FuncTestErrorCode_TangageMoveToStartTimeout   = 0x20,
  FuncTestErrorCode_TangageArrieterTimeout      = 0x40,
  FuncTestErrorCode_TangageRotationTimeout      = 0x80,
  FuncTestErrorCode_TangageVusVelocityIncorrect = 0x100,
} FuncTestErrorCode_t;

// Функциональный тест. Эпаты арретирования - выход на стартовую позицию/тест АР
typedef enum 
{ 
  FuncTestArMode_MoveToStart,           // выход в начальную точку
  FuncTestArMode_GetNextTestAngle       // задать следующий угол из тестовых
} FuncTestArrieterMode_t;







void gk_init();         // инициализация
void gk_setFailure(GSFailure_t error);
void gk_clearFailure(GSFailure_t error);
void gk_checkSpeedProtection();     // защита по скорости
void gk_checkNSetupPin();
void gk_moveNext();
  
void gk_setModeEngineOffBySpeedProtection();    // Выключить двигатели после срабатывания защиты по скорости
void gk_setModeAR(uint32_t sysAngleU);  // Встать на заданный угол (арретир)
void gk_setModeVUS(int16_t velocityCode32); // Установить режим ВУС
void gk_setModeVelocityCalibration();   // Установить режим калибровки по скорости
void gk_setModeEngineOff();             // Выключение двигателей
void gk_setModeTudaSudaVUS(uint16_t period, int16_t velocity);       // Движение туда-сюда ВУС
void gk_setModeTudaSudaAR(uint16_t period, uint32_t angle1, uint32_t angle2); // Движение туда-сюда АР
void gk_setModeTP(uint32_t angleU);     // Транспортное положение
void gk_setModeVOU(uint32_t sysAngleU, int16_t velocityCode32);
uint8_t gk_setModeSelfControl();           // Самоконтроль
void gk_setModeDusTemperatureCalibration();// Калибровка ДУС по температуре



void gkModeControl();

#endif //__GK_CONTROL_MODE_H__
