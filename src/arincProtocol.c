#include "stdlib.h"
#include "stdio.h"
#include "MDR1986VE1T.h"
#include "global.h"
#include "settings.h"
#include "arinc.h"
#include "arincProtocol.h"
#include "canmonitor.h"
//#include "canterminal.h" 
#include "timers.h"
#include "gkControlMode.h"
#include "dusFilter.h"
#include "angelSensor.h"
#include "tangageControl.h"

/*
По команде "контроль" ГС входит в режим Самоконтроль и запускает функциональный тест!
После выполнения функционального тест матрица состояния СЛОВА СОСТОЯНИЯ выставляется в зависимости от результата функционального теста
В матрицах остальных слов передается ФУНКЦИОНАЛЬНЫЙ ТЕСТ до тех пор пока не завершится самоконтроль

*/

uint8_t DEBUG_SEND_TEST_VALUE_IN_FUNCTIONAL_TEST = 1;   // для отладки - отправка тестовых значений в функциональном тесте

// Arinc протокол - состояние ГС
uint8_t gs_preparationNow = 1;             // идет подготовка
uint8_t gs_selfControlDisable = 1;         // установлен запрет контроля
uint8_t gs_readyTakeCmd = 1;               // готов принять команду (ГОТ)
uint8_t gs_rxChannelIsFaulted = 0;         // неисправность канала приема УИ
uint8_t gs_rxCommandIsFaulted = 0;         // неисправность принятой УИ
uint8_t gs_rxNewCommand = 0;               // получена новая УИ
uint8_t gs_commandCanBeProcessed = 0;      // возможность выполнения принятой УИ
uint8_t gs_selfControlPerformed = 0;       // выполняется самоконтроль

// Принятая команда
uint32_t g_lastCmdWord = 0;                  // последнее принятое командное слово
uint32_t g_lastCmdCourseAngleWord = 0;       // последнее принятое слово с углом азимута
uint32_t g_lastCmdTangageAngleWord = 0;      // последнее принятое слово с углом тангажа
uint32_t g_lastCmdCourseVelocityWord = 0;    // последнее принятое слово с улгловой скоростью по азимуту
uint32_t g_lastCmdTangageVelocityWord = 0;   // последнее принятое слово с угловой скоростью тангажа
uint32_t g_lastWordRxTime = 0;               // время приема последнего слова
uint8_t  g_rxCmdFlags = 0;                   // флаги приема соответствующих команд. Когда он будет равен b111_111, все пакеты считаются принятыми

// Последние принятые значения углов и угловых скоростей для отправки по CAN
int16_t g_cmdCourseVelocityVUS = 0;          // заданная скорость в режиме ВУС
int16_t g_cmdCourseAngleAR = 0;              // заданный угол в режиме АР
int16_t g_cmdCourseVelocityVUO = 0;          // заданная скорость в режиме ВУО    
int16_t g_cmdCourseAngleVUO = 0;             // заданный угол в режиме ВУО
// Матрицы состояний слов из последней УИ
uint8_t g_cmdMatrix;
uint8_t g_courseAngleMatrix;
uint8_t g_tangageAngleMatrix;
uint8_t g_courseVelocityMatrix;
uint8_t g_tangageVelocityMatrix;
// Значения углов и скоростей из последней УИ
int16_t g_courseAngle128;
int16_t g_tangAngle128;
int16_t g_courseVelocity128;
int16_t g_tangVelocity128;



// Текущая шкала углов в КИ (аппаратная/логическая)
uint8_t g_courseAngleInLogicalScale = 0;    // выдавать курсовой угол в логической шкале
uint8_t g_tangageAngleInLogicalScale = 0;   // выдавать тангажный угол в логической шкале

// Текущие углы и поправки в аппаратным и логическим шкалам
extern uint32_t g_sysAngle360;                  // системный угол с датчика угла
extern uint32_t g_tangageSysAngle360;           // системный угол тангажного контура
extern uint32_t g_hardwareZeroAngleCorrection;  // ноль аппаратной шкалы
extern uint32_t g_logicalZeroAngleCorrection;   // ноль логической шкалы
extern uint32_t g_tangageHardwareZeroAngleCorrection; // ноль аппаратной шкалы тангажного контура
extern uint32_t g_tangageLogicalZeroAngleCorrection;  // ноль логической шкалы тангажного контура

// Текущий режим работы ГС
extern GKControlMode_t gk_controlMode;
extern GSFailure_t     gs_failure;              // отказ ГС (в процессе работы или функционального теста)
extern uint8_t         functionalTestDone;      // завершен функциональный тест




void _arretieredInCurrentPosition();    // арретирование в текущей позиции


// Обработать принятое сообщение
void arinc_processRxWord(uint32_t word)
{    
    switch(word & 0xFF)
    {
      case LABEL_CMD:               // командное слово
        {
            if(!ARINC_WORD_ID_IS_CORRECT(word)) return;
            g_lastCmdWord = word;
            g_lastWordRxTime = system_time;
            g_rxCmdFlags |= 1<<0;
        }
        break;
          
      case LABEL_CMD_COURSE_ANGEL:  // курсовой угол
        {
            if(!ARINC_WORD_ID_IS_CORRECT(word)) return;
            g_lastCmdCourseAngleWord = word;
            g_lastWordRxTime = system_time;
            g_rxCmdFlags |= 1<<1;
        }
        break;
        
      case LABEL_CMD_TANGAGE_ANGEL: // тангажный угол
        {
            if(!ARINC_WORD_ID_IS_CORRECT(word)) return;
            g_lastCmdTangageAngleWord = word;
            g_lastWordRxTime = system_time;
            g_rxCmdFlags |= 1<<2;
        }
        break;
          
      case LABEL_CMD_COURSE_VELOCITY:  // угловая скорость по курсу
        {
            if(!ARINC_WORD_ID_IS_CORRECT(word)) return;
            g_lastCmdCourseVelocityWord = word;
            g_lastWordRxTime = system_time;
            g_rxCmdFlags |= 1<<3;
        }
        break;
          
      case LABEL_CMD_TANGAGE_VELOCITY: // угловая скорость потангажу
        {
            if(!ARINC_WORD_ID_IS_CORRECT(word)) return;
            g_lastCmdTangageVelocityWord = word;
            g_lastWordRxTime = system_time;
            g_rxCmdFlags |= 1<<4;
        }
        break;       
          
      default: break;
    }
}


// Сервис
void arincService()
{
    static uint32_t lastSendTime = 0;    
    if(elapsed(&lastSendTime, ARINC_SEND_STATE_PERIOD))// Отправка состояния
    {
       arincGS_sendState();
    }
    
    // Включение прерываний
    tangageCtrl_enableRxAngleViaCANIfNeeded();
  
    // Прием команды
    if(g_rxCmdFlags != 0)
    {       
       NVIC_DisableIRQ(ARINC429R_IRQn);                                         // отключение прерываний приемника
       if(elapsed(&g_lastWordRxTime, ARINC_CMD_TIMEOUT) && (g_rxCmdFlags != 0x1F))  // если истекло время ожидания пакета 
       {
          g_rxCmdFlags = 0;
          gs_rxCommandIsFaulted = 1; // неисправность УИ
          gs_rxChannelIsFaulted = updateChannelState(gs_rxCommandIsFaulted);
          arincGS_onCmdTimeout();
       }
       else if(g_rxCmdFlags == 0x1F)
       {
          g_rxCmdFlags = 0;
          gs_rxCommandIsFaulted = 0; // исправность УИ
          gs_rxChannelIsFaulted = updateChannelState(gs_rxCommandIsFaulted);
          gs_rxNewCommand = 1;       // получена новая УИ
          arincGS_onCmdReceived();
       }
       NVIC_EnableIRQ(ARINC429R_IRQn);                                          // разрешение прерываний приемника
    }    
}

// Таймаут ожидания команды
void arincGS_onCmdTimeout()
{    
    canMonitor_printf("ARINC COMMAND TIMEOUT");
    arincGS_sendState();
}

// Пришла новая команда от БОУ
void arincGS_onCmdReceived()
{
    // матрицы состояния 
    g_cmdMatrix = ARINC_WORD_GET_MATRIX(g_lastCmdWord);
    g_courseAngleMatrix = ARINC_WORD_GET_MATRIX(g_lastCmdCourseAngleWord);
    g_tangageAngleMatrix = ARINC_WORD_GET_MATRIX(g_lastCmdTangageAngleWord);
    g_courseVelocityMatrix = ARINC_WORD_GET_MATRIX(g_lastCmdCourseVelocityWord);
    g_tangageVelocityMatrix = ARINC_WORD_GET_MATRIX(g_lastCmdTangageVelocityWord);
    
    // курсовой угол
    int16_t tmpValue;
    tmpValue = (g_lastCmdCourseAngleWord>>13) & 0x7FFF;
    g_courseAngle128 = (g_lastCmdCourseAngleWord&1<<28) ? (tmpValue|0x8000) : tmpValue;
    // тангажный угол
    tmpValue = (g_lastCmdTangageAngleWord>>13) & 0x7FFF;
    g_tangAngle128 = (g_lastCmdTangageAngleWord&1<<28) ? (tmpValue|0x8000) : tmpValue;            
    // курсовая скорость
    tmpValue = (g_lastCmdCourseVelocityWord>>14) & 0x1FFF;
    g_courseVelocity128 = (g_lastCmdCourseVelocityWord&1<<28) ? (tmpValue|0xE000) : tmpValue;
    // тангажная скорость
    tmpValue = (g_lastCmdTangageVelocityWord>>14) & 0x1FFF;
    g_tangVelocity128 = (g_lastCmdTangageVelocityWord&1<<28) ? (tmpValue|0xE000) : tmpValue;

    
    uint16_t allMatrixMask = GET_ALL_WORDS_MATRIX_MASK(g_cmdMatrix, g_courseAngleMatrix, g_tangageAngleMatrix, g_courseVelocityMatrix, g_tangageVelocityMatrix);
    if(allMatrixMask == ALL_WORDS_MATRIX_IS_WARNING_MASK)
    {// все слова с матрицей состояния ПРЕДУПРЕЖДЕНИЕ ОБ ОТКАЗЕ
        _arretieredInCurrentPosition();
    }
    else if(allMatrixMask == ALL_WORDS_MATRIX_IS_FUNC_TEST_MASK)
    {// все слова с матрицей состояния ФУНКЦИОНАЛЬНЫЙ ТЕСТ      
      if(((g_courseAngle128>>7) != ARINC_CMD_ANGLE_TEST_VALUE) || ((g_tangAngle128>>7) != ARINC_CMD_ANGLE_TEST_VALUE) ||  
         ((g_courseVelocity128>>7) != ARINC_CMD_VELOCITY_TEST_VALUE) || ((g_tangVelocity128>>7) != ARINC_CMD_VELOCITY_TEST_VALUE))
      {
         gs_rxCommandIsFaulted = 1; // неисправность УИ
         arincGS_sendState();
         return;
      }
       
      //  Слово используется только для контроля УИ.
      //  Если ГС выполняет функциональный тест - то он продолжает его выполнять.
      //  Если не выполняет - то встает в арретир
      if(gs_selfControlPerformed)
      {
         _arretieredInCurrentPosition();
      }
    }
    else 
    {
        switch(g_cmdMatrix)
        {
          case UI_CMD_WORD_MATRIX_STATE_NO_DATA:
            arincGS_sendState();
            break;
          case UI_CMD_WORD_MATRIX_STATE_FUNC_TEST: // должны быть ВСЕ слова с этими матрицами, в противном случае - неисправная УИ
          case UI_CMD_WORD_MATRIX_STATE_WARNING:
            {
               gs_rxCommandIsFaulted = 1; // неисправность УИ
               arincGS_sendState();
               return;
            }
            break;
          case UI_CMD_WORD_MATRIX_STATE_NORMAL:    
            {
              uint8_t numOfCommands = getBitsInWord(g_lastCmdWord&0xFF000);
              if(numOfCommands>1)
              {
                 gs_rxCommandIsFaulted = 1; // неисправность УИ
                 arincGS_sendState();
                 return;
              }

              arincGS_processCmd();
            }
            break;
        }
    }
}

void arincGS_processCmd()
{
   if(gs_preparationNow) // идет подготовка
   {
     gs_commandCanBeProcessed = 0;
     arincGS_sendState();
     return;
   }
  
   if(g_lastCmdWord & ARINC_CMD_RKN)          // РК - разрешение контроля
   {
     cmd_RKN();
   }
   if(g_lastCmdWord & ARINC_CMD_ZKN)          // ЗКН - запрет контроля контроля
   {
     cmd_ZKN();
   }
   else if(g_lastCmdWord & ARINC_CMD_KN)      // Самоконтроль
   {
     cmd_KN();
   }
   else if(g_lastCmdWord & ARINC_CMD_VUS)     // ВУС
   { 
     cmd_VUS(g_courseVelocity128, g_tangVelocity128);
   }
   else if(g_lastCmdWord & ARINC_CMD_VOU)     // ВОУ - внешнее управление ориентацией
   {
     cmd_VUO(g_courseAngle128, g_tangAngle128, g_courseVelocity128, g_tangVelocity128);
   }
   else if(g_lastCmdWord & ARINC_CMD_UST0)    // УСТ0 - установка нулей
   {
     cmd_UST0();
   }
   else if(g_lastCmdWord & ARINC_CMD_AR)      // АР - арретирование
   { 
     cmd_AR(g_courseAngle128, g_tangAngle128);
   }   
   else if(g_lastCmdWord & ARINC_CMD_TP)      // транспортное положение
   {
     cmd_TP();
   }
}

// Разрешить самоконтроль
void cmd_RKN()
{
   if(gs_selfControlDisable)
   {
      gs_commandCanBeProcessed = 1;
      gs_selfControlDisable = 0;
   }
   else
   {
      gs_commandCanBeProcessed = 0;
   }
   arincGS_sendState();
}

// Запретить самоконтроль
void cmd_ZKN()
{
   if(!gs_selfControlDisable) // запрет контроля не установлен
   {
     gs_commandCanBeProcessed = 1;
     gs_selfControlDisable    = 1;
     
     if(gs_selfControlPerformed)   // если выполняется самоконтроль - прерываем его и встаем в АР
     {
       gs_selfControlPerformed = 0;                              // завершить самоконтроль
       functionalTestDone = 1;                                   // функциональный тест завершен
       gs_readyTakeCmd         = (gs_failure == GgFailure_None); // ГОТ=1, если нет ошибок
       cmd_AR(g_courseAngle128, g_tangAngle128);
     }
   }
   else 
   {
     gs_commandCanBeProcessed = 0;        
   }
   arincGS_sendState();
}

// Контроль
void cmd_KN()
{
   if((gs_selfControlDisable==0) && (gs_selfControlPerformed == 0))
   {
     gs_commandCanBeProcessed = gk_setModeSelfControl();
     if(gs_commandCanBeProcessed)
     {
       gk_clearFailure(GgFailure_FuncTestError);
       gs_selfControlPerformed = 1;     // самоконтроль выполняется
       gs_readyTakeCmd = 0;             // ГС сможет принять команду только после запрета контроля, если не будет ошибок
       g_courseAngleInLogicalScale = 1;
       g_tangageAngleInLogicalScale = 1;
     
#ifdef ENABLE_TANGAGE_CONTROL
       tangageCtrl_setModeSelfControl();
#endif
     }
   }
   else
   {
     gs_commandCanBeProcessed = 0;
   }
   arincGS_sendState();
}

// Установка нулей
void cmd_UST0()
{
   if(gs_readyTakeCmd) // ГОТ - готов принят команду
   {
     gs_commandCanBeProcessed = 1;
     
     extern void saveUserDataInFlash();
     g_logicalZeroAngleCorrection = g_sysAngle360;             // ноль логической шкалы
     g_tangageLogicalZeroAngleCorrection = g_tangageSysAngle360;  // ноль логической шкалы тангажного контура
     
     _arretieredInCurrentPosition();
     saveUserDataInFlash();
   }
   else gs_commandCanBeProcessed = 0;
   arincGS_sendState();
}

// Установить режим ВУС
void cmd_VUS(int16_t _courseVelocity128, int16_t _tangVelocity128)
{  
   if(gs_readyTakeCmd)
   {
       gs_commandCanBeProcessed = 1; 
       
       g_cmdCourseVelocityVUS = _courseVelocity128;
       if(_courseVelocity128 > MAX_ABS_COURSE_VELOCITY128) _courseVelocity128 = MAX_ABS_COURSE_VELOCITY128;
       else if(_courseVelocity128 < -MAX_ABS_COURSE_VELOCITY128) _courseVelocity128 = -MAX_ABS_COURSE_VELOCITY128;
      
       int16_t courseVel32 = _courseVelocity128>>2;
       gk_setModeVUS(courseVel32);
#ifdef ENABLE_TANGAGE_CONTROL       
       if(_tangVelocity128 > MAX_ABS_TANGAGE_VELOCITY128) _tangVelocity128 = MAX_ABS_TANGAGE_VELOCITY128;
       else if(_tangVelocity128 < -MAX_ABS_TANGAGE_VELOCITY128) _tangVelocity128 = -MAX_ABS_TANGAGE_VELOCITY128;
       tangageCtrl_setModeVUS(_tangVelocity128);
#endif
   }
   else
   {
     gs_commandCanBeProcessed = 0;
   }
   arincGS_sendState();
}

// Установить режим ВУО
void cmd_VUO(int16_t _courseAngle128, int16_t _tangAngle128, int16_t _courseVelocity128, int16_t _tangVelocity128)
{  
   if(gs_readyTakeCmd)
   {
       gs_commandCanBeProcessed = 1; 
       
       g_courseAngleInLogicalScale = (g_lastCmdCourseAngleWord>>10)&1;
       g_tangageAngleInLogicalScale = (g_lastCmdTangageAngleWord>>10)&1;
       
       g_cmdCourseVelocityVUO = _courseVelocity128;
       g_cmdCourseAngleVUO = _courseAngle128;
       
       // Ограничение скорости
       if(_courseVelocity128 > MAX_ABS_COURSE_VELOCITY128) _courseVelocity128 = MAX_ABS_COURSE_VELOCITY128;
       else if(_courseVelocity128 < -MAX_ABS_COURSE_VELOCITY128) _courseVelocity128 = -MAX_ABS_COURSE_VELOCITY128;
       
       if(_tangVelocity128 > MAX_ABS_TANGAGE_VELOCITY128) _tangVelocity128 = MAX_ABS_TANGAGE_VELOCITY128;
       else if(_tangVelocity128 < -MAX_ABS_TANGAGE_VELOCITY128) _tangVelocity128 = -MAX_ABS_TANGAGE_VELOCITY128;
       
      
       uint32_t correction = g_courseAngleInLogicalScale ? g_logicalZeroAngleCorrection : g_hardwareZeroAngleCorrection;
       uint32_t sysAngleU = angle_convertCmdS2SysU(_courseAngle128, correction, MAX_SYSANGLE_CODE);
       int16_t courseVel32 = _courseVelocity128>>2;
       gk_setModeVOU(sysAngleU, courseVel32);
         
#ifdef ENABLE_TANGAGE_CONTROL
       correction = g_tangageAngleInLogicalScale ? g_tangageLogicalZeroAngleCorrection : g_tangageHardwareZeroAngleCorrection;
       uint32_t tangSysAngle = angle_convertCmdS2SysU(_tangAngle128, correction, MAX_TANGAGE_SYSANGLE_CODE);
       tangageCtrl_setModeVUO(tangSysAngle, _tangAngle128, _tangVelocity128);
#endif
   }
   else 
   {
     gs_commandCanBeProcessed = 0;
   }
   arincGS_sendState();
}

// Установить режим АР
void cmd_AR(int16_t _courseAngle128, int16_t _tangAngle128)
{   
   if(gs_readyTakeCmd)
   {
       gs_commandCanBeProcessed = 1; 
       
       g_courseAngleInLogicalScale = (g_lastCmdCourseAngleWord>>10)&1;
       g_tangageAngleInLogicalScale = (g_lastCmdTangageAngleWord>>10)&1;
              
       g_cmdCourseAngleAR = _courseAngle128;         
       uint32_t correction = g_courseAngleInLogicalScale ? g_logicalZeroAngleCorrection : g_hardwareZeroAngleCorrection;
       uint32_t sysAngleU = angle_convertCmdS2SysU(_courseAngle128, correction, MAX_SYSANGLE_CODE);
       gk_setModeAR(sysAngleU);
         
#ifdef ENABLE_TANGAGE_CONTROL
       correction = g_tangageAngleInLogicalScale ? g_tangageLogicalZeroAngleCorrection : g_tangageHardwareZeroAngleCorrection;
       uint32_t tangSysAngle = angle_convertCmdS2SysU(_tangAngle128, correction, MAX_TANGAGE_SYSANGLE_CODE);
       tangageCtrl_setModeAR(tangSysAngle, _tangAngle128);
#endif    
   }
   else
   {
       gs_commandCanBeProcessed = 0;
   }
   arincGS_sendState();
}

// Установить режим ТП
void cmd_TP()
{
   if(gs_readyTakeCmd)
   {
       gs_commandCanBeProcessed = 1; 
       g_courseAngleInLogicalScale = 1;
       g_tangageAngleInLogicalScale = 1;
       
       uint32_t angleU = angle_convertCmdS2SysU(0*128, g_logicalZeroAngleCorrection, MAX_SYSANGLE_CODE);
       gk_setModeTP(angleU);
         
#ifdef ENABLE_TANGAGE_CONTROL
       uint32_t tangAngleU = angle_convertCmdS2SysU(90*128, g_tangageLogicalZeroAngleCorrection, MAX_TANGAGE_SYSANGLE_CODE);
       tangageCtrl_setModeTP(tangAngleU);
#endif
   }
   else
   {
       gs_commandCanBeProcessed = 0;
   }
   arincGS_sendState();
}


// Встать в арретир в текущей позиции
void _arretieredInCurrentPosition()
{
   // Для корректного отображения текущих углов арретирования в CAN-овской программе 
   // вычисляем углы в шкале с шагом 1/128 градуса, как если бы пришла команда АР с текущими углами в качестве параметров!
   uint32_t correction;
   int16_t angle128;
  
   // курсовой контур в арретир
   correction = g_courseAngleInLogicalScale ? g_logicalZeroAngleCorrection : g_hardwareZeroAngleCorrection;
   g_cmdCourseAngleAR = angle_convertSysU2CmdS(g_sysAngle360, correction, MAX_SYSANGLE_CODE);        
   gk_setModeAR(g_sysAngle360);
  
   // установить тангажный контур в арретир
   correction = g_tangageAngleInLogicalScale ? g_tangageLogicalZeroAngleCorrection : g_tangageHardwareZeroAngleCorrection;
   angle128 = angle_convertSysU2CmdS(g_tangageSysAngle360, correction, MAX_TANGAGE_SYSANGLE_CODE);
   tangageCtrl_setModeAR(g_tangageSysAngle360, angle128);
}


// Обновить состояние канала приема исходя из исправности КУ (последние три или 3 из шести...)
// cmdIsFault = 1 - управляющая команда неисправна
uint8_t updateChannelState(uint8_t cmdIsFault)
{
  static uint8_t channelFault = 0;         // флаг неисправности канала
  static uint8_t cmdFaultSequence = 0;     // флаги исправности последних 6 КУ
  cmdFaultSequence = (cmdFaultSequence>>1 | (uint8_t)(cmdIsFault<<5)) & 0x3F;
  
  switch(cmdFaultSequence)
  {
     case 0x00: channelFault = 0; break;
     case 0x01: channelFault = 0; break;
     case 0x02: channelFault = 0; break;
     case 0x03: channelFault = 0; break;
     case 0x04: channelFault = 0; break;
     case 0x05: channelFault = 0; break;
     case 0x06: channelFault = 0; break;
     case 0x07: channelFault = 0; break;
          
     case 0x0B: channelFault = 1; break;
     case 0x0D: channelFault = 1; break;
     case 0x0E: channelFault = 1; break;
     case 0x0F: channelFault = 1; break;
     case 0x13: channelFault = 1; break;
     case 0x15: channelFault = 1; break;
     case 0x16: channelFault = 1; break;
     case 0x17: channelFault = 1; break;
     case 0x19: channelFault = 1; break;
     case 0x1A: channelFault = 1; break;
     case 0x1B: channelFault = 1; break;
     case 0x1C: channelFault = 1; break;
     case 0x1D: channelFault = 1; break;
     case 0x1E: channelFault = 1; break;
     case 0x1F: channelFault = 1; break;
     case 0x23: channelFault = 1; break;
     case 0x25: channelFault = 1; break;
     case 0x26: channelFault = 1; break;
     case 0x27: channelFault = 1; break;
     case 0x29: channelFault = 1; break;     
     case 0x2A: channelFault = 1; break;
     case 0x2B: channelFault = 1; break;
     case 0x2C: channelFault = 1; break;
     case 0x2D: channelFault = 1; break;
     case 0x2E: channelFault = 1; break;
     case 0x2F: channelFault = 1; break;     
     case 0x31: channelFault = 1; break;
     case 0x32: channelFault = 1; break;
     case 0x33: channelFault = 1; break;
     case 0x34: channelFault = 1; break;
     case 0x35: channelFault = 1; break;
     case 0x36: channelFault = 1; break;
     case 0x37: channelFault = 1; break;    
     case 0x38: channelFault = 1; break;
     case 0x39: channelFault = 1; break;
     case 0x3A: channelFault = 1; break;
     case 0x3B: channelFault = 1; break;
     case 0x3C: channelFault = 1; break;
     case 0x3D: channelFault = 1; break;
     case 0x3E: channelFault = 1; break;
     case 0x3F: channelFault = 1; break;   
     default : break;
  }
    
  // Состояние канала приема
  return channelFault;
}




// Отправить состояние
void arincGS_sendState()
{
   uint32_t stateWords[] = {
              getGSStateWord(),
              getAligmentCourseAngel(),
              getAligmentTangageAngel(),
              getCourseAngel(),
              getTangageAngel(),
              getCourseVelocity(),
              getTangageVelocity(),
        };        
   //for(int i=0; i<7; i++) printf("%.8x\n", stateWords[i]);
   //printf("\n");
   arincSendArray(stateWords, sizeof(stateWords)/4);
}

// Слово состояния
uint32_t getGSStateWord()
{
  extern VUOStage_t vuoStage;
  extern uint8_t courseArretieredNow;           // курсовой контур находится в арретире
  extern uint8_t tangageArretieredNow;          // тангажный контур находится в арретире
  extern FuncTestStage_t functionalTestStage;
  
  GKControlMode_t mode = gk_controlMode;
  uint8_t gsArretiered = courseArretieredNow & tangageArretieredNow;
  uint8_t RVUS = (mode == GkMode_VUS) ||                                        // режим управления по угловым скоростям (ВУС)
                 ((mode==GkMode_VUO) && (vuoStage==VUO_VUS)) ||
                 ((mode==GkMode_SelfControl) && ((functionalTestStage==FUNCTEST_VUS_TEST) || (functionalTestStage==FUNCTEST_COMPLETE_WITH_ERROR)));
  uint8_t RUU = (!gsArretiered) && (                                            // управления ориентацией ГС по углам
                                     (mode == GkMode_AR) || 
                                     ((mode==GkMode_VUO) && (vuoStage==VUO_AR)) || 
                                     (mode == GkMode_TP) ||
                                     ((mode==GkMode_SelfControl) && ((functionalTestStage==FUNCTEST_AR_TEST) || (functionalTestStage==FUNCTEST_COMPLETE_SUCCESS)))
                                   );
  uint8_t UAR = gsArretiered && (                                               // находимся в электрическом арретире
                                  (mode == GkMode_AR) ||
                                  (mode==GkMode_SelfControl)
                                 );
  uint8_t UTP = (mode == GkMode_TP) && gsArretiered;                            // установлен в транспортное положение
  
  uint32_t currentGsState = gs_preparationNow<<10 |
                            gs_selfControlDisable<<12 |
                            gs_readyTakeCmd<<14 | 
                            RVUS<<15 |
                            RUU<<16 |
                            UAR<<17 |
                            UTP<<18 |
                            1<<19 |                     // неограниченный круговой поворот по азимуту (курсу)
                            1<<20 |                     // неограниченный круговой поворот по наклону (тангажу)
                            gs_rxChannelIsFaulted<<24 |
                            gs_rxCommandIsFaulted<<26 |
                            gs_rxNewCommand<<27 |
                            gs_commandCanBeProcessed<<28;

  uint8_t matrixState;

  if(gs_failure) matrixState = KI_STATE_WORD_MATRIX_STATE_WARNING;
  else if(gs_preparationNow) matrixState = KI_STATE_WORD_MATRIX_STATE_NO_DATA;
  else if(!functionalTestDone) matrixState = KI_STATE_WORD_MATRIX_STATE_FUNC_TEST; // выполняется функциональный тест
  else matrixState = KI_STATE_WORD_MATRIX_STATE_NORMAL;
  

//  if(gs_preparationNow) matrixState = KI_STATE_WORD_MATRIX_STATE_NO_DATA;
//  else if(!functionalTestDone) matrixState = KI_STATE_WORD_MATRIX_STATE_FUNC_TEST; // выполняется функциональный тест
//  else if(gs_failure) matrixState = KI_STATE_WORD_MATRIX_STATE_WARNING;
//  else matrixState = KI_STATE_WORD_MATRIX_STATE_NORMAL;
 
  return (matrixState<<29 | currentGsState | LABEL_STATE_GS);
}
// Угол юстировки по курсу
uint32_t getAligmentCourseAngel()
{
  int16_t angle;
  uint8_t sign;
  uint8_t logicalScale = 0;
  uint8_t matrixState;  
  if(gs_preparationNow) matrixState = KI_DATA_WORD_MATRIX_STATE_NO_DATA;
  else if(gs_selfControlPerformed) matrixState = KI_DATA_WORD_MATRIX_STATE_FUNC_TEST;
  else if(gs_failure) matrixState = KI_DATA_WORD_MATRIX_STATE_WARNING;
  else matrixState = KI_DATA_WORD_MATRIX_STATE_NORMAL;
  
  if(gs_selfControlPerformed) // выполняется самоконтроль
  {
     angle = ARINC_CMD_ANGLE_TEST_VALUE*128;
     sign = 0;
  }
  else 
  {
     angle = angle_convertSysU2CmdS(g_logicalZeroAngleCorrection, g_hardwareZeroAngleCorrection, MAX_SYSANGLE_CODE);
     sign = (angle < 0);
     angle = angle & 0x7FFF; 
  }
  uint32_t word = LABEL_STATE_ALIGMENT_COURSE_ANGEL |
                  logicalScale<<10 |
                  angle<<13 | 
                  sign<<28 |
                  matrixState<<29;
  return word;
}
// Угол юстировки по тангажу
uint32_t getAligmentTangageAngel()
{
  int16_t angle;
  uint8_t sign;
  uint8_t logicalScale = 0;
  uint8_t matrixState;
  if(gs_preparationNow) matrixState = KI_DATA_WORD_MATRIX_STATE_NO_DATA;
  else if(gs_selfControlPerformed) matrixState = KI_DATA_WORD_MATRIX_STATE_FUNC_TEST;
  else if(gs_failure) matrixState = KI_DATA_WORD_MATRIX_STATE_WARNING;   
  else matrixState = KI_DATA_WORD_MATRIX_STATE_NORMAL;
  
  if(gs_selfControlPerformed) // выполняется самоконтроль
  {
     angle = ARINC_CMD_ANGLE_TEST_VALUE*128;
     sign = 0;
  }
  else 
  {
     angle = angle_convertSysU2CmdS(g_tangageLogicalZeroAngleCorrection, g_tangageHardwareZeroAngleCorrection, MAX_TANGAGE_SYSANGLE_CODE);
     sign = (angle < 0);
     angle = angle & 0x7FFF; 
  }
  uint32_t word = LABEL_STATE_ALIGMENT_TANGAGE_ANGEL |
                  logicalScale<<10 |
                  angle<<13 | 
                  sign<<28 |
                  matrixState<<29;
  return word;
}





// Угол по курсу
uint32_t getCourseAngel()
{
  int16_t angle;
  uint8_t sign;
  uint8_t logicalScale = 0;
  uint8_t matrixState;
  if(gs_preparationNow) matrixState = KI_DATA_WORD_MATRIX_STATE_NO_DATA;
  else if(gs_selfControlPerformed) matrixState = KI_DATA_WORD_MATRIX_STATE_FUNC_TEST;
  else if(gs_failure) matrixState = KI_DATA_WORD_MATRIX_STATE_WARNING;   
  else matrixState = KI_DATA_WORD_MATRIX_STATE_NORMAL;
  
  if(gs_selfControlPerformed & DEBUG_SEND_TEST_VALUE_IN_FUNCTIONAL_TEST) // выполняется самоконтроль
  {
     angle = ARINC_CMD_ANGLE_TEST_VALUE*128;
     sign = 0;
  }
  else 
  {
     uint32_t correction = g_courseAngleInLogicalScale ? g_logicalZeroAngleCorrection : g_hardwareZeroAngleCorrection;
     angle = angle_convertSysU2CmdS(g_sysAngle360, correction, MAX_SYSANGLE_CODE);
     sign = (angle < 0);
     angle = angle & 0x7FFF;   
     logicalScale = g_courseAngleInLogicalScale;
  }
  uint32_t word = LABEL_STATE_COURSE_ANGEL |
                  logicalScale<<10 |
                  angle<<13 | 
                  sign<<28 |
                  matrixState<<29;
  return word;  
}
// Угол по тангажу
uint32_t getTangageAngel()
{
  int16_t angle;
  uint8_t sign;
  uint8_t logicalScale = 0;
  uint8_t matrixState;
  if(gs_preparationNow) matrixState = KI_DATA_WORD_MATRIX_STATE_NO_DATA;
  else if(gs_selfControlPerformed) matrixState = KI_DATA_WORD_MATRIX_STATE_FUNC_TEST;
  else if(gs_failure) matrixState = KI_DATA_WORD_MATRIX_STATE_WARNING;   
  else matrixState = KI_DATA_WORD_MATRIX_STATE_NORMAL;
  
  if(gs_selfControlPerformed & DEBUG_SEND_TEST_VALUE_IN_FUNCTIONAL_TEST) // выполняется самоконтроль
  {
     angle = ARINC_CMD_ANGLE_TEST_VALUE*128;
     sign = 0;
  }
  else 
  {
     uint32_t correction = g_tangageAngleInLogicalScale ? g_tangageLogicalZeroAngleCorrection : g_tangageHardwareZeroAngleCorrection;
     angle = angle_convertSysU2CmdS(g_tangageSysAngle360, correction, MAX_TANGAGE_SYSANGLE_CODE);
     sign = (angle < 0);
     angle = angle & 0x7FFF;   
     logicalScale = g_tangageAngleInLogicalScale;
  }
  uint32_t word = LABEL_STATE_TANGAGE_ANGEL |
                  logicalScale<<10 |
                  angle<<13 | 
                  sign<<28 |
                  matrixState<<29;
  return word;  
}


// Угловая скорость по курсу
uint32_t getCourseVelocity()
{
  static int numOfFrame = 0;
  if(++numOfFrame == 16) numOfFrame = 1;  

  uint8_t sign;
  uint16_t sample128;
  uint8_t matrixState;
  if(gs_preparationNow) matrixState = KI_DATA_WORD_MATRIX_STATE_NO_DATA;
  else if(gs_selfControlPerformed) matrixState = KI_DATA_WORD_MATRIX_STATE_FUNC_TEST;
  else if(gs_failure) matrixState = KI_DATA_WORD_MATRIX_STATE_WARNING;   
  else matrixState = KI_DATA_WORD_MATRIX_STATE_NORMAL;
    
  if(gs_selfControlPerformed & DEBUG_SEND_TEST_VALUE_IN_FUNCTIONAL_TEST) // выполняется самоконтроль
  {
     sign = 0;
     sample128 = ARINC_CMD_VELOCITY_TEST_VALUE*128;
  }
  else
  {
     extern int16_t dusAmplitude; // амплитуда с ценой деления 1/32 град.
     int16_t amplitudeCode = dusAmplitude;   
     sign = amplitudeCode<0;   
     sample128 = amplitudeCode<<2;                         // переходим к шкале 1/128 градуса    
  }
  uint32_t word = LABEL_STATE_COURSE_VELOCITY |
                  (numOfFrame<<10) |
                  (sample128&0x3FFF)<<14 |
                  sign<<28 |
                  matrixState<<29;
  return word;
}
// Угловая скорость по тангажу
uint32_t getTangageVelocity()
{
  static int numOfFrame = 0;
  if(++numOfFrame == 16) numOfFrame = 1;  

  uint8_t sign;
  uint16_t sample128;
  uint8_t matrixState;
  if(gs_preparationNow) matrixState = KI_DATA_WORD_MATRIX_STATE_NO_DATA;
  else if(gs_selfControlPerformed) matrixState = KI_DATA_WORD_MATRIX_STATE_FUNC_TEST;
  else if(gs_failure) matrixState = KI_DATA_WORD_MATRIX_STATE_WARNING;   
  else matrixState = KI_DATA_WORD_MATRIX_STATE_NORMAL;
    
  if(gs_selfControlPerformed & DEBUG_SEND_TEST_VALUE_IN_FUNCTIONAL_TEST) // выполняется самоконтроль
  {
     sign = 0;
     sample128 = ARINC_CMD_VELOCITY_TEST_VALUE*128;
  }
  else
  {
     extern int16_t tangageAmplitudeCode; // амплитуда с ценой деления 1/32 град.
     int16_t amplitudeCode = tangageAmplitudeCode;
     sign = amplitudeCode<0;   
     sample128 = amplitudeCode<<2;                         // переходим к шкале 1/128 градуса    
  }
  uint32_t word = LABEL_STATE_TANGAGE_VELOCITY |
                  (numOfFrame<<10) |
                  (sample128&0x3FFF)<<14 |
                  sign<<28 |
                  matrixState<<29;
  return word;
}
  
