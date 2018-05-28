#include "tangageControl.h"
#include "can.h"
#include "global.h"
#include "gkControlMode.h"
#include "canMonitor.h"
#include "canFrameQueue.h"

extern GSFailure_t gs_failure;

// Очередь сообщений
CanSoftwareBuffer_t *tangageControlSoftBuffer;

//Прием угла
uint32_t lastTangageAngleRxTime=0;             // время последнего приема тангажного угла
uint32_t g_tangageSysAngle360;
uint32_t g_tangageHardwareZeroAngleCorrection; // ноль аппаратной шкалы
uint32_t g_tangageLogicalZeroAngleCorrection;  // ноль логической шкалы
int16_t  tangageAmplitudeCode;                 // амплитуда ДУС

uint32_t tangageArretieredRequiredAngleU32;    // заданный угол арретира тангажного контура
GKControlMode_t tangageControlMode;            // режим работы тангажного контура


// тест шины CAN
uint8_t canBusTest_enable = 0;
uint32_t canBusTest_value = 0;
uint32_t canBusTest_errorCount = 0;
uint32_t canBusTestStartTime = 0;       // время приема команды СТАРТ

uint8_t tangageCtrl_send(uint8_t *pData, uint8_t len);
void tangageCtrl_rxAngleEnable();
void tangageCtrl_rxAngleDisable();


void tangageCtrl_init()
{
  tangageControlSoftBuffer = canSwBuffer_create(MDR_CAN1, 25, CAN_TANGAGE_CONTROL_TX_BUF);
}


// Отправить данные по CAN
uint8_t tangageCtrl_send(uint8_t *pData, uint8_t len)
{
#ifndef USE_DYNAMIC_CAN_QUEUE 
   return can1_send_packet(CAN_TANGAGE_CONTROL_TX_BUF, CANID_TANGAGE_CONTORL_TX, pData, len);  
#else
   return canSwBuffer_enqueue(tangageControlSoftBuffer, pData, len, CANID_TANGAGE_CONTORL_TX);
#endif
}
// Разрешение приема угла с платы АЦПВТ
void tangageCtrl_rxAngleEnable()
{
   MDR_CAN1->INT_RX |= 1<<CAN_TANGAGE_ANGLE_RX_BUF;
}
// Запрет приема угла с платы АЦПВТ
void tangageCtrl_rxAngleDisable()
{
   MDR_CAN1->INT_RX &= ~(1<<CAN_TANGAGE_ANGLE_RX_BUF);
}



// Разрешить прием угла, если необходимо
void tangageCtrl_enableRxAngleViaCANIfNeeded()
{
  if(system_time - lastTangageAngleRxTime >= TANGAGE_ANGLE_RX_INTERVAL)
  {
    tangageCtrl_rxAngleEnable();
  } 
}

// Принято сообщение от АЦПВТ (передается с частотой 2kHz) 
void tangageCtrl_rxAngle(uint8_t *buf, uint8_t len)
{
  if(len != 3) return;
  g_tangageSysAngle360 = buf[0]<<16 | buf[1]<<8 | buf[2];
  lastTangageAngleRxTime = system_time;
  tangageCtrl_rxAngleDisable();
}

// Принято сообщение с текущей скоростью и состоянием от тангажного контура 
void tangageCtrl_rxMsg(uint8_t *buf, uint8_t len)
{
  if(len != 5) return;
  
  // Отдалище
////  if(buf[0] == 0xFF && buf[1] == 0xFF && buf[2] == 0xFF && buf[3] == 0xFF && buf[4] == 0xFF)
////  {
////   canBusTest_stop();
////   return;
////  }
  
  extern int16_t dusTemperature;
  
  tangageAmplitudeCode = (int16_t)(buf[0]<<8 | buf[1]);
  tangageControlMode = (GKControlMode_t)buf[2];
  dusTemperature = (int16_t)(buf[3]<<8 | buf[4]);
    
  __disable_irq();
  if(tangageControlMode == GkMode_EngineOffBySpeedProtection) gk_setFailure(GgFailure_TangageSpeedProtect);
  else gk_clearFailure(GgFailure_TangageSpeedProtect); 
  __enable_irq();
}




// Установить режим ВУС тангажного контура
uint8_t tangageCtrl_setModeVUS(int16_t velocity128)
{ 
  uint8_t buf[3];
  buf[0] = TANGAGE_CTRL_SETMODE_VUS;
  buf[1] = (uint8_t)(velocity128>>8); 
  buf[2] = (uint8_t)velocity128;
  return tangageCtrl_send(buf, sizeof(buf));
}
 
// Установить режим АРРЕТИР тангажного контура
// angleU - 18 битный угол с поправками, для управления
// dbgCmdAngle - угол переданный в команде по ARINC. Передается в тангажный контур для can-монитора
uint8_t tangageCtrl_setModeAR(uint32_t angleU, int16_t dbgCmdAngle)
{
  tangageArretieredRequiredAngleU32 = angleU;
  
  uint8_t buf[6];
  buf[0] = TANGAGE_CTRL_SETMODE_AR;
  buf[1] = (uint8_t)(angleU>>16);
  buf[2] = (uint8_t)(angleU>>8);
  buf[3] = (uint8_t)angleU;
  buf[4] = (uint8_t)(dbgCmdAngle>>8);
  buf[5] = (uint8_t)dbgCmdAngle;
  return tangageCtrl_send(buf, sizeof(buf));
}

// Выключить двигатели
uint8_t tangageCtrl_setModeEngineOff()
{
  uint8_t buf[1];
  buf[0] = TANGAGE_CTRL_SETMODE_ENGINE_OFF;
  return tangageCtrl_send(buf, sizeof(buf));
}

// Выключить двигатели
uint8_t tangageCtrl_setModeDisableEngine()
{
  uint8_t buf[1];
  buf[0] = TANGAGE_CTRL_SETMODE_DISABLE_ENGINE;
  return tangageCtrl_send(buf, sizeof(buf));
}

// Транспортное положение
uint8_t tangageCtrl_setModeTP(uint32_t angleU)
{
  tangageArretieredRequiredAngleU32 = angleU;
  
  uint8_t buf[5];
  buf[0] = TANGAGE_CTRL_SETMODE_TP;
  buf[1] = (uint8_t)(angleU>>24);
  buf[2] = (uint8_t)(angleU>>16);
  buf[3] = (uint8_t)(angleU>>8);
  buf[4] = (uint8_t)angleU;
  return tangageCtrl_send(buf, sizeof(buf));
}

// ВУО
uint8_t tangageCtrl_setModeVUO(uint32_t angleU, int16_t dbgCmdAngle, int16_t velocity128)
{  
  tangageArretieredRequiredAngleU32 = angleU;
  
  uint8_t buf[8];
  buf[0] = TANGAGE_CTRL_SETMODE_VUO;
  buf[1] = (uint8_t)(angleU>>16);
  buf[2] = (uint8_t)(angleU>>8);
  buf[3] = (uint8_t)angleU;
  buf[4] = (uint8_t)(dbgCmdAngle>>8);
  buf[5] = (uint8_t)dbgCmdAngle;  
  buf[6] = (uint8_t)(velocity128>>8);
  buf[7] = (uint8_t)velocity128;
  return tangageCtrl_send(buf, sizeof(buf));
}

// Задать начальный угол на который должен встать контур при калибровке
uint8_t tangageCtrl_sendAngleCorretion(uint32_t hardwareCorrection, uint32_t logicalCorrection)
{  
  uint8_t buf[7];
  buf[0] = TANGAGE_CTRL_ANGLE_CORRECTIONS;
  buf[1] = (uint8_t)(hardwareCorrection>>16);
  buf[2] = (uint8_t)(hardwareCorrection>>8);
  buf[3] = (uint8_t)hardwareCorrection;
  buf[4] = (uint8_t)(logicalCorrection>>16);
  buf[5] = (uint8_t)(logicalCorrection>>8);
  buf[6] = (uint8_t)logicalCorrection;
  return tangageCtrl_send(buf, sizeof(buf));
}

// Установить режим самоконтроля
uint8_t tangageCtrl_setModeSelfControl()
{
  uint8_t buf[2];
  buf[0] = TANGAGE_CTRL_SETSUBMODE_SELFCONTROL;
  buf[1] = (uint8_t)TANGAGE_SELFCONTROL_INIT;
  return tangageCtrl_send(buf, sizeof(buf));
}
// Установить режим ВУС при самоконтроле
uint8_t tangageCtrl_setSelfControlSubmodeVUS(int16_t velocity32)
{
  uint8_t buf[5];
  buf[0] = TANGAGE_CTRL_SETSUBMODE_SELFCONTROL;
  buf[1] = (uint8_t)TANGAGE_SELFCONTROL_VUS;
  buf[2] = 0;   // резерв
  buf[3] = (uint8_t)(velocity32>>8);
  buf[4] = (uint8_t)velocity32;  
  return tangageCtrl_send(buf, sizeof(buf));
}

// Установить режим АР при самоконтроле
// angleU - 18битный системный угол с учетом поправок на который должен встать шар
// angle128 - угол с шагом 1/128 градуса соответствующий angleU. Для отображения в CAN-овском мониторе
uint8_t tangageCtrl_setSelfControlSubmodeAR(uint32_t angleU, int16_t angle128)
{  
  tangageArretieredRequiredAngleU32 = angleU;
  
  uint8_t buf[8];
  buf[0] = TANGAGE_CTRL_SETSUBMODE_SELFCONTROL;
  buf[1] = (uint8_t)TANGAGE_SELFCONTROL_AR;  
  buf[2] = 0;   // резерв
  buf[3] = (uint8_t)(angleU>>16);
  buf[4] = (uint8_t)(angleU>>8);
  buf[5] = (uint8_t)angleU;
  buf[6] = (uint8_t)(angle128>>8);
  buf[7] = (uint8_t)angle128;  
  return tangageCtrl_send(buf, sizeof(buf));
}

// Завершить функциональный тест с ошибкой или без
uint8_t tangageCtrl_setSelfControlSubmodeEndOfTest(FuncTestStage_t stage, FuncTestErrorCode_t errorCode)
{
  uint8_t buf[4];
  buf[0] = TANGAGE_CTRL_SETSUBMODE_SELFCONTROL;
  buf[1] = (uint8_t)stage;
  buf[2] = (uint8_t)(errorCode>>8);
  buf[3] = (uint8_t)errorCode;
  return tangageCtrl_send(buf, sizeof(buf));
}


// Начать тест шины CAN
void canBusTest_start()
{
  canBusTest_enable = 1;
  canBusTest_value = 0;
  canBusTest_errorCount = 0;
  canBusTestStartTime = system_time;
    
  uint8_t buf[2];
  buf[0] = TANGAGE_CTRL_CAN_BUS_TEST;
  buf[1] = 1;
  tangageCtrl_send(buf, sizeof(buf));
  
  ////canMonitor_sendString("CANTEST started");
  canMonitor_printf("CANTEST started");
}

// Завершить тест шины CAN
void canBusTest_stop()
{
  canBusTest_enable = 0;
  ////canMonitor_sendString("CANTEST stopped. Frames: %d, Errors: %d", canBusTest_value, canBusTest_errorCount);
  canMonitor_printf("CANTEST stopped. Frames: %d, Errors: %d", canBusTest_value, canBusTest_errorCount);
  
  uint8_t buf[2];
  buf[0] = TANGAGE_CTRL_CAN_BUS_TEST;
  buf[1] = 0;
  tangageCtrl_send(buf, sizeof(buf));
}

// Тест шины CAN
void canBusTest_sendNext()
{
  //static uint32_t numOfCall = 0;
  static uint32_t lastReportTime = 0;
  
  if(!canBusTest_enable) return;
  if(system_time - canBusTestStartTime < 200) return;
  //if((++numOfCall % 2) != 0) return;
    
  uint8_t buf[5];
  buf[0] = TANGAGE_CTRL_CAN_BUS_TEST;
  buf[1] = (uint8_t)(canBusTest_value>>24);
  buf[2] = (uint8_t)(canBusTest_value>>16);
  buf[3] = (uint8_t)(canBusTest_value>>8);
  buf[4] = (uint8_t)(canBusTest_value>>0);  
  canBusTest_value++;  
  uint8_t ok = tangageCtrl_send(buf, sizeof(buf));

  if(!ok)
  {
    canBusTest_errorCount++;
    
    if(system_time - lastReportTime < 1000) return;
    lastReportTime = system_time;
    ////canMonitor_sendString("CANTEST ERR: frame not sent");
    canMonitor_printf("CANTEST ERR: frame not sent");
  }
}