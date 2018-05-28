#ifndef __TANGAGE_CONTROL_H__
#define __TANGAGE_CONTROL_H__


#include "MDR1986VE1T.h"
#include "gkControlMode.h"

#define TANGAGE_ANGLE_RX_INTERVAL      50       // обновлять значение тангажного угла каждные X мс

#define MAX_TANGAGE_SYSANGLE_CODE               0x3FFFF  // максимальное значение угла тангажного контура (18 бит)
#define TANGAGE_SYSANGLE_TO_FLOAT(code32)      (float)(code32*360.0/MAX_TANGAGE_SYSANGLE_CODE)
#define TANGAGE_USYSANGLE_TO_CODE(angleF)      (uint32_t)(angleF*MAX_TANGAGE_SYSANGLE_CODE/360.0)




// Id команд управления тангажным контуром
#define TANGAGE_CTRL_SETMODE_VUS                0x01          // установить режим ВУС
#define TANGAGE_CTRL_SETMODE_AR                 0x02          // установить режим АР
#define TANGAGE_CTRL_SETMODE_ENGINE_OFF         0x03          // останов
#define TANGAGE_CTRL_SETMODE_DISABLE_ENGINE     0x04          // запрет управлениями двигателями
#define TANGAGE_CTRL_SETMODE_TP                 0x05          // транспортное положениеы
#define TANGAGE_CTRL_SETMODE_VUO                0x06          // ВУО
#define TANGAGE_CTRL_SETSUBMODE_SELFCONTROL     0x07          // подрежим самоконтроля
#define TANGAGE_CTRL_ANGLE_CORRECTIONS          0x08          // аппаратная и логическая поправка к шкалам

#define TANGAGE_CTRL_CAN_BUS_TEST               0xDB          // тест шины CAN


// Стадии режима Самоконтроль
typedef enum 
{ 
  TANGAGE_SELFCONTROL_INIT         = 0x00,      // инициализация режима самоконтроля
  TANGAGE_SELFCONTROL_VUS          = 0x01,      // задать движение с заданной скоростью
  TANGAGE_SELFCONTROL_AR           = 0x02,      // задать необходимое положение
} SelfControlTangageSubmode_t;


void tangageCtrl_init();

void tangageCtrl_enableRxAngleViaCANIfNeeded();
void tangageCtrl_rxAngle(uint8_t *buf, uint8_t len);
void tangageCtrl_rxMsg(uint8_t *buf, uint8_t len);

uint8_t tangageCtrl_setModeVUS(int16_t velocity128);       // установить режим ВУС
uint8_t tangageCtrl_setModeAR(uint32_t angleU, int16_t dbgCmdAngle); // установить режим арретир
uint8_t tangageCtrl_setModeEngineOff();                    // выключить двигатели
uint8_t tangageCtrl_setModeDisableEngine();                // запрет управлением двигателями
uint8_t tangageCtrl_setModeTP(uint32_t angleU);            // транспортное положение
uint8_t tangageCtrl_setModeVUO(uint32_t angleU, int16_t dbgCmdAngle, int16_t velocity128);     // ВУО          
uint8_t tangageCtrl_sendAngleCorretion(uint32_t hardwareCorrection, uint32_t logicalCorrection);    // аппаратная и логическая поправка к шкалам

uint8_t tangageCtrl_setModeSelfControl();                    // самоконтроль
uint8_t tangageCtrl_setSelfControlSubmodeVUS(int16_t velocity32);
uint8_t tangageCtrl_setSelfControlSubmodeAR(uint32_t angleU, int16_t angle128);
uint8_t tangageCtrl_setSelfControlSubmodeEndOfTest(FuncTestStage_t stage, FuncTestErrorCode_t errorCode);

// Тест шины CAN
void canBusTest_start();
void canBusTest_stop();
void canBusTest_sendNext();

#endif //__TANGAGE_CONTROL_H__