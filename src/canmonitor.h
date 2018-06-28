#ifndef __CANMONITOR_H__
#define __CANMONITOR_H__

#include "MDR1986VE1T.h"
#include "canmonitorText.h"
#include "can.h"

#define CANMONITOR_QUEUE_CAPACITY               50


// Идентификаторы команд
#define CANMONITOR_STATE_REQUEST                0x01    // запрос состояния
#define CANMONITOR_EXTENDED_CMD                 0xF0    // расширенная команда
#define CANMONITOR_PID_KOEF_REQUEST             0x02    // запрос коэффициентов ПИД-регулятора
#define CANMONITOR_PID_KOEF_SET_P               0x03
#define CANMONITOR_PID_KOEF_SET_I               0x04
#define CANMONITOR_PID_KOEF_SET_D               0x05
#define CANMONITOR_PID_KOEF_SET_N               0x51
#define CANMONITOR_SET_T_DISKR                  0x06
#define CANMONITOR_SAVE_PID_KOEF_IN_FLASH       0x07
#define CANMONITOR_START_VELOCITY_CALIBRATION   0x08
#define CANMONITOR_SET_MODE_TUDA_SUDA_VUS       0x09
#define CANMONITOR_SET_OUTPUT_DEBUGING_INFO     0x0A
#define CANMONITOR_ENGINE_SWITCH_OFF            0x0B    // выключить двигатели
#define CANMONITOR_SAVE_NEW_HARDWARE_SCALE_COURSE   0x0C    // сохранить новый ноль аппаратаной шкалы курсового контура
#define CANMONITOR_SAVE_NEW_HARDWARE_SCALE_TANGAGE  0x0D    // сохранить новый ноль аппаратаной шкалы тангажного контура
#define CANMONITOR_HARDWARE_SCALE_OFFSETS_REQUEST   0x0E    // запрос смещений аппаратной шкалы курсового и тангажного  контура относительно железной
#define CANMONITOR_GS_ENGINE_OFF                0x10    // команда для обоих контуров
#define CANMONITOR_GS_SET_GOT                   0x11    // установить флаг готовность приема команды. Для отладки
#define CANMONITOR_SET_MODE_TUDA_SUDA_AR        0x0F 
#define CANMONITOR_SET_MODE_DUS_TEMP_CALIBRATION 0x12

#define CANMONITOR_TX_VALUE_1                   0x20
#define CANMONITOR_TX_VALUE_2                   0x21
#define CANMONITOR_TX_VALUE_3                   0x22
#define CANMONITOR_TX_VALUE_4                   0x23
#define CANMONITOR_TX_VALUE_5                   0x24
#define CANMONITOR_TX_VALUE_6                   0x25



// Служебные команды
#define CANMONITOR_DEBUG                        0xDB    // отладочная команда
#define CANMONITOR_EXT_CMD_STRING_ID            0xBB    // идентификатор того, что  данные в расширенной команде нужно представлять как строку



// Идентификаторы ответов
#define CANMONITOR_STATE                        0x0E    // слово состояния
#define CANMONITOR_WORKING_TIME_MS              0x50    // время в работы в мс от подачи питания
#define CANMONITOR_EXTENDED_ANSWER              0xF0    // расширенная команда
#define CANMONITOR_COURSE_VELOCITY              0x51    // угловая скорость
#define CANMONITOR_COURSE_UPR                   0x52    // управляющее воздействие
#define CANMONITOR_COURSE_GET_P                 0x53    // P коэф.
#define CANMONITOR_COURSE_GET_I                 0x54    // I коэф.
#define CANMONITOR_COURSE_GET_D                 0x55    // D коэф.
#define CANMONITOR_COURSE_GET_N                 0x59    // N коэф.
#define CANMONITOR_COURSE_GET_T                 0x56    // T дискр.
#define CANMONITOR_SEND_CURRENT_ANGLE           0x0A    // Текущий угол
#define CANMONITOR_SEND_TEST_VALUE_1            0x0B
#define CANMONITOR_DBG_SEND_SAVED_VALUE         0x0C    // отправить сохраненные значения
#define CANMONITOR_COURSE_HARDWARE_SCALE_OFFSET  0x0D     // смещение аппаратной шкалы курсового контура относительно железной
#define CANMONITOR_TANGAGE_HARDWARE_SCALE_OFFSET 0x0F     // смещение аппаратной шкалы тангажного контура относительно железной
#define CANMONITOR_GS_STATE                     0xE2    // слово состояния ГС
#define CANMONITOR_ANGLE_SAMPLE                 0x60    // отсчет датчика угла

// Расширенная команда
#define EXT_CMD_START                           0x00
#define EXT_CMD_CONT                            0x01
#define EXT_CMD_END                             0x02



// Текстовые команды для отладки и контроля
#define DBG_CMD___HELP                          ("help")                // список команд
#define DBG_CMD___PING                          ("ping")                // проверка связи
#define DBG_CMD___CAN_TEST_START                ("cantest start")       // запустить can-тест
#define DBG_CMD___CAN_TEST_STOP                 ("cantest stop")        // остановить can-тест
#define DBG_CMD___CAN_TEST_STATISTIC            ("cantest statistic")   // статистика can-теста
#define DBG_CMD___SET_USE_DUS_IN_ARRETIER       ("set use dus in arretier ")     // управление в арретире по ДУС
#define DBG_CMD___RESET_MK                      ("reset course mk")
#define DBG_CMD___GET_DUS_CORR                  ("get dus corr")        // прочитать текущую поправку к ДУС
#define DBG_CMD___SET_DUS_CORR                  ("set dus corr ")       // прочитать текущую поправку к ДУС
#define DBG_CMD___SET_DUS_CORR_ENABLE           ("set enable dus corr ")// прочитать текущую поправку к ДУС
#define DBG_CMD___UPDATE_TANGAGE_SCALES         ("update tangage scales")// отправка поправок к аппаратной и логической шкалам тангажного контура
#define DBG_CMD___GET_SCALES                    ("get scales")          // текущие поправки
#define DBG_CMD___SET_PREPARATION               ("set preparation")     // установить бит ГОТ
#define DBG_CMD___GET_CALIBRATION_COEF          ("get dus coef")        // коэффициенты калибровки ДУС по температуре
#define DBG_CMD___GET_CALIBRATION_COEF_DEFAULT  ("get dus coef default")   // !!!! реализуется по другому
#define DBG_CMD___GET_TEMP                      ("get temp")            // текущая температура
#define DBG_CMD___DYNAMIC_RAM_STATISTIC         ("dram")                // статистика использования памяти
#define DBG_CMD___INFINITE_FUNC_TEST_ENABLE     ("set infinite functest enable ")// бесконечный функциональный тест
#define DBG_CMD___SET_ENABLE_AR_CALIB_LOG       ("set enable calibration log ")                            // вкл/выкл логгирование калибровки в арретире





void canMonitor_init();

void canMonitor_rxFrameHandler(uint8_t *buf, uint8_t len);      // Принят пакет от монитора
void canMonitor_send(uint8_t *pData, uint8_t len);              // Отправить данные CAN-монитору


void canMonitor_rxExtCmd(uint8_t *pData, uint8_t len);          // принять расширенную команду
void canMonitor_sendExtHex(uint8_t *pBuf, uint8_t len);         // отправить ответ на расширенную команду
void canMonitor_printf(uint8_t *p, ...);                 // отправить строку



void canMonitor_sendStateWord();        // отправить слово состояния курсового контура
void canMonitor_fillBufferWithCurrentState(uint8_t *msg);
void canMonitor_sendWorkingTime();      // время наработки
void canMonitor_sendCourseVelocity();   // текущая угловая скорость
void canMonitor_sendCourseUpr();        // управляющее воздействие
void canMonitor_sendPIDKoef();          // отправить текущие значения коэф. ПИД-рег.
void canMonitor_sendAngleCode();
void canMonitor_sendTestValue1(float value);
void canMonitor_setModeTudaSudaVUS(uint8_t *buf, uint8_t len);
void canMonitor_setModeTudaSudaAR(uint8_t *buf, uint8_t len);
void canMonitor_setOutputDebugingInfo(uint8_t *buf, uint8_t len);
void canMonitor_sendDbgSavedValue(float value);
void canMonitor_saveNewHardwareScaleCourse();
void canMonitor_saveNewHardwareScaleTangage();
void canMonitor_sendHardwareScaleOffsets();
void canMonitor_GsSendState();          // отправить слово состояния ГС




void canMonitor_sendAngle(float value);
void canMonitor_sendSpeed(float value);
void canMonitor_sendUpr(float value);
void canMonitor_sendValue1(float value);
void canMonitor_sendValue2(float value);


#endif //__CANMONITOR_H__
