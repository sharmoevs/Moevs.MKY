#ifndef __DUS_CALIBRATION_H__
#define __DUS_CALIBRATION_H__

#include "MDR1986VE1T.h"


#define DUS_TEMPCALIB_MIN_TEMP             (-60)                                // наименьшая температура
#define DUS_TEMPCALIB_MAX_TEMP             (60)                                 // наибольшая температура
#define DUS_TEMPCALIB_TEMPERATURE_STEP     4                                    // ширина температурного калибровочного диапазона
#define DUS_TEMPCALIB_RANGES_COUNT         ((DUS_TEMPCALIB_MAX_TEMP - DUS_TEMPCALIB_MIN_TEMP)/DUS_TEMPCALIB_TEMPERATURE_STEP)+2 // кол-во диапазонов включая [-∞:min] и [max:+∞]
#define DUS_TEMPCALIB_AVER_TEMP_OFFSET     1                                    // задает ширину области калибровки

#define GET_LOW_TEMPERATURE(rangeIndex)    (DUS_TEMPCALIB_MIN_TEMP + (rangeIndex-1)*DUS_TEMPCALIB_TEMPERATURE_STEP)     // минимальная температура заданного диапазона
#define GET_HIGH_TEMPERATURE(rangeIndex)   (DUS_TEMPCALIB_MIN_TEMP + (rangeIndex)*DUS_TEMPCALIB_TEMPERATURE_STEP)       // максимальная температура заданного диапазона

#define DUS_TEMPCALIB_CALIBRATION_INTERVAL 30*1000*60                       // через какое время можно будет повторно сохранить калибровочный коэф. в эту ячейку



// Стадия калибровки ДУС в арретире
typedef enum
{
  AR_CALIB_STATE__GRAB_ANGLE    = 0x00,     // захватить текущий угол
  AR_CALIB_STATE__CHECK_ANGLE   = 0x01,     // проверить, что не ушли с захваченного угла
  AR_CALIB_STATE__ACCUM_SAMPLES = 0x02,     // накопить отсчеты
  AR_CALIB_STATE__WAIT_NEXT_CALIB = 0x03    // ожидание следующей калибровки
} ArCalibrationState_t;

// Калибровка ДУС в арретире
typedef struct
{
  ArCalibrationState_t  stage;          // состояние калибровки
  uint32_t              repereAngle;    // угол, который мы захватываем в качестве калибровочного
  uint8_t               numOfAr;        // сколько раз удержались в захваченном угле
  uint32_t              startOperationTime;     // время начала операции
  int32_t               dusAccumulator; // аккумулятор для отсчетов
  uint16_t              totalSamplesCount;   // количество отсчетов калибровки
  uint16_t              iSample;        // номер отсчета
  int16_t               dusCorrection;  // рассчитанная поправка к ДУС
  uint32_t              calibrationInterval;    // через какое время повторить
  uint8_t               done;           // калибровка завершена
}ArCalibration_t;


// Структура калибровочного коэффициента ДУС по температуре
typedef struct
{
  uint32_t time;         // время калибровки
  int16_t  correction;   // значение корректировки
  uint8_t  available;    // идентификатор наличия калибровки для данного диапазона
}DusTempCorrection_t;

// Тип поправки к ДУС
typedef enum
{
  DUS_CALIB_COEF__NONE  = 0x00,
  DUS_CALIB_COEF__RAM   = 0x01,
  DUS_CALIB_COEF__FLASH = 0x02
} DusCalibrationCoefDestination_t;



void dus_calibrate();
void dusCalibration_init(uint16_t totalSamplesCount, uint32_t calibrationInterval);






#endif //__DUS_CALIBRATION_H__