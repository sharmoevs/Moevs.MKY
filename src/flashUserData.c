#include "flashUserData.h"
#include "global.h"
#include "angelSensor.h"
#include "dusCalibration.h"

/*

  Карта памяти
         _______________
        |               | 0x0001_0FFC
        |               |                   Область самозагрузчика              page #31
        |_______________| 0x0001_F000
        |               | 0x0001_EFFC
        |               |                   Область пользовательских настроек   page #30
        |_______________| 0x0001_E000
        |               |
        |               |                                                       page #29
        |_______________| 0x0001_D000
        |               |
        |               |                                                       page #28
        |_______________|
        |               |
        |               |                                                       page #27
        |_______________|
        |               |
        |               |
        |_______________|
        |               | 0x0000_0FFC
        |               |
        |_______________| 0x0000_0000

*/


extern float koef_P;
extern float koef_I;
extern float koef_D;
extern float koef_N;
extern float tDiskr;            // период дескритизации
extern double dusCalibrationKoef;

extern uint32_t g_engineZeroAngleCorrection;
extern uint32_t g_hardwareZeroAngleCorrection;          // ноль аппаратной шкалы курсового контура
extern uint32_t g_tangageHardwareZeroAngleCorrection;   // ноль аппаратной шкалы тангажного контура
extern uint32_t g_logicalZeroAngleCorrection;           // ноль логической шкалы курсового контура
extern uint32_t g_tangageLogicalZeroAngleCorrection;    // ноль логической шкалы тангажного контура

extern DusTempCorrection_t dusDefaultTemperatureCorrections[DUS_TEMPCALIB_RANGES_COUNT];
extern DusTempCorrection_t dusTemperatureCorrections[DUS_TEMPCALIB_RANGES_COUNT];

#define DUS_KORR_KOEF_DEFAULT   0.72392910
#define KOEF_P_DEFAULT          1
#define KOEF_I_DEFAULT          0
#define KOEF_D_DEFAULT          0
#define KOEF_N_DEFAULT          0
#define TDISKR_DEFAULT          0.0005
#define ENGINE_ZERO_CORRECTION_DEFAULT  0
#define COURSE_HARDWARE_ZERO_ANGLE_CORRECTION_DEFAULT  0
#define COURSE_LOGICAL_ZERO_ANGLE_CORRECTION_DEFAULT   0
#define TANGAGE_HARDWARE_ZERO_ANGLE_CORRECTION_DEFAULT  0
#define TANGAGE_LOGICAL_ZERO_ANGLE_CORRECTION_DEFAULT   0


// Прочитать параметры из памяти
void readUserDataFromFlash()
{
   koef_P = *((float*)(PID_KOEF_P_ADDR));
   if(koef_P != koef_P) koef_P = KOEF_P_DEFAULT;
   
   koef_I = *((float*)(PID_KOEF_I_ADDR));
   if(koef_I != koef_I) koef_I = KOEF_I_DEFAULT;
   
   koef_D = *((float*)(PID_KOEF_D_ADDR));
   if(koef_D != koef_D) koef_D = KOEF_D_DEFAULT;
   koef_D = 10;
   
   koef_N = *((float*)(PID_KOEF_N_ADDR));
   if(koef_N != koef_N) koef_N = KOEF_N_DEFAULT;
   
   tDiskr = *((float*)(PID_TDISKR_ADDR));
   if(tDiskr != tDiskr) tDiskr = TDISKR_DEFAULT;
   
   dusCalibrationKoef = *((double*)DUS_KORR_KOER_ADDR);
   if(dusCalibrationKoef != dusCalibrationKoef) dusCalibrationKoef = DUS_KORR_KOEF_DEFAULT;
   
   
   g_engineZeroAngleCorrection = *((uint32_t*)ENGINE_ZERO_ANGLE_ADDR);
   if(g_engineZeroAngleCorrection == 0xFFFFFFFF) g_engineZeroAngleCorrection = ENGINE_ZERO_CORRECTION_DEFAULT;
      
   g_hardwareZeroAngleCorrection = *((uint32_t*)HARDWARE_ZERO_ANGLE_COURSE_ADDR);
   if(g_hardwareZeroAngleCorrection == 0xFFFFFFFF) g_hardwareZeroAngleCorrection = COURSE_HARDWARE_ZERO_ANGLE_CORRECTION_DEFAULT;
   
   g_tangageHardwareZeroAngleCorrection = *((uint32_t*)HARDWARE_ZERO_ANGLE_TANGAGE_ADDR);  
   if(g_tangageHardwareZeroAngleCorrection == 0xFFFFFFFF) g_tangageHardwareZeroAngleCorrection = TANGAGE_HARDWARE_ZERO_ANGLE_CORRECTION_DEFAULT;
   
   g_logicalZeroAngleCorrection = *((uint32_t*)LOGICAL_ZERO_ANGLE_COURSE_ADDR);
   if(g_logicalZeroAngleCorrection == 0xFFFFFFFF) g_logicalZeroAngleCorrection = COURSE_LOGICAL_ZERO_ANGLE_CORRECTION_DEFAULT;
   
   g_tangageLogicalZeroAngleCorrection = *((uint32_t*)LOGICAL_ZERO_ANGLE_TANGAGE_ADDR);
   if(g_tangageLogicalZeroAngleCorrection == 0xFFFFFFFF) g_tangageLogicalZeroAngleCorrection = TANGAGE_LOGICAL_ZERO_ANGLE_CORRECTION_DEFAULT;
   
   // Температурная поправка к ДУС
   for(int i=0; i<DUS_TEMPCALIB_RANGES_COUNT; i++)
   {
     uint32_t value = *((uint32_t*)(DUS_TEMP_CALIB_KOEF_START_ADDR + i*4));
     dusDefaultTemperatureCorrections[i].correction = (int16_t)value;
     dusDefaultTemperatureCorrections[i].available = (value != 0xFFFFFFFF);
     dusDefaultTemperatureCorrections[i].time = 0;
     
     dusTemperatureCorrections[i].available = 0;
     dusTemperatureCorrections[i].time = 0;
   }
   
   
   // отладка
 
   //g_hardwareZeroAngleCorrection = 0;          // ноль аппаратной шкалы курсового контура
   //g_tangageHardwareZeroAngleCorrection = 0;   // ноль аппаратной шкалы тангажного контура
   //g_logicalZeroAngleCorrection = 0;           // ноль логической шкалы курсового контура
   //g_tangageLogicalZeroAngleCorrection = 0;    // ноль логической шкалы тангажного контура
  
}

// Сохранить пользовательские данные во флеши
void saveUserDataInFlash()
{
   uint32_t tmp;
   uint8_t *p;

    __disable_irq();
    erase_flash_page(USER_DATA_BASE_FLASH_ADDR);
    
    // P-коэф.
    p = (uint8_t*)(&koef_P);
    tmp = p[3]<<24 | p[2]<<16 | p[1]<<8 | p[0];
    flash_write_word(PID_KOEF_P_ADDR, tmp);
    
    // I-коэф.
    p = (uint8_t*)(&koef_I);
    tmp = p[3]<<24 | p[2]<<16 | p[1]<<8 | p[0];
    flash_write_word(PID_KOEF_I_ADDR, tmp);
    
    // D-коэф.
    p = (uint8_t*)(&koef_D);
    tmp = p[3]<<24 | p[2]<<16 | p[1]<<8 | p[0];
    flash_write_word(PID_KOEF_D_ADDR, tmp);
    
    // N-коэф.
    p = (uint8_t*)(&koef_N);
    tmp = p[3]<<24 | p[2]<<16 | p[1]<<8 | p[0];
    flash_write_word(PID_KOEF_N_ADDR, tmp);
    
    // Интервал дискретизации.
    p = (uint8_t*)(&tDiskr);
    tmp = p[3]<<24 | p[2]<<16 | p[1]<<8 | p[0];
    flash_write_word(PID_TDISKR_ADDR, tmp);        
        
    // Калибровочный коэф ДУС
    p = (uint8_t*)(&dusCalibrationKoef);
    tmp = p[3]<<24 | p[2]<<16 | p[1]<<8 | p[0];
    flash_write_word(DUS_KORR_KOER_ADDR, tmp);   
    tmp = p[7]<<24 | p[6]<<16 | p[5]<<8 | p[4];
    flash_write_word(DUS_KORR_KOER_ADDR+4, tmp);
    
    // Ноль двигателей
    flash_write_word(ENGINE_ZERO_ANGLE_ADDR, g_engineZeroAngleCorrection);
    
    // Ноль аппаратной шкалы угла курсового контура
    flash_write_word(HARDWARE_ZERO_ANGLE_COURSE_ADDR, g_hardwareZeroAngleCorrection);
    
    // Ноль аппаратной шкалы угла тангажного контура
    flash_write_word(HARDWARE_ZERO_ANGLE_TANGAGE_ADDR, g_tangageHardwareZeroAngleCorrection);
    
    // Ноль логической шкалы угла курсового контура
    flash_write_word(LOGICAL_ZERO_ANGLE_COURSE_ADDR, g_logicalZeroAngleCorrection);
    
    // Ноль логической шкалы угла тангажного контура
    flash_write_word(LOGICAL_ZERO_ANGLE_TANGAGE_ADDR, g_tangageLogicalZeroAngleCorrection);
        
    // Температурная поправка к ДУС
    for(int i=0; i<DUS_TEMPCALIB_RANGES_COUNT; i++)
    {
      tmp = dusDefaultTemperatureCorrections[i].correction;
      if(dusDefaultTemperatureCorrections[i].available) tmp &= 0xFFFF;
      flash_write_word(DUS_TEMP_CALIB_KOEF_START_ADDR + i*4, tmp);
    }
    
    __enable_irq();
}