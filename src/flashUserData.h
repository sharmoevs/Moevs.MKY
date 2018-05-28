#ifndef __FLASH_USER_DATA_H__
#define __FLASH_USER_DATA_H__

#include "MDR1986VE1T.h"
#include "flash1986ve1t.h"


#define USER_DATA_BASE_FLASH_ADDR       GET_FLASH_PAGE_ADDR(30)                 // базовый адрес во флешь-памяти для пользовательских настроек
#define PID_KOEF_P_ADDR                 (USER_DATA_BASE_FLASH_ADDR + 4)         // значениe коэффициента P PID-регулятора
#define PID_KOEF_I_ADDR                 (USER_DATA_BASE_FLASH_ADDR + 8)         // значениe коэффициента I PID-регулятора
#define PID_KOEF_D_ADDR                 (USER_DATA_BASE_FLASH_ADDR + 12)        // значениe коэффициента D PID-регулятора
#define PID_TDISKR_ADDR                 (USER_DATA_BASE_FLASH_ADDR + 16)        // значениe времени дискретизации
#define DUS_KORR_KOER_ADDR              (USER_DATA_BASE_FLASH_ADDR + 20)        // калибровочный коэф. ДУС
#define PID_KOEF_N_ADDR                 (USER_DATA_BASE_FLASH_ADDR + 28)        // значениe коэффициента N PID-регулятора
#define HARDWARE_ZERO_ANGLE_COURSE_ADDR (USER_DATA_BASE_FLASH_ADDR + 32)
#define HARDWARE_ZERO_ANGLE_TANGAGE_ADDR (USER_DATA_BASE_FLASH_ADDR + 36)
#define ENGINE_ZERO_ANGLE_ADDR          (USER_DATA_BASE_FLASH_ADDR + 40)
#define LOGICAL_ZERO_ANGLE_COURSE_ADDR  (USER_DATA_BASE_FLASH_ADDR + 44)        // ноль логической шкалы курсового контура
#define LOGICAL_ZERO_ANGLE_TANGAGE_ADDR (USER_DATA_BASE_FLASH_ADDR + 48)        // ноль логической шкалы тангажного контура

#define DUS_TEMP_CALIB_KOEF_START_ADDR  (USER_DATA_BASE_FLASH_ADDR + 200)       // температурная поправка к ДУС

void readUserDataFromFlash();
void saveUserDataInFlash();


#endif //__FLASH_USER_DATA_H__