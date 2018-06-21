#ifndef __SETTINGS_H__
#define __SETTINGS_H__


#ifdef MOEVS_BOARD // ПЛАТА МОЭВС 
  #define CPU_CLOCK_MHZ           120000000                                     // частота CPU в МГц
  #define EXT_GENERATOR_FREQ      8000000                                       // частота внешнего генератора 16/2 = 8МГц
#else // ОТЛАДОЧНАЯ ПЛАТА
  #define CPU_CLOCK_MHZ           125000000                                     // частота CPU в МГц
  #define EXT_GENERATOR_FREQ      12500000                                      // частота внешнего генератора 25/2 = 12.5
#endif //MOEVS_BOARD
#define CPU_PLL_KOEF              ((CPU_CLOCK_MHZ/EXT_GENERATOR_FREQ)-1)        // коэф. умножения PLL

// Частота ШИМ
#define TIM_PWM_CLK                     30000000        // частота тактирования таймера ШИМ
#define PWM_FREQ                        15000           // частота ШИМ
#define TIM_PWM_ARR_REG                 ((TIM_PWM_CLK/PWM_FREQ)-1)
#define TIM_PWM_GET_CCR3(percentage)    (uint16_t)((percentage*TIM_PWM_ARR_REG)/100)
#define TIM_PWM_60_PERCENTAGE_CCR3_REG  TIM_PWM_GET_CCR3(60)
//#define TIM_PWM_60_PERCENTAGE_CCR3_REG  (uint16_t)((60*TIM_PWM_ARR_REG)/100)

// Идентификатор и имя загрузчика
#define SELF_LOADER_DEFAULT_NAME        ("MK Y")
#define SELFLOADER_DEFAULT_ID           0x80

// Защита по скорости
#define ENABLE_VELOCITY_PROTECTION                      // ВКЛЮЧИТЬ ЗАЩИТУ ПО СКОРОСТИ
#define AVERAGE_VELOCITY_INTERVAL       100             // период усреднения
#define VELOCITY_PROTECTION_THRESHOLD   50.0

// Частота дискретизации ДУС
#define DUS_SAMPLING_FREQUENCY          2000                                    // частота дискеретизации ДУС
#define DUS_SAMPLING_TIMER_TIMEOUT_mcs  (1000000/DUS_SAMPLING_FREQUENCY)        // период таймера ДУС, мкс
#define DUS_TIMER_COUNT_PER_ms          (1000/DUS_SAMPLING_TIMER_TIMEOUT_mcs)   // кол-во отсчетов таймера ДУС за миллисекунду

// Установка в Арретир
#define ARRETIR_VELOCITY                35.0                                    // скорость переброса в режиме АР
#define AR_THRESHOLD                    USYSANGLE_TO_CODE(2)                    // ограничение рассогласования по углу в схеме управления!
#define COURSE_ARRETIERED_DELTA_FI      (USYSANGLE_TO_CODE(0.4))                // разница между текущим углом и требуемым, при которой считается, что ГС находится в арретире
#define TANGAGE_ARRETIERED_DELTA_FI     (TANGAGE_USYSANGLE_TO_CODE(0.4))

// Калибровка ДУС
#define MAX_START_UP_CALIBRATION_TIME   60000                                   // максимальное время калибровки

#define ENGINE_STARTUP_DELAY            2000                                    // задержка включения двигателей после подачи питания
#define DUS_STARTUP_CALIBRATION_SAMPLES 5000                                    // количество отсчетов для усреднения ДУС
#define ARRETIER_CALIB_MAX_DEVIATION    USYSANGLE_TO_CODE(10.0/128.0)           // максимальное отклонение от арретира при калибровке в арретире
#define ARRETIER_CALIBRATION_SAMPLES    2000                                    // количество отсчетов для усреднения ДУС при калибровке в арретире
#define TEMPERATURE_CALIBRATION_SAMPLES 5000                                    // количество отсчетов для усреднения ДУС при калибровке по температуре


// Максимальная скорость вращения контуров
#define MAX_ABS_COURSE_VELOCITY128      (40*128)
#define MAX_ABS_TANGAGE_VELOCITY128     (40*128)


// Разрешить управление тангажным контуром
#define ENABLE_TANGAGE_CONTROL

// Использовать CAN с очередью и динамической памятью
#define USE_DYNAMIC_CAN_QUEUE



// Самоконтроль
#define SELFCONTROL_COURSE_CIRCLE_VELOCITY      25      // скорость прохождения круга, град/сек
#define SELFCONTROL_TANGAGE_CIRCLE_VELOCITY     25      // скорость оборота тангажного контура
#define SELFCONTROL_MAX_VELOCITY_ERROR          2       // максимальная ошибка скорости
#define SELFCONTROL_MIN_VELOCITY                (MIN(SELFCONTROL_COURSE_CIRCLE_VELOCITY, SELFCONTROL_TANGAGE_CIRCLE_VELOCITY)) // скорости самого медленного контура
#define SELFCONTROL_MAX_CIRCLE_TIME             (360.0/(SELFCONTROL_MIN_VELOCITY-SELFCONTROL_MAX_VELOCITY_ERROR)*1000)        // максимально возможное время оборота самого медленного контура

#define SELFCONTROL_COURSE_START_POSITION       (SELFCONTROL_COURSE_ANGLE_4)
#define SELFCONTROL_TANGAGE_START_POSITION      (SELFCONTROL_TANGAGE_ANGLE_4)

#define SELFCONTROL_COURSE_ANGLE_1              (-90*128)
#define SELFCONTROL_TANGAGE_ANGLE_1             (-90*128)

#define SELFCONTROL_COURSE_ANGLE_2              (-180*128)
#define SELFCONTROL_TANGAGE_ANGLE_2             (-180*128)

#define SELFCONTROL_COURSE_ANGLE_3              (90*128)
#define SELFCONTROL_TANGAGE_ANGLE_3             (90*128)

#define SELFCONTROL_COURSE_ANGLE_4              (0*128)
#define SELFCONTROL_TANGAGE_ANGLE_4             (0*128)




// Фильтр ДУС
#define DUS_FILTER_NONE
//#define DUS_FILTER_F4kHz_B100Hz
//#define DUS_FILTER_F4kHz_B200Hz
//#define DUS_FILTER_F4kHz_B300Hz
//#define DUS_FILTER_F4kHz_B500Hz
//#define DUS_FILTER_F2kHz_B200Hz_500Hz_20dB_80dB
//#define DUS_FILTER_F2kHz_B200Hz_SPECIFY_ORDER_2
//#define DUS_FILTER_F2kHz_B200Hz_500Hz_1dB_60dB                  // *
//#define DUS_FILTER_F2kHz_B500Hz_700Hz_1dB_60dB
//#define DUS_PID_FILTER_F2kHz_B200Hz_700Hz_2dB_26dB


// Фильтр ПИД
//#define PID_FILTER_NONE
//#define PID_FILTER_F2kHz_B200Hz_500Hz_20dB_80dB
//#define PID_FILTER_F2kHz_B200Hz_SPECIFY_ORDER_2
#define PID_FILTER_F2kHz_B200Hz_500Hz_1dB_60dB                //*
//#define PID_FILTER_F2kHz_B500Hz_700Hz_1dB_60dB
//#define PID_FILTER_F2kHz_B200Hz_700Hz_2dB_26dB







#if (DUS_SAMPLING_FREQUENCY != 2000) && (DUS_SAMPLING_FREQUENCY != 4000)
  #error "Incorrect DUS_SAMPLING_FREQUENCY value" 
#endif










#endif //__SETTINGS_H__