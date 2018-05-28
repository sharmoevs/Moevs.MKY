#ifndef __ADC_H__
#define __ADC_H__

#include "MDR1986VE1T.h"

#define ADC_AVERAGES_COUNT      64      // кол-во усреднений измерений АЦП


#if    ADC_AVERAGES_COUNT == 2
#define ADC_DIV_SHIFT 1                 // на сколько бит нужно сдвинуть результат, что бы усреднить
#elif  ADC_AVERAGES_COUNT == 4
#define ADC_DIV_SHIFT 2
#elif  ADC_AVERAGES_COUNT == 8
#define ADC_DIV_SHIFT 3
#elif  ADC_AVERAGES_COUNT == 16
#define ADC_DIV_SHIFT 4
#elif  ADC_AVERAGES_COUNT == 32
#define ADC_DIV_SHIFT 5
#elif  ADC_AVERAGES_COUNT == 64
#define ADC_DIV_SHIFT 6
#else
#error "Некорректное значение количества усреднений АЦП"
#endif


// Номера каналов АЦП
#define ADC_CHANNEL_0           0
#define ADC_CHANNEL_1           1
#define ADC_CHANNEL_2           2
#define ADC_CHANNEL_3           3
#define ADC_CHANNEL_4           4
#define ADC_CHANNEL_5           5
#define ADC_CHANNEL_6           6
#define ADC_CHANNEL_7           7 
#define ADC_CHANNEL_30          30 
#define ADC_CHANNEL_31          31 

#define ADC_MEASURE(ch)         (MDR_ADC->ADC1_CFG = 0xF<<28|0x7<<25|0x4<<12|ch<<4|1<<1|1<<0)

//#define ADC_REF_VOLTAGE         3.3                                             // опорное напряжение АЦП
//#define ADC_CODE_TO_VOLTS(code) ((ADC_REF_VOLTAGE*code)/0xFFF)                  // перевод кода АЦП в напряжение




#define ANGEL_VELOCITY_SENSOR_ADC_CHANNEL               ADC_CHANNEL_5           // номер канала АЦП c датчиком угловых скоростей
#define REFERENCE_VOLTAGE_1_ADC_CHANNEL                 ADC_CHANNEL_2
#define REFERENCE_VOLTAGE_2_ADC_CHANNEL                 ADC_CHANNEL_3





void adc_init();                                // инициализация АЦП
void adc_startMeasure();                        // произвести измерение по выбранному каналу
//void correctDusZeroVelocity();                  // скорректировать датчик угловой скорости по нулевой
void adcService();                              // сервер АЦП


void _checkRef();
//void _updateCourseVelocity();
void _updateUpr();







uint16_t adc_start(uint8_t channel);
uint16_t adc_startAverage(uint8_t channel);

uint16_t adc_getVelocityCode();                  // значение АЦП с ДУС

//float calculateCourseVelocity(uint16_t adcCode); // рассчитать угловую скорость по показаниям АЦП




#endif //__ADC_H__