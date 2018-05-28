#include "adc.h"
#include "MDR1986VE1T.h"
#include "global.h"
#include "canterminal.h"
#include "timers.h"

uint8_t adcConvertDone = 1;     // завершен цикл измерений АЦП
uint32_t adcAccum = 0;          // аккумулирование результата для усреднения
uint8_t adcMeasureCounter = 0;  // счетчик преобразований
uint8_t adcCurrentChannel;      // номер канала АЦП
uint16_t adcValues[8];          // усредненные значения измерения АЦП


// Поправки
int16_t vRefCodeAmmend;         // поправка к АЦП
//int16_t dusCodeAmmend;          // попкравка к АЦП ДУС
float vRefAmmend;               // поправка к показаниям АЦП
float uprVolts;                 // опорное напряжение АЦП

// Скорректированные показания АЦП
//uint16_t adcDusCode;            // показания с АЦП ДУС


void test();

// Инициализация АЦП
void adc_init()
{
   MDR_RST_CLK->PER_CLOCK |= 1<<17;         // тактирование АЦП
   MDR_ADC->ADC1_CFG = 0xF<<28 |            // задержка 15 тактов CLK между началом преобразования ADC1
                       0x7<<25 |            // задержка 7 тактов CLK перед началом следующего преобразования после завершения предыдущего при последовательном переборе каналов
                       0x4<<12 |            // коэф деления входной частоты
                       1<<0;                // включить АЦП
                         
   //MDR_ADC->ADC1_STATUS = 1<<4;             // прерывание по окончанию преобразования
   //NVIC_EnableIRQ(ADC_IRQn);                // разрешить прерывания
   
     
   //correctDusZeroVelocity();                // скорректировать ДУС по нулевой угловой скорости
  
   // test
   // while(1)
   // {
   //   test();
   // }
}


// Оцифровка заданного канала
uint16_t adc_start(uint8_t channel)
{
  ADC_MEASURE(channel);
  while(!(MDR_ADC->ADC1_STATUS & (1<<2)));
  return (uint16_t)(MDR_ADC->ADC1_RESULT);  
}
// Оцифровка заданного канала с усреднением
uint16_t adc_startAverage(uint8_t channel)
{
  uint32_t sum = 0;
  for(int i=0; i<ADC_AVERAGES_COUNT; i++)
  {
     sum += adc_start(channel);
  }
  uint16_t res = sum>>ADC_DIV_SHIFT;
  return res;
}

// Замер времени преобразования
void test()
{
   __START_STOPWATCH;  
   uint16_t val = adc_startAverage(ADC_CHANNEL_2);
   __STOP_STOPWATCH;
   
   canTerminal_printf("%d %d", __STOPWATCH_ELAPSED, val);   
   delay_ms(100);
}

// ДУС
uint16_t adc_getVelocityCode()
{
  uint16_t code = adc_start(ANGEL_VELOCITY_SENSOR_ADC_CHANNEL);
  return code;
}










// Корректировка опорного напряжения 3.3В
void _checkRef()
{
  uint16_t ref1Adc = adcValues[REFERENCE_VOLTAGE_1_ADC_CHANNEL];  
  vRefAmmend = (1.65 - (ref1Adc*1.65)/0x800);
  vRefCodeAmmend = 0x800 - ref1Adc;
}

// Обновить УПР
void _updateUpr()
{  
   uint16_t uprAdc = adcValues[7] + vRefCodeAmmend;
   uprVolts = (3.3*uprAdc)/0xFFF;
}






















