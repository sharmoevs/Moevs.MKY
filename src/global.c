#include "global.h"

volatile uint32_t system_time = 0;

/*
Отладочные команды по CAN_MONITOR:

ОБЩЕЕ
  reset course mk       - сброс контроллера

CAN TEST
  cantest start         - запустить тест
  cantest stop          - остановить тест
  cantest statistic     - статистика CAN

ДУС (цифровой и дифференцированный угол)
  get dus correction    - запросить поправку к аналоговому ДУС
  set dus correction X  - устаноивть новую поправку к аналоговому ДУС
  set dus enable Х      - вкл/выкл аналоговый ДУС
  set dus filter enable  Х - фильтр на цифровой ДУС

СБОР ДАННЫХ 
  save samples          - сохранить отсчеты скорости и угла
  get samples dus       - считать сохраненные отсчеты ДУС
  get samples angle     - считать сохраненные отсчеты датчика угла


*/


// Возвращает колчисечтво одиничных бит в слове
uint32_t getBitsInWord(uint32_t x)
{
   x = (x & 0x55555555) + ((x>>1) & 0x55555555);
   x = (x & 0x33333333) + ((x>>2) & 0x33333333);
   x = (x & 0x0F0F0F0F) + ((x>>4) & 0x0F0F0F0F);
   x = (x & 0x00FF00FF) + ((x>>8) & 0x00FF00FF);
   x = (x & 0x0000FFFF) + ((x>>16) & 0x0000FFFF);
   return x;
}
/*
// Отключает прерывания, возвращает маску разрешенных прерываний.
uint32_t __DISABLE_IRQ()
{
  uint32_t iser = NVIC->ISER;
  NVIC->ICER = 0xFFFFFFFF;
  return iser;
}

void __ENABLE_IRQ(uint32_t iser)
{
  NVIC->ISER = iser;
}
*/