#ifndef __ARINC__H__
#define __ARINC__H__

#include "MDR1986VE1T.h"


#define ARINC_SYS_FREQ_PRESCALER        (CPU_CLOCK_MHZ/1000000)
#if ARINC_SYS_FREQ_PRESCALER > 127
  #error "Предделитель системной частоты для ARINC должен быть меньше 127"
#endif


#define ARINC_CH1       1
#define ARINC_CH2       2
#define ARINC_CH3       3
#define ARINC_CH4       4
#define ARINC_CH5       5
#define ARINC_CH6       6
#define ARINC_CH7       7
#define ARINC_CH8       8



// Выбрать канал, доступ к ФИФО которого будет осуществляться
#define ARINC_SET_CHANNEL(channel)               (MDR_ARINC429R->CHANNEL = channel-1)
// Проверить наличие данных в приемном ФИФО канала channel
#define ARINC_RX_BUF_NOT_EMPTY(channel)          (MDR_ARINC429R->STATUS1 & (1<<(channel-1)))
// Проверить наличие ошибок в канале channel
#define ARINC_RX_CHANNEL_ERROR(channel)          (MDR_ARINC429R->STATUS1 & (1<<(13+channel)))
// Сбросить приемный канал channel
#define ARINC_RX_CHANNEL_RESET(channel)           MDR_ARINC429R->CONTROL1 &= ~(1<<(channel-1)); \
   


void arinc_init();
void arincSendArray(uint32_t *pwords, uint8_t cnt);



#endif //__ARINC__H__