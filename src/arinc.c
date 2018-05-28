#include "arinc.h"
#include "settings.h"
#include "arincProtocol.h"

// Инициализация ARINC
void arinc_init()
{ 
   MDR_RST_CLK->PER_CLOCK |= 1<<26 | 1<<28;                        // тактирование  ARINC429R и ARINC429T 
   
// Настройка приемников
   MDR_ARINC429R->CONTROL1 = 1<<0;                                 // разрешение работы канала 1
   MDR_ARINC429R->CONTROL3 = 1<<28;                                // разрешение прерывания при наличии данных в FIFO
   MDR_ARINC429R->CONTROL4 = ARINC_SYS_FREQ_PRESCALER<<0;          // делитель частоты ядра для получения опорной частоты канала 1
   MDR_ARINC429R->CHANNEL = 0;                                     // выбор первого канала
   
// Настройка передатчиков
   MDR_ARINC429T->CONTROL1 = 1<<4 |     // разрешение работы канала 2
                             1<<6 |     // 32-м битом передается бит паритета канала 2
                             1<<7;     // бит четности - дополнение до нечетного канала 2
                           // (ARINC_SYS_FREQ_PRESCALER&0x7F)<<8; // делитель частоты ядра до 1МГц
   MDR_ARINC429T->CONTROL3 = ARINC_SYS_FREQ_PRESCALER<<8;   
   NVIC_SetPriority(ARINC429R_IRQn, 1);                                         // снижение приоритета на 1
   NVIC_EnableIRQ(ARINC429R_IRQn);      // разрешение прерываний приемника
}

// Отправить массив данных
void arincSendArray(uint32_t *pwords, uint8_t cnt)
{
   for(int i=0; i<cnt; i++)
   {
      while(MDR_ARINC429T->STATUS & 1<<4);      //FIFO канала 2 заполнено
      MDR_ARINC429T->DATA2_T = pwords[i]; 
   }
}

// Обработчик прерываний по приему данных
void ARINC429R_IRQHandler()
{ 
  uint32_t word;  
  while(MDR_ARINC429R->STATUS1 & 1<<0)
  {
     word = MDR_ARINC429R->DATA_R;
     arinc_processRxWord(word);
  }
}
