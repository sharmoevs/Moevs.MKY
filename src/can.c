#include "can.h"
#include "global.h"
#include "timers.h"


void _can1_initRxBuffer(uint8_t numOfBuf, uint32_t canFilterID);
void _can1_initTxBuffer(uint8_t numOfBuf, uint8_t enableIrq);



// Инициализация can1
void can1_init()
{
   // Разрешение тактирование блока CAN и предделитель частоты 
   MDR_RST_CLK->PER_CLOCK |= 1<<0;             // тактирование CAN1    
   MDR_RST_CLK->CAN_CLOCK = 0<<0 |             // делитель тактовой частоты CAN1 (0-0, 1-2, 2-4 ect)
                            1<<24;             // разрешение тактовой частоты на CAN1
   
   // Настройка скорости шины CAN
#ifdef MOEVS_BOARD
   MDR_CAN1->BITTMNG = 1<<27 |                 // трехкратное семплирование с мажоритарным контролем
                       1<<25 |                 // размер фазы SJW - это максимальное значение, на которое происходит подстройка приема и передачи при работе на шине CAN. Приемник подстраивается на значение ошибки, но не более чем SJW.
                       5<<22 |                 // размер фазы SEG2 - это время, используемое для сокращения битового интервала при подстройке.
                       5<<19 |                 // размер фазы SEG1 - это время, используемое для увеличения битового интервала при подстройке.
                       6<<16 |                 // размер фазы PSEG - это время, компенсирующее задержку распространения сигналов в шине CAN
                       5<<0;                   // предделитель системной частоты CLK = PCLK/(BRP + 1);  TQ(us) = (BRP + 1)/CLK(MHz)
#else // Отладочная плата
   MDR_CAN1->BITTMNG = 1<<27 |                 // трехкратное семплирование с мажоритарным контролем
                       1<<25 |                 // размер фазы SJW - это максимальное значение, на которое происходит подстройка приема и передачи при работе на шине CAN. Приемник подстраивается на значение ошибки, но не более чем SJW.
                       7<<22 |                 // размер фазы SEG2 - это время, используемое для сокращения битового интервала при подстройке.
                       7<<19 |                 // размер фазы SEG1 - это время, используемое для увеличения битового интервала при подстройке.
                       7<<16 |                 // размер фазы PSEG - это время, компенсирующее задержку распространения сигналов в шине CAN
                       4<<0;                   // предделитель системной частоты CLK = PCLK/(BRP + 1);  TQ(us) = (BRP + 1)/CLK(MHz)
#endif //MOEVS_BOARD
   
   
   
   _can1_initTxBuffer(CAN_LDR_TX_BUF,              0); // Передающий буфер - для перепрошивки
   _can1_initTxBuffer(CAN_TERM_TX_BUF,             0); // Передающий буфер - для терминала
   _can1_initTxBuffer(CAN_MONITOR_TX_BUF,          0); // Передающий буфер - для монитора
   _can1_initTxBuffer(CAN_TANGAGE_CONTROL_TX_BUF,  0); // Передающий буфер - для тангажного контура
   _can1_initTxBuffer(CAN_DBG_TEXT_MESSAGE_TX_BUF, 0); // Передающий буфер для текста
   
   _can1_initRxBuffer(CAN_LDR_RX_BUF,             CAN_RX_FILTER_PROGRAM);         // Приемный буфер - для перепрошивки
   _can1_initRxBuffer(CAN_TERM_RX_BUF,            CAN_RX_FILTER_TERMINAL);        // Приемный буфер - для терминала
   _can1_initRxBuffer(CAN_MONITOR_RX_BUF,         CAN_RX_FILTER_MONITOR);         // Приемный буфер - для монитора
   _can1_initRxBuffer(CAN_TANGAGE_ANGLE_RX_BUF,   CAN_RX_FILTER_TANGAGE_ANGLE);   // Приемный буфер - для тангажного угла
   _can1_initRxBuffer(CAN_TANGAGE_CONTROL_RX_BUF, CAN_RX_FILTER_TANGAGE_CONTROL); // Приемный буфер - для тангажного контура
   
  
   // Настройка прерываний 
   MDR_CAN1->INT_EN = //1<<4 |                                                    // разрешение прерывания по превышению TEC или REC допустимого значения в ERROR_MAX
                      //1<<3 |                                                    // разрешение прерывания по возникновению ошибки
                      //1<<2 |                                                    // разрешение прерывания по возможности передачи
                      1<<1 |                                                    // разрешениe прерывания по приему сообщений
                      1<<0;                                                     // глобальное разрешение прерывания блока CAN   
  
   MDR_CAN1->CONTROL = 1<<0;                                                    // разрешение работы контроллера CAN  
   NVIC_SetPriority(CAN1_IRQn, 1);                                              // снижение приоритета на 1
}

// Инициализация приемного буфера
void _can1_initRxBuffer(uint8_t numOfBuf, uint32_t canFilterID)
{
   // Приемный буфер - для монитора
   MDR_CAN1->BUF_CON[numOfBuf] = 1<<1 |                               // буфер работает на прием
                                 1<<0;                                // разрешение работы буфера
   MDR_CAN1->CAN_BUF_FILTER[numOfBuf].MASK = CAN_STANDART_ID_MASK;    // ID & CAN_BUF_FILTER[x].MASK == CAN_BUF_FILTER[x].FILTER
   MDR_CAN1->CAN_BUF_FILTER[numOfBuf].FILTER = canFilterID;
   MDR_CAN1->INT_RX |= 1<<numOfBuf;                                   // разрешение прерывания первого буфера
}

// Инициализация передающего буфера
void _can1_initTxBuffer(uint8_t numOfBuf, uint8_t enableIrq)
{
   MDR_CAN1->BUF_CON[numOfBuf] = 0<<1 |                               // буфер работает на передачу
                                 1<<0;                                // разрешение работы буфера
   if(enableIrq) MDR_CAN1->INT_TX |= 1<<numOfBuf;                               // разрешение прерывания первого буфера
}


// Принять новый пакет
__ramfunc uint8_t can1_rx_new_packet(uint8_t can_buf_indx, uint8_t *buf, uint8_t *len)
{
   if(MDR_CAN1->BUF_CON[can_buf_indx] & 1<<6)
   {
      ((uint32_t*)buf)[0] = MDR_CAN1->CAN_BUF[can_buf_indx].DATAL;
      ((uint32_t*)buf)[1] = MDR_CAN1->CAN_BUF[can_buf_indx].DATAH;
     
      MDR_CAN1->BUF_CON[can_buf_indx] &= ~(1<<6);
      *len = MDR_CAN1->CAN_BUF[can_buf_indx].DLC&0xF;
      return 1;
   }
   else return 0;
}


// Передать пакет. Возвращает 1, если передача завершилась успешно, 0 - в противном случае
__ramfunc uint8_t can1_send_packet(uint8_t can_buf_indx, uint16_t msgId, uint8_t *pData, uint8_t len)
{   
    uint32_t transfer_time = system_time;
    while(CAN_TX_REQ(MDR_CAN1, can_buf_indx)) // ожидание освобождения буфера передатчика
    {
      if(elapsed(&transfer_time, CAN_TX_TIMEOUT_MS)) // истекло время ожидания 
      {
         CAN_TX_REQ_CLR(MDR_CAN1, can_buf_indx);                                // снять запрос на передачу пакета
         return 0;
      }
    }
  
    // Ожидание, пока завершится передача пакета или превысится лимит ошибок передачи
    //while(CAN_TX_REQ(MDR_CAN1, can_buf_indx) && !CAN_ERROR_OVER(MDR_CAN1));
    
    if(len>8) len=8;
    MDR_CAN1->CAN_BUF[can_buf_indx].ID = msgId<<18;
    MDR_CAN1->CAN_BUF[can_buf_indx].DLC = len | 1<<11;    
    MDR_CAN1->CAN_BUF[can_buf_indx].DATAL = *((uint32_t*)pData);
    MDR_CAN1->CAN_BUF[can_buf_indx].DATAH = *((uint32_t*)pData+1);
    MDR_CAN1->BUF_CON[can_buf_indx] |= 1<<5;              // запрос на отправку
           
    return 1;
}


// Записать пакет в аппаратный буфер передатчика
void can_putPackage(MDR_CAN_TypeDef *can, uint8_t can_buf_indx, uint16_t msgId, uint8_t *pData, uint8_t len)
{
    if(len>8) len=8;
    can->CAN_BUF[can_buf_indx].ID = msgId<<18;
    can->CAN_BUF[can_buf_indx].DLC = len | 1<<11;    
    can->CAN_BUF[can_buf_indx].DATAL = *((uint32_t*)pData);
    can->CAN_BUF[can_buf_indx].DATAH = *((uint32_t*)pData+1);
    can->BUF_CON[can_buf_indx] |= 1<<5;              // запрос на отправку
}


// Записать пакет в аппаратный буфер передатчика
void can1_putPackage(uint8_t can_buf_indx, uint16_t msgId, uint8_t *pData, uint8_t len)
{
    if(len>8) len=8;
    MDR_CAN1->CAN_BUF[can_buf_indx].ID = msgId<<18;
    MDR_CAN1->CAN_BUF[can_buf_indx].DLC = len | 1<<11;    
    MDR_CAN1->CAN_BUF[can_buf_indx].DATAL = *((uint32_t*)pData);
    MDR_CAN1->CAN_BUF[can_buf_indx].DATAH = *((uint32_t*)pData+1);
    MDR_CAN1->BUF_CON[can_buf_indx] |= 1<<5;              // запрос на отправку
}


// Обработчик прерываний CAN1
void CAN1_IRQHandler()
{ 
    uint8_t rxBuf[8];
    uint8_t len;
    if(can1_rx_new_packet(CAN_LDR_RX_BUF, rxBuf, &len))         // перепрошивка
    {
       extern void process_canloader_frame(uint8_t*, uint8_t);
       process_canloader_frame(rxBuf, len);
    }
    
    //if(can1_rx_new_packet(CAN_TERM_RX_BUF, rxBuf, &len))        // терминал
    //{
    //   extern void term_receive_msg(uint8_t *buf, uint8_t len);
    //   term_receive_msg(rxBuf, len);
    //}
    
    if(can1_rx_new_packet(CAN_MONITOR_RX_BUF, rxBuf, &len))     // монитор
    {
       canMonitor_rxFrameHandler(rxBuf, len);
    }
    
    if(can1_rx_new_packet(CAN_TANGAGE_ANGLE_RX_BUF, rxBuf, &len))     // тангажный угол
    {
      tangageCtrl_rxAngle(rxBuf, len);
    }
    
    if(can1_rx_new_packet(CAN_TANGAGE_CONTROL_RX_BUF, rxBuf, &len))   // тангажный контур
    {
      tangageCtrl_rxMsg(rxBuf, len);
    }
}

