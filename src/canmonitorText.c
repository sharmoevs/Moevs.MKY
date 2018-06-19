/*
  ver 1.0       28.11.17
*/

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include "can.h"
#include "canMonitorText.h"
#include "dataQueue.h"




DataBuffer_t canTextBuffers[CAN_QUEUE_TEXT_SIZE];
DataQueue_t canTextQueue;
CanTextTransmitter_t textTransmitter;


void _canText_getNextMessage();
void _canText_sendNextChunk();
void _canMonitor_putAndWait(MDR_CAN_TypeDef *can, uint8_t canHwBuf, uint8_t *pBuf, uint8_t len);

  
// Инициализация очереди CAN
void canText_init(MDR_CAN_TypeDef *can)
{
  dataQueue_init(&canTextQueue, canTextBuffers, CAN_QUEUE_TEXT_SIZE);
  textTransmitter.can = can;
  textTransmitter.done = 1;
}

// Сервер по передаче строк
void canMonitorText_server()
{
   if(textTransmitter.done)   // передача сообщения завершена
   {
      if(canTextQueue.count > 0)  // если в очереди есть сообщения на передачу
      {
        _canText_getNextMessage();
        _canText_sendNextChunk();
      }
   }
   else // идет передача сообщения
   {     
     if(CAN_TX_REQ(textTransmitter.can, CAN_DBG_TEXT_MESSAGE_TX_BUF)) return; // передача не завершена
     
     textTransmitter.numOfTransferedBytes += textTransmitter.chunkSize;
     if(textTransmitter.numOfTransferedBytes == textTransmitter.msgSize) // передано всё сообщение
     {
       uint32_t irqDisabled = __get_PRIMASK();
       __disable_irq();
       free(textTransmitter.pData);     // очистить память
       if(!irqDisabled) __enable_irq();
       
       textTransmitter.done = 1;
     }
     else
     {
        _canText_sendNextChunk();
     }     
   }
}

// Достать следующее сообщение из очереди
void _canText_getNextMessage()
{
  uint32_t irqDisabled = __get_PRIMASK();
  __disable_irq();
  DataBuffer_t *dataBuffer = dataQueue_dequeue(&canTextQueue);
  if(!irqDisabled) __enable_irq();

  textTransmitter.pData = dataBuffer->pData;
  textTransmitter.msgSize = dataBuffer->len;
  textTransmitter.offset = 0;
  textTransmitter.numOfTransferedBytes = 0;
  textTransmitter.stage = CAN_TEXT_TX_START;
  textTransmitter.done = 0;
}

// Передать строку
void _canText_sendNextChunk()
{ 
  static const uint8_t MAX_DATA_LEN = 7;
  uint8_t data[8];
  uint16_t offset = textTransmitter.offset;
  uint8_t *src = textTransmitter.pData;
  uint8_t *dst = &data[1];
  
  uint16_t remainingBytes = textTransmitter.msgSize - textTransmitter.numOfTransferedBytes;   // кол-во байт для передачи
  textTransmitter.chunkSize = remainingBytes<=MAX_DATA_LEN ? remainingBytes : MAX_DATA_LEN;

  switch(textTransmitter.stage)
  {
    case CAN_TEXT_TX_START:
      data[0] = (remainingBytes <= MAX_DATA_LEN) ? CAN_TEXT_TX_END : CAN_TEXT_TX_START;
      textTransmitter.stage = CAN_TEXT_TX_CONTINUE;     // следующий пакет будет "Продолжением"
      break;
      
    case CAN_TEXT_TX_CONTINUE: 
      data[0] = (remainingBytes <= MAX_DATA_LEN) ? CAN_TEXT_TX_END : CAN_TEXT_TX_CONTINUE;      
      break;
  }
  
  for(uint8_t i=0; i<textTransmitter.chunkSize; i++) *dst++ = *(src+offset+i);
  textTransmitter.offset += textTransmitter.chunkSize;  
  
  can_putPackage(textTransmitter.can, CAN_DBG_TEXT_MESSAGE_TX_BUF, CANID_DBG_TEXT_TX, data, 1+textTransmitter.chunkSize);// первый байт служебный
}

#ifdef USE_DYNAMIC_CAN_QUEUE
// Отправить строку
void canMonitor_printf(uint8_t *p, ...)
{
  uint8_t buf[256];
  
  va_list args;
  va_start(args, p);
  int len = vsnprintf((char*)buf, sizeof(buf), (char*)p, args);
  va_end(args);
 
  if(len<=0) return;
    
  uint32_t irqDisabled = __get_PRIMASK();
  __disable_irq();
  uint8_t *pMem = (uint8_t*)malloc(len);
  if(!irqDisabled) __enable_irq();
  
  if(pMem == 0) 
  {
    return;
  }
  memcpy(pMem, buf, len);
  
  __disable_irq();
  uint8_t ok = dataQueue_enqueque(&canTextQueue, pMem, len);
  if(!ok)
  {
    free(pMem);
    int x=0; 
    x++;
  }
  if(!irqDisabled) __enable_irq();
}
#endif //USE_DYNAMIC_CAN_QUEUE




// =============================================================================
// ================================ blocking ===================================
// =============================================================================

// Записать пакет в буфер и дождаться его отправки
void _canMonitor_putAndWait(MDR_CAN_TypeDef *can, uint8_t canHwBuf, uint8_t *pBuf, uint8_t len)
{
   uint8_t msg[8];
   uint8_t needToSend = 0;          // сколько байт осталось отправить
   uint8_t sendedBytes = 0;         // кол-во отправленных байт данных
   uint8_t dataBytesInFrame = 0;    // кол-во байт в передаваемом пакете
   uint8_t indx = 0;                // индекс

#define MAX_DATA_BYTES  7   
   while(sendedBytes != len)
   {
      needToSend = len - sendedBytes;
      if(needToSend <= MAX_DATA_BYTES) // если оставшиеся данные влезают в один пакет - то он последний
      {
          msg[0] = CAN_TEXT_TX_END;
      }
      else
      {
          if(sendedBytes == 0) msg[0] = CAN_TEXT_TX_START;
          else msg[0] = CAN_TEXT_TX_CONTINUE;
      }

      dataBytesInFrame = needToSend <= MAX_DATA_BYTES ? needToSend : MAX_DATA_BYTES;
      sendedBytes += dataBytesInFrame;

      for(uint8_t i=0; i<dataBytesInFrame; i++)
      {
         msg[i+1] = pBuf[indx++];
      }
      can_putPackage(can, canHwBuf, CANID_DBG_TEXT_TX, msg, dataBytesInFrame+1);// первый байт служебный
      while(CAN_TX_REQ(can, canHwBuf));
   }  
}

// Блокирующая версия printf
void canMonitor_blockingPrintf(MDR_CAN_TypeDef *can, uint8_t canHwBuf, uint8_t *format, ...)
{
  uint8_t buf[256];  
  va_list args;
  va_start(args, format);
  int len = vsnprintf((char*)buf, sizeof(buf), (char*)format, args);
  va_end(args);
  
  if(len<=0) return;
  
  _canMonitor_putAndWait(can, canHwBuf, buf, len);
}