/*
  ver 1.0       28.11.17
*/

#ifndef __CANMONITOR_TEXT_H__
#define __CANMONITOR_TEXT_H__


#include "global.h"


#define CAN_QUEUE_TEXT_SIZE                     100           // глубина очереди


typedef enum
{
  CAN_TEXT_TX_START = 0x00,
  CAN_TEXT_TX_CONTINUE = 0x01,
  CAN_TEXT_TX_END = 0x02
} CanTextTransferStage;


typedef struct
{
  MDR_CAN_TypeDef *can;  // аппаратный модуль передачи CAN
  
  uint8_t *pData;        // указатель 
  uint16_t msgSize;      // длина сообщения
  uint16_t offset;       // смещение от начала сообщения
  
  uint16_t chunkSize;    // длина передаваемого куска данных
  uint16_t numOfTransferedBytes;    // кол-во уже переданных байт
  
  CanTextTransferStage stage;       // стадия передачи пакета 0-начало, 1-продолжение, 2-конец  
  uint8_t done;          // сообщение передано
} CanTextTransmitter_t;



void canText_init(MDR_CAN_TypeDef *can);
void canMonitorText_server();
void canMonitor_printf(uint8_t *p, ...);
void canMonitor_blockingPrintf(MDR_CAN_TypeDef *can, uint8_t canHwBuf, uint8_t *format, ...);


#endif //__CANMONITOR_TEXT_H__