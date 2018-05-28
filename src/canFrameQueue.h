/*
  ver 1.0       21.11.17
*/

#ifndef __CAN_FRAME_QUEUE_H__
#define __CAN_FRAME_QUEUE_H__

//#include "MDR1986VE1T.h"
#include "linkedList.h"


// Структура программного буфера CAN-овского сообщения
typedef struct 
{
  uint8_t data[8];      // само сообщение
  uint8_t len;          // длина сообшения
  uint16_t id;          // индентификатор CAN-овского сообщения
} CanMessage_t;

// Программная очередь для одного буфера
typedef struct
{
  MDR_CAN_TypeDef *can; // аппаратный буфер CAN
  LinkedList_t* queue;  // очередь сообщений
  uint8_t hwBuf;        // аппаратный буфер
  uint32_t bdg_maxCount;
} CanSoftwareBuffer_t;



CanSoftwareBuffer_t* canSwBuffer_create(MDR_CAN_TypeDef *can, int capacity, uint8_t hwBuf);
void canSwBuffer_delete(CanSoftwareBuffer_t* queue);

uint8_t canSwBuffer_enqueue(CanSoftwareBuffer_t* buffer, uint8_t *data, uint8_t len, uint16_t canID);
void canSwBuffer_service(CanSoftwareBuffer_t* buffer);








#endif //__CAN_FRAME_QUEUE_H__