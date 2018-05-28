/*
  ver 1.0       21.11.17
*/

#include <stdlib.h>
#include "can.h"
#include "canFrameQueue.h"


CanMessage_t* canSwBuffer_dequeue(LinkedList_t *queue);
  
  
// Создать программную очередь для буфера
CanSoftwareBuffer_t* canSwBuffer_create(MDR_CAN_TypeDef *can, int capacity, uint8_t hwBuf)
{
   uint32_t irqDisabled = __get_PRIMASK();
  __disable_irq();
  
  CanSoftwareBuffer_t* softBuffer = (CanSoftwareBuffer_t*)malloc(sizeof(CanSoftwareBuffer_t));
  if(softBuffer == 0)
  {
    if(!irqDisabled) __enable_irq();
    return 0;
  }  
  
  softBuffer->can = can;
  softBuffer->queue = LinkedList_createNew(capacity);
  softBuffer->hwBuf = hwBuf;
  softBuffer->bdg_maxCount = 0;
  
  if(!irqDisabled) __enable_irq();
  return softBuffer;
}

// Удалить программный буфер
void canSwBuffer_delete(CanSoftwareBuffer_t* queue)
{
  uint32_t irqDisabled = __get_PRIMASK();
  __disable_irq();
  LinkedList_delete(queue->queue);
  free(queue);
  if(!irqDisabled) __enable_irq();
}


// Добавить элемент в конец очереди
uint8_t canSwBuffer_enqueue(CanSoftwareBuffer_t* buffer, uint8_t *data, uint8_t len, uint16_t canID)
{
  uint32_t irqDisabled = __get_PRIMASK();
  __disable_irq(); 
  if(buffer->queue->count == buffer->queue->capacity) 
  {
    if(!irqDisabled) __enable_irq();
    return 0;
  }
  
  CanMessage_t *msg = (CanMessage_t*)malloc(sizeof(CanMessage_t));
  if(msg == 0)
  {
    if(!irqDisabled) __enable_irq();
    return 0;    
  }
  
  for(uint8_t i=0; i<len; i++) msg->data[i] = data[i];
  msg->len = len;
  msg->id = canID;  
  uint8_t ok = linkedList_addLast(buffer->queue, msg);
 
  // Отладка
  if(ok)
  {
    if(buffer->queue->count > buffer->bdg_maxCount)
    {
      buffer->bdg_maxCount = buffer->queue->count;
    }
  }  
  
  if(!irqDisabled) __enable_irq();
  return ok;
}

// Взять первый элемент из очереди
CanMessage_t* canSwBuffer_dequeue(LinkedList_t *queue)
{
  CanMessage_t *swBuf = (CanMessage_t *)linkedList_removeFirst(queue);
  return swBuf;
}




// Сервер  отправки сообщений из программного буфера
void canSwBuffer_service(CanSoftwareBuffer_t* buffer)
{
  if(CAN_TX_REQ(buffer->can, buffer->hwBuf)) return;    // передача не завершена  
  
  uint32_t irqDisabled = __get_PRIMASK();
  __disable_irq();
  
  if(buffer->queue->count == 0)
  {
    if(!irqDisabled) __enable_irq();
    return;
  }
    
  CanMessage_t *msg = canSwBuffer_dequeue(buffer->queue);  
  can_putPackage(buffer->can, buffer->hwBuf, msg->id, msg->data, msg->len);
  free(msg);
  if(!irqDisabled) __enable_irq();
}






