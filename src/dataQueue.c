/*
  ver 1.0       21.11.17
*/

#include <stdio.h>
#include "dataQueue.h"


// Инициализировать структуру CAN
void dataQueue_init(DataQueue_t *queue, DataBuffer_t *array, int arraySize)
{
  queue->array = array;
  queue->size = arraySize;
  queue->head = 0;
  queue->tail = 0;
  queue->count = 0;
}

// Добавить команду в конец очереди
// Возвращает 1, если команда была добавлена в очередь
uint8_t dataQueue_enqueque(DataQueue_t *queue, uint8_t* pData, uint8_t len)
{
  uint8_t size = queue->size;
  if(queue->count == size) return 0;
  uint8_t tail = queue->tail;
  
  queue->array[tail].pData = pData;
  queue->array[tail].len = len;
  
  queue->tail = (tail+1) % size;
  queue->count++;
  
  if(queue->count > queue->dbg_maxCount) queue->dbg_maxCount = queue->count;
  return 1;
}

// Удалить первый элемент из очереди
DataBuffer_t* dataQueue_dequeue(DataQueue_t *queue)
{
   if(queue->count == 0) return 0;
   
   uint8_t size = queue->size;
   uint8_t head = queue->head;
   DataBuffer_t *element = &queue->array[head];
   
   queue->head = (head+1) % size;
   queue->count--;
   return element;
}



/*
void queueTest1()
{
  CanSoftBuf_t buffers[10];
  CanSoftBuf_t *pBuf;
  for(int i=0; i<sizeof(buffers)/sizeof(CanSoftBuf_t); i++)
  {
    for(int k=0; k<8; k++) buffers[i].data[k] = 0;
  }
  
  QueueCanMsg_t queue;
  queueCAN_init(&queue, buffers, 10);
  
  uint8_t value = 0;
  uint8_t data[8];
  for(int i=0; i<sizeof(buffers)/sizeof(CanSoftBuf_t); i++)
  {
    for(int k=0; k<8; k++) data[k] = k+value;
    queueCAN_enqueque(&queue, data, 8);
    value++;
  }
  
  for(int i=0; i<14; i++)
  {
    pBuf = queueCAN_dequeue(&queue);
  }
  */
  
  
  /*
  int x = queue.size;
  int y = buffers[0].len;
  x = pBuf->len;

}
*/
