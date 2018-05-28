/*
  ver 1.0       21.11.17
*/

#ifndef __DATA_QUEUE_H__
#define __DATA_QUEUE_H__

#include <stdint.h>


// Структура буфера
typedef struct
{
  uint8_t *pData;
  uint16_t len;
} DataBuffer_t;

// Структура очередь
typedef struct
{
  DataBuffer_t *array;  // указатель на массив слотов
  uint16_t size;        // размер очереди (максимальное кол-во элементов)
  
  uint16_t dbg_maxCount;// максимальное количество элементов, которое было в очереди
  uint16_t count;       // количество элементов в очереди  
  uint8_t  head;        // указатель на первый элемент в очереди
  uint8_t  tail;        // указатель на последний элемент в очереди  
} DataQueue_t;



void          dataQueue_init(DataQueue_t *queue, DataBuffer_t *array, int arraySize);
uint8_t       dataQueue_enqueque(DataQueue_t *queue, uint8_t* pData, uint8_t len);
DataBuffer_t* dataQueue_dequeue(DataQueue_t *queue);


#endif //__DATA_QUEUE_H__