/*
  ver 1.0       21.11.17
*/

#include <stdlib.h>
#include <stdio.h>
#include "linkedList.h"


// Инициализация 
void LinkedList_init(LinkedList_t *list, uint32_t capacity)
{
  list->head = 0;
  list->tail = 0;
  list->count = 0;
  list->capacity = capacity;
}

// Создать новый список 
LinkedList_t *LinkedList_createNew(uint32_t capacity)
{
  LinkedList_t *list = (LinkedList_t*)malloc(sizeof(LinkedList_t));
  if(list == 0) return 0;

  LinkedList_init(list, capacity);
  return list;
}

// Удалить список и все его элементы
void LinkedList_delete(LinkedList_t *list)
{
  while(linkedList_removeFirst(list)); // очистить список
  free(list);
}


// Добавить в начало списка
uint8_t linkedList_addFirst(LinkedList_t *list, void *data)
{
  llnode_t *newNode = (llnode_t*)malloc(sizeof(llnode_t));
  if(newNode == 0) return 0;                    // не удалось выделить память
  if(list->count == list->capacity) return 0;   // список заполнен
  
  newNode->data = data;
  newNode->next = 0;
  newNode->prev = list->head;
  
  if(list->count == 0)
  {
    list->tail = newNode;
  }
  
  list->head->next = newNode;
  list->head = newNode;
  list->count++;
  return 1;
}

// Добавить в конец списка
uint8_t linkedList_addLast(LinkedList_t *list, void *data)
{
  llnode_t *newNode = (llnode_t*)malloc(sizeof(llnode_t));
  if(newNode == 0) return 0;                    // не удалось выделить память
  if(list->count == list->capacity) return 0;   // список заполнен
  
  newNode->data = data;
  newNode->prev = 0;
  newNode->next = list->tail;
  
  if(list->count == 0)  // список пуст
  {
     list->head = newNode;
  }
  
  list->tail->prev = newNode;
  list->tail = newNode;
  list->count++;    
  return 1;
}


// Возвращает и удаляет первый элемент из очереди
void* linkedList_removeFirst(LinkedList_t *list)
{
  if(list->count == 0) return 0;

  llnode_t *node = list->head;
  void *value = node->data;
  
  if(list->count == 1) // последний элемент
  {
    list->head = 0;
    list->tail = 0;
  }
  else
  {  
    list->head = list->head->prev;  // указатель на первый элемент списка
    list->head->next = 0;           // указатель на следующий элемент списка
  }
  list->count--;  
  free(node);
  return value;
}

// Возвращает и удаляет последний элемент из очереди
void* linkedList_removeLast(LinkedList_t *list)
{
  if(list->count == 0) return 0;

  llnode_t *node = list->tail;
  void *value = node->data;
  
  if(list->count == 1) // последний элемент
  {
    list->head = 0;
    list->tail = 0;
  }
  else
  {
    list->tail = list->tail->next;  // указатель на последний элемент списка
    list->tail->prev = 0;           // указатель на предыдущий элемент списка
  }
  list->count--;
  free(node);
  return value;
}


// Для отладки. Распечатать связанный список
void dbg_linkedList_print(LinkedList_t *list)
{
  int i=0;
  llnode_t *node = list->head;
  while(node != 0)
  {
    int value = *((int*)node->data);
    printf("#%d: %d\n", i, value);
    node = node->prev;
    i++;
  }
  
  if(i==0) printf("empty\n");
  printf("\n");
  
  node->next +=0;
  list->count +=0;
}
