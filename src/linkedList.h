/*
  ver 1.0       21.11.17
*/

#ifndef __LINKED_LIST_H__
#define __LINKED_LIST_H__

#include <stdint.h>


// Структура элемента связанного списка
typedef struct LLNode
{
  struct LLNode *next;
  struct LLNode *prev;
  void* data;
} llnode_t;

// Структура связанного списка
typedef struct
{
  llnode_t *head;       // указатель на первый элемент в списке
  llnode_t *tail;       // указатель на последний элемент в списке
  uint32_t count;       // количество элементов в списке
  uint32_t capacity;    // максимальное количество элементов
} LinkedList_t;



// Инициализация
void LinkedList_init(LinkedList_t *list, uint32_t capacity);
LinkedList_t *LinkedList_createNew(uint32_t capacity);
void LinkedList_delete(LinkedList_t *list);

// Добавление элементов
uint8_t linkedList_addFirst(LinkedList_t *list, void *data);    // добавить элемент вначало списка
uint8_t linkedList_addLast(LinkedList_t *list, void *data);     // добавить элемент в конец списка

// Удаление элементов
void* linkedList_removeFirst(LinkedList_t *list);               // удалить первый элемент списка
void* linkedList_removeLast(LinkedList_t *list);                // удалить последний элемент списка


// Отладка
void dbg_linkedList_print(LinkedList_t *list);                  // распечатать текущий список


#endif //__LINKED_LIST_H__