#ifndef __LOADERS__
#define __LOADERS__

#include "MDR1986VE1T.h"


// Расположение полей загрузчика во флеши:
//  1     1         1        1        
//  Id    NameLen   reserv   reserv                первое слово во флеши
//
//  1..255 
//  Name                                           второе слово во флеши
typedef struct LoaderStruct
{
  uint32_t flashPageAdder;      // адрес страницы во флеши, где хранятся id name загрузчика
  uint8_t *pName;               // указатель на имя загрузчика
  uint8_t nameLen;              // длина имени загрузчика
  uint8_t state;                // загрузчик находится в режиме программирования
  uint8_t id;                   // идентификатор загрузчика  
  uint8_t isCritical;           // критичность смены загрузчика 1 - для самозагрузчиков
  
  uint32_t memPageSize;         // виртуальный размер страницы памяти
  uint16_t memPagesCnt;         // количество виртуальных страниц памяти
  uint16_t memOffset;           // смещение от начала страницы 
  uint16_t flags;               // индивидуальные настройки для различных типов флеши
}Loader_TypeDef;


// Указатели на загрузчики
#define SELF_LOADER                            (&loaders[0])
#define LOADERS_COUNT                          1


// Протокол
#define CANLOADER_MSG_ID                       0x1                              // запрос кто в сети?
#define CANLOADER_MSG_NAME                     0x2                              // запрос имени
#define CANLOADER_MSG_LOADER_MODE              0x3                              // управление загрузчиком
#define CANLOADER_MSG_RECEIVE_NEW_NAME         0x4                              // задать новое имя
#define CANLOADER_MSG_SAVE_NEW_NAME            0x5                              // сохранить новое имя и id
#define CANLOADER_MSG_MEM_PARAM                0x6                              // запрос параметров памяи
#define CANLOADER_MSG_ERASE_MEM                0x7                              // стереть память
#define CANLOADER_MSG_SET_MEM_OFFSET           0x8                              // смещение внутри страницы
#define CANLOADER_MSG_LOAD_PAGE                0x9                              // загрузить страницу
#define CANLOADER_MSG_WRITE_PAGE               0xA                              // записать страницу
#define CANLOADER_MSG_READ_PAGE                0xB                              // прочитать страницу


// Состояние загрузчика загрузчика
#define CANLOADER_STATE_ACTIVE                 1
#define CANLOADER_STATE_INACTIVE               0



#define VIRTUAL_PAGE_SIZE                       256
#define DIV_ON_VIRTUAL_PAGE_SIZE(val)           (val>>8)                // /256







void setupLoader(Loader_TypeDef *loader, uint32_t pageAddr, uint8_t isCritical, 
                uint32_t memPageSize, uint16_t memPagesCnt,
                uint8_t defaultId, uint8_t defaultNameLen, uint8_t* pDefaultName);




void loaders_init();
__ramfunc uint8_t exist_loader(uint8_t id, Loader_TypeDef **loader);

__ramfunc void process_canloader_frame(uint8_t *buf, uint8_t len);
__ramfunc void send_loader_id(Loader_TypeDef *loader);
__ramfunc void send_loader_name(Loader_TypeDef *loader);
__ramfunc void change_loader_mode(Loader_TypeDef *loader, uint8_t newState);
__ramfunc void receive_new_loader_name(Loader_TypeDef *loader, uint8_t *pData, uint8_t len);
__ramfunc void save_new_loader_name(Loader_TypeDef *loader, uint8_t newId, uint8_t nameLen);




__ramfunc void send_loader_mem_param(Loader_TypeDef *loader);
__ramfunc void load_software_page(Loader_TypeDef *loader, uint8_t *pData, uint8_t len);









#endif //__LOADERS__

