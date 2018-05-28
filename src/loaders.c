#include "loaders.h"
#include "selfLoader.h"
#include "settings.h"
#include "can.h"


uint8_t newLoaderNameBuf[256];                  // буфер для приема нового имени загрузчика
uint8_t softwarePageBuf[VIRTUAL_PAGE_SIZE];     // буфер для приема страницы прошивки
uint16_t memOffset;                             // смещение от начала страницы

Loader_TypeDef loaders[LOADERS_COUNT];

// Инициализация загрузчика
void setupLoader(Loader_TypeDef *loader, uint32_t pageAddr, uint8_t isCritical,
                 uint32_t memPageSize, uint16_t memPagesCnt,
                 uint8_t defaultId, uint8_t defaultNameLen, uint8_t* pDefaultName)
{
    loader->flashPageAdder = pageAddr;
    loader->state = 0;
    loader->isCritical = isCritical;
    
    loader->memPageSize = memPageSize;
    loader->memPagesCnt = memPagesCnt;
    
    uint8_t id = *((uint8_t*)pageAddr);    
    if(id == 0xFF) 
    {
       loader->id = defaultId;
       loader->nameLen = defaultNameLen;
       loader->pName = pDefaultName;
    }
    else 
    {
       loader->id = *((uint8_t*)pageAddr);
       loader->nameLen = *((uint8_t*)(pageAddr+1));
       loader->pName =  (uint8_t*)(pageAddr+4);       
    }    
}

// Инициализация загрузчиков
void loaders_init()
{     
  // Самозагрузчик
  uint16_t selfLoader_mem_pages_cnt = (FLASH1986VE1T_PAGES_CNT - LOADERS_COUNT) - 1;     // кол-во страниц для прошивки: одна для загрузчика и одна для параметров
  setupLoader(SELF_LOADER, GET_FLASH_PAGE_ADDR(31), 1,
             MK_FLASH_PAGE_SIZE, selfLoader_mem_pages_cnt,
             SELFLOADER_DEFAULT_ID, sizeof(SELF_LOADER_DEFAULT_NAME), SELF_LOADER_DEFAULT_NAME);
}

// Возвращает загрузчик с требуемым id
uint8_t exist_loader(uint8_t id, Loader_TypeDef **loader)
{
   for(int i=0; i<LOADERS_COUNT; i++)
   {
      if(loaders[i].id == id)
      {
         *loader = &loaders[i];
         return 1;
      }
   }  
   return 0;
}



// Обработка принятого can-фрейма
__ramfunc void process_canloader_frame(uint8_t *rxBuf, uint8_t len)
{
  uint8_t id = rxBuf[0];
  uint8_t msgType = rxBuf[1];
    
  // Запрос "Кто в сети?"
  if((id == 0xFF) && (msgType == CANLOADER_MSG_ID)) 
  {
     if(len != 2) return;     
     for(int i=0; i<LOADERS_COUNT; i++)
     {   
         send_loader_id(&loaders[i]);   
     } 
     return;
  }  
  
  Loader_TypeDef *loader;  
  if(!exist_loader(id, &loader)) return;        // не найден загрузчик с таким id
  
  switch(msgType)
  {    
      case CANLOADER_MSG_NAME:                  // запрос имени загрузчика
        {  
           if(len != 2) return;
           send_loader_name(loader); 
        } 
        break;
        
      case CANLOADER_MSG_LOADER_MODE:           // задание режима загрузчика
        {
           if(len != 3) return;           
           change_loader_mode(loader, rxBuf[2]);
        }
        break;
        
      case CANLOADER_MSG_RECEIVE_NEW_NAME:      // принять новое имя загрузчика
        {
           receive_new_loader_name(loader, rxBuf, len);
        }
        break;
        
      case CANLOADER_MSG_SAVE_NEW_NAME:         // сохранить новое имя загрузчика
        {
           if(len != 4) return;     
           save_new_loader_name(loader, rxBuf[2], rxBuf[3]);
        }
        break;
                
      case CANLOADER_MSG_MEM_PARAM:             // отправить параметры памяти
        {          
          if(len!=2) return;
          send_loader_mem_param(loader);  
        }
        break;
        
      case CANLOADER_MSG_ERASE_MEM:             // стереть память
       {
          if(len!=4) return;
          uint16_t pages_cnt = (uint16_t)rxBuf[2]<<8 | (uint16_t)rxBuf[3];          
          selfloader_erase_mem(loader,pages_cnt);
       }    
       break;  
     
      case CANLOADER_MSG_SET_MEM_OFFSET:        // задать смещение в страницу памяти  
       {
          if(len!=4) return;
          memOffset = (uint16_t)rxBuf[2]<<8 | (uint16_t)rxBuf[3];         
       }    
       break; 
              
      case CANLOADER_MSG_LOAD_PAGE:             // загрузить страницу
       {
          if(len<3) return;
          load_software_page(loader, rxBuf, len);  
       }  
       break;
       
     case CANLOADER_MSG_WRITE_PAGE:             // записать страницу 
       {
          if(len!=4) return;
          uint16_t numOfPage = (uint16_t)rxBuf[2]<<8 | (uint16_t)rxBuf[3];         
          selfloader_write_flash_page(loader,numOfPage);
        }
       break;
     
     case CANLOADER_MSG_READ_PAGE:              // прочитать страницу
       {
          if(len!=4) return;
          uint16_t numOfPage = (uint16_t)rxBuf[2]<<8 | (uint16_t)rxBuf[3];               
          selfloader_read_flash_page(loader, numOfPage);
       }
       break;
  }
}


// Ответ на запрос "Кто в сети"
__ramfunc void send_loader_id(Loader_TypeDef *loader)
{
  uint8_t txBuf[4];
  txBuf[0] = loader->id;               // идентификатор устройства
  txBuf[1] = CANLOADER_MSG_ID;         // тип сообщения - ID
  txBuf[2] = loader->nameLen;          // размер имени
  txBuf[3] = loader->isCritical;       // флаг осторожного стирания
  
  can1_send_packet(CAN_LDR_TX_BUF, CANID_LDR_ANSWER, txBuf, sizeof(txBuf));
}

// Отправить имя загрузчика                        
__ramfunc void send_loader_name(Loader_TypeDef *loader)
{
  uint8_t txBuf[8];
  txBuf[0] = loader->id;
  txBuf[1] = CANLOADER_MSG_NAME;
  
  uint8_t offset = 0;
  uint8_t cnt;          // кол-во передаваемый байт в текущей транзакции
  
  while(offset < loader->nameLen)
  {
     cnt = loader->nameLen - offset;
     if(cnt >= 5)
     {
        txBuf[2] = offset; 
        txBuf[3] = loader->pName[offset++]; 
        txBuf[4] = loader->pName[offset++];
        txBuf[5] = loader->pName[offset++];
        txBuf[6] = loader->pName[offset++];
        txBuf[7] = loader->pName[offset++];
        
        can1_send_packet(CAN_LDR_TX_BUF, CANID_LDR_ANSWER, txBuf, sizeof(txBuf));
     }
     else 
     {
        txBuf[2] = offset; 
        uint8_t i;
        for(i=3; i<cnt+3; i++)  
           txBuf[i] = loader->pName[offset++]; 

        can1_send_packet(CAN_LDR_TX_BUF, CANID_LDR_ANSWER, txBuf, i);
     }
  }  
}

// Перевести загрузчик в активный режим
__ramfunc void change_loader_mode(Loader_TypeDef *loader, uint8_t newState)
{
   uint8_t txBuf[3];
   txBuf[0] = loader->id;
   txBuf[1] = CANLOADER_MSG_LOADER_MODE;
   
   if(loader->state == newState)
   {
      txBuf[2] = loader->state;
      can1_send_packet(CAN_LDR_TX_BUF, CANID_LDR_ANSWER, txBuf, sizeof(txBuf));
      return;
   }
   
   loader->state = newState;
   
   if(loader->state == CANLOADER_STATE_ACTIVE)                           // вход в режмм программирования
   {
      asm volatile ("cpsid i");                    // запрет прерываний
      txBuf[2] = loader->state;
      can1_send_packet(CAN_LDR_TX_BUF, CANID_LDR_ANSWER, txBuf, sizeof(txBuf));

      uint8_t can_rx_buf[8];
      uint8_t len;
      while(1)
      {
          if(can1_rx_new_packet(CAN_LDR_RX_BUF, can_rx_buf, &len)) 
            process_canloader_frame(can_rx_buf, len);
      }     
   }
   else 
   {          
      txBuf[2] = loader->state;
      can1_send_packet(CAN_LDR_TX_BUF, CANID_LDR_ANSWER, txBuf, sizeof(txBuf));
      while(CAN_TX_REQ(MDR_CAN1, CAN_LDR_TX_BUF));
      asm volatile ("cpsid i");            // разрешение прерываний
            
      SYSTEM_RESET();
      while(1);
   }  
}

// Сохранить в буфере новое имя загрузчика
__ramfunc void receive_new_loader_name(Loader_TypeDef *loader, uint8_t *pData, uint8_t len)
{
   if(loader->state != CANLOADER_STATE_ACTIVE) return;
   
   uint8_t offset = pData[2];
   for(uint8_t i=3; i<len; i++)
      newLoaderNameBuf[offset++] = pData[i];
}

// Сохранить новое имя и идентификатор
__ramfunc void save_new_loader_name(Loader_TypeDef *loader, uint8_t newId, uint8_t nameLen)
{       
   erase_flash_page(loader->flashPageAdder);     // стирание страницы с id и именем загрузчика
  
   // Отчет о статусе сохранения имени 
   uint8_t txBuf[3];
   txBuf[0] = loader->id;                       // идентификатор устройства. В ответе старый ID
   txBuf[1] = CANLOADER_MSG_SAVE_NEW_NAME;      // тип сообщения - ID
   txBuf[2] = 0x1;                              // сохранение выполняется
   can1_send_packet(CAN_LDR_TX_BUF, CANID_LDR_ANSWER, txBuf, sizeof(txBuf));
   
   
   // сохранение нового id и длины имени
   uint32_t tmp = newId | 
                  nameLen<<8;
   flash_write_word(loader->flashPageAdder, tmp);
       
   uint32_t addrName = loader->flashPageAdder + 4;
   uint32_t recBytesCnt = 0;                     // число записанных байт имени
   
   while(recBytesCnt < nameLen)
   {
      tmp = (uint32_t)newLoaderNameBuf[recBytesCnt] |
            (uint32_t)newLoaderNameBuf[recBytesCnt+1]<<8 |
            (uint32_t)newLoaderNameBuf[recBytesCnt+2]<<16 |
            (uint32_t)newLoaderNameBuf[recBytesCnt+3]<<24;      
                   
      flash_write_word(addrName, tmp);
      addrName+=4;
      recBytesCnt+=4;
           
      // отчет о состоянии записи - выполняется
      can1_send_packet(CAN_LDR_TX_BUF, CANID_LDR_ANSWER, txBuf, sizeof(txBuf));
   }
   
   // Отчет о статусе сохранения имени - сохранение завершено
   txBuf[2] = 0x2;                           
   can1_send_packet(CAN_LDR_TX_BUF, CANID_LDR_ANSWER, txBuf, sizeof(txBuf));
      
   loader->id = newId;                          // смена ID
}

// Отправить параметры памяти самозагрузчика
__ramfunc void send_loader_mem_param(Loader_TypeDef *loader)
{    
    uint8_t txBuf[7];  
    uint32_t page_size = loader->memPageSize;
    uint16_t pages_cnt = loader->memPagesCnt;

    uint16_t virtualPageSize = VIRTUAL_PAGE_SIZE;                               // размер виртуальной страницы
    uint16_t virtualPageCnt = DIV_ON_VIRTUAL_PAGE_SIZE(page_size)*pages_cnt;    // количество виртуальных страниц

    txBuf[0] = loader->id;               // идентификатор устройства
    txBuf[1] = CANLOADER_MSG_MEM_PARAM;  // тип сообщения - ID
    txBuf[2] = 0x01;                     // зарезервировано
    txBuf[3] = virtualPageSize>>8;       // размер страницы, старший байт
    txBuf[4] = (uint8_t)virtualPageSize; // младший байт
    txBuf[5] = virtualPageCnt>>8;        // кол-во страниц, старший байт
    txBuf[6] = (uint8_t)virtualPageCnt;  // младший байт
    
    can1_send_packet(CAN_LDR_TX_BUF, CANID_LDR_ANSWER, txBuf, sizeof(txBuf));    
}

// Загрузить страницу в буфер для последующей записи
__ramfunc void load_software_page(Loader_TypeDef *loader, uint8_t *pData, uint8_t len)
{ 
   if(loader->state != CANLOADER_STATE_ACTIVE) return;
    
   uint8_t i = 2;
   while(i<len)
   {
      softwarePageBuf[memOffset++] = pData[i++];
   }
}

