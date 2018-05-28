#include "selfLoader.h"
#include "can.h"


extern uint8_t softwarePageBuf[VIRTUAL_PAGE_SIZE]; 


// Стереть память самозагрузчика
__ramfunc void selfloader_erase_mem(Loader_TypeDef *loader, uint16_t virtualPagesCnt)
{     
    if(loader->state != CANLOADER_STATE_ACTIVE) return;        
    
    uint16_t virtPagesInPage = DIV_ON_VIRTUAL_PAGE_SIZE(loader->memPageSize);      // количество виртуальных страниц в реальной 
    uint16_t virtualPagesMax = virtPagesInPage*(loader->memPagesCnt);
    if(virtualPagesCnt > virtualPagesMax) virtualPagesCnt = virtualPagesMax;
    
    uint16_t pagesToErase = 0;                // количество реальных страниц, которые нужно стереть    
    uint16_t tmp = 0;
    while(tmp < virtualPagesCnt)
    {  
        pagesToErase++;
        tmp += virtPagesInPage;
    }    
    
    uint8_t txBuf[4];
    txBuf[0] = loader->id;
    txBuf[1] = CANLOADER_MSG_ERASE_MEM;
    txBuf[2] = 0x1;                       // стирание осуществляется
    txBuf[3] = 0;                         // процент стиранияы
    
    uint32_t pageAddr = MK_FLASH_START_ADDR;       // адрес нулевой страницы
    uint32_t pageSize = MK_FLASH_PAGE_SIZE;        // размер страницы    
    
    for(uint8_t i=0; i<pagesToErase; i++)
    {
       erase_flash_page(pageAddr);
       pageAddr += pageSize;              // переходим к следующей странице
       txBuf[3]+=3;                       // процент выполнения операции
       
       can1_send_packet(CAN_LDR_TX_BUF, CANID_LDR_ANSWER, txBuf, sizeof(txBuf));
    }

    txBuf[2] = 0x2;                       // стирание завершено
    can1_send_packet(CAN_LDR_TX_BUF, CANID_LDR_ANSWER, txBuf, 3);       
}


// Записать страницу в память
__ramfunc void selfloader_write_flash_page(Loader_TypeDef *loader, uint16_t numOfPage)
{
   if(loader->state != CANLOADER_STATE_ACTIVE) return;
   
   uint32_t addr = numOfPage*VIRTUAL_PAGE_SIZE;        
   uint32_t *pData = (uint32_t*)softwarePageBuf;
   uint16_t words = VIRTUAL_PAGE_SIZE>>2;               // кол-во записываемых слов
   
   for(int i=0; i<words; i++)
   {
      flash_write_word(addr, *pData);
      addr+=4;
      pData++;
   }
   
   // отчет о записи 
   uint8_t txBuf[3];
   txBuf[0] = loader->id;
   txBuf[1] = CANLOADER_MSG_WRITE_PAGE;
   txBuf[2] = 0x2;                       // запись завершена
   can1_send_packet(CAN_LDR_TX_BUF, CANID_LDR_ANSWER, txBuf, sizeof(txBuf));
}



// Считать страницу памяти для верификации
__ramfunc void selfloader_read_flash_page(Loader_TypeDef *loader, uint16_t numOfPage)
{
   if(loader->state != CANLOADER_STATE_ACTIVE) return;
        
   uint16_t page_size = loader->memPageSize;
   uint16_t pages_cnt = loader->memPagesCnt;
   uint16_t virtualPageCnt = DIV_ON_VIRTUAL_PAGE_SIZE(page_size)*pages_cnt;    // количество виртуальных страниц
     
   
   uint8_t txBuf[8];
   txBuf[0] = loader->id;
   txBuf[1] = CANLOADER_MSG_READ_PAGE;
 
   uint32_t offsetMem = numOfPage*VIRTUAL_PAGE_SIZE;
   uint32_t sizeMem = virtualPageCnt*VIRTUAL_PAGE_SIZE;
   if((offsetMem + VIRTUAL_PAGE_SIZE)>sizeMem) return;
   
   uint8_t *pMemory = (uint8_t*)MK_FLASH_START_ADDR;
   uint16_t needTxBytes = VIRTUAL_PAGE_SIZE;
   
   
   while(needTxBytes)
   {
     uint8_t freeBytesInMsg = 6;
     uint8_t indexMsg = 2;
     
     while(freeBytesInMsg&&needTxBytes)
     {
        txBuf[indexMsg++] = pMemory[offsetMem++];
        --freeBytesInMsg;
        --needTxBytes;
     }
     can1_send_packet(CAN_LDR_TX_BUF, CANID_LDR_ANSWER, txBuf, indexMsg);
   }
   
   // отчет о завершении передачи страницы
   can1_send_packet(CAN_LDR_TX_BUF, CANID_LDR_ANSWER, txBuf, 2);
}