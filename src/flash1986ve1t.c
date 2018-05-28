#include "flash1986ve1t.h"
#include "timers.h"


// Стирание одной страницы основной флешь-памяти
__ramfunc void erase_flash_page(uint32_t addr)
{  
   uint32_t enable_irq = NVIC->ISER;    // сохранение разрешенных прерываний 
   NVIC->ICER = 0xFFFFFFFF;             // запрет всех прерываний
         
   MDR_EEPROM->KEY = EEPROM_KEY;   
   MDR_EEPROM->CMD |= 1<<CON;           // переключаемся в режим программирования
   MDR_EEPROM->CMD &= ~(1<<IFREN);      // выбор памяти (1 - для информационной, 0 - для основной)
   
   MDR_EEPROM->ADR = addr;
   
   for(uint8_t i=0; i<4; i++)           // стирание всей памяти для секторов A,B,C,D
   {
      MDR_EEPROM->ADR &= ~(0x3<<2);     // сброс номера сектора
      MDR_EEPROM->ADR |= i<<2;          // номер сектора (0-A, 1-B, 2-C, 3-D)
      
      MDR_EEPROM->CMD |= 1<<XE | 1<<ERASE;      
      delay_us(5);                      // задержка на 5мкс      
      MDR_EEPROM->CMD |= 1<<NVSTR;      
      delay_us(40*1000);                     // задержка на 40мс - полное стирание памяти
      MDR_EEPROM->CMD &= ~(1<<ERASE);   
      delay_us(5);                      // задержка на 5мкс      
      MDR_EEPROM->CMD &= ~(1<<XE | 1<<NVSTR);      
      delay_us(1);                      // задержка на 1мкс    
   }
      
   MDR_EEPROM->CMD &= ~(1<<CON);        // выход из режима программирования   
   MDR_EEPROM->KEY = 0;
      
   NVIC->ISER = enable_irq;             // разрешение прерываний
}

// Записать 32б слово data по адресу addr в основную флеш-память
__ramfunc void flash_write_word(uint32_t addr, uint32_t data)
{  
   uint32_t enable_irq = NVIC->ISER;           // сохранение разрешенных прерываний 
   NVIC->ICER  = 0xFFFFFFFF;                   // запрет всех прерываний
         
   MDR_EEPROM->KEY = EEPROM_KEY;   
   MDR_EEPROM->CMD |= 1<<CON;                  // переключаемся в режим программирования
   MDR_EEPROM->CMD &= ~(1<<IFREN);             // основная память  
   
   MDR_EEPROM->ADR = addr;
   MDR_EEPROM->DI = data;
   
   MDR_EEPROM->CMD |= 1<<XE | 1<<PROG;
   delay_us(5);                                // задержка на 5мкс    
   MDR_EEPROM->CMD |= 1<<NVSTR;       
   delay_us(10);                               // задержка на 10мкс    
   MDR_EEPROM->CMD |= 1<<YE;      
   delay_us(40);                               // задержка на 40мкс  Запись в память
   MDR_EEPROM->CMD &= ~(1<<YE);      
   delay_us(1);                                // задержка на 20нс    
   MDR_EEPROM->CMD &= ~(1<<PROG);      
   delay_us(5);                                // задержка на 5мкс    
   MDR_EEPROM->CMD &= ~(1<<XE | 1<<NVSTR);      
   delay_us(1);                                // задержка на 1мкс    
      
   MDR_EEPROM->CMD &= ~(1<<CON | 1<<IFREN);    // выход из режима программирования                          
   MDR_EEPROM->KEY = 0;
      
   NVIC->ISER = enable_irq;                    // разрешение прерываний
}



/*
// Записать массив из cnt слов в основную флеш-память по адресу addr
__ramfunc void flash_write_array(uint32_t addr, uint32_t *pData, uint16_t cnt)
{  
   uint32_t enable_irq = NVIC->ISER[0];         // сохранение разрешенных прерываний 
   NVIC->ICER[0]  = 0xFFFFFFFF;                 // запрет всех прерываний
   
   MDR_EEPROM->KEY = EEPROM_KEY;   
   MDR_EEPROM->CMD |= 1<<CON;                   // переключаемся в режим программирования
   MDR_EEPROM->CMD &= ~(1<<IFREN);              // основная память  
   
   MDR_EEPROM->ADR = addr;
   //MDR_EEPROM->DI = *pData++;
   //addr+=4;
   
   MDR_EEPROM->CMD |= 1<<XE | 1<<PROG;
   delay_mcs(5);                                // задержка на 5мкс    
   MDR_EEPROM->CMD |= 1<<NVSTR;       
   delay_mcs(10);                               // задержка на 10мкс    
   //MDR_EEPROM->CMD |= 1<<YE;      
   //delay_mcs(40);                               // задержка на 40мкс  Запись в память
   //MDR_EEPROM->CMD &= ~(1<<YE);    
   //delay_mcs(1);                                // задержка на 20нс      
   
   for(uint16_t i=0; i<cnt-1; i++)
   {
      MDR_EEPROM->ADR = addr;
      MDR_EEPROM->DI = *pData++;
      addr+=4;
      
      MDR_EEPROM->CMD |= 1<<YE;      
      delay_mcs(40);                               // задержка на 40мкс  Запись в память
      MDR_EEPROM->CMD &= ~(1<<YE);        
      delay_mcs(1);                                // задержка на 20нс 
   }
   
   MDR_EEPROM->CMD &= ~(1<<PROG);      
   delay_mcs(5);                                // задержка на 5мкс    
   MDR_EEPROM->CMD &= ~(1<<XE | 1<<NVSTR);      
   delay_mcs(1);                                // задержка на 1мкс    
      
   MDR_EEPROM->CMD &= ~(1<<CON | 1<<IFREN);     // выход из режима программирования                          
   MDR_EEPROM->KEY = 0;
      
   NVIC->ISER[0] = enable_irq;          // разрешение прерываний
}
*/


// =============================================================================
// =================== Работа с информационной флеш-памятью ====================
// =============================================================================

// Записать 32б слово data по адресу addr в информационную флеш-память
__ramfunc void flash_info_write_word(uint32_t addr, uint32_t data)
{
   uint32_t enable_irq = NVIC->ISER;           // сохранение разрешенных прерываний 
   NVIC->ICER  = 0xFFFFFFFF;                   // запрет всех прерываний
   
   MDR_EEPROM->KEY = EEPROM_KEY;   
   MDR_EEPROM->CMD |= 1<<CON;                  // переключаемся в режим программирования
   MDR_EEPROM->CMD |= 1<<IFREN;                // информационная память
   
   MDR_EEPROM->ADR = addr;
   MDR_EEPROM->DI = data;
   
   MDR_EEPROM->CMD |= 1<<XE | 1<<PROG;
   delay_us(5);                                // задержка на 5мкс
   MDR_EEPROM->CMD |= 1<<NVSTR;
   delay_us(10);                               // задержка на 10мкс
   MDR_EEPROM->CMD |= 1<<YE;
   delay_us(40);                               // задержка на 40мкс  Запись в память
   MDR_EEPROM->CMD &= ~(1<<YE);
   delay_us(20);                               // задержка на 20нс
   MDR_EEPROM->CMD &= ~(1<<PROG);
   delay_us(5);                                // задержка на 5мкс
   MDR_EEPROM->CMD &= ~(1<<XE | 1<<NVSTR);
   delay_us(1);                                // задержка на 1мкс
   MDR_EEPROM->CMD &= ~(1<<CON | 1<<IFREN);    // выход из режима программирования
   MDR_EEPROM->KEY = 0;
   
   NVIC->ISER = enable_irq;                    // разрешение прерываний
}

// Чтение одного 32б слова из информационной памяти 
__ramfunc uint32_t flash_info_read_word(uint32_t addr)
{   
   uint32_t enable_irq = NVIC->ISER;            // сохранение разрешенных прерываний 
   NVIC->ICER  = 0xFFFFFFFF;                    // запрет всех прерываний
   
   MDR_EEPROM->KEY = EEPROM_KEY;   
   MDR_EEPROM->CMD |= 1<<CON;                   // переключаемся в режим программирования
   MDR_EEPROM->CMD |= 1<<IFREN;                 // информационная память   
   MDR_EEPROM->ADR = addr;      
   MDR_EEPROM->CMD |= (1<<XE | 1<<YE | 1<<SE);
   delay_us(1);                                 // задержка на 30нс      
   uint32_t data = MDR_EEPROM->DO;    
   MDR_EEPROM->CMD &= ~(1<<XE | 1<<YE | 1<<SE | 1<<CON | 1<<IFREN);   
   MDR_EEPROM->KEY = 0;
   
   NVIC->ISER = enable_irq;                     // разрешение прерываний
   return data;
}

// Стирание одного сектора одной страницы информационной флеш-памяти
__ramfunc void flash_info_erase_sector(uint32_t addr)
{    
   uint32_t enable_irq = NVIC->ISER;            // сохранение разрешенных прерываний 
   NVIC->ICER  = 0xFFFFFFFF;                    // запрет всех прерываний
   
   // Определение сектора
   uint8_t page_addr = (addr>>12)&0xFF;         // номер стираемой страницы 0xaaaA_Aaaa
   uint8_t tmp8 = (uint8_t)addr&0xF;            // младшие 4 бита 
   uint8_t sector;   
   switch(tmp8)
   {
      case 0x0: sector = 0; break;
      case 0x4: sector = 1; break;
      case 0x8: sector = 2; break;
      case 0xC: sector = 3; break;
      default : return;
   }      
   MDR_EEPROM->KEY = EEPROM_KEY;
   MDR_EEPROM->CMD |= 1<<CON;                   // переключаемся в режим программирования      
   MDR_EEPROM->CMD |= 1<<IFREN;                 // информационная память   
   MDR_EEPROM->ADR = page_addr<<12 | sector<<2; // установить номер страницы и сектора
   MDR_EEPROM->CMD |= 1<<XE | 1<<ERASE;                                 
   delay_us(5);                                 // задержка на 5мкс       
   MDR_EEPROM->CMD |= 1<<NVSTR;      
   delay_us(40000);                             // задержка на 40мс - стирание одной страницы памяти
   MDR_EEPROM->CMD &= ~(1<<ERASE);   
   delay_us(5);                                 // задержка на 5мкс    
   MDR_EEPROM->CMD &= ~(1<<XE | 1<<NVSTR);                              
   delay_us(1);                                 // задержка на 1мкс           
   MDR_EEPROM->CMD &= ~(1<<CON);                // выход из режима программирования   
   MDR_EEPROM->KEY = 0;
     
   NVIC->ISER = enable_irq;                     // разрешение прерываний
}

