#ifndef __FLASH_1986VE9x_H__
#define __FLASH_1986VE9x_H__

#include "MDR1986VE1T.h"

#define FLASH1986VE1T_START_ADDR                0x00000000
#define FLASH1986VE1T_PAGE_SIZE                 0x1000
#define FLASH1986VE1T_PAGES_CNT                 32
#define GET_FLASH_PAGE_ADDR(pageNum)            (FLASH1986VE1T_START_ADDR + (pageNum<<12))


// ключ для работы с EEPROM
#define EEPROM_KEY                      0x8AAA5551  

// Битовые поля регистра EEPROM_CMD
#define CON                             0
#define WR                              1
#define RD                              2
#define DELAY                           3
#define XE                              6
#define YE                              7
#define SE                              8
#define IFREN                           9
#define ERASE                           10
#define MAS1                            11
#define PROG                            12
#define NVSTR                           13



__ramfunc void erase_flash_page(uint32_t addr);
__ramfunc void flash_write_word(uint32_t addr, uint32_t data);

// Работа с информационной памятью
__ramfunc void flash_info_write_word(uint32_t addr, uint32_t data);
__ramfunc uint32_t flash_info_read_word(uint32_t addr);
__ramfunc void flash_info_erase_sector(uint32_t addr);


#endif //__FLASH_1986VE9x_H__