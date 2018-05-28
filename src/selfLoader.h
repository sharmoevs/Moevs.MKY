#ifndef __SELF_LOADER__
#define __SELF_LOADER__

#include "MDR1986VE1T.h"
#include "flash1986ve1t.h"
#include "loaders.h"


#define MK_FLASH_START_ADDR             FLASH1986VE1T_START_ADDR                
#define MK_FLASH_PAGE_SIZE              FLASH1986VE1T_PAGE_SIZE
#define MK_FLASH_PAGES_CNT         FLASH1986VE1T_PAGES_CNT



__ramfunc void selfloader_erase_mem(Loader_TypeDef *loader, uint16_t pagesCnt);
__ramfunc void selfloader_write_flash_page(Loader_TypeDef *loader, uint16_t numOfPage);
__ramfunc void selfloader_read_flash_page(Loader_TypeDef *loader, uint16_t numOfPage);






#endif //__SELF_LOADER__