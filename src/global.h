#ifndef __GLOBAL_H__
#define __GLOBAL_H__

#include "MDR1986VE1T.h"
#include "settings.h"



// Системные часы
extern volatile uint32_t system_time;


// ============ Отладка ===============
// Секундомер 
#define __START_STOPWATCH            MDR_TIMER3->PSG = 120-1;\
                                     MDR_TIMER3->ARR = 0xFFFFFFFF;\
                                     MDR_TIMER3->CNT = 0;\
                                     uint32_t ___startTime = MDR_TIMER3->CNT;
#define __STOP_STOPWATCH             uint32_t ___endTime = MDR_TIMER3->CNT;
#define __STOPWATCH_ELAPSED          (___endTime - ___startTime)
// ====================================

                                     
                                    
#define MAX(x,y)                     ((x>y) ? x : y)
#define MIN(x,y)                     ((x<y) ? x : y)
#define DIFFERENCE_MODULE(x,y)       ((x>y) ? (x-y) : (y-x))
#define ABS(x)                       (x>=0 ? x : -x)


                                     
                                     
                                     

uint32_t getBitsInWord(uint32_t word);
/*
uint32_t __DISABLE_IRQ();
void     __ENABLE_IRQ(uint32_t iser);
  */                                
                                   






#endif //__GLOBAL_H__