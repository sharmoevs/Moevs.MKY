#ifndef __ENGENE_CONTROL_H__
#define __ENGENE_CONTROL_H__

#include "MDR1986VE1T.h"


// Управление ключами
#define AH_HI()                         (MDR_PORTE->SETTX = 1<<10)
#define AH_LO()                         (MDR_PORTE->CLRTX = 1<<10)
                                          
#define AL_HI()                         (MDR_PORTE->SETTX = 1<<11)
#define AL_LO()                         (MDR_PORTE->CLRTX = 1<<11)
                                          
#define BH_HI()                         (MDR_PORTE->SETTX = 1<<12)
#define BH_LO()                         (MDR_PORTE->CLRTX = 1<<12)
                                          
#define BL_HI()                         (MDR_PORTE->SETTX = 1<<13)
#define BL_LO()                         (MDR_PORTE->CLRTX = 1<<13)

#define CH_HI()                         (MDR_PORTE->SETTX = 1<<14)
#define CH_LO()                         (MDR_PORTE->CLRTX = 1<<14)
          
#define CL_HI()                         (MDR_PORTE->SETTX = 1<<15)
#define CL_LO()                         (MDR_PORTE->CLRTX = 1<<15)

#define GK_PHASE_SWITCH_DELAY_us        25              // задержка между переключениями фаз, мкс




void engine_disableSpin();   // выключить двигатели
void engine_spinCCW();       // вращение против часовой стрелке
void engine_spinCW();        // вращение по часовой стрелке



                
#endif //__ENGENE_CONTROL_H__
