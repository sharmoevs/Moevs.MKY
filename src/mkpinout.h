#ifndef __MKPINOUT_H__
#define __MKPINOUT_H__

#include "MDR1986VE1T.h"


//сделать управление датчиком угла

#define AS_ROUGH_ENABLE                 (MDR_PORTD->CLRTX = 1<<1)
#define AS_ROUGH_DISABLE                (MDR_PORTD->SETTX = 1<<1)
#define AS_ACCURATE_ENABLE              (MDR_PORTD->CLRTX = 1<<0)
#define AS_ACCURATE_DISABLE             (MDR_PORTD->SETTX = 1<<0)
// Выбор микросхемы
#define AS_SELECT_ROUGH                 AS_ACCURATE_DISABLE;\
                                        AS_ROUGH_ENABLE                         // выбор грубого
#define AS_SELECT_ACCURATE              AS_ROUGH_DISABLE;\
                                        AS_ACCURATE_ENABLE                      // выбор точного
                                           
#define AS_BYTE_SELECT_HI               (MDR_PORTC->SETTX = 1<<0)               // выбор старшего байта
#define AS_BYTE_SELECT_LO               (MDR_PORTC->CLRTX = 1<<0)               // выбор младшего байта
                                          
#define AS_INHIBIT_HI                   (MDR_PORTC->SETTX = 1<<13)              // защелка выхода
#define AS_INHIBIT_LO                   (MDR_PORTC->CLRTX = 1<<13)       
                                           
#define AS_DATA                         ((MDR_PORTF->RXTX>>5)&0xFF)             // данные с платы
                                         

#define IS_ENGINE_ENABLE                ((MDR_PORTB->RXTX>>7)&1)                // разрешено управлять двигателями



#endif //__MKPINOUT_H__