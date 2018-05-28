#ifndef __TIMERS__
#define __TIMERS__

#include "MDR1986VE1T.h"


//void timer1_init(uint16_t ms);
void timer1_pwm_init();
void setPwmFilling(uint8_t percentages);
void setPwmFillingF(float percentages);
void setPwmFillingF_2(uint16_t upr);
uint8_t getPwmFilling();
void delayTimer_init();
void timer_setEnable(MDR_TIMER_TypeDef *timer, uint8_t enable);

void dus_timer4_init(uint32_t us);
void gk_timer2_init(uint32_t us);

__ramfunc void delay_tics(uint32_t tics);
__ramfunc void delay_us(uint32_t us);
__ramfunc void delay_ms(uint32_t ms);
__ramfunc uint8_t elapsed(uint32_t *var, const uint32_t delay_ms);
void resetProgramTimer(uint32_t *var);




#endif //__TIMERS__