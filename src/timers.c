#include "timers.h"
#include "global.h"


// Включить/Выключить таймер
void timer_setEnable(MDR_TIMER_TypeDef *timer, uint8_t enable)
{
    if(enable) timer->CNTRL |= TIMER_CNTRL_CNT_EN;
    else timer->CNTRL &= ~(TIMER_CNTRL_CNT_EN);
}

/*// Инициализация Timer1
void timer1_init(uint16_t ms)
{
    MDR_RST_CLK->PER_CLOCK |= 1<<14;                              // тактирование TIMER1    
    MDR_RST_CLK->TIM_CLOCK |= 0x2<<0 |                            // HCLCK/4 - такктирование от 31.25MHz при F=125 (30МГц при F=120)
                              1<<24;                              // разрешение тактирование таймера
    MDR_TIMER1->CNT = 0;    
#ifdef MOEVS_BOARD
    MDR_TIMER1->PSG = 30000-1;                                    // 1000 kHz
#else
    MDR_TIMER1->PSG = 31250-1;                                    // 1000 kHz    
#endif//MOEVS_BOARD    
    MDR_TIMER1->ARR = ms;                                         // раз в msc миллисекунд при f=125MHz    
    MDR_TIMER1->IE |= TIMER_IE_CNT_ARR_EVENT_IE;  
    NVIC_EnableIRQ(TIMER1_IRQn);
}
*/

// Инициализация таймера в режиме ШИМ
void timer1_pwm_init()
{
    MDR_RST_CLK->PER_CLOCK |= 1<<14;                              // тактирование TIMER2    
    MDR_RST_CLK->TIM_CLOCK |= 0x2<<0 |                            // HCLCK/4 - такктирование от 31.25MHz при F=125 (30МГц при F=120)
                              1<<24;                              // разрешение тактирование таймера

    MDR_TIMER1->CH3_CNTRL = 0<<0 |      // канал работает в режиме ШИМ
                            6<<9;       // 1, если DIR= 0 (счет вверх), CNT<CCR, иначе 0
    MDR_TIMER1->CH3_CNTRL1 = 1<<0 |     // всегда на OE выдается 1, канал всегда работает на выход
                             2<<2;      // на выход выдается сигнал REF    
    MDR_TIMER1->CNT = 0;
    //MDR_TIMER1->ARR = 2000-1;
    MDR_TIMER1->ARR = TIM_PWM_ARR_REG;
    

    MDR_TIMER1->CNTRL |= 1<<0;         // запустить таймер
}
// Установить заполнение ШИМ
void setPwmFilling(uint8_t percentages)
{
   uint32_t value = percentages * 20;
   MDR_TIMER1->CCR3 = value;
}
// Установить заполнение ШИМ
void setPwmFillingF(float percentages)
{
   uint32_t value = (uint32_t)(percentages * 20);
   MDR_TIMER1->CCR3 = value;
}
// Установить заполнение ШИМ
void setPwmFillingF_2(uint16_t upr)
{
   uint16_t ccr3;
   //if(upr > 1200)  ccr3 = 1200;
   if(upr > TIM_PWM_60_PERCENTAGE_CCR3_REG) ccr3 = TIM_PWM_60_PERCENTAGE_CCR3_REG;   
   else ccr3 = upr;
   
   uint16_t tmp = TIM_PWM_60_PERCENTAGE_CCR3_REG;
   tmp++;
   
   MDR_TIMER1->CCR3 = ccr3;
}

// Возвращает текущее заполнение ШИМ
uint8_t getPwmFilling()
{
  uint32_t value = MDR_TIMER1->CCR3;
  return value/20;
}





// ================================== Delay ====================================

// Задержка в итерациях. При HCLK=80МГц, Т=12.5нс
__ramfunc void delay_tics(uint32_t tics)
{ 
   for(volatile uint32_t i=0; i<tics; i++);
}

// Задержка на mcs микросекунд для CLK=120МГц!
__ramfunc void delay_us(uint32_t us)
{    
#define TICK 10   // 1 такт - 8нс
   uint32_t tics = (us*100);
   tics >>= 4;   // delay_tics - задержка не в тактах, а в итерациях, поэтому вычисляется дольше
   delay_tics(tics);
}

// Задержка на ms миллисекунд
__ramfunc void delay_ms(uint32_t ms)
{
   uint32_t startDelayTime = system_time;
   while((system_time-startDelayTime) < ms);
}



// Миллиссекундная и микросекундная задержка на основе таймера общего назначеня
// Инициализация Timer3. Т = 1 мкс
void delayTimer_init()
{
   // HCLCK = 125.000.000 Гц / 120.000.000 Гц

   MDR_RST_CLK->PER_CLOCK |= 1<<16;                              // тактирование TIMER2
   MDR_RST_CLK->TIM_CLOCK |= 0x0<<16 |                           // HCLCK/1 - такктирование от 125MHz / 120МГц
                             1<<26;                              // разрешение тактирование таймера

   MDR_TIMER3->CNT = 0;
#ifdef MOEVS_BOARD  // 120МГц
   MDR_TIMER3->PSG = 120-1;                                      // 1000000 kHz   
#else // 125МГц
   MDR_TIMER3->PSG = 125-1;                                      // 1000000 kHz
#endif//MOEVS_BOARD
   MDR_TIMER3->ARR = 0xFFFFFFFF;                                 // раз в msc миллисекунд при f=125MHz / 120МГц

   timer_setEnable(MDR_TIMER3, 1); 
}



// Инициализация Timer4 - ДУС и системное время
void dus_timer4_init(uint32_t us)
{
    MDR_RST_CLK->PER_CLOCK |= 1<<19;                              // тактирование TIMER4
      MDR_TIMER4->CNT = 0;
#ifdef MOEVS_BOARD
    MDR_RST_CLK->UART_CLOCK |= 0x2<<16 |                          // HCLCK/4 - такктирование от 30MHz (при F=120)
                               1<<26;                             // разрешение тактирование таймера TIMER4
    MDR_TIMER4->PSG = 30-1;                                       // 1  MHz
#else
    MDR_RST_CLK->UART_CLOCK |= 0x0<<16 |                          // HCLCK/1 - такктирование от 125MHz
                               1<<26;                             // разрешение тактирование таймера TIMER4
    MDR_TIMER4->PSG = 125-1;                                      // 1  MHz
#endif//MOEVS_BOARD    
    MDR_TIMER4->ARR = us;                                         // раз в us миллисекунд при f=120MHz
    MDR_TIMER4->IE |= TIMER_IE_CNT_ARR_EVENT_IE;  
    NVIC_EnableIRQ(TIMER4_IRQn);
}


// Инициализация TIMER_2 - Управление ГК и ПИД-регулятор
void gk_timer2_init(uint32_t us)
{      
    MDR_RST_CLK->PER_CLOCK |= 1<<15;                              // тактирование TIMER2
    MDR_RST_CLK->TIM_CLOCK |= 0x2<<8 |                            // HCLCK/4 - такктирование от 31.25MHz при F=125 (30МГц при F=120)
                              1<<25;                              // разрешение тактирование таймера
  
    MDR_TIMER2->CNT = 0;
#ifdef MOEVS_BOARD
    MDR_TIMER2->PSG = 30-1;                                       // 1 MHz
#else 
    MDR_TIMER2->PSG = 31250-1;                                    // 1000 Hz    
#endif 
    MDR_TIMER2->ARR = us;                                         // T = 1мс при f=120MHz      
    MDR_TIMER2->IE |= TIMER_IE_CNT_ARR_EVENT_IE;
    //NVIC_SetPriority(TIMER2_IRQn, 1);                             // снижение приоритета на 1
    NVIC_EnableIRQ(TIMER2_IRQn);
}






// Возвращает 1, если прошло delay_ms от момента *var, относительно системного времени
__ramfunc uint8_t elapsed(uint32_t *var, const uint32_t delay_ms) 
{
   if ((system_time - *var) >= delay_ms) 
   {
      *var = system_time; // сброс
      return 1;
   }
   else return 0;
}

// Сброс программного таймера
void resetProgramTimer(uint32_t *var)
{
   *var = system_time;
}






