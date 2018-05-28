#include "MDR1986VE1T.h"
#include "settings.h"
#include "timers.h"
#include "can.h"
#include "arinc.h"
#include "adc.h"
#include "global.h"
#include "mkpinout.h"
#include "engineControl.h"
#include "dac.h"
#include "canMonitor.h"
#include "canMonitorText.h"
#include "GkControlMode.h"

void rcc_init();
void ports_init();
void initGKControlLines();

void SystemInit()
{
   rcc_init();                         // настройка тактирования ядра
   delayTimer_init();                  // инициализация таймера для задержек (мкс, мс)
 
   ports_init();                       // инициализация портов
   can1_init();                        // инициализация can1
   arinc_init();                       // инициализация arinc
   dac_init();                         // инициализация ЦАП
   adc_init();                         // инициализация АЦП
   
   timer1_pwm_init();                  // таймер для формирования ШИМ
   dus_timer4_init(DUS_SAMPLING_TIMER_TIMEOUT_mcs);     // системный таймер и ДУС
   //gk_timer2_init(1000);               // таймер управления ГК
}




// Инициализация тактирования
void rcc_init()
{
#ifdef MOEVS_BOARD // ПЛАТА МОЭВС
    MDR_RST_CLK->TIM_CLOCK = 0;         // сброс предделителей для таймеров
    MDR_RST_CLK->CPU_CLOCK = 0;
    MDR_EEPROM->CMD = 0x4<<3;                                                   // 4 такта паузы при работе на частотах до 150МГц
    
    MDR_RST_CLK->HS_CONTROL = 0<<1 |                                            // Режим внешнего осцилятора
                              RST_CLK_HS_CONTROL_HSE_ON;                        // Включить внешний генератор
    while(!(MDR_RST_CLK->CLOCK_STATUS & RST_CLK_CLOCK_STATUS_HSE_RDY));         // Ждем 
    MDR_RST_CLK->PLL_CONTROL = CPU_PLL_KOEF<<8 |                                // CPU PLL
                               1<<2;                                            // CPU PLL On
    while(!(MDR_RST_CLK->CLOCK_STATUS & RST_CLK_CLOCK_STATUS_PLL_CPU_RDY));     // waiting cpu pll ready
    MDR_RST_CLK->CPU_CLOCK |= 3<<0 |                                            // CPU_C1 = HSE/2
                              1<<2 |                                            // CPU_C2 = PLLCPUo
                              0<<7 |                                            // CPU_C3 = CPU_C2
                              1<<8;                                             // HCLK = CPU_C3

    MDR_RST_CLK->PER_CLOCK = 1<<3 |   // EEPROM_CNTRL
                             1<<4;    // RST_CLK
#else  // ОТЛАДОЧНАЯ ПЛАТА  
    MDR_RST_CLK->TIM_CLOCK = 0;         // сброс предделителей для таймеров
    MDR_RST_CLK->CPU_CLOCK = 0;  
    MDR_EEPROM->CMD = 0x5<<3;                                                   // 5 такта паузы при работе на частотах до 150МГц
  
    MDR_RST_CLK->HS_CONTROL = RST_CLK_HS_CONTROL_HSE_BYP |                      // Режим внешнего генератора
                              RST_CLK_HS_CONTROL_HSE_ON;                        // Включить внешний генератор
    while(!(MDR_RST_CLK->CLOCK_STATUS & RST_CLK_CLOCK_STATUS_HSE_RDY));         // Ждем 
    MDR_RST_CLK->PLL_CONTROL = CPU_PLL_KOEF<<8 |                                // CPU PLL         
                               1<<2;                                            // CPU PLL On
    while(!(MDR_RST_CLK->CLOCK_STATUS & RST_CLK_CLOCK_STATUS_PLL_CPU_RDY));     // waiting cpu pll ready
    MDR_RST_CLK->CPU_CLOCK |= 3<<0 |                                            // CPU_C1 = HSE/2
                              1<<2 |                                            // CPU_C2 = PLLCPUo
                              0<<7 |                                            // CPU_C3 = CPU_C2
                              1<<8;                                             // HCLK = CPU_C3

    MDR_RST_CLK->PER_CLOCK = 1<<3 |   // EEPROM_CNTRL
                             1<<4;    // RST_CLK    
#endif //MOEVS_BOARD
}





// Инициализация портов
void ports_init()
{
#ifdef MOEVS_BOARD // ПЛАТА МОЭВС
    MDR_RST_CLK->PER_CLOCK |= //1<<21 |                  // PORTA
                              1<<22 |                  // PORTB
                              1<<23 |                  // PORTC
                              1<<24 |                  // PORTD
                              1<<25 |                  // PORTE
                              1<<29;                   // PORTF

    // can1, защелка выхода датчика угла, выбор ст/мл байта
    MDR_PORTC->FUNC = 0x0<<0 |        // выбор байта д. угла
                      0x3<<18 |       // can1 rx
                      0x3<<20 |       // can1 tx
                      0x0<<26;        // port - защелка выхода
    MDR_PORTC->OE = 1<<0 |      // выбор байта угла
                    0<<9 |      // can1 rx
                    1<<10 |     // can1 tx
                    1<<13;      // защелка выхода
    MDR_PORTC->ANALOG = 1<<0 | 1<<9 | 1<<10 | 1<<13;
    MDR_PORTC->PWR = 0x3<<0 | 0x3<<18 | 0x3<<20 | 0x3<<26;
    
    // arinc-429 in
    MDR_PORTB->FUNC = 0x2<<0 |          // arinc DI0+
                      0x2<<2 |          // arinc DI0-
                      0x0<<14;          // NSETUP - работа двигателей
    MDR_PORTB->OE       = 0<<0 | 0<<1 | 0<<7;
    MDR_PORTB->ANALOG   = 1<<0 | 1<<1 | 1<<7;
    MDR_PORTB->PWR  = 0x3<<0 | 0x3<<2 | 0x2<<14;
    
    // arinc-429 out
    MDR_PORTF->FUNC = 0x1<<26 |         // arinc OUT2+
                      0x1<<28;          // arinc OUT2-
    MDR_PORTF->OE       = 1<<13 | 1<<14 |    // arinc OUT2+, OUT2-
                          0<<5 | 0<<6 | 0<<7 | 0<<8 | 0<<9 | 0<<10 | 0<<11 | 0<<12; // данные датчика угла                         
    MDR_PORTF->ANALOG   = 1<<13 | 1<<14 |    // arinc OUT2+, OUT2-                                 
                          1<<5 | 1<<6 | 1<<7 | 1<<8 | 1<<9 | 1<<10 | 1<<11 | 1<<12; // данные датчика угла
    MDR_PORTF->PWR  = 0x3<<26 | 0x3<<28 |    // arinc OUT2+, OUT2-   
                      0x3<<10 | 0x3<<12 | 0x3<<14 | 0x3<<16 | 0x3<<18 | 0x3<<20 | 0x3<<22 | 0x3<<24;// данные датчика угла
    
    // АЦП
    MDR_PORTD->OE = 1<<0 |              // датчик угла - точно
                    1<<1 |              // датчик угла - грубо
                    0<<9 | 
                    0<<10 | 
                    0<<12 | 
                    0<<14;        // вход АЦП
    MDR_PORTD->ANALOG = 0<<9 | 0<<10 | 0<<12 | 0<<14 |  // аналоговый
                        1<<0 | 1<<1;    // точность датчика угла
    MDR_PORTD->PWR = 0x3<<0 | 0x3<<2;
    
    // TIM_PWM, ЦАП
    MDR_PORTE->FUNC = 1<<10;            // PWM
    MDR_PORTE->OE = 1<<5 |
                    1<<1 |              // ЦАП выход SUY
                    1<<2;               // ЦАП выход DUY
    MDR_PORTE->ANALOG = 1<<5;           // точность датчика угла
    MDR_PORTE->PWR = 0x3<<10;
    
    initGKControlLines();               // линии управления ключами
    engine_disableSpin();
         
    AS_ROUGH_DISABLE;
    AS_ACCURATE_DISABLE;
    AS_INHIBIT_HI;
    
#else // ОТЛАДОЧНАЯ ПЛАТА
    MDR_RST_CLK->PER_CLOCK |= 1<<21 |                  // PORTA
                              1<<22 |                  // PORTB
                              1<<23 |                  // PORTC
                              1<<24 |                  // PORTD
                              1<<25 |                  // PORTE
                              1<<29;                   // PORTF

    // can1
    MDR_PORTC->FUNC = 0x3<<18  |       // can1 rx
                      0x3<<20;         // can1 tx
    MDR_PORTC->OE = 0<<9 |      // can1 rx
                    1<<10;      // can1 tx
    MDR_PORTC->ANALOG = 1<<9 | 1<<10;
    MDR_PORTC->PWR = 0x3<<18 | 0x3<<20;

    // arinc-429 in
    MDR_PORTB->FUNC = 0x2<<0 |          // arinc DI0+
                      0x2<<2;           // arinc DI0-
    MDR_PORTB->OE       = 0<<0 | 0<<1;
    MDR_PORTB->ANALOG   = 1<<0 | 1<<1;
    MDR_PORTB->PWR  = 0x3<<0 | 0x3<<2;
  
    // arinc-429 out
    MDR_PORTF->FUNC = 0x1<<26 |         // arinc OUT2+
                      0x1<<28;          // arinc OUT2-
    MDR_PORTF->OE       = 1<<13 | 1<<14;
    MDR_PORTF->ANALOG   = 1<<13 | 1<<14;
    MDR_PORTF->PWR  = 0x3<<26 | 0x3<<28;
  
    // ЦАП
    MDR_PORTE->FUNC = 0;
    MDR_PORTE->OE = 1<<1;               // ЦАП выход SUY
    MDR_PORTE->ANALOG = 0;           // точность датчика угла
    //MDR_PORTE->PWR = 0;
    
    
#endif //MOEVS_BOARD
}

// Инициализировать линии управления ГК
void initGKControlLines()
{ 
    MDR_PORTE->OE |= 1<<10 | 1<<11 | 1<<12 | 1<<13 | 1<<14 | 1<<15;
    MDR_PORTE->ANALOG |= 1<<10 | 1<<11 | 1<<12 | 1<<13 | 1<<14 | 1<<15;
    MDR_PORTE->PWR |= 0x3<<20 | 0x3<<22 | 0x3<<24 | 0x3<<26 | 0x3<<28 | 0x3<<30;
}












//==============================================================================
//==============================================================================
//========================== Обработчик исключений =============================
//==============================================================================
//==============================================================================

void Hard_fault_handler_c(unsigned int* hardfault_args);

void HardFault_Handler()
{
   engine_disableSpin();
   
   uint32_t contr_reg = __get_CONTROL();
   if(contr_reg&2)
   {
      asm("MRS R0, PSP");
   }
   else
   {
      asm("MRS R0, MSP");
   }
   asm("B    (Hard_fault_handler_c)");                    //top of stack is in R0. It is passed to C-function.
}

void Hard_fault_handler_c(unsigned int* hardfault_args)
{
   unsigned int stacked_r0 = ((unsigned long) hardfault_args[0]);
   unsigned int stacked_r1 = ((unsigned long) hardfault_args[1]);
   unsigned int stacked_r2 = ((unsigned long) hardfault_args[2]);
   unsigned int stacked_r3 = ((unsigned long) hardfault_args[3]);
   unsigned int stacked_r12 = ((unsigned long) hardfault_args[4]);
   unsigned int stacked_lr = ((unsigned long) hardfault_args[5]);
   unsigned int stacked_pc = ((unsigned long) hardfault_args[6]);
   unsigned int stacked_psr = ((unsigned long) hardfault_args[7]);

   extern GKControlMode_t gk_controlMode;
   gk_controlMode = GkMode_HardFault;
   uint8_t msg[8];
   canMonitor_fillBufferWithCurrentState(msg);
   
   while(1)
   {
     can_putPackage(MDR_CAN1, CAN_MONITOR_TX_BUF, CANID_MONITOR_TX, msg, sizeof(msg));
     while(CAN_TX_REQ(MDR_CAN1, CAN_MONITOR_TX_BUF));
     
     canMonitor_blockingPrintf(MDR_CAN1, CAN_DBG_TEXT_MESSAGE_TX_BUF, 
                               "HARDFAULT!\n"
                               "R0  = %.8x\n"
                               "R1  = %.8x\n"
                               "R2  = %.8x\n"
                               "R3  = %.8x\n"
                               "R12 = %.8x\n"
                               "LR  = %.8x\n"
                               "PC  = %.8x\n"
                               "PSR = %.8x\n\n",  stacked_r0, stacked_r1, stacked_r2,
                                                  stacked_r3, stacked_r12, stacked_lr,
                                                  stacked_pc, stacked_psr);
     delay_us(5*1000*1000);
   }
}













