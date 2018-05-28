#include "dac.h"


//extern float vRefAmmend;                // поправка 
extern int16_t vRefCodeAmmend;          // поправка к АЦП


// Инициализация ЦАП
void dac_init()
{
  MDR_RST_CLK->PER_CLOCK |= 1<<18;      // тактирование ЦАП
  MDR_DAC->CFG = 1<<2 |                 // включение DAC0
                 1<<3;                  // включение DAC1
  
  dac_setAngle(0);
}

// Установка скорости
void dac_setSpeed(float speed)
{
  float volts = 1.65 + speed*0.02;
  uint16_t codeAmmend = 0xC; // 0x1C;
  
  uint16_t dacCode = (uint16_t)((volts*0xFFF)/3.3) - codeAmmend;
  MDR_DAC->DAC1_DATA = dacCode;
}

// Установка угла
void dac_setAngle()
{
  //float volts = 1.65;
  //uint16_t codeAmmend = 11;
  
  uint16_t dacCode =  2047 - 0x1a;
  dacCode = 0x7F9;
  MDR_DAC->DAC2_DATA = dacCode;
}
