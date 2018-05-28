#include "engineControl.h"
#include "timers.h"
#include "global.h"
#include "dac.h"
#include "pidRegulator.h"
#include "canterminal.h"
#include "angelSensor.h"


extern uint32_t courseAngleCodeX32;
extern uint32_t g_engineControlAngle360;
int8_t prevSectorCW = -1;       // предыдущий сектор при пращении по часовой стрелке
int8_t prevSectorCCW = -1;      // предыдущий сектор при вращении против часовой стрелке



// Отключение управляющих линий
void engine_disableSpin()
{
  AH_LO();
  AL_LO();
  BH_LO();
  BL_LO();
  CH_LO();
  CL_LO();
}


/*
#define ANGLE_TO_CODE(angle)      ((uint16_t)(0xFFFF*angle/360.0))
#define ANGLE_7_5                 ANGLE_TO_CODE(7.5)
#define ANGLE_15                  ANGLE_TO_CODE(15)
#define ANGLE_22_5                ANGLE_TO_CODE(22.5)
#define ANGLE_30                  ANGLE_TO_CODE(30)
#define ANGLE_37_5                ANGLE_TO_CODE(37.5)
#define ANGLE_45                  ANGLE_TO_CODE(45)
*/

// Вращение по часовой стрелке
void engine_spinCW()
{
   prevSectorCCW = -1;   // сбросить сектор CCW
     
   //float _angle = CONVERT_ANGLE32_TO_FLOAT(courseAngleCodeX32);
   //float tmp = _angle - 45*(int)(_angle / 45);
   
   uint32_t tmp = g_engineControlAngle360 - USYSANGLE_TO_CODE(45)*(int)(g_engineControlAngle360/USYSANGLE_TO_CODE(45));
   
   if(tmp <= USYSANGLE_TO_CODE(7.5))
   {
     if(prevSectorCW == 1) return; 
     prevSectorCW = 1;
     
     AH_LO();
     BH_LO();
     AL_LO();
     CL_LO();
     delay_us(GK_PHASE_SWITCH_DELAY_us);
     CH_HI();
     BL_HI();
   }
   else if(tmp <= USYSANGLE_TO_CODE(15))
   {    
     if(prevSectorCW == 2) return;
     prevSectorCW = 2;
    
     BH_LO();
     CH_LO();
     AL_LO();
     CL_LO();
     delay_us(GK_PHASE_SWITCH_DELAY_us);
     AH_HI();
     BL_HI();
   }
   else if(tmp <= USYSANGLE_TO_CODE(22.5))
   {    
     if(prevSectorCW == 3) return;
     prevSectorCW = 3;
     
     BH_LO();
     CH_LO();
     AL_LO();
     BL_LO();  
     delay_us(GK_PHASE_SWITCH_DELAY_us);
     AH_HI();
     CL_HI();
   }
   else if(tmp <= USYSANGLE_TO_CODE(30))
   {   
     if(prevSectorCW == 4) return;
     prevSectorCW = 4;
      
     AH_LO();
     CH_LO();
     AL_LO();
     BL_LO();
     delay_us(GK_PHASE_SWITCH_DELAY_us);
     BH_HI();
     CL_HI();
   }
   else if(tmp <= USYSANGLE_TO_CODE(37.5))
   {    
     if(prevSectorCW == 5) return;
     prevSectorCW = 5;
     
     AH_LO();
     CH_LO();
     BL_LO();
     CL_LO();
     delay_us(GK_PHASE_SWITCH_DELAY_us);
     BH_HI();
     AL_HI();
   }
   else if(tmp <= USYSANGLE_TO_CODE(45))
   {   
     if(prevSectorCW == 6) return;
     prevSectorCW = 6;
     
     AH_LO();
     BH_LO();
     BL_LO();
     CL_LO();      
     delay_us(GK_PHASE_SWITCH_DELAY_us);
     CH_HI();
     AL_HI();
   }
}


// Вращение против часовой стрелке
void engine_spinCCW()
{  
   prevSectorCW = -1;   // сбросить сектор CW     
   
   //float _angle = CONVERT_ANGLE32_TO_FLOAT(courseAngleCodeX32);
   //float tmp = _angle - 45*(int)(_angle / 45);   
   //uint16_t tmp = g_engineControlAngle360 - ANGLE_45*(int)(g_engineControlAngle360/ANGLE_45);
   uint32_t tmp = g_engineControlAngle360 - USYSANGLE_TO_CODE(45)*(int)(g_engineControlAngle360/USYSANGLE_TO_CODE(45));
   if(tmp <= USYSANGLE_TO_CODE(7.5))
   {
     if(prevSectorCCW == 1) return; 
     prevSectorCCW = 1;
     
     AH_LO();
     AL_LO();
     BL_LO();
     CH_LO();
     delay_us(GK_PHASE_SWITCH_DELAY_us);
     BH_HI();
     CL_HI();
   }
   else if(tmp <= USYSANGLE_TO_CODE(15))
   {    
     if(prevSectorCCW == 2) return;
     prevSectorCCW = 2;
    
     AH_LO();
     BL_LO();
     CH_LO();
     CL_LO();
     delay_us(GK_PHASE_SWITCH_DELAY_us);
     BH_HI();
     AL_HI();
   }
   else if(tmp <= USYSANGLE_TO_CODE(22.5))
   {    
     if(prevSectorCCW == 3) return;
     prevSectorCCW = 3;
     
     AH_LO();
     BH_LO();
     BL_LO();
     CL_LO();  
     delay_us(GK_PHASE_SWITCH_DELAY_us);
     CH_HI();
     AL_HI();
   }
   else if(tmp <= USYSANGLE_TO_CODE(30))
   {   
     if(prevSectorCCW == 4) return;
     prevSectorCCW = 4;
      
     AH_LO();
     AL_LO();
     BH_LO();
     CL_LO();
     delay_us(GK_PHASE_SWITCH_DELAY_us);
     CH_HI();
     BL_HI();
   }
   else if(tmp <= USYSANGLE_TO_CODE(37.5))
   {    
     if(prevSectorCCW == 5) return;
     prevSectorCCW = 5;
     
     AL_LO();
     BH_LO();
     CH_LO();
     CL_LO();
     delay_us(GK_PHASE_SWITCH_DELAY_us);
     AH_HI();
     BL_HI();
   }
   else if(tmp <= USYSANGLE_TO_CODE(45))
   {   
     if(prevSectorCCW == 6) return;
     prevSectorCCW = 6;
     
     AL_LO();
     BH_LO();
     BL_LO();
     CH_LO();      
     delay_us(GK_PHASE_SWITCH_DELAY_us);
     AH_HI();
     CL_HI();
   }
}



/*



// Вращение против часовой стрелке
void engine_spinCCW()
{  
   prevSectorCW = -1;   // сбросить сектор CW     
   
   float _angle = CONVERT_ANGLE32_TO_FLOAT(courseAngleCodeX32);
   float tmp = _angle - 45*(int)(_angle / 45);
   if(tmp <= 7.5)
   {
     if(prevSectorCCW == 1) return; 
     prevSectorCCW = 1;
     
     AH_LO();
     AL_LO();
     BL_LO();
     CH_LO();
     delay_us(GK_PHASE_SWITCH_DELAY_us);
     BH_HI();
     CL_HI();
   }
   else if(tmp <= 15)
   {    
     if(prevSectorCCW == 2) return;
     prevSectorCCW = 2;
    
     AH_LO();
     BL_LO();
     CH_LO();
     CL_LO();
     delay_us(GK_PHASE_SWITCH_DELAY_us);
     BH_HI();
     AL_HI();
   }
   else if(tmp <= 22.5)
   {    
     if(prevSectorCCW == 3) return;
     prevSectorCCW = 3;
     
     AH_LO();
     BH_LO();
     BL_LO();
     CL_LO();  
     delay_us(GK_PHASE_SWITCH_DELAY_us);
     CH_HI();
     AL_HI();
   }
   else if(tmp <= 30)
   {   
     if(prevSectorCCW == 4) return;
     prevSectorCCW = 4;
      
     AH_LO();
     AL_LO();
     BH_LO();
     CL_LO();
     delay_us(GK_PHASE_SWITCH_DELAY_us);
     CH_HI();
     BL_HI();
   }
   else if(tmp <= 37.5)
   {    
     if(prevSectorCCW == 5) return;
     prevSectorCCW = 5;
     
     AL_LO();
     BH_LO();
     CH_LO();
     CL_LO();
     delay_us(GK_PHASE_SWITCH_DELAY_us);
     AH_HI();
     BL_HI();
   }
   else if(tmp <= 45)
   {   
     if(prevSectorCCW == 6) return;
     prevSectorCCW = 6;
     
     AL_LO();
     BH_LO();
     BL_LO();
     CH_LO();      
     delay_us(GK_PHASE_SWITCH_DELAY_us);
     AH_HI();
     CL_HI();
   }
}

// Вращение по часовой стрелке
void engine_spinCW()
{
   prevSectorCCW = -1;   // сбросить сектор CCW
     
   float _angle = CONVERT_ANGLE32_TO_FLOAT(courseAngleCodeX32);
   float tmp = _angle - 45*(int)(_angle / 45);
   if(tmp <= 7.5)
   {
     if(prevSectorCW == 1) return; 
     prevSectorCW = 1;
     
     AH_LO();
     BH_LO();
     AL_LO();
     CL_LO();
     delay_us(GK_PHASE_SWITCH_DELAY_us);
     CH_HI();
     BL_HI();
   }
   else if(tmp <= 15)
   {    
     if(prevSectorCW == 2) return;
     prevSectorCW = 2;
    
     BH_LO();
     CH_LO();
     AL_LO();
     CL_LO();
     delay_us(GK_PHASE_SWITCH_DELAY_us);
     AH_HI();
     BL_HI();
   }
   else if(tmp <= 22.5)
   {    
     if(prevSectorCW == 3) return;
     prevSectorCW = 3;
     
     BH_LO();
     CH_LO();
     AL_LO();
     BL_LO();  
     delay_us(GK_PHASE_SWITCH_DELAY_us);
     AH_HI();
     CL_HI();
   }
   else if(tmp <= 30)
   {   
     if(prevSectorCW == 4) return;
     prevSectorCW = 4;
      
     AH_LO();
     CH_LO();
     AL_LO();
     BL_LO();
     delay_us(GK_PHASE_SWITCH_DELAY_us);
     BH_HI();
     CL_HI();
   }
   else if(tmp <= 37.5)
   {    
     if(prevSectorCW == 5) return;
     prevSectorCW = 5;
     
     AH_LO();
     CH_LO();
     BL_LO();
     CL_LO();
     delay_us(GK_PHASE_SWITCH_DELAY_us);
     BH_HI();
     AL_HI();
   }
   else if(tmp <= 45)
   {   
     if(prevSectorCW == 6) return;
     prevSectorCW = 6;
     
     AH_LO();
     BH_LO();
     BL_LO();
     CL_LO();      
     delay_us(GK_PHASE_SWITCH_DELAY_us);
     CH_HI();
     AL_HI();
   }
}



*/