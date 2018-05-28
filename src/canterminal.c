#include <stdio.h>
#include <stdarg.h>
#include "canterminal.h"
#include "global.h"
#include "settings.h"
#include "can.h"



// Принять сообщение
void term_receive_msg(uint8_t *buf, uint8_t len)
{
   static uint8_t canTermBuf[256];              
   static uint8_t msgLen = 0;                   
   static uint8_t offset = 0;                   
                 
   if(buf[0] != CAN_TERMINAL_DEVICE_ID) return; // i
      
   uint8_t frameType = buf[1];                  // frameType (start/cont/end)
   uint8_t dataLen = len-2;                     
   switch(frameType)       
   {
       case TERM_TRANSACTION_STATE_START:       
            offset = 0;
            msgLen = 0;
            for(int i=0; i<dataLen; i++)
            {
                canTermBuf[offset++] = buf[2+i];
                msgLen++;
            }            
         break;
         
       case TERM_TRANSACTION_STATE_CONTINIOUS: 
              for(int i=0; i<dataLen; i++)
              {
                  canTermBuf[offset++] = buf[2+i];
                  msgLen++;
              }          
         break;
         
       case TERM_TRANSACTION_STATE_END:        
              for(int i=0; i<dataLen; i++)
              {
                  canTermBuf[offset++] = buf[2+i];
                  msgLen++;
              }   
              
              process_rx_frame(canTermBuf, msgLen);
              offset = 0;
              msgLen = 0;
         break;
   }
}

// Отправить сообщение по терминалу
void send_to_term(uint8_t *p, uint8_t len)
{
   uint8_t buf[8];
   uint8_t need_tx = len;
   uint8_t state = TERM_TRANSACTION_STATE_START;   
   buf[0] = CAN_TERMINAL_DEVICE_ID;  
  
   while(need_tx != 0)
   {
      if(need_tx <= 6)
      {
         buf[1] = TERM_TRANSACTION_STATE_END;
         for(int i=2; i<need_tx+2; i++) buf[i] = *p++; 
         
         if(!can1_send_packet(CAN_TERM_TX_BUF, CANID_TERM_TX, buf, need_tx+2))
           return;
         need_tx = 0;
      }
      else 
      {
         if(state == TERM_TRANSACTION_STATE_START) 
         {
            buf[1] = TERM_TRANSACTION_STATE_START;
            state = TERM_TRANSACTION_STATE_CONTINIOUS;
         }         
         else buf[1] = TERM_TRANSACTION_STATE_CONTINIOUS;
  
         for(int i=2; i<8; i++) buf[i] = *p++; 
         if(!can1_send_packet(CAN_TERM_TX_BUF, CANID_TERM_TX, buf, sizeof(buf)))
            return;
         need_tx-=6;
      }
   }   
}

// Отправить строку по терминалу
void canTerminal_printf(uint8_t *p, ...)
{
  uint8_t buf[256];
  va_list args;
  va_start(args, p);
  int len = vsprintf((char*)buf, (char*)p, args);
  va_end(args);
 
  if(len<0) return;  
  send_to_term(buf, len);
}


// Сравнить принятое сообщение со строкой
uint8_t check_cmd(uint8_t* pCmd, uint8_t* pData, uint8_t len)
{
    int i=0;
    while(pCmd[i] != 0)
    {
       if(pCmd[i] == pData[i])
       {
          i++;
          continue;
       }
       else return 0;
    }

    if(i != len) return 0;      // проверка длины
    else return 1;
}

// Проверить, что принятое сообщение начинается с pCmd[]
uint8_t starts_with(uint8_t* pCmd, uint8_t* pData, uint8_t len)
{
    int i=0;
    while(pCmd[i] != 0)
    {
       if(pCmd[i] == pData[i])
       {
          i++;
          continue;
       }
       else return 0;
    }
    return 1;
}


// Обработать принятое сообщение
void process_rx_frame(uint8_t *buf, uint8_t len)
{    
    if(check_cmd("echo", buf, len))     // эхо
    {       
       canTerminal_printf("MK Y: echo");
    }
    if(check_cmd("time", buf, len))     // время работы
    {
       uint32_t timeH = 0, timeM = 0, timeS = 0, timeMs = 0; 
       
       timeMs = system_time % 1000;     // кол-во миллисекунд (0...999)
       timeS = system_time / 1000;      // кол-во прошедших секунд (>60)
       
       timeM = timeS / 60;              // кол-во минут (>60)
       timeS = timeS % 60;              // кол-во секунд (0..59)
       
       timeH = timeM/60;                // кол-во часов
       timeM = timeM%60;                // кол-во минут (0..60)
      
       canTerminal_printf("working time %d:%.2d:%.2d.%.3d", timeH, timeM, timeS, timeMs);
    }
}

