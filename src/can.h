#ifndef __CAN_H__
#define __CAN_H__

#include "MDR1986VE1T.h"


#define CAN_TX_TIMEOUT_MS               4                                       // таймаут на передачу пакета

#define CAN_TX_REQ(can, bufNum)         (can->BUF_CON[bufNum] & 1<<5)           // запрос на отправку пакета, 0 - пакет отправлен, 1 - передача не завершена
#define CAN_TX_REQ_CLR(can, bufNum)     (can->BUF_CON[bufNum] &= ~(1<<5))       // снять запрос на отправку пакета
#define CAN_ERROR_OVER(can)             (can->STATUS & 1<<2)                    // ошибок больше, чем ERR_MAX



#define CAN_STANDART_ID_MASK               0x7FF<<18                            // маска стандартного ID


// Индексы буферов для перепрограммирования 
#define CAN_LDR_RX_BUF                     0             // приемный буфер для перепрограммирования 
#define CAN_LDR_TX_BUF                     15            // передающий буфер для перепрограммировния

// Индексы буферов для терминала
#define CAN_TERM_RX_BUF                    1             // приемный буфер для терминала 
#define CAN_TERM_TX_BUF                    16            // передающий буфер для терминала

// Индексы буферов для мониторинга
#define CAN_MONITOR_RX_BUF                 2             // приемный буфер для монитора состояния камеры 
#define CAN_MONITOR_TX_BUF                 17            // передающий буфер для монитора состояния камеры

// Индексы буферов для тангажного угла
#define CAN_TANGAGE_ANGLE_RX_BUF           3

// Индексы буферов для тангажного контура
#define CAN_TANGAGE_CONTROL_RX_BUF         4
#define CAN_TANGAGE_CONTROL_TX_BUF         18

// Индекс буфера для передачи текстовых сообщений
#define CAN_DBG_TEXT_MESSAGE_TX_BUF        19            // отладочные сообщения



// CAN-Идентификаторы 
#define CANID_LDR_REQ                      0x01          // перепрошивка - id=1
#define CANID_LDR_ANSWER                   0x02          // девайсы отвечают сообщениями с id=2 при перепрограммировании

#define CANID_TERM_RX                      0x04          // id сообщений от терминала 
#define CANID_TERM_TX                      0x03          // id сообщений терминалу

#define CANID_MONITOR_RX                   0x80          // id сообщений от монитора состояний
#define CANID_MONITOR_TX                   0x70          // id сообщения монитору состояний


#define CANID_TANGAGE_ANGlE_RX             0x07          // id сообщений от тангажного контура

#define CANID_TANGAGE_CONTORL_RX           0x08          // id сообщений от тангажного контура
#define CANID_TANGAGE_CONTORL_TX           0x09          // id сообщения тангажному контору

#define CANID_DBG_TEXT_TX                  0x90          // id сообщений текстовой отладки


// Маски id
#define CAN_RX_FILTER_PROGRAM              (CANID_LDR_REQ<<18)                  // команды перепрошивки
#define CAN_RX_FILTER_TERMINAL             (CANID_TERM_RX<<18)                  // сообщения от терминала
#define CAN_RX_FILTER_MONITOR              (CANID_MONITOR_RX<<18)               // сообщения от монитора
#define CAN_RX_FILTER_TANGAGE_ANGLE        (CANID_TANGAGE_ANGlE_RX<<18)         // сообщения от тангажного конитура
#define CAN_RX_FILTER_TANGAGE_CONTROL      (CANID_TANGAGE_CONTORL_RX<<18)       // сообщения от тангажного конитура




extern void canMonitor_rxFrameHandler(uint8_t *buf, uint8_t len);
extern void term_receive_msg(uint8_t *buf, uint8_t len);
extern void tangageCtrl_rxAngle(uint8_t *buf, uint8_t len);
extern void tangageCtrl_rxMsg(uint8_t *buf, uint8_t len);

void can1_init();

__ramfunc uint8_t can1_rx_new_packet(uint8_t can_buf_indx, uint8_t *buf, uint8_t *len);
__ramfunc uint8_t can1_send_packet(uint8_t can_buf_indx, uint16_t msgId, uint8_t *pData, uint8_t len);


void can_putPackage(MDR_CAN_TypeDef *can, uint8_t can_buf_indx, uint16_t msgId, uint8_t *pData, uint8_t len);
void can1_putPackage(uint8_t can_buf_indx, uint16_t msgId, uint8_t *pData, uint8_t len);







#endif //__CAN_H__