#ifndef __CAN_TERMINAL__
#define __CAN_TERMINAL__


#include "MDR1986VE1T.h"


#define CAN_TERMINAL_DEVICE_ID          0                                              

// Состояние приема массива пакетов
#define TERM_TRANSACTION_STATE_START           0
#define TERM_TRANSACTION_STATE_CONTINIOUS      1
#define TERM_TRANSACTION_STATE_END             2



void term_receive_msg(uint8_t *buf, uint8_t len);
void process_rx_frame(uint8_t *buf, uint8_t len);
void send_to_term(uint8_t *p, uint8_t len);
void canTerminal_printf(uint8_t *p, ...);

uint8_t check_cmd(uint8_t* pCmd, uint8_t* pData, uint8_t len);
uint8_t starts_with(uint8_t* pCmd, uint8_t* pData, uint8_t len);



#endif //__CAN_TERMINAL__