#ifndef __PC__H
#define __PC__H
#include "stm32f4xx_hal.h"
#include "usart.h"



#define PCUART  huart5
#define PC_IRQn UART5_IRQn

extern uint8_t recvBuf[30];
extern uint8_t recvBuf1[30];
extern uint8_t recvBuf2[30];
extern uint8_t recv_index;
extern uint8_t recv_new;
void pc_task();

#endif
