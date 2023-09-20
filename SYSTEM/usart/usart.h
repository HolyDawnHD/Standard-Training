#ifndef __USART_H
#define __USART_H
#include "sys.h"
#include "stdio.h"
//#include "stm32f4xx.h"

#define USART_REC_LEN 200
#define EN_USART1_RX 1
#define EN_UART5_RX 1


extern uint8_t  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern uint16_t USART_RX_STA;  

//接收中断函数
void uart_init(uint32_t bound);

#endif
