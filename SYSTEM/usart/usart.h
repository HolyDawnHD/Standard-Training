#ifndef __USART_H
#define __USART_H
#include "sys.h"
#include "stdio.h"
//#include "stm32f4xx.h"

#define USART_REC_LEN 200
#define EN_USART1_RX 1
#define EN_UART5_RX 1


extern uint8_t  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern uint16_t USART_RX_STA;  

//�����жϺ���
void uart_init(uint32_t bound);

#endif
