#include "RC.h"
#include "sys.h"
#include "CAN.h"
#include "LED.h"
#include "stm32f4xx.h"
#include "delay.h"


//键鼠
// #pragma pack(1) 
// typedef union 
// { 
// struct 
// { 
// struct 
// { 
// uint8_t ch0_h:8; //!< Byte 0 
// 
// uint8_t ch0_l:3; //!< Byte 1 
// uint8_t ch1_h:5; 
// 
// uint8_t ch1_l:6; //!< Byte 2 
// uint8_t ch2_h:2; 
// 
// uint8_t ch2_m:8; //!< Byte 3 
// 
// uint8_t ch2_l:1; //!< Byte 4 
// uint8_t ch3_h:7; 
// 
// uint8_t ch3_l:4; //!< Byte 5 
// uint8_t s1:2; 
// uint8_t s2:2; 
// }rc; 
// 
// struct 
// { 
// int16_t x; //!< Byte 6-7 
// int16_t y; //!< Byte 8-9 
// int16_t z; //!< Byte 10-11 
// uint8_t press_l; //!< Byte 12 
// uint8_t press_r; //!< Byte 13 
// }mouse; 
// 
// struct 
// { 
// uint16_t v; //!< Byte 14-15 
// }key; 
// 
// uint16_t resv; //!< Byte 16-17 
// }; 
// uint8_t buf[18]; //!< Union --> Byte<0-17> 
// }RC_Ctl_Define_t; 


volatile uint8_t sbus_rx_buffer[25]; 

/* ----------------------- Function Implements ---------------------------- */ 
/****************************************************************************** 
 * @fn RC_Init 
 * 
 * @brief configure stm32 usart2 port 
 * - USART Parameters 
 * - 100Kbps 
 * - 8-N-1 
 * - DMA Mode 
 * 
 * @return None. 
 * 
 * @note This code is fully tested on STM32F405RGT6 Platform, You can port it 
 * to the other platform. 
 */ 
void RC_Init(void) 
{ 
	 /* -------------- Enable Module Clock Source ----------------------------*/ 
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_DMA2, ENABLE); 
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); 
	 GPIO_PinAFConfig(GPIOA,GPIO_PinSource10, GPIO_AF_USART1); 
	 /* -------------- Configure GPIO ---------------------------------------*/ 
	 { 
	 GPIO_InitTypeDef gpio; 
	 USART_InitTypeDef usart1; 
	 gpio.GPIO_Pin = GPIO_Pin_10 ; 
	 gpio.GPIO_Mode = GPIO_Mode_AF; 
	 gpio.GPIO_OType = GPIO_OType_PP; 
	 gpio.GPIO_Speed = GPIO_Speed_100MHz; 
	 gpio.GPIO_PuPd = GPIO_PuPd_NOPULL; 
	 GPIO_Init(GPIOA, &gpio); 
	 
	 USART_DeInit(USART1); 
	 usart1.USART_BaudRate = 100000; 
	 usart1.USART_WordLength = USART_WordLength_8b;
	 usart1.USART_StopBits = USART_StopBits_1; 
	 usart1.USART_Parity = USART_Parity_Even; 
	 usart1.USART_Mode = USART_Mode_Rx;
	 usart1.USART_HardwareFlowControl = USART_HardwareFlowControl_None; 
	 USART_Init(USART1,&usart1); 
	 
	 USART_Cmd(USART1,ENABLE); 
	 USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE); 
	 } 
	 /* -------------- Configure NVIC ---------------------------------------*/ 
	 { 
	 NVIC_InitTypeDef nvic; 
	 nvic.NVIC_IRQChannel = DMA2_Stream2_IRQn; 
	 nvic.NVIC_IRQChannelPreemptionPriority = 1; 
	 nvic.NVIC_IRQChannelSubPriority = 1; 
	 nvic.NVIC_IRQChannelCmd = ENABLE; 
	 NVIC_Init(&nvic); 
	 } 
	 /* -------------- Configure DMA -----------------------------------------*/ 
	 { 
	 DMA_InitTypeDef dma; 
	 DMA_DeInit(DMA2_Stream2); 
	 dma.DMA_Channel = DMA_Channel_4; 
	 dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
	 dma.DMA_Memory0BaseAddr = (uint32_t)sbus_rx_buffer; 
	 dma.DMA_DIR = DMA_DIR_PeripheralToMemory; 
	 dma.DMA_BufferSize = 18; 
	 dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
	 dma.DMA_MemoryInc = DMA_MemoryInc_Enable; 
	 dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; 
	 dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	 dma.DMA_Mode = DMA_Mode_Circular; 
	 dma.DMA_Priority = DMA_Priority_VeryHigh; 
	 dma.DMA_FIFOMode = DMA_FIFOMode_Disable; 
	 dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull; 
	 dma.DMA_MemoryBurst = DMA_Mode_Normal; 
	 dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; 
	 DMA_Init(DMA2_Stream2,&dma); 
	 DMA_ITConfig(DMA2_Stream2,DMA_IT_TC,ENABLE); 
	 DMA_Cmd(DMA2_Stream2,DISABLE); /******先关闭*****/
	 DMA_Cmd(DMA2_Stream2,ENABLE); 
	 } 
} 
/****************************************************************************** 
 * @fn DMA1_Stream5_IRQHandler 
 * 
 * @brief USART2 DMA ISR 
 * 
 * @return None. 
 * 
 * @note This code is fully tested on STM32F405RGT6 Platform, You can port it 
 * to the other platform. 
 */ 

int32_t rc_time=0,close_rc=1;

void DMA2_Stream2_IRQHandler(void) 
{ 
	 if(DMA_GetITStatus(DMA2_Stream2, DMA_IT_TCIF2)) 
	 { 
		 DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2); 
		 DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2); 
		 RC_Ctl.rc.ch0 = (sbus_rx_buffer[0]| (sbus_rx_buffer[1] << 8)) & 0x07ff; //!< Channel 0 
		 RC_Ctl.rc.ch1 = ((sbus_rx_buffer[1] >> 3) | (sbus_rx_buffer[2] << 5)) & 0x07ff; //!< Channel 1 
		 RC_Ctl.rc.ch2 = ((sbus_rx_buffer[2] >> 6) | (sbus_rx_buffer[3] << 2) | //!< Channel 2 
		 (sbus_rx_buffer[4] << 10)) & 0x07ff; 
		 RC_Ctl.rc.ch3 = ((sbus_rx_buffer[4] >> 1) | (sbus_rx_buffer[5] << 7)) & 0x07ff; //!< Channel 3 
		 RC_Ctl.rc.s1 = ((sbus_rx_buffer[5] >> 4)& 0x000C) >> 2; //!< Switch left 
		 RC_Ctl.rc.s2 = ((sbus_rx_buffer[5] >> 4)& 0x0003); //!< Switch right
		 
		 rc_time=0;
		 
	 }
}

void send_data_to_chassis(void)  //给底盘发送遥控器数据
{
	CAN2_RC_TRANSMIT(sbus_rx_buffer);
}



void RC_unable(void)
{
        USART_Cmd(USART1, DISABLE);
}

void RC_restart(uint16_t dma_buf_num)
{
        USART_Cmd(USART1, DISABLE);
        DMA_Cmd(DMA2_Stream2, DISABLE);
        DMA_SetCurrDataCounter(DMA2_Stream2, dma_buf_num);

        USART_ClearFlag(USART1, USART_FLAG_IDLE);

        DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
        DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2);
        DMA_Cmd(DMA2_Stream2, ENABLE);
        USART_Cmd(USART1, ENABLE);
}


