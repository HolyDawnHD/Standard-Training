#include "CAN.h"
#include "sys.h"
#include "PID.h"
#include "usart.h"
#include "LED.h"
#include "delay.h"





motor_measure_t motor[5];


const motor_measure_t *Motor_Measure_Point(void)
{
    return &motor[5];
}

void CAN1_mode_init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp)
{
	GPIO_InitTypeDef GPIO_InitStructure;
    CAN_InitTypeDef CAN_InitStructure;
    CAN_FilterInitTypeDef CAN_FilterInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
	
	//gpiob 8 9 can1 rx tx
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	
//	RCC_APB1PeriphResetCmd(RCC_APB1Periph_CAN1, ENABLE);//复位使相应寄存器为0,
//    RCC_APB1PeriphResetCmd(RCC_APB1Periph_CAN1, DISABLE);//取消复位，使寄存器能够正常使用
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_CAN1);
	
	CAN_InitStructure.CAN_TTCM = DISABLE;
    CAN_InitStructure.CAN_ABOM = DISABLE;
    CAN_InitStructure.CAN_AWUM = DISABLE;
    CAN_InitStructure.CAN_NART = ENABLE;
    CAN_InitStructure.CAN_RFLM = DISABLE;
    CAN_InitStructure.CAN_TXFP = DISABLE;
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
    CAN_InitStructure.CAN_SJW = tsjw;
    CAN_InitStructure.CAN_BS1 = tbs1;
    CAN_InitStructure.CAN_BS2 = tbs2;
    CAN_InitStructure.CAN_Prescaler = brp;
    CAN_Init(CAN1, &CAN_InitStructure);
	
	CAN_FilterInitStructure.CAN_FilterNumber = 0;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);
	
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
	
}

void CAN2_mode_init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp)
{
	GPIO_InitTypeDef GPIO_InitStructure;
    CAN_InitTypeDef CAN_InitStructure;
    CAN_FilterInitTypeDef CAN_FilterInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
	
	//gpiob 5 6 can1 rx tx
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);
	
//	RCC_APB1PeriphResetCmd(RCC_APB1Periph_CAN1, ENABLE);//复位使相应寄存器为0,
//    RCC_APB1PeriphResetCmd(RCC_APB1Periph_CAN1, DISABLE);//取消复位，使寄存器能够正常使用
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_CAN2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_CAN2);
	
	CAN_InitStructure.CAN_TTCM = DISABLE;
    CAN_InitStructure.CAN_ABOM = DISABLE;
    CAN_InitStructure.CAN_AWUM = DISABLE;
    CAN_InitStructure.CAN_NART = ENABLE;
    CAN_InitStructure.CAN_RFLM = DISABLE;
    CAN_InitStructure.CAN_TXFP = DISABLE;
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
    CAN_InitStructure.CAN_SJW = tsjw;
    CAN_InitStructure.CAN_BS1 = tbs1;
    CAN_InitStructure.CAN_BS2 = tbs2;
    CAN_InitStructure.CAN_Prescaler = brp;
    CAN_Init(CAN2, &CAN_InitStructure);
	
	CAN_FilterInitStructure.CAN_FilterNumber = 15;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);
	
	CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	

}

#define get_motor_measure(ptr, rx_message)                                                     \
{                                                                                              \
        (ptr)->last_ecd = (ptr)->ecd;                                                          \
        (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
        (ptr)->speed_rpm = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);     \
        (ptr)->given_current = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]); \
        (ptr)->temperate = (rx_message)->Data[6];                                              \
		if((ptr)->ecd-(ptr)->last_ecd<-4096)   \
		{                                    \
			(ptr)->round++;                  \
		}                                    \
		if((ptr)->ecd-(ptr)->last_ecd>4096)    \
		{                                    \
			(ptr)->round--;                   \
		}                                    \
		(ptr)->real_motor_ecd_degree=(fp32)(ptr)->ecd/8191.0f+(ptr)->round;   \
}



void CAN1_RX0_IRQHandler(void)
{
    static CanRxMsg rx1_message;

    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_Receive(CAN1, CAN_FIFO0, &rx1_message);
		switch(rx1_message.StdId)
		{
			case 0x201:
				get_motor_measure(&motor[1], &rx1_message);
				break;
			case 0x202:
				get_motor_measure(&motor[2], &rx1_message);
				break;
			case 0x203:
				get_motor_measure(&motor[3], &rx1_message);
				break;
			case 0x204:
				get_motor_measure(&motor[4], &rx1_message);
				break;
			
		}
		
		
    }
}


RC_Ctl_t RC_Crl_chassis;

void get_rc_measure(RC_Ctl_t *RC,CanRxMsg *rx_message)
{
	
	
	RC->rc.ch0 = (rx_message->Data[0]| (rx_message->Data[1] << 8)) & 0x07ff; //!< Channel 0 
	RC->rc.ch1 = ((rx_message->Data[1]>> 3) | (rx_message->Data[2] << 5)) & 0x07ff; //!< Channel 1 
	RC->rc.ch2 = ((rx_message->Data[2] >> 6) | (rx_message->Data[3] << 2) | //!< Channel 2 
	(rx_message->Data[4] << 10)) & 0x07ff; 
	RC->rc.ch3 = ((rx_message->Data[4] >> 1) | (rx_message->Data[5] << 7)) & 0x07ff; //!< Channel 3 
	RC->rc.s1 = ((rx_message->Data[5] >> 4)& 0x000C) >> 2; //!< Switch left 
	RC->rc.s2 = ((rx_message->Data[5]>> 4)& 0x0003); //!< Switch right
	
	
}



void CAN2_RX0_IRQHandler(void)
{
    static CanRxMsg rx2_message;

    if (CAN_GetITStatus(CAN2, CAN_IT_FMP0) != RESET)
    {
		
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
        CAN_Receive(CAN2, CAN_FIFO0, &rx2_message);
		if(rx2_message.StdId==0x107)
			get_rc_measure(&RC_Crl_chassis, &rx2_message);
    }
		
}



uint8_t mailbox;

u8 CAN2_RC_TRANSMIT(volatile uint8_t *send_data)                //发送函数
{
	u16 i=0;
	static CanTxMsg TxMessage;
	
    TxMessage.StdId =0x107 ; //0x1FF  0x200
	TxMessage.ExtId=0;
    TxMessage.IDE = CAN_Id_Standard;
    TxMessage.RTR = CAN_RTR_Data;
    TxMessage.DLC = 0x08;
	
	
	TxMessage.Data[0]= send_data[0];
	TxMessage.Data[1]= send_data[1];
	TxMessage.Data[2]= send_data[2];
	TxMessage.Data[3]= send_data[3];
	TxMessage.Data[4]= send_data[4];
	TxMessage.Data[5]= send_data[5];
	TxMessage.Data[6]=0;
	TxMessage.Data[7]=0;
	
    mailbox=CAN_Transmit( CAN2,  &TxMessage);
	
	while((CAN_TransmitStatus(CAN2,mailbox)==CANTXFAILED) && (i!=0XFF))
	{
		i++;
	}
	if(i>=0XFF)
	{
		return 0;
	}
	else 
	{
		return 1;
	}
	
}

void CAN1_CHASSIS_MOTOR_TRANSMIT(int16_t val1,int16_t val2,int16_t val3,int16_t val4)//发送函数
{
	static CanTxMsg TxMessage;
	
    TxMessage.StdId =0x200 ; //0x1FF  0x200
	
    TxMessage.IDE = 0x00000000;
    TxMessage.RTR = 0x00000000;
    TxMessage.DLC = 0x08;
	
	
    TxMessage.Data[0] = (val1 >> 8);
    TxMessage.Data[1] = val1;
	TxMessage.Data[2] = (val2 >> 8);
    TxMessage.Data[3] = val2;
	TxMessage.Data[4] = (val3 >> 8);
    TxMessage.Data[5] = val3;
	TxMessage.Data[6] = (val4 >> 8);
    TxMessage.Data[7] = val4;


    CAN_Transmit( CAN1,  &TxMessage);

}


