#include "stm32f4xx.h"
#include "sys.h"
#include "delay.h"
#include "RC.h"
#include "CAN.h"
#include "LED.h"
#include "usart.h"
#include "chassis_tasks.h"
#include "gimbal_tasks.h"

u8 gimbal_Init(void)
{
	
	RC_Init();
	LED_Init();
	CAN1_mode_init(CAN_SJW_1tq, CAN_BS2_7tq, CAN_BS1_6tq, 3);
	CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_7tq, CAN_BS1_6tq, 3);
	
	imu_Init();
	LED_G=0;
	return 0;
	
}

u8 imu_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
		
	//pc6 pc7��Ϊtx rx ����ΪUSART6_TX  USART6_RX
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);
		

	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6);
	//GPIOC ����Ϊ USART6

		
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; //GPIOC6
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOC,&GPIO_InitStructure); //��ʼ�� PC12��PD2
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; //GPIOC7
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOC,&GPIO_InitStructure);
		
		
	 //UART(ͨ���첽�շ����� ��ʼ�����ã�ͬusart
	USART_InitStructure.USART_BaudRate = 921600;//һ������Ϊ 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl =USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//�շ�
	USART_Init(USART6, &USART_InitStructure); //��ʼ������
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);//�����ж�
	USART_Cmd(USART6, ENABLE); //ʹ�ܴ���
		
		
	//#if EN_USART1_RX 
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);//�����ж�
	//UART5 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//��ռ���ȼ� 0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0; //��Ӧ���ȼ� 0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ ͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);
	return 0;
}

int16_t num_buf=0;
imu imu_buf;

int16_t BUF[55];
void USART6_IRQHandler(void) //���� 6 �жϷ������
{
	if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)
	{
		USART_ClearFlag(USART6, USART_IT_RXNE);
		BUF[num_buf] =USART_ReceiveData(USART6);//(USART6->DR); //��ȡ���յ�������
		num_buf++;
		if(num_buf>=41)
		{
			trans_data(BUF,&imu_buf);
			num_buf=0;
		}
		
		if(BUF[0]!=0x5A)
			num_buf=0;
		else if(BUF[1]!=0xA5 && BUF[1]!=0x00)
			num_buf=0;
	}
	if(USART_GetITStatus(USART6, USART_IT_ORE) != RESET)
	{
		USART_ClearFlag(USART6,USART_IT_ORE);
	}
	
}

void trans_data(int16_t *get,imu *set)
{
	imu_buf.a_x=get[10]<<8 | get[9];
	imu_buf.a_y=get[12]<<8 | get[11];
	imu_buf.a_z=get[14]<<8 | get[13];
	imu_buf.angle_speed_x=get[17]<<8 | get[16];
	imu_buf.angle_speed_y=get[19]<<8 | get[18];
	imu_buf.angle_speed_z=get[21]<<8 | get[20];
	imu_buf.pitch=get[31]<<8 | get[30];
	imu_buf.roll=get[33]<<8 | get[32];
	imu_buf.yaw= get[35]<<8 | get[34];
	
	
}

