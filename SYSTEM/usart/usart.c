#include "usart.h"
#include "sys.h"

u8 USART_RX_BUF[USART_REC_LEN];
u16 USART_RX_STA=0;

#if 1
#pragma import(__use_no_semihosting)
	//��׼����Ҫ��֧�ֺ���
	struct __FILE
	{
		int handle;
	};
	FILE __stdout;
	//����_sys_exit()�Ա���ʹ�ð�����ģʽ
	void sys_exit(int x)
	{
		x = x;
	}
	//�ض��� fputc ����
	int fputc(int ch, FILE *f)
	{
		while(USART_GetFlagStatus(UART5,USART_FLAG_TC)==RESET);
		USART_SendData(UART5,(uint8_t)ch);
		return ch;
	}
#endif

	
#if EN_USART1_RX  
	
	void uart_init(uint32_t bound)
	{
		
		//usart2 pa2tx  pa3rx

		GPIO_InitTypeDef GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
		
		//rx txʹ��pd2 pc12������Ϊuart5
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOA,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,ENABLE);
		//RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
		
		
		
		
		GPIO_PinAFConfig(GPIOD,GPIO_PinSource2,GPIO_AF_UART5);
		GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_UART5);
		//GPIOC12 ����Ϊ USART1

		
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; //GPIOC12 �� GPIOD2
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
		GPIO_Init(GPIOC,&GPIO_InitStructure); //��ʼ�� PC12��PD2
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //GPIOC12 �� GPIOD2
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
		GPIO_Init(GPIOD,&GPIO_InitStructure);
		

		

		
		 //UART(ͨ���첽�շ����� ��ʼ�����ã�ͬusart
		USART_InitStructure.USART_BaudRate = bound;//һ������Ϊ 115200;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_HardwareFlowControl =USART_HardwareFlowControl_None;//��Ӳ������������
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//�շ�
		USART_Init(UART5, &USART_InitStructure); //��ʼ������
		USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);//�����ж�
		USART_Cmd(UART5, ENABLE); //ʹ�ܴ���
		//USART_ClearFlag(UART5, USART_FLAG_TC);
		
		
		//#if EN_USART1_RX 
		USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);//�����ж�
		//UART5 NVIC ����
		NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//��ռ���ȼ� 2
		NVIC_InitStructure.NVIC_IRQChannelSubPriority =2; //��Ӧ���ȼ� 2
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ ͨ��ʹ��
		NVIC_Init(&NVIC_InitStructure);
		//#endif

		
		/*zhe shi pa2 pa3*/
//		GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
//		GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);
		
//		 //UART(ͨ���첽�շ����� ��ʼ		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
//		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
//		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
//		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
//		GPIO_Init(GPIOA,&GPIO_InitStructure); 
//		
//		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; 
//		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
//		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
//		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
//		GPIO_Init(GPIOA,&GPIO_InitStructure);
//		USART_InitStructure.USART_BaudRate = bound;//һ������Ϊ 9600;
//		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//		USART_InitStructure.USART_StopBits = USART_StopBits_1;
//		USART_InitStructure.USART_Parity = USART_Parity_No;
//		USART_InitStructure.USART_HardwareFlowControl =USART_HardwareFlowControl_None;//��Ӳ������������
//		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//�շ�
//		USART_Init(USART2, &USART_InitStructure); //��ʼ������
//		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�����ж�
//		USART_Cmd(USART2, ENABLE); //ʹ�ܴ���
//		//USART_ClearFlag(UART5, USART_FLAG_TC);
//		
//		
//		//#if EN_USART1_RX 
//		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�����ж�
//		//UART5 NVIC ����
//		NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
//		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//��ռ���ȼ� 2
//		NVIC_InitStructure.NVIC_IRQChannelSubPriority =2; //��Ӧ���ȼ� 2
//		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ ͨ��ʹ��
//		NVIC_Init(&NVIC_InitStructure);
//		//#endif
		
	}
	
	

	void UART5_IRQHandler(void) //���� 5 �жϷ������
	{
//		u8 Res;
		#if SYSTEM_SUPPORT_OS //��� SYSTEM_SUPPORT_OS Ϊ�棬����Ҫ֧�� OS.
			OSIntEnter();
		#endif
		
		
		if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
		//�����ж�(���յ������ݱ����� 0x0d 0x0a ��β)
		{
//			Res =USART_ReceiveData(UART5);//(UART5->DR); //��ȡ���յ�������
//			if((USART_RX_STA&0x8000)==0)//����δ���
//			{
//				if(USART_RX_STA&0x4000)//���յ��� 0x0d
//				{
//					if(Res!=0x0a)
//						USART_RX_STA=0;//���մ���,���¿�ʼ
//					else 
//						USART_RX_STA|=0x8000; //���������
//				}
//				else //��û�յ� 0X0D
//				{
//					if(Res==0x0d)
//						USART_RX_STA|=0x4000;
//					else
//					{
//						USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
//						USART_RX_STA++;
//						if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;
//						//�������ݴ���,���¿�ʼ����
//					}
//				}
//			}
//			
		(void)UART5->SR;
		(void)UART5->DR; 
			
			
		}
		#if SYSTEM_SUPPORT_OS //��� SYSTEM_SUPPORT_OS Ϊ�棬����Ҫ֧�� OS.
			OSIntExit();
		#endif
	}
#endif
