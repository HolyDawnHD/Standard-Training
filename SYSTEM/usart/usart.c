#include "usart.h"
#include "sys.h"

u8 USART_RX_BUF[USART_REC_LEN];
u16 USART_RX_STA=0;

#if 1
#pragma import(__use_no_semihosting)
	//标准库需要的支持函数
	struct __FILE
	{
		int handle;
	};
	FILE __stdout;
	//定义_sys_exit()以避免使用半主机模式
	void sys_exit(int x)
	{
		x = x;
	}
	//重定义 fputc 函数
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
		
		//rx tx使用pd2 pc12，复用为uart5
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOA,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,ENABLE);
		//RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
		
		
		
		
		GPIO_PinAFConfig(GPIOD,GPIO_PinSource2,GPIO_AF_UART5);
		GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_UART5);
		//GPIOC12 复用为 USART1

		
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; //GPIOC12 与 GPIOD2
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
		GPIO_Init(GPIOC,&GPIO_InitStructure); //初始化 PC12，PD2
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //GPIOC12 与 GPIOD2
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
		GPIO_Init(GPIOD,&GPIO_InitStructure);
		

		

		
		 //UART(通用异步收发器） 初始化设置，同usart
		USART_InitStructure.USART_BaudRate = bound;//一般设置为 115200;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_HardwareFlowControl =USART_HardwareFlowControl_None;//无硬件数据流控制
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//收发
		USART_Init(UART5, &USART_InitStructure); //初始化串口
		USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);//开启中断
		USART_Cmd(UART5, ENABLE); //使能串口
		//USART_ClearFlag(UART5, USART_FLAG_TC);
		
		
		//#if EN_USART1_RX 
		USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);//开启中断
		//UART5 NVIC 配置
		NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//抢占优先级 2
		NVIC_InitStructure.NVIC_IRQChannelSubPriority =2; //响应优先级 2
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ 通道使能
		NVIC_Init(&NVIC_InitStructure);
		//#endif

		
		/*zhe shi pa2 pa3*/
//		GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
//		GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);
		
//		 //UART(通用异步收发器） 初始		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
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
//		USART_InitStructure.USART_BaudRate = bound;//一般设置为 9600;
//		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//		USART_InitStructure.USART_StopBits = USART_StopBits_1;
//		USART_InitStructure.USART_Parity = USART_Parity_No;
//		USART_InitStructure.USART_HardwareFlowControl =USART_HardwareFlowControl_None;//无硬件数据流控制
//		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//收发
//		USART_Init(USART2, &USART_InitStructure); //初始化串口
//		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启中断
//		USART_Cmd(USART2, ENABLE); //使能串口
//		//USART_ClearFlag(UART5, USART_FLAG_TC);
//		
//		
//		//#if EN_USART1_RX 
//		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启中断
//		//UART5 NVIC 配置
//		NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
//		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//抢占优先级 2
//		NVIC_InitStructure.NVIC_IRQChannelSubPriority =2; //响应优先级 2
//		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ 通道使能
//		NVIC_Init(&NVIC_InitStructure);
//		//#endif
		
	}
	
	

	void UART5_IRQHandler(void) //串口 5 中断服务程序
	{
//		u8 Res;
		#if SYSTEM_SUPPORT_OS //如果 SYSTEM_SUPPORT_OS 为真，则需要支持 OS.
			OSIntEnter();
		#endif
		
		
		if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
		//接收中断(接收到的数据必须是 0x0d 0x0a 结尾)
		{
//			Res =USART_ReceiveData(UART5);//(UART5->DR); //读取接收到的数据
//			if((USART_RX_STA&0x8000)==0)//接收未完成
//			{
//				if(USART_RX_STA&0x4000)//接收到了 0x0d
//				{
//					if(Res!=0x0a)
//						USART_RX_STA=0;//接收错误,重新开始
//					else 
//						USART_RX_STA|=0x8000; //接收完成了
//				}
//				else //还没收到 0X0D
//				{
//					if(Res==0x0d)
//						USART_RX_STA|=0x4000;
//					else
//					{
//						USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
//						USART_RX_STA++;
//						if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;
//						//接收数据错误,重新开始接收
//					}
//				}
//			}
//			
		(void)UART5->SR;
		(void)UART5->DR; 
			
			
		}
		#if SYSTEM_SUPPORT_OS //如果 SYSTEM_SUPPORT_OS 为真，则需要支持 OS.
			OSIntExit();
		#endif
	}
#endif
