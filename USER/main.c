#include "stm32f4xx.h"
#include "sys.h"
#include "delay.h"
#include "RC.h"
#include "CAN.h"
#include "LED.h"
#include "usart.h"
#include "chassis_tasks.h"
#include "gimbal_tasks.h"
#include "main.h"


u8 now_use=gimbal;  //ÉÕÂ¼ÔÆÌ¨ gimbal or µ×ÅÌ´úÂë chassis
int main()
{
	delay_init(168);
	uart_init(115200);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	if(now_use==gimbal)
	{
		gimbal_Init();
	}
	else if(now_use==chassis)
	{
		chassis_Init();
	}
	
	
	
	while(1)
	{
		delay_ms(1);
		if(now_use==gimbal)
		{
			send_data_to_chassis();
		}
		else if(now_use==chassis)
		{
			chassis_move();
		}
		
		
	}
}


