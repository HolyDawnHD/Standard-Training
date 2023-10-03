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
	last_pos_yaw=imu_buf.yaw;
	rc_time=56;
	
	while(1)
	{
		delay_ms(1);
		if(now_use==gimbal)
		{
			send_data_to_chassis();
			gimbal_move();
			trigger_move();
			rc_time++;
			if(rc_time>50)
			{
				close_rc=1;
			}
			else 
			{
				close_rc=0;
			}
			if(trigger_sent)
			{
				trigger_time++;
			}
			else
			{
				trigger_time=0;
			}
		}
		else if(now_use==chassis)
		{
			chassis_move();
		}
		if(rc_time>1000)
		{
			rc_time=51;
		}
		
	}
}


