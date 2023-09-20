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
	LED_G=0;
	return 0;
	
}


