#include "stm32f4xx.h"
#include "sys.h"
#include "delay.h"
#include "RC.h"


int main()
{
	delay_init(168);
	RC_Init();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	while(1)
	{
		
	}
}


