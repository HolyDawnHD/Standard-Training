#include "stm32f4xx.h"
#include "sys.h"
#include "delay.h"
#include "RC.h"
#include "CAN.h"
#include "LED.h"
#include "usart.h"
#include "chassis_tasks.h"
#include "gimbal_tasks.h"

fp32 kp_speed=20;
fp32 ki_speed=0.003;
fp32 kd_speed=2;
fp32 kp_ecd=200;
fp32 ki_ecd=0;
fp32 kd_ecd=0;
	
PidTypeDef Pid_Speed[5],Pid_Ecd[5];

u8 chassis_Init(void)
{
	
	PID_Init(&Pid_Speed[1],kp_speed,ki_speed,kd_speed,16384.0f,5000.0f);
	PID_Init(&Pid_Speed[2],kp_speed,ki_speed,kd_speed,16384.0f,5000.0f);
	PID_Init(&Pid_Speed[3],kp_speed,ki_speed,kd_speed,16384.0f,5000.0f);
	PID_Init(&Pid_Speed[4],kp_speed,ki_speed,kd_speed,16384.0f,5000.0f);
	
	PID_Init(&Pid_Ecd[1],kp_ecd,ki_ecd,kd_ecd,16384.0f,5000.0f);
	PID_Init(&Pid_Ecd[2],kp_ecd,ki_ecd,kd_ecd,16384.0f,5000.0f);
	PID_Init(&Pid_Ecd[3],kp_ecd,ki_ecd,kd_ecd,16384.0f,5000.0f);
	PID_Init(&Pid_Ecd[4],kp_ecd,ki_ecd,kd_ecd,16384.0f,5000.0f);
	
//	PID_Init(&Pid_Speed[1][2],kp_speed,ki_speed,kd_speed,16384.0f,5000.0f);
//	PID_Init(&Pid_Speed[2][2],kp_speed,ki_speed,kd_speed,16384.0f,5000.0f);
//	PID_Init(&Pid_Speed[3][2],kp_speed,ki_speed,kd_speed,16384.0f,5000.0f);
//	PID_Init(&Pid_Speed[4][2],kp_speed,ki_speed,kd_speed,16384.0f,5000.0f);
//	
//	PID_Init(&Pid_Speed[1][3],kp_speed,ki_speed,kd_speed,16384.0f,5000.0f);
//	PID_Init(&Pid_Speed[2][3],kp_speed,ki_speed,kd_speed,16384.0f,5000.0f);
//	PID_Init(&Pid_Speed[3][3],kp_speed,ki_speed,kd_speed,16384.0f,5000.0f);
//	PID_Init(&Pid_Speed[4][3],kp_speed,ki_speed,kd_speed,16384.0f,5000.0f);
	LED_Init();
	CAN1_mode_init(CAN_SJW_1tq, CAN_BS2_7tq, CAN_BS1_6tq, 3);
	CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_7tq, CAN_BS1_6tq, 3);
	return 0;
	
}


int32_t set_speed1,set_speedy,set_speedx,set_speedw;
int16_t pid_speed1;
int16_t pid_speed2;
int16_t pid_speed3;
int16_t pid_speed4;
int16_t speed_wheel[5];
float memory_ecd[5];

void chassis_move(void)
{
	//u8 chassis_flag=0;
	
	//遥控器拨杆换算速度
	set_speedx=(RC_Crl_chassis.rc.ch3-1024)/660.0f*7000.0f;
	set_speedy=(RC_Crl_chassis.rc.ch2-1024)/660.0f*7000.0f;
	set_speedw=(RC_Crl_chassis.rc.ch0-1024)/660.0f*7000.0f;   
	
	//运动合成
	speed_wheel[1]=-set_speedx-set_speedy+set_speedw;
	speed_wheel[2]=set_speedx-set_speedy+set_speedw;
	speed_wheel[3]=set_speedx+set_speedy+set_speedw;
	speed_wheel[4]=-set_speedx+set_speedy+set_speedw;
	
	if(speed_wheel[1]||speed_wheel[2]||speed_wheel[3]||speed_wheel[4])
	{
		//chassis_flag=1;
		pid_speed1=PID_Calc(&Pid_Speed[1],motor[1].speed_rpm,speed_wheel[1]);
		pid_speed2=PID_Calc(&Pid_Speed[2],motor[2].speed_rpm,speed_wheel[2]);
		pid_speed3=PID_Calc(&Pid_Speed[3],motor[3].speed_rpm,speed_wheel[3]);
		pid_speed4=PID_Calc(&Pid_Speed[4],motor[4].speed_rpm,speed_wheel[4]);
		
		CAN1_CHASSIS_MOTOR_TRANSMIT(pid_speed1,pid_speed2,pid_speed3,pid_speed4);
		
		memory_ecd[1]=motor[1].real_motor_ecd_degree;
		memory_ecd[2]=motor[2].real_motor_ecd_degree;
		memory_ecd[3]=motor[3].real_motor_ecd_degree;
		memory_ecd[4]=motor[4].real_motor_ecd_degree;
		
	}
	else
	{
		
//		PID_OVER_ZERO(&memory_ecd[1],&motor[1].ecd);
//		PID_OVER_ZERO(&memory_ecd[2],&motor[2].ecd);
//		PID_OVER_ZERO(&memory_ecd[3],&motor[3].ecd);
//		PID_OVER_ZERO(&memory_ecd[4],&motor[4].ecd);
		
		pid_speed1=PID_Calc(&Pid_Speed[1],motor[1].speed_rpm,PID_Calc(&Pid_Ecd[1],motor[1].real_motor_ecd_degree,memory_ecd[1]));
		pid_speed2=PID_Calc(&Pid_Speed[2],motor[2].speed_rpm,PID_Calc(&Pid_Ecd[2],motor[2].real_motor_ecd_degree,memory_ecd[2]));
		pid_speed3=PID_Calc(&Pid_Speed[3],motor[3].speed_rpm,PID_Calc(&Pid_Ecd[3],motor[3].real_motor_ecd_degree,memory_ecd[3]));
		pid_speed4=PID_Calc(&Pid_Speed[4],motor[4].speed_rpm,PID_Calc(&Pid_Ecd[4],motor[4].real_motor_ecd_degree,memory_ecd[4]));
		
		CAN1_CHASSIS_MOTOR_TRANSMIT(pid_speed1,pid_speed2,pid_speed3,pid_speed4);
	}
	
	
}

void PID_OVER_ZERO (int16_t *set,int16_t *get)
{
	if(*set - *get> 4096)
	{
		*get += 8192;
		
	}
	else if (*set - *get< -4096)
	{
		*get -= 8192;
	}
	
	
	
	
}


