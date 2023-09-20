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
fp32 kp_ecd=5;
fp32 ki_ecd=0;
fp32 kd_ecd=0;
	
PidTypeDef Pid_Speed[4][4];

u8 chassis_Init(void)
{
	
	PID_Init(&Pid_Speed[1][0],kp_speed,ki_speed,kd_speed,16384.0f,5000.0f);
	PID_Init(&Pid_Speed[2][0],kp_speed,ki_speed,kd_speed,16384.0f,5000.0f);
	PID_Init(&Pid_Speed[3][0],kp_speed,ki_speed,kd_speed,16384.0f,5000.0f);
	PID_Init(&Pid_Speed[4][0],kp_speed,ki_speed,kd_speed,16384.0f,5000.0f);
	
	PID_Init(&Pid_Speed[1][2],kp_speed,ki_speed,kd_speed,16384.0f,5000.0f);
	PID_Init(&Pid_Speed[2][2],kp_speed,ki_speed,kd_speed,16384.0f,5000.0f);
	PID_Init(&Pid_Speed[3][2],kp_speed,ki_speed,kd_speed,16384.0f,5000.0f);
	PID_Init(&Pid_Speed[4][2],kp_speed,ki_speed,kd_speed,16384.0f,5000.0f);
	
	PID_Init(&Pid_Speed[1][3],kp_speed,ki_speed,kd_speed,16384.0f,5000.0f);
	PID_Init(&Pid_Speed[2][3],kp_speed,ki_speed,kd_speed,16384.0f,5000.0f);
	PID_Init(&Pid_Speed[3][3],kp_speed,ki_speed,kd_speed,16384.0f,5000.0f);
	PID_Init(&Pid_Speed[4][3],kp_speed,ki_speed,kd_speed,16384.0f,5000.0f);
	LED_Init();
	CAN1_mode_init(CAN_SJW_1tq, CAN_BS2_7tq, CAN_BS1_6tq, 3);
	CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_7tq, CAN_BS1_6tq, 3);
	return 0;
	
}


int32_t set_speed1,set_speed2,set_speed3,set_speed0;
int16_t speed1;
int16_t speed2;
int16_t speed3;
int16_t speed4;


void chassis_move(void)
{
	u8 chassis_flag=0;
	set_speed3=(RC_Crl_chassis.rc.ch3-1024)/660.0f*7000.0f;
	set_speed2=(RC_Crl_chassis.rc.ch2-1024)/660.0f*7000.0f;
	set_speed0=(RC_Crl_chassis.rc.ch0-1024)/660.0f*7000.0f;   
	
	//Ç°ºó
	if(set_speed3)
	{
		chassis_flag=1;
		speed1=PID_Calc(&Pid_Speed[1][3],motor[1].speed_rpm,-set_speed3);
		speed2=PID_Calc(&Pid_Speed[2][3],motor[2].speed_rpm,set_speed3);
		speed3=PID_Calc(&Pid_Speed[3][3],motor[3].speed_rpm,set_speed3);
		speed4=PID_Calc(&Pid_Speed[4][3],motor[4].speed_rpm,-set_speed3);
		
		CAN1_CHASSIS_MOTOR_TRANSMIT(speed1,speed2,speed3,speed4);
	}
	//×óÓÒ
	if(set_speed2)
	{
		chassis_flag=1;
		speed1=PID_Calc(&Pid_Speed[1][2],motor[1].speed_rpm,-set_speed2);
		speed2=PID_Calc(&Pid_Speed[2][2],motor[2].speed_rpm,-set_speed2);
		speed3=PID_Calc(&Pid_Speed[3][2],motor[3].speed_rpm,set_speed2);
		speed4=PID_Calc(&Pid_Speed[4][2],motor[4].speed_rpm,set_speed2);
		
		CAN1_CHASSIS_MOTOR_TRANSMIT(speed1,speed2,speed3,speed4);
	}
	//×óÓÒÐý×ª
	if(set_speed0)
	{
		chassis_flag=1;
		speed1=PID_Calc(&Pid_Speed[1][0],motor[1].speed_rpm,set_speed0);
		speed2=PID_Calc(&Pid_Speed[2][0],motor[2].speed_rpm,set_speed0);
		speed3=PID_Calc(&Pid_Speed[3][0],motor[3].speed_rpm,set_speed0);
		speed4=PID_Calc(&Pid_Speed[4][0],motor[4].speed_rpm,set_speed0);
		
		CAN1_CHASSIS_MOTOR_TRANSMIT(speed1,speed2,speed3,speed4);
	}
	if(chassis_flag==0)
	{
		CAN1_CHASSIS_MOTOR_TRANSMIT(0,0,0,0);
	}
	
	
}




