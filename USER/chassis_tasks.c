#include "stm32f4xx.h"
#include "sys.h"
#include "delay.h"
#include "RC.h"
#include "CAN.h"
#include "LED.h"
#include "usart.h"
#include "chassis_tasks.h"
#include "gimbal_tasks.h"
#include "math.h"

#define pi 3.1415926

PidTypeDef Pid_Speed_chassis[5],Pid_Ecd_chassis[5],Pid_Speed_yaw0,Pid_Ecd_yaw0;
int8_t close_chassis=1;
int32_t set_yaw_pos=5120;

fp32 kp_speed_chassis=20;
fp32 ki_speed_chassis=0.003;
fp32 kd_speed_chassis=2;
fp32 kp_ecd_chassis=250;  //200
fp32 ki_ecd_chassis=0;
fp32 kd_ecd_chassis=0;

fp32 kp_speed_yaw0=3;
fp32 ki_speed_yaw0=0;
fp32 kd_speed_yaw0=0;
fp32 kp_ecd_yaw0=5;
fp32 ki_ecd_yaw0=0;
fp32 kd_ecd_yaw0=0;


/************************底盘初始化**********************/
u8 chassis_Init(void)
{
	PID_Init(&Pid_Speed_yaw0,kp_speed_yaw0,ki_speed_yaw0,kd_speed_yaw0,25000.0f,5000.0f);
	PID_Init(&Pid_Ecd_yaw0,kp_ecd_yaw0,ki_ecd_yaw0,kd_ecd_yaw0,25000.0f,5000.0f);
	
	PID_Init(&Pid_Speed_chassis[1],kp_speed_chassis,ki_speed_chassis,kd_speed_chassis,16384.0f,5000.0f);
	PID_Init(&Pid_Speed_chassis[2],kp_speed_chassis,ki_speed_chassis,kd_speed_chassis,16384.0f,5000.0f);
	PID_Init(&Pid_Speed_chassis[3],kp_speed_chassis,ki_speed_chassis,kd_speed_chassis,16384.0f,5000.0f);
	PID_Init(&Pid_Speed_chassis[4],kp_speed_chassis,ki_speed_chassis,kd_speed_chassis,16384.0f,5000.0f);
	
	PID_Init(&Pid_Ecd_chassis[1],kp_ecd_chassis,ki_ecd_chassis,kd_ecd_chassis,16384.0f,5000.0f);
	PID_Init(&Pid_Ecd_chassis[2],kp_ecd_chassis,ki_ecd_chassis,kd_ecd_chassis,16384.0f,5000.0f);
	PID_Init(&Pid_Ecd_chassis[3],kp_ecd_chassis,ki_ecd_chassis,kd_ecd_chassis,16384.0f,5000.0f);
	PID_Init(&Pid_Ecd_chassis[4],kp_ecd_chassis,ki_ecd_chassis,kd_ecd_chassis,16384.0f,5000.0f);
	
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
	
	if(id==id4)
	{
		set_yaw_pos=5120;
	}
	if(id==id3)
	{
		set_yaw_pos=0;
	}
	
	return 0;
	
}

/*********************************底盘移动函数******************/
float set_speed1,set_speedy,set_speedx,set_speedw;
int16_t pid_speed1;
int16_t pid_speed2;
int16_t pid_speed3;
int16_t pid_speed4;
float speed_wheel[5];
float memory_ecd[5];

int32_t now_yaw_pos;

void chassis_move(void)
{
	
	//遥控器拨杆换算速度
	if(RC_Crl_chassis.rc.s1==3)
	{
		set_speedx=(RC_Crl_chassis.rc.ch3-1024)/660.0f*3000.0f;
		set_speedy=(RC_Crl_chassis.rc.ch2-1024)/660.0f*3000.0f;
		now_yaw_pos=yaw.ecd;
		PID_OVER_ZERO(&set_yaw_pos,&now_yaw_pos);
		set_speedw=PID_Calc(&Pid_Speed_yaw0,yaw.speed_rpm,PID_Calc(&Pid_Ecd_yaw0,now_yaw_pos,set_yaw_pos))/25000.0f*3500.0f;
		
	}
	if(RC_Crl_chassis.rc.s1==1)
	{
		set_speedx=0;
		set_speedy=0;
		set_speedw=0;
	}
	if(RC_Crl_chassis.rc.s1==2)
	{
		set_speedw=3000.0f;
		if(id==id4)
		{
			set_speedx=((RC_Crl_chassis.rc.ch3-1024.0f)/660.0f*3000.0f)*cos((double)2.0f*pi*(5176.0f-yaw.ecd)/8191.0f) - ((RC_Crl_chassis.rc.ch2-1024.0f)/660.0f*3000.0f)*sin((double)2.0f*pi*(5176.0f-yaw.ecd)/8191.0f);
			set_speedy=((RC_Crl_chassis.rc.ch3-1024.0f)/660.0f*3000.0f)*sin((double)2.0f*pi*(5176.0f-yaw.ecd)/8191.0f) + ((RC_Crl_chassis.rc.ch2-1024.0f)/660.0f*3000.0f)*cos((double)2.0f*pi*(5176.0f-yaw.ecd)/8191.0f);
		}
		if(id==id3)
		{
			set_speedx=((RC_Crl_chassis.rc.ch3-1024.0f)/660.0f*3000.0f)*cos((double)2.0f*pi*(0.0f-yaw.ecd)/8191.0f) - ((RC_Crl_chassis.rc.ch2-1024.0f)/660.0f*3000.0f)*sin((double)2.0f*pi*(0.0f-yaw.ecd)/8191.0f);
			set_speedy=((RC_Crl_chassis.rc.ch3-1024.0f)/660.0f*3000.0f)*sin((double)2.0f*pi*(0.0f-yaw.ecd)/8191.0f) + ((RC_Crl_chassis.rc.ch2-1024.0f)/660.0f*3000.0f)*cos((double)2.0f*pi*(0.0f-yaw.ecd)/8191.0f);
		}
	}
	
	
	//set_speedw=0;  
	
	//运动合成
	speed_wheel[1]=-set_speedx-set_speedy+set_speedw;
	speed_wheel[2]=set_speedx-set_speedy+set_speedw;
	speed_wheel[3]=set_speedx+set_speedy+set_speedw;
	speed_wheel[4]=-set_speedx+set_speedy+set_speedw;
	
	if((speed_wheel[1]||speed_wheel[2]||speed_wheel[3]||speed_wheel[4])&&close_chassis==0)
	{
		//chassis_flag=1;
		pid_speed1=PID_Calc(&Pid_Speed_chassis[1],motor[1].speed_rpm,speed_wheel[1]);
		pid_speed2=PID_Calc(&Pid_Speed_chassis[2],motor[2].speed_rpm,speed_wheel[2]);
		pid_speed3=PID_Calc(&Pid_Speed_chassis[3],motor[3].speed_rpm,speed_wheel[3]);
		pid_speed4=PID_Calc(&Pid_Speed_chassis[4],motor[4].speed_rpm,speed_wheel[4]);
		
		CAN1_CHASSIS_MOTOR_TRANSMIT(pid_speed1,pid_speed2,pid_speed3,pid_speed4);
		
		memory_ecd[1]=motor[1].real_motor_ecd_degree;
		memory_ecd[2]=motor[2].real_motor_ecd_degree;
		memory_ecd[3]=motor[3].real_motor_ecd_degree;
		memory_ecd[4]=motor[4].real_motor_ecd_degree;
		
	}
	else if(close_chassis==0)
	{
		
//		pid_speed1=PID_Calc(&Pid_Speed_chassis[1],motor[1].speed_rpm,PID_Calc(&Pid_Ecd_chassis[1],motor[1].real_motor_ecd_degree,memory_ecd[1]));
//		pid_speed2=PID_Calc(&Pid_Speed_chassis[2],motor[2].speed_rpm,PID_Calc(&Pid_Ecd_chassis[2],motor[2].real_motor_ecd_degree,memory_ecd[2]));
//		pid_speed3=PID_Calc(&Pid_Speed_chassis[3],motor[3].speed_rpm,PID_Calc(&Pid_Ecd_chassis[3],motor[3].real_motor_ecd_degree,memory_ecd[3]));
//		pid_speed4=PID_Calc(&Pid_Speed_chassis[4],motor[4].speed_rpm,PID_Calc(&Pid_Ecd_chassis[4],motor[4].real_motor_ecd_degree,memory_ecd[4]));
		
		pid_speed1=PID_Calc(&Pid_Speed_chassis[1],motor[1].speed_rpm,0);
		pid_speed2=PID_Calc(&Pid_Speed_chassis[2],motor[2].speed_rpm,0);
		pid_speed3=PID_Calc(&Pid_Speed_chassis[3],motor[3].speed_rpm,0);
		pid_speed4=PID_Calc(&Pid_Speed_chassis[4],motor[4].speed_rpm,0);
		
		CAN1_CHASSIS_MOTOR_TRANSMIT(pid_speed1,pid_speed2,pid_speed3,pid_speed4);
	}
	else
	{
		
		CAN1_CHASSIS_MOTOR_TRANSMIT(0,0,0,0);
		
		memory_ecd[1]=motor[1].real_motor_ecd_degree;
		memory_ecd[2]=motor[2].real_motor_ecd_degree;
		memory_ecd[3]=motor[3].real_motor_ecd_degree;
		memory_ecd[4]=motor[4].real_motor_ecd_degree;
	}
	
	
}


/***********************过零点************************/
void PID_OVER_ZERO (int32_t *set,int32_t *get)
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


