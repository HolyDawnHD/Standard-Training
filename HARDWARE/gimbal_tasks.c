#include "stm32f4xx.h"
#include "sys.h"
#include "delay.h"
#include "RC.h"
#include "CAN.h"
#include "LED.h"
#include "usart.h"
#include "chassis_tasks.h"
#include "gimbal_tasks.h"
#include "PID.h"


u16 id=id4;
PidTypeDef Pid_Speed_pitch,Pid_Ecd_pitch,Pid_Speed_yaw,Pid_Ecd_yaw,Pid_Speed_trigger,Pid_Ecd_trigger;

fp32 kp_speed_pitch=480;
fp32 ki_speed_pitch=0;
fp32 kd_speed_pitch=0;
fp32 kp_ecd_pitch=0.2;
fp32 ki_ecd_pitch=0;
fp32 kd_ecd_pitch=0;

/********id4**********/
fp32 kp_speed_yaw=480;
fp32 ki_speed_yaw=0;
fp32 kd_speed_yaw=0;
fp32 kp_ecd_yaw=0.2;
fp32 ki_ecd_yaw=0;
fp32 kd_ecd_yaw=0;



/********id3**********/
//fp32 kp_speed_yaw=480;
//fp32 ki_speed_yaw=0;
//fp32 kd_speed_yaw=0;
//fp32 kp_ecd_yaw=0.2;
//fp32 ki_ecd_yaw=0;
//fp32 kd_ecd_yaw=0;


fp32 kp_speed_trigger=10;
fp32 ki_speed_trigger=0;
fp32 kd_speed_trigger=0;
fp32 kp_ecd_trigger=300;
fp32 ki_ecd_trigger=0;
fp32 kd_ecd_trigger=0;

motor_measure_t pitch,yaw,trigger;
int32_t set_pitch,get_pitch,now_pitch,last_pitch_pos;
int32_t first_open;
float last_pos_yaw,now_pos_yaw;
RC_Ctl_t RC_Ctl;

/****************云台初始化**********************************/

int16_t pid_init_pos;
u8 gimbal_Init(void)
{
	
	RC_Init();
	LED_Init();
	CAN1_mode_init(CAN_SJW_1tq, CAN_BS2_7tq, CAN_BS1_6tq, 3);
	CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_7tq, CAN_BS1_6tq, 3);
	
	imu_Init();
	
	PID_Init(&Pid_Speed_pitch,kp_speed_pitch,ki_speed_pitch,kd_speed_pitch,30000.0f,5000.0f);
	PID_Init(&Pid_Ecd_pitch,kp_ecd_pitch,ki_ecd_pitch,kd_ecd_pitch,30000.0f,8191.0f);
	
	PID_Init(&Pid_Speed_yaw,kp_speed_yaw,ki_speed_yaw,kd_speed_yaw,30000.0f,3000.0f);
	PID_Init(&Pid_Ecd_yaw,kp_ecd_yaw,ki_ecd_yaw,kd_ecd_yaw,15000.0f,5000.0f);
	
	PID_Init(&Pid_Speed_trigger,kp_speed_trigger,ki_speed_trigger,kd_speed_trigger,10000.0f,5000.0f);
	PID_Init(&Pid_Ecd_trigger,kp_ecd_trigger,ki_ecd_trigger,kd_ecd_trigger,10000.0f,5000.0f);
	
	first_open=1;
	last_pos_yaw=imu_buf.yaw;
	if(id==id4)
	{
		last_pitch_pos=3975; //680 3975
	}
	else if(id==id3)
	{
		last_pitch_pos=680;
	}
	
	LED_G=0;
	return 0;
	
}

/***************************陀螺仪初始化*********************/
u8 imu_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
		
	//pc6 pc7分为tx rx 复用为USART6_TX  USART6_RX
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);
		

	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6);
	//GPIOC 复用为 USART6

		
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; //GPIOC6
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOC,&GPIO_InitStructure); //初始化 PC12，PD2
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; //GPIOC7
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOC,&GPIO_InitStructure);
		
		
	 //UART(通用异步收发器） 初始化设置，同usart
	USART_InitStructure.USART_BaudRate = 921600;//一般设置为 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl =USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//收发
	USART_Init(USART6, &USART_InitStructure); //初始化串口
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);//开启中断
	USART_Cmd(USART6, ENABLE); //使能串口
		
		
	//#if EN_USART1_RX 
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);//开启中断
	//UART5 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//抢占优先级 0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0; //响应优先级 0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ 通道使能
	NVIC_Init(&NVIC_InitStructure);
	return 0;
}


/****************接收陀螺仪数据**********************************/
int16_t num_buf=0;
imu imu_buf;

int16_t BUF[55];
void USART6_IRQHandler(void) //串口 6 中断服务程序
{
	if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)
	{
		USART_ClearFlag(USART6, USART_IT_RXNE);
		BUF[num_buf] =USART_ReceiveData(USART6);//(USART6->DR); //读取接收到的数据
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


/***********************转存陀螺仪数据**************************/
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


/*************************云台pitch yaw移动********************/

float set_speed_yaw,set_yaw;
int16_t pid_yaw;
float set_ecd_yaw=0.0f;
float now_pos;
int16_t pid_trigger;
u32 yaw_time,yaw_sent;
float change_kpmax_speed=80.0f,change_kpmin_speed=400.0f,change_kimax_speed=0.0f,change_kimin_speed=0.0f,change_kpmax_ecd=0.4f,change_kpmin_ecd=0.2f,change_kimax_ecd=0.0f,change_kimin_ecd=0.0f;

void gimbal_move(void)
{
/*****************pitch电机控制***************************/
	//get_pitch=(RC_Ctl.rc.ch1-1024)/660.0f; 
//	if(RC_Ctl.rc.ch1-1024>300)
//	{
//		get_pitch=1;
//	}
//	else if(RC_Ctl.rc.ch1-1024<-300&&RC_Ctl.rc.ch1!=0)
//	{
//		get_pitch=-1;
//	}
	if(RC_Ctl.rc.ch1-1024!=0)
	{
		get_pitch=(RC_Ctl.rc.ch1-1024)/660.f*2.0f;
	}
	else
	{
		get_pitch=0;
	}
	
	if(get_pitch&&close_rc==0)
	{
		set_pitch+=get_pitch;
		if(id==id4)
		{
			if(set_pitch>=4375) //1100  4375
			{
				set_pitch=4375;
			}
			if(set_pitch<=3245) //-150  3245
			{
				set_pitch=3245;
			}
		}
		if(id==id3)
		{
			if(set_pitch>=1100) //1100  4375
			{
				set_pitch=1100;
			}
			if(set_pitch<=-150) //-150  3245
			{
				set_pitch=-150;
			}
		}
		
		last_pitch_pos=set_pitch;
		
	}
	else if(close_rc==0)
	{
		set_pitch=last_pitch_pos;
	}
	else
	{
		last_pitch_pos=pitch.ecd;
	}
	
	if(close_rc==0)
	{
		
		now_pitch=pitch.ecd;
		PID_OVER_ZERO(&set_pitch,&now_pitch);
		pid_init_pos=PID_Calc(&Pid_Speed_pitch,pitch.speed_rpm,PID_Calc(&Pid_Ecd_pitch,now_pitch,set_pitch));
		CAN1_GIMBAL_PITCH_TRIGGER_TRANSMIT(pid_init_pos,pid_trigger);
		
	}
	else
	{
		CAN1_GIMBAL_PITCH_TRIGGER_TRANSMIT(0,pid_trigger);
		
	}
//	fp32 now_pitch_pos=pitch.real_motor_ecd_degree*8191.0f;
	
/*****************************yaw电机控制***************************/

	
	/***********这是调试代码**************/
//	now_pos=imu_buf.yaw;
//	pid_yaw=PID_Calc(&Pid_Speed_yaw,yaw.speed_rpm,PID_Calc(&Pid_Ecd_yaw,now_pos,set_ecd_yaw));
//	CAN2_GIMBAL_YAW_TRANSMIT(pid_yaw);
	/************************************/
	
	/**********这是速度环转头代码*********/
	
//	set_speed_yaw=-(RC_Ctl.rc.ch0-1024)/660.0f*300.0f;
//	if(set_speed_yaw&&close_rc==0)
//	{
//			
//			pid_yaw=PID_Calc(&Pid_Speed_yaw,yaw.speed_rpm,set_speed_yaw);
//			CAN2_GIMBAL_YAW_TRANSMIT(pid_yaw);
//			last_pos_yaw=imu_buf.yaw/3600.0f;
//		
//	
//	}
//	else if(close_rc==0)
//	{

//			now_pos_yaw=imu_buf.yaw/3600.0f;
//			PID_OVER_ZERO_YAW(&last_pos_yaw,&now_pos_yaw);
//			pid_yaw=PID_Calc(&Pid_Speed_yaw,yaw.speed_rpm,PID_Calc(&Pid_Ecd_yaw,now_pos_yaw,last_pos_yaw));
//			CAN2_GIMBAL_YAW_TRANSMIT(pid_yaw);
//		
//	}
//	else
//	{
//		CAN2_GIMBAL_YAW_TRANSMIT(0);
//		last_pos_yaw=imu_buf.yaw/3600.0f;
//		first_open=1;
//		
//	}

	
	/*********这是位置环转头代码*********/
//	if(RC_Ctl.rc.ch0-1024>100)
//	{
//		set_speed_yaw=-2.0f;
//	}
//	else if(RC_Ctl.rc.ch0-1024<-100&&RC_Ctl.rc.ch0!=0)
//	{
//		set_speed_yaw=2.0f;
//	}
	if(RC_Ctl.rc.ch0-1024!=0)
	{
		set_speed_yaw=(1024-RC_Ctl.rc.ch0)/660.f*2.0f;
	}
	else
	{
		set_speed_yaw=0.0f;
	}
	
	if(set_speed_yaw)
	{
		set_yaw+=set_speed_yaw;
		if(set_yaw>1800)
		{
			set_yaw=-1800;
		}
		if(set_yaw<-1800)
		{
			set_yaw=1800;
		}
		last_pos_yaw=set_yaw;
	}
	else
	{
		set_yaw=last_pos_yaw;
	}
	if(close_rc==0)
	{
		
		now_pos=imu_buf.yaw;
		PID_OVER_ZERO_IMU(&set_yaw,&now_pos);
		pid_yaw=PID_Calc(&Pid_Speed_yaw,yaw.speed_rpm,PID_Calc(&Pid_Ecd_yaw,now_pos,set_yaw));
		
		//pid_yaw=averageFilter(pid_yaw);
		//pid_yaw=PID_Change_Calc(&Pid_Speed_yaw,yaw.speed_rpm,PID_Change_Calc(&Pid_Ecd_yaw,now_pos,set_yaw,change_kpmin_ecd,change_kpmax_ecd,change_kimin_ecd,change_kimax_ecd),change_kpmin_speed,change_kpmax_speed,change_kimin_speed,change_kimax_speed);
		
		CAN2_GIMBAL_YAW_TRANSMIT(pid_yaw);
		//last_pos_yaw=now_pos;
	}
	else
	{
		CAN2_GIMBAL_YAW_TRANSMIT(0);
		last_pos_yaw=imu_buf.yaw;
	}
}

/*****************************trigger电机控制*****************************/
float now_trigger_pos,set_trigger_pos=20.0f,last_trigger_pos=0.0f;
float trigger_speed=3000;
u32 trigger_time=0,trigger_sent=0;
void trigger_move(void)
{
	if(close_rc==0)
	{
		if(trigger_time>=1500)
		{
			now_trigger_pos=trigger.real_motor_ecd_degree;
			set_trigger_pos=last_trigger_pos-3.6f;
			
			pid_trigger=PID_Calc(&Pid_Speed_trigger,trigger.speed_rpm,PID_Calc(&Pid_Ecd_trigger,now_trigger_pos,set_trigger_pos));
			CAN1_GIMBAL_PITCH_TRIGGER_TRANSMIT(pid_init_pos,pid_trigger);
			if(now_trigger_pos-set_trigger_pos<=0.1f&&now_trigger_pos-set_trigger_pos>=-0.1f)
			{
				trigger_sent=0;
				trigger_time=0;
				last_trigger_pos=last_trigger_pos-3.6f;
			}
			else
				trigger_sent=1;
			
		}
		else
		{
			if(RC_Ctl.rc.s2==1)
			{
				now_trigger_pos=trigger.real_motor_ecd_degree;
				set_trigger_pos=last_trigger_pos+3.6f;
				pid_trigger=PID_Calc(&Pid_Speed_trigger,trigger.speed_rpm,PID_Calc(&Pid_Ecd_trigger,now_trigger_pos,set_trigger_pos));
				CAN1_GIMBAL_PITCH_TRIGGER_TRANSMIT(pid_init_pos,pid_trigger);
				if((now_trigger_pos - set_trigger_pos<=0.1f)&&(now_trigger_pos - set_trigger_pos>=-0.1f))
				{
					trigger_sent=0;
					last_trigger_pos=trigger.real_motor_ecd_degree;
				}
				else
					trigger_sent=1;
			}
			if(RC_Ctl.rc.s2==3)
			{
				now_trigger_pos=trigger.real_motor_ecd_degree;
				
				pid_trigger=PID_Calc(&Pid_Speed_trigger,trigger.speed_rpm,PID_Calc(&Pid_Ecd_trigger,now_trigger_pos,set_trigger_pos));
				CAN1_GIMBAL_PITCH_TRIGGER_TRANSMIT(pid_init_pos,pid_trigger);
				if((now_trigger_pos - set_trigger_pos<=0.1f)&&(now_trigger_pos - set_trigger_pos>=-0.1f))
				{
					trigger_sent=0;
					last_trigger_pos=trigger.real_motor_ecd_degree;
				}
				else
					trigger_sent=1;
				
				
			}
			if(RC_Ctl.rc.s2==2)
			{
				now_trigger_pos=trigger.real_motor_ecd_degree;
				set_trigger_pos=last_trigger_pos+3.6f;
				
				pid_trigger=PID_Calc(&Pid_Speed_trigger,trigger.speed_rpm,PID_Calc(&Pid_Ecd_trigger,now_trigger_pos,set_trigger_pos));
				CAN1_GIMBAL_PITCH_TRIGGER_TRANSMIT(pid_init_pos,pid_trigger);
				if((now_trigger_pos - set_trigger_pos<=0.1f)&&(now_trigger_pos - set_trigger_pos>=-0.1f))
				{
					trigger_sent=0;
					//last_trigger_pos=trigger.real_motor_ecd_degree;
				}
				else
					trigger_sent=1;
				
			}
			
		}
	}
	else if(close_rc==1)
	{
		now_trigger_pos=trigger.real_motor_ecd_degree;
		set_trigger_pos=last_trigger_pos;
		pid_trigger=PID_Calc(&Pid_Speed_trigger,trigger.speed_rpm,PID_Calc(&Pid_Ecd_trigger,now_trigger_pos,set_trigger_pos));
		CAN1_GIMBAL_PITCH_TRIGGER_TRANSMIT(pid_init_pos,0);
		
	}
	
	
}


/***************归一化的过零点**********************/

void PID_OVER_ZERO_YAW (float *set,float *get)
{
	if(*set - *get> 0.5f)
	{
		*get += 1.0f;
	}
	else if (*set - *get< -0.5f)
	{
		*get -= 1.0f;
	}
	
}

/***************陀螺仪的过零点**********************/

void PID_OVER_ZERO_IMU (float *set,float *get)
{
	if(*set - *get> 1800.0f)
	{
		*get += 3600.0f;
	}
	else if (*set - *get< -1800.0f)
	{
		*get -= 3600.0f;
	}
	
}
