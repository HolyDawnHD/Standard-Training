#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "sys.h"
#define fp32 float
#define id3 3
#define id4 4

u8 gimbal_Init(void);
u8 imu_Init(void);


typedef struct 
{
	int16_t angle_speed_x;
	int16_t angle_speed_y;
	int16_t angle_speed_z;
	int16_t pitch;
	int16_t yaw;
	int16_t roll;
	u16 fourdim;
	int16_t a_x;
	int16_t a_y;
	int16_t a_z;
	
}imu;

extern imu imu_buf;
extern u32 trigger_time,trigger_sent;
extern u32 yaw_time,yaw_sent;
extern u16 id;

void trans_data(int16_t *get,imu *set);
void gimbal_move(void);
void trigger_move(void);
void PID_OVER_ZERO_YAW(float *set,float *get);
void PID_OVER_ZERO_IMU (float *set,float *get);

#endif


