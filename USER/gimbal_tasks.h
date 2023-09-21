#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "sys.h"

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

void trans_data(int16_t *get,imu *set);

#endif


