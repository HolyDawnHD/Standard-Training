#ifndef __CAN_H
#define __CAN_H

#include "sys.h"
#include "PID.h"
#include "RC.h"
#include "stm32f4xx_can.h"

#define speed 0
#define position 1

void CAN1_mode_init(uint8_t tsjw,uint8_t tbs2,uint8_t tbs1,uint16_t brp);
void CAN1_RX0_IRQHandler(void);
void CAN1_RC_TRANSMIT(void);
void CAN2_mode_init(uint8_t tsjw,uint8_t tbs2,uint8_t tbs1,uint16_t brp);
void CAN2_RX0_IRQHandler(void);
uint8_t CAN2_RC_TRANSMIT(volatile uint8_t *send_data);
void get_rc_measure(RC_Ctl_t *rc,CanRxMsg *rx_message);
void CAN1_CHASSIS_MOTOR_TRANSMIT(int16_t val1,int16_t val2,int16_t val3,int16_t val4);
void CAN1_GIMBAL_PITCH_TRIGGER_TRANSMIT(int16_t val_pitch,int16_t val_trigger);
void CAN2_GIMBAL_YAW_TRANSMIT(int16_t val);
//void CAN1_GIMBAL_TRIGGER_TRANSMIT(int16_t val);


typedef struct
{
    int16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
	int16_t round;
	float one_ecd;
	float real_motor_ecd_degree;
} motor_measure_t;

void get_pitch_yaw_measure(motor_measure_t *motor,CanRxMsg *rx_message);
void get_trigger_measure(motor_measure_t *motor,CanRxMsg *rx_message);
extern motor_measure_t motor[5],pitch,yaw,trigger;
extern int32_t rc_time,close_rc,last_pitch_pos;
extern float last_pos_yaw;


#endif

