#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "sys.h"

extern int8_t close_chassis;

u8 chassis_Init(void);
void chassis_move(void);
void PID_OVER_ZERO (int32_t *set,int32_t *get);

#endif


