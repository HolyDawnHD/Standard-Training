#ifndef __PID_H
#define __PID_H

#include "sys.h"

typedef float fp32;


enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};



typedef struct
{
    uint8_t mode;
    //PID 三参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //最大输出
    fp32 max_iout; //最大积分输出

    fp32 set;//设定转速
    fp32 now_speed;//当前转速

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
    fp32 error[3]; //误差项 0最新 1上一次 2上上次

} PidTypeDef;


void PID_Init(PidTypeDef *pid, const fp32 kp,const fp32 ki,const fp32 kd, fp32 max_out, fp32 max_iout);
extern fp32 PID_Calc(PidTypeDef *pid, fp32 ref, fp32 set);
extern fp32 PID_Change_Calc(PidTypeDef *pid, fp32 ref, fp32 set,fp32 p_min,fp32 p_max,fp32 i_min,fp32 i_max);
extern void PID_clear(PidTypeDef *pid);
int16_t averageFilter(int16_t in_data);
extern int16_t data[10];


#endif

