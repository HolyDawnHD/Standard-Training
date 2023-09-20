#include "PID.h"
#include "usart.h"


#ifndef NULL
#define NULL 0
#endif

#define LimitMax(input,max) \
{                           \
	if(input>max)           \
	{                       \
		input=max;          \
		                    \
	}                       \
	else if(input<-max)     \
	{                       \
		input=-max;         \
	}                       \
}                           \

void PID_Init(PidTypeDef *pid, const fp32 kp,const fp32 ki,const fp32 kd, fp32 max_out, fp32 max_iout)
{
    pid->Kp =kp;
    pid->Ki =ki;
    pid->Kd =kd;
    pid->max_out = max_out;              //pid限幅
    pid->max_iout = max_iout;           //积分限幅
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

fp32 PID_Calc(PidTypeDef *pid, fp32 ref, int32_t set)
{
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->now_speed = ref;
    pid->error[0] = set - ref;
//位置式pid
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
		//printf("%f\r\n",pid->out);
		return pid->out;
	

    
}


void PID_clear(PidTypeDef *pid)
{
    if (pid == NULL)
    {
        return;
    }
	
    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->now_speed = pid->set = 0.0f;
}

