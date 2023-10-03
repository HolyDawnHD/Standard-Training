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

fp32 PID_Calc(PidTypeDef *pid, fp32 ref, fp32 set)
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
fp32 PID_Change_Calc(PidTypeDef *pid, fp32 ref, fp32 set,fp32 p_min,fp32 p_max,fp32 i_min,fp32 i_max)
{
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->now_speed = ref;
    pid->error[0] = set - ref;
	
	float err=pid->error[0]<0? -pid->error[0]:pid->error[0];
	pid->Kp=p_min+p_max*(1.0f-1.0f/(1+(0.01f*err)));
	//pid->Ki=i_min+i_max*(1.0f/(1.0f+0.01f*err));
	
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

int16_t data[10];
int16_t averageFilter(int16_t in_data)
{
	int16_t sum=0;
	for(int i=0;i<9;i++)
	{
		data[i]=data[i+1];
		sum=sum+data[i];
	}
	data[9]=in_data;
	sum=sum+data[9];
	
	return (sum/10);
}

