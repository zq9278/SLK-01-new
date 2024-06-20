#include "pid.h"



PID_typedef HeatPID;

void HeatPIDInit(void)
{
	HeatPID.target_val=42.5;
	HeatPID.actual_val=0; 
	HeatPID.err=0;
	HeatPID.err_last=0;
	HeatPID.integral=0;
	
	HeatPID.maxIntegral=100;
	HeatPID.maxOutput=255;
	
	HeatPID.Kp=0.2;
	HeatPID.Ki=0.3;
	HeatPID.Kd=60;
	
}

u8 PID_realize(PID_typedef *pid,float temp_val) 
{
    pid->err=pid->target_val-temp_val;
    pid->integral+=pid->err;
    Limit(pid->integral,-pid->maxIntegral,pid->maxIntegral);
    pid->actual_val=pid->Kp*pid->err+pid->Ki*pid->integral+pid->Kd*(pid->err-pid->err_last);
    Limit(pid->actual_val,0,pid->maxOutput);
    pid->err_last=pid->err;
    return pid->actual_val;
}
























