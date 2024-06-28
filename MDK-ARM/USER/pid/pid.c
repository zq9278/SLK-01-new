/*
 * @Author: zhangqi zq9278@gmail.com
 * @Date: 2024-06-15 13:31:10
 * @LastEditors: zhangqi zq9278@gmail.com
 * @LastEditTime: 2024-06-20 10:56:46
 * @FilePath: \EIDEd:\Project\SLK01\Software\SLK-01-new\MDK-ARM\USER\pid\pid.c
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
// pid.c

#include "pid.h"
PID_TypeDef HeatPID;
void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float integral_max, float integral_min, float output_max, float output_min, float setpoint) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->previous_error = 0;
    pid->integral = 0;
    pid->integral_max = integral_max;
    pid->integral_min = integral_min;
    pid->output_max = output_max;
    pid->output_min = output_min;
    pid->setpoint = 0; // 初始化设定值为0
}

float PID_Compute(PID_TypeDef *pid, float measured_value, float setpoint) {
    pid->setpoint = setpoint;  // 更新设定值

    float error = pid->setpoint - measured_value;
    pid->integral += error;
    
    // 对积分项进行限幅
    Limit(pid->integral, pid->integral_min, pid->integral_max);

    float derivative = error - pid->previous_error;
    pid->previous_error = error;
    
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    
    // 对输出值进行限幅
    Limit(output, pid->output_min, pid->output_max);
    
    return output;
}
