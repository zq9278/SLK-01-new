// pid.h

#ifndef PID_H
#define PID_H

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float previous_error;
    float integral;
    float integral_max;   // �����޷�ֵ
    float integral_min;   // �������޷�ֵ
    float output_max;     // ����޷�ֵ
    float output_min;     // ������޷�ֵ
    float setpoint;       // �趨ֵ
} PID_TypeDef;

// ��ʼ��PID������
void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float integral_max, float integral_min, float output_max, float output_min, float setpoint);

// ����PID�������
float PID_Compute(PID_TypeDef *pid, float measured_value, float setpoint);

// �޷���
#define Limit(x, min, max) ((x) = (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x))))

#endif // PID_H
