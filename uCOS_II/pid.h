#ifndef PID_H
#define PID_H

typedef struct {
    float Kp, Ki, Kd;
} PIDParam;

void PitchPID(float, PIDParam, PIDParam);
void RollPID(float, PIDParam, PIDParam);
void YawPID(float, PIDParam, PIDParam);

#endif