typedef struct PID
{
    float Kp;
    float Ki;
    float Kd;
    float tau;
    float integrator_lmt;
    float setpoint;
    float measured;
    float integrator;
    float differentiator;
    float prev_err;
    float prev_measured;
    absolute_time_t time_prev;
} PID;

void PID_Coefficents(struct PID *pid_struct, float Kp, float Ki, float Kd, float tau, float intergrator_lmt);

float PID_Compute(absolute_time_t time, struct PID *pid);
