#ifndef PID_H
#define PID_H

#include <math.h>

class PID
{

private:
    float min_val_, max_val_;
    float KP, KI, KD;

    float kp, kpT,ki,kiT,kd,kdT;

    float angular_vel_Prev;

    struct e
    {
        float proportional;
        float integral;
        float derivative;
        float u;
        float previous;
    }err;

    float lowpass_filt = 0;
    float lowpass_prev = 0;
    float radian = 0;
    float eProportional;
    float deg2target = 0;

    float encPrev;

    float angular_vel_Filt = 0;
    float angular_vel = 0;

    float PPR = 0;

    float error_integral = 0;
    float error_previous = 0;

    float eIntegral = 0;
    float prevError = 0;

public:

    PID(float MIN_VAL, float MAX_VAL, float kp_, float ki_, float kd_);
    void parameter(float kp_, float ki_, float kd_);
    void parameterT(float kp_, float ki_, float kd_);
    void ppr_total(float total_ppr);
    void reset();
    void resetIntegral();



    float control_angle(float target,float enc, float pwm, float deltaT);
    float control_count(float target, float enc, float pwm, float deltaT);
    float control_angle_speed(float target_angle, float target_speed, float enc, float deltaT);
    float control_speed(float target, float enc, float deltaT);
    float control_base(float error, float speed, int condition, float deltaT);
    float control_default(float target, float curr, float deltaT);
    float convert_speed(float enc, float deltaT);

    float get_filt_vel();
    float get_error();

};

#endif