#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "Arduino.h"

class Kinematic
{
public:
    enum base
    {
        DIFFERENSTIAL_DRIVE,
        SKID_STEER,
        MECANUM,
        OMNI
    };

    base base_platform_;

    struct rps
    {
        float motor1;
        float motor2;
        float motor3;
        float motor4;
    };

    struct velocities
    {
        float linear_x;
        float linear_y;
        float angular_z;
    };

    struct position
    {
        float linear_x;
        float linear_y;
        float angular_z;
    };

    struct pwm
    {
        float motor1;
        float motor2;
        float motor3;
        float motor4;
    };

    Kinematic(base robot_base, int motor_max_rps, float max_rps_ratio,
              float motor_operating_voltage, float motor_power_max_voltage,
              float wheel_diameter, float robot_diameter);

    velocities getVelocities(float rps1, float rps2, float rps3, float rps4);
    rps getRPS(float linear_x, float linear_y, float angular_z, float imu_angular_z);
    float getMaxRPS();
    float toRad(float deg);
    float toDeg(float rad);

private:
    rps calculateRPS(float linear_x, float linear_y, float angular_z, float imu_angular_z);
    int getTotalWheels(base robot_base);

    float max_rps_;
    float wheels_y_distance_;
    float pwm_res_;
    float wheel_circumference_;
    float robot_circumference_;
    float wheel_radius_;
    int total_wheels_;
    float robot_radius_ = 0.175;
    float enc_wheel_diameter = 0.0985;
    // float enc_wheel_diameter = 4.635 / 100;
    float total_enc_pulse = 1024;
    float enc_robot_circumference = 2 * PI * robot_radius_;
    float enc_wheel_circumference = enc_wheel_diameter * PI;
};

#endif