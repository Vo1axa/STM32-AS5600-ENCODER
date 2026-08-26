#ifndef KINEMATIC_H
#define KINEMATIC_H

// #include "Arduino.h"
#include <cmath>
#include <algorithm>

// ============================================================
// Kinematic - Swerve Drive version
// Struktur & naming convention mengikuti versi mecanum/skid-steer
// sebelumnya, tapi logic matematika mengikuti inverse/forward
// kinematics matrix swerve (persamaan 11.14 - 11.17).
// ============================================================
class Kinematic
{
public:
    enum base
    {
        SWERVE_DRIVE
    };

    base base_platform_;

    // Kecepatan tiap module (hasil inverse kinematics)
    // speed  -> dikirim ke DRIVE MOTOR (m/s, sudah dikonversi ke rps di getRPS jika perlu)
    // angle  -> dikirim ke STEERING MOTOR (rad)
    struct module_state
    {
        float speed1, angle1;
        float speed2, angle2;
        float speed3, angle3;
        float speed4, angle4;
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

    // Constructor
    // wheel_diameter    : diameter wheel (m), untuk convert speed (m/s) -> rps
    // wheelbase_length  : jarak antar wheel depan-belakang (m)
    // wheelbase_width   : jarak antar wheel kiri-kanan (m)
    Kinematic(base robot_base, int motor_max_rpm, float max_rpm_ratio,
              float motor_operating_voltage, float motor_power_max_voltage,
              float wheel_diameter, float wheelbase_length, float wheelbase_width);

    // INVERSE KINEMATICS: chassis speed (vx, vy, omega) -> speed+angle tiap module
    module_state getModuleStates(float linear_x, float linear_y, float angular_z);

    // FORWARD KINEMATICS: speed+angle tiap module -> chassis speed (vx, vy, omega)
    // dipakai untuk odometry
    velocities getVelocities(float speed1, float angle1,
                              float speed2, float angle2,
                              float speed3, float angle3,
                              float speed4, float angle4);

    // Optimisasi steering: minimalkan rotasi module (hindari muter >90 derajat)
    // currentAngle -> sudut wheel sekarang (rad, dari encoder steering)
    // desiredSpeed/desiredAngle -> target hasil inverse kinematics
    // outSpeed/outAngle -> hasil setelah optimisasi (referensi, diisi function ini)
    void optimizeModule(float desiredSpeed, float desiredAngle, float currentAngle,
                         float &outSpeed, float &outAngle);

    float getMaxRPS();
    float toRad(float deg);
    float toDeg(float rad);

private:
    module_state calculateModuleStates(float linear_x, float linear_y, float angular_z);

    float max_rps_;
    float wheel_radius_;
    float wheel_circumference_;
    float wheelbase_length_;   // jarak depan-belakang (total, bukan setengah)
    float wheelbase_width_;    // jarak kiri-kanan (total, bukan setengah)
    float half_length_;
    float half_width_;
};

#endif