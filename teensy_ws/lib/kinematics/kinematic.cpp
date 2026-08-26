#include "kinematic.h"
#include <cmath>

// ============================================================
// Constructor — Layout segitiga sama sisi
//   module 1 = Depan        (rx = 0,               ry = +distance_from_center)
//   module 2 = Kiri Belakang(rx = -distance*√3/2,  ry = -distance/2)
//   module 3 = Kanan Belakang(rx = +distance*√3/2, ry = -distance/2)
// ============================================================
Kinematic::Kinematic(base robot_base, int motor_max_rpm, float max_rpm_ratio,
                     float motor_operating_voltage, float motor_power_max_voltage,
                     float wheel_diameter, float wheel_distance_from_center)
    : base_platform_(robot_base),
      wheel_radius_(wheel_diameter / 2.0f),
      wheel_circumference_(M_PI * wheel_diameter),
      wheel_distance_(wheel_distance_from_center)
{
    motor_power_max_voltage = std::clamp(motor_power_max_voltage, 0.0f, motor_operating_voltage);
    max_rps_ = ((motor_power_max_voltage / motor_operating_voltage) * motor_max_rpm) * max_rpm_ratio;

    // Posisi roda relatif terhadap pusat chassis (meter)
    rx_[0] = 0.0f;
    ry_[0] = +wheel_distance_;

    rx_[1] = -wheel_distance_ * 0.8660254f;  // -R * cos(30°) = -R * √3/2
    ry_[1] = -wheel_distance_ * 0.5f;        // -R * sin(30°)

    rx_[2] = +wheel_distance_ * 0.8660254f;  // +R * √3/2
    ry_[2] = -wheel_distance_ * 0.5f;

    // Untuk forward kinematics: sum of rx^2 + ry^2
    sum_r2_ = 0.0f;
    for (int i = 0; i < 3; i++) {
        sum_r2_ += (rx_[i] * rx_[i] + ry_[i] * ry_[i]);
    }
}

// ============================================================
// INVERSE KINEMATICS
// chassis speed (vx, vy, omega) -> 3 module speed (rps) + angle (rad)
// ============================================================
Kinematic::module_state Kinematic::calculateModuleStates(float linear_x, float linear_y, float angular_z)
{
    float vix[3], viy[3], speed_mps[3], angle[3];

    for (int i = 0; i < 3; i++)
    {
        vix[i] = linear_x - angular_z * ry_[i];
        viy[i] = linear_y + angular_z * rx_[i];

        speed_mps[i] = sqrtf(vix[i] * vix[i] + viy[i] * viy[i]);
        angle[i]     = atan2f(viy[i], vix[i]);
    }

    Kinematic::module_state state;

    state.speed1 = std::clamp(speed_mps[0] / wheel_circumference_, -max_rps_, max_rps_);
    state.angle1 = angle[0];

    state.speed2 = std::clamp(speed_mps[1] / wheel_circumference_, -max_rps_, max_rps_);
    state.angle2 = angle[1];

    state.speed3 = std::clamp(speed_mps[2] / wheel_circumference_, -max_rps_, max_rps_);
    state.angle3 = angle[2];

    // Module 4 tidak dipakai di 3-wheel, set ke 0
    state.speed4 = 0.0f;
    state.angle4 = 0.0f;

    return state;
}

Kinematic::module_state Kinematic::getModuleStates(float linear_x, float linear_y, float angular_z)
{
    return calculateModuleStates(linear_x, linear_y, angular_z);
}

// ============================================================
// FORWARD KINEMATICS
// 3 module speed (rps) + angle (rad) -> chassis speed (vx, vy, omega)
//
// Least-squares closed form untuk 3 roda:
//   vx    = (v1x + v2x + v3x) / 3
//   vy    = (v1y + v2y + v3y) / 3
//   omega = Σ(rx_i * v_iy - ry_i * v_ix) / Σ(rx_i² + ry_i²)
// ============================================================
Kinematic::velocities Kinematic::getVelocities(float speed1, float angle1,
                                               float speed2, float angle2,
                                               float speed3, float angle3,
                                               float speed4, float angle4)
{
    (void)speed4;  // Unused in 3-wheel
    (void)angle4;

    // rps -> m/s
    float v1x = (speed1 * wheel_circumference_) * cosf(angle1);
    float v1y = (speed1 * wheel_circumference_) * sinf(angle1);

    float v2x = (speed2 * wheel_circumference_) * cosf(angle2);
    float v2y = (speed2 * wheel_circumference_) * sinf(angle2);

    float v3x = (speed3 * wheel_circumference_) * cosf(angle3);
    float v3y = (speed3 * wheel_circumference_) * sinf(angle3);

    Kinematic::velocities vel;

    // vx, vy: rata-rata (least-squares untuk translasi)
    vel.linear_x = (v1x + v2x + v3x) / 3.0f;
    vel.linear_y = (v1y + v2y + v3y) / 3.0f;

    // omega: least-squares untuk rotasi
    float cross_sum = (rx_[0] * v1y - ry_[0] * v1x)
                    + (rx_[1] * v2y - ry_[1] * v2x)
                    + (rx_[2] * v3y - ry_[2] * v3x);

    vel.angular_z = cross_sum / sum_r2_;

    return vel;
}

// ============================================================
// Steering optimization (sama persis, dipanggil per module)
// ============================================================
void Kinematic::optimizeModule(float desiredSpeed, float desiredAngle, float currentAngle,
                               float &outSpeed, float &outAngle)
{
    float delta = desiredAngle - currentAngle;

    while (delta > M_PI)  delta -= 2.0f * M_PI;
    while (delta < -M_PI) delta += 2.0f * M_PI;

    if (fabsf(delta) > M_PI / 2.0f)
    {
        outSpeed = -desiredSpeed;
        outAngle = currentAngle + delta - (delta > 0.0f ? M_PI : -M_PI);
    }
    else
    {
        outSpeed = desiredSpeed;
        outAngle = currentAngle + delta;
    }
}

float Kinematic::getMaxRPS()
{
    return max_rps_;
}

float Kinematic::toRad(float deg)
{
    return deg * M_PI / 180.0f;
}

float Kinematic::toDeg(float rad)
{
    return rad * 180.0f / M_PI;
}