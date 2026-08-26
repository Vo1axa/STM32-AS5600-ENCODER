#ifndef SWERVE_CONFIG_H
#define SWERVE_CONFIG_H

// uncomment the base you're building
// #define ROBOT_2 DIFFERENTIAL_DRIVE       // 2WD and Tracked robot w/ 2 motors
// #define ROBOT_2 SKID_STEER              // 4WD robot
// #define ROBOT_2 MECANUM                // Mecanum drive robot
#define ROBOT_2 SWERVE
// #define ROBOT_1 OMNI // Omniwheel robot

// uncomment the motor driver you're using
// #define USE_GENERIC_2_IN_MOTOR_DRIVER         // Motor drivers with 2 Direction Pins(INA, INB) and 1 PWM(ENABLE) pin ie. L298, L293, VNH5019
// #define USE_GENERIC_1_IN_MOTOR_DRIVER        // Motor drivers with 1 Direction Pin(INA) and 1 PWM(ENABLE) pin.
#define USE_BTS7960_MOTOR_DRIVER // BTS7970 Motor Driver
// #define USE_ESC_MOTOR_DRIVER               // Motor ESC for brushless motors

// uncomment the IMU you're using
// #define USE_GY85_IMU
// #define USE_MPU6050_IMU
// #define USE_MPU9150_IMU
// #define USE_MPU9250_IMU
// #define USE_BNO055_IMU

#define K_P 50      // 55
#define K_I 0 // 0.045454545
#define K_D 0             // 0

// define your robot' specs here
#define MOTOR_MAX_RPS 8.4               // motor's max RPM
#define MAX_RPS_RATIO 0.85              // max RPM allowed for each MAX_RPM_ALLOWED = MOTOR_MAX_RPM * MAX_RPM_RATIO
#define MOTOR_OPERATING_VOLTAGE 24      // motor's operating voltage (used to calculate max RPM)
#define MOTOR_POWER_MAX_VOLTAGE 24      // max voltage of the motor's power source (used to calculate max RPM)
#define MOTOR_POWER_MEASURED_VOLTAGE 24 // current voltage reading of the power connected to the motor (used for calibration)
#define COUNTS_PER_REV1 320             // steering encoder's no of ticks per rev
#define COUNTS_PER_REV2 1000            // wheel2 encoder's no of ticks per rev
#define COUNTS_PER_REV3 1000            // wheel3 encoder's no of ticks per rev
#define COUNTS_PER_REV4 1000            // wheel4 encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.0985           // wheel's diameter in meters
#define ROBOT_DIAMETER 0.80             // 800       // distance between left and right wheels
#define ROBOT_RADIUS 0.40               // 400
#define PWM_BITS 8
// #define PWM_FREQUENCY 25000 // 1000

// // INVERT ENCODER COUNTS
// #define MOTOR1_ENCODER_INV false
// #define MOTOR2_ENCODER_INV false
// #define MOTOR3_ENCODER_INV false
// #define MOTOR4_ENCODER_INV false

// // INVERT MOTOR DIRECTIONS
// #define MOTOR1_INV false
// #define MOTOR2_INV false
// #define MOTOR3_INV false
// #define MOTOR4_INV false

// #define IR_PIN PA6

// ENCODER PINS
#define MOTOR1_ENCODER_A PB14
#define MOTOR1_ENCODER_B PB15

// #define MOTOR2_ENCODER_A PB1
// #define MOTOR2_ENCODER_B PB10

// MOTOR PINS
#define MOTOR1_IN_A PA6
#define MOTOR1_IN_B PB1

#define MOTOR2_IN_A PA7
#define MOTOR2_IN_B PB0

#define MOTOR3_IN_A PB8
#define MOTOR3_IN_B PB9

//PROXY PINS
#define PROXY1_LEFT PA15 //PB9
#define PROXY1_RIGHT PB3 //PB8

#define PROXY2_LEFT PB5
#define PROXY2_RIGHT PB4

//LIMIT PINS
#define LIMIT1 A0
#define LIMIT2 A1
#define LIMIT3 A2
#define LIMIT4 A3
#define REED_SWITCH PA8

//SOLENOID PINS
#define solenoidHolder A5
#define solenoidGrip PB11
#define solenoidExtend A4

#define TOF_MIN_DIST 50
#define TOF_MAX_DIST 74
#define TOF_JUMP_MAX 100

// IMU 18,19 / SDA0,SCL0
// IMU 16,17 / SDA1,SCL1

#define NUM_MOTORS 3
#define NUM_ENCODERS 1

const int cw[NUM_MOTORS] = {
    MOTOR1_IN_A,
    MOTOR2_IN_A,
    MOTOR3_IN_A};

const int ccw[NUM_MOTORS] = {
    MOTOR1_IN_B,
    MOTOR2_IN_B,
    MOTOR3_IN_B};

// encoder in array
const int enca[NUM_ENCODERS] = {
    MOTOR1_ENCODER_A};
const int encb[NUM_ENCODERS] = {
    MOTOR1_ENCODER_B};

// const int pwm[4] ={-1,-1,-1,-1};
#define PWM_MAX pow(2, PWM_BITS) - 1
#define PWM_MIN -PWM_MAX

#endif