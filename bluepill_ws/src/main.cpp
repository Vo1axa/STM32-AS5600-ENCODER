#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "as5600.h"

// ---- AS5600 steering sensor on I2C2 (PB10 = SCL, PB11 = SDA) ----
// PB6/PB7 are already used by serial1 (Teensy link), so I2C1 is not available.
// You can move these defines into config.h if you prefer (config.h wins).
#ifndef AS5600_SDA
#define AS5600_SDA PB11
#endif
#ifndef AS5600_SCL
#define AS5600_SCL PB10
#endif
#ifndef AS5600_ZERO_DEG
#define AS5600_ZERO_DEG 0.0f   // rawDeg when the wheel points straight (see 'z' command)
#endif
#ifndef AS5600_INVERT
#define AS5600_INVERT false    // flip if positive PWM makes the angle go DOWN (see 't' command)
#endif

#define DRIVE_CMD_TIMEOUT 500  // ms, auto-stop kalau tidak ada command
#define TICKS_PER_REV 320.0
#define CMD_HEADER 0xAA
#define PWM_MIN_MOVE 130       // PWM minimal biar motor steering mau gerak

volatile long pos[NUM_ENCODERS];
float currentPwm[2];
bool homingDone = false;

// ---- UART command dari Teensy ----
struct __attribute__((packed)) DriveCmd {
  float angle;  // target sudut steering (derajat)
  float speed;  // target speed roda (-150..150)
};

uint8_t rxBuffer[sizeof(DriveCmd) + 1];  // payload + checksum
uint8_t rxIndex = 0;
bool waitingHeader = true;

float targetAngleCmd = 0;
float targetSpeedCmd = 0;
unsigned long lastCmdTime = 0;

template <int j>
void readEncoder();
void setMotor(int cwPin, int ccwPin, float pwmVal);
void homing();
float getAngle();
void goToAngle(float targetAngle);
void readSerialCommand();
void handleDebugCommand();
void testSteeringDirection();

void (*const encoderIsr[NUM_ENCODERS])() = {readEncoder<0>};

HardwareSerial serial1(PB7, PB6);

// AS5600 absolute steering sensor
TwoWire as5600Wire(AS5600_SDA, AS5600_SCL);
AS5600 steeringSensor(as5600Wire, AS5600_ZERO_DEG, AS5600_INVERT);
bool useAs5600 = false;

void setup()
{
  Serial.begin(115200);
  serial1.begin(115200);

  // Steering encoder
  pinMode(MOTOR1_ENCODER_A, INPUT);
  pinMode(MOTOR1_ENCODER_B, INPUT);

  // Steering & drive pins
  pinMode(REED_SWITCH, INPUT_PULLUP);
  pinMode(MOTOR1_IN_A, OUTPUT);
  pinMode(MOTOR1_IN_B, OUTPUT);
  pinMode(MOTOR2_IN_A, OUTPUT);
  pinMode(MOTOR2_IN_B, OUTPUT);

  for (int i = 0; i < NUM_MOTORS; i++)
  {
    pinMode(cw[i], OUTPUT);
    pinMode(ccw[i], OUTPUT);

    // analogWriteResolution(PWM_BITS);
    // analogWrite(cw[i], 0);
    // analogWrite(ccw[i], 0);
  }

  for (int i = 0; i < NUM_ENCODERS; i++)
  {
    pinMode(enca[i], INPUT);
    pinMode(encb[i], INPUT);

    attachInterrupt(digitalPinToInterrupt(enca[i]), encoderIsr[i], RISING);
  }

  // AS5600: absolute angle -> no reed switch homing needed.
  // If it is not found, fall back to the old reed switch + quadrature encoder.
  useAs5600 = steeringSensor.begin();
  if (useAs5600)
  {
    homingDone = true;
    Serial.println("AS5600 OK: using absolute steering angle");
  }
  else
  {
    Serial.print("AS5600 not found / no magnet (status 0x");
    Serial.print(steeringSensor.status(), HEX);
    Serial.println(") -> using reed switch homing + quadrature encoder");
  }
}

void loop()
{
  handleDebugCommand();

  if (useAs5600) steeringSensor.update();

  homing();

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 100)  // print tiap 100ms
  {
    lastPrint = millis();
    Serial.print("pos[0]: ");
    Serial.print(pos[0]);
    Serial.print(" || PWM M1: ");
    Serial.print(currentPwm[0]);
    Serial.print(" | PWM M2: ");
    Serial.print(currentPwm[1]);
    Serial.print(" | angleCmd: ");
    Serial.print(targetAngleCmd);
    Serial.print(" | speedCmd: ");
    Serial.print(targetSpeedCmd);
    if (useAs5600)
    {
      Serial.print(" || AS5600 angle: ");
      Serial.print(steeringSensor.angleDeg(), 1);
      Serial.print(" | dir: ");
      Serial.print(steeringSensor.directionStr());
      Serial.print(" | vel: ");
      Serial.print(steeringSensor.velocityDps(), 0);
      Serial.print(" deg/s");
    }
    Serial.println();
  }

  if (!homingDone) return;

  readSerialCommand();

  if (millis() - lastCmdTime > DRIVE_CMD_TIMEOUT)
  {
    // link putus / belum pernah terima command -> stop total
    setMotor(MOTOR1_IN_A, MOTOR1_IN_B, 0);
    setMotor(MOTOR2_IN_A, MOTOR2_IN_B, 0);
    return;
  }

  // lost the steering sensor while running -> stop everything (don't drive blind)
  if (useAs5600 && !steeringSensor.healthy())
  {
    setMotor(MOTOR1_IN_A, MOTOR1_IN_B, 0);
    setMotor(MOTOR2_IN_A, MOTOR2_IN_B, 0);
    return;
  }

  goToAngle(targetAngleCmd);
  setMotor(MOTOR2_IN_A, MOTOR2_IN_B, targetSpeedCmd);
}

// Serial monitor commands (USB):
//   't' = test: pulse steering motor with +PWM and report which way the angle moved
//   'z' = print the raw angle at the current position (use it for AS5600_ZERO_DEG)
void handleDebugCommand()
{
  while (Serial.available())
  {
    char c = Serial.read();
    if (!useAs5600) continue;

    if (c == 'z')
    {
      Serial.print(">> Raw AS5600 angle here = ");
      Serial.print(steeringSensor.rawDeg(), 2);
      Serial.println(" deg. Put this in AS5600_ZERO_DEG (wheel must point straight).");
    }
    else if (c == 't')
    {
      testSteeringDirection();
    }
  }
}

// WARNING: moves the steering motor for ~0.2 s. Keep the module free to turn.
void testSteeringDirection()
{
  steeringSensor.update();
  float before = steeringSensor.angleDeg();

  setMotor(MOTOR1_IN_A, MOTOR1_IN_B, PWM_MIN_MOVE);
  delay(200);
  setMotor(MOTOR1_IN_A, MOTOR1_IN_B, 0);
  delay(100);

  steeringSensor.update();
  float moved = fmod(steeringSensor.angleDeg() - before + 540.0f, 360.0f) - 180.0f;

  Serial.print(">> Positive PWM changed the angle by ");
  Serial.print(moved, 1);
  Serial.println(" deg");

  if (moved > 1.0f)
    Serial.println(">> Direction OK (keep AS5600_INVERT as it is)");
  else if (moved < -1.0f)
    Serial.println(">> Direction REVERSED: flip AS5600_INVERT, upload, run 't' again");
  else
    Serial.println(">> No movement detected (motor blocked, or PWM too low?)");
}

void readSerialCommand()
{
  while (serial1.available())
  {
    uint8_t b = serial1.read();

    if (waitingHeader)
    {
      if (b == CMD_HEADER)
      {
        waitingHeader = false;
        rxIndex = 0;
      }
      continue;
    }

    rxBuffer[rxIndex++] = b;

    if (rxIndex == sizeof(DriveCmd) + 1)
    {
      uint8_t checksum = 0;
      for (size_t i = 0; i < sizeof(DriveCmd); i++) checksum ^= rxBuffer[i];

      if (checksum == rxBuffer[sizeof(DriveCmd)])
      {
        DriveCmd cmd;
        memcpy(&cmd, rxBuffer, sizeof(DriveCmd));
        targetAngleCmd = cmd.angle;
        targetSpeedCmd = cmd.speed;
        lastCmdTime = millis();
      }
      // checksum salah -> paket dibuang, tunggu header berikutnya

      waitingHeader = true;
      rxIndex = 0;
    }
  }
}

void homing()
{
  if (homingDone) return;

  if (digitalRead(REED_SWITCH) == HIGH)
  {
    setMotor(MOTOR1_IN_A, MOTOR1_IN_B, -50);
  }
  else
  {
    setMotor(MOTOR1_IN_A, MOTOR1_IN_B, 0);
    pos[0] = 0;
    homingDone = true;
  }
}

float getAngle()
{
  if (useAs5600) return steeringSensor.angleDeg();   // absolute, 0..360
  return (pos[0] / TICKS_PER_REV) * 360.0;           // fallback: quadrature encoder
}

void goToAngle(float targetAngle)
{
  float currentAngle = getAngle();
  float error = targetAngle - currentAngle;

  // bungkus error ke rentang [-180, 180] biar selalu ambil jalur terpendek
  error = fmod(error + 540.0f, 360.0f) - 180.0f;

  if (fabs(error) <= 1.0)
  {
    setMotor(MOTOR1_IN_A, MOTOR1_IN_B, 0);
    return;
  }

  float kp = 5.0;  // tuning gain, sesuain sendiri
  float pwm = kp * error;

  pwm = constrain(pwm, -255, 255);  // batasi speed max

  setMotor(MOTOR1_IN_A, MOTOR1_IN_B, pwm);
}

template <int j>
void readEncoder()
{
  int b = digitalRead(encb[j]);
  if (b > 0)
  {
    pos[j]++;
  }
  else
  {
    pos[j]--;
  }
}

void setMotor(int cwPin, int ccwPin, float pwmVal)
{
  if (cwPin == MOTOR1_IN_A) {
    currentPwm[0] = pwmVal;
  } else if (cwPin == MOTOR2_IN_A) {
    currentPwm[1] = pwmVal;
  }

  if (pwmVal > 0)
  {
    analogWrite(cwPin, fabs(pwmVal));
    analogWrite(ccwPin, 0);
  }
  else if (pwmVal < 0)
  {
    analogWrite(cwPin, 0);
    analogWrite(ccwPin, fabs(pwmVal));
  }
  else
  {
    analogWrite(cwPin, 0);
    analogWrite(ccwPin, 0);
  }
}