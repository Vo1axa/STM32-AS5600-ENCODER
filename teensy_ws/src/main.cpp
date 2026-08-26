#include <Arduino.h>
#include <USBHost_t36.h>

USBHost usb;
USBHub hub1(usb);
USBHIDParser hid1(usb);
JoystickController joystick(usb);

// ============================================================
// JOYSTICK CONFIG
// ============================================================
#define AXIS_LX 0   // stick KIRI, sumbu X -> strafe (vy)
#define AXIS_LY 1   // stick KIRI, sumbu Y -> maju/mundur (vx)
#define AXIS_RX 2   // stick KANAN, sumbu X -> rotasi (omega)
#define AXIS_MIN 0
#define AXIS_MAX 255
#define AXIS_CENTER 128
#define AXIS_DEADZONE 15

// ============================================================
// KINEMATIK 3-RODA SWERVE
// ============================================================
#define WHEEL_DISTANCE 0.25f
#define MAX_CMD_SPEED  150.0f

class Kinematic3 {
public:
  struct ModuleState {
    float speed[3];
    float angle[3];
  };

  explicit Kinematic3(float distance) {
    rx_[0] = 0.0f;               ry_[0] = +distance;
    rx_[1] = -distance * 0.8660254f;  ry_[1] = -distance * 0.5f;
    rx_[2] = +distance * 0.8660254f;  ry_[2] = -distance * 0.5f;
  }

  ModuleState calculate(float vx, float vy, float omega) {
    ModuleState s;
    for (int i = 0; i < 3; i++) {
      float vix = vx - omega * ry_[i];
      float viy = vy + omega * rx_[i];
      s.speed[i] = sqrtf(vix * vix + viy * viy);
      s.angle[i] = atan2f(viy, vix);
    }
    return s;
  }

  static float radToDeg(float rad) { return rad * 180.0f / PI; }

  // Steering optimization: minimalkan rotasi module (hindari muter >90 derajat)
  // Jika delta > 90°, balik arah drive dan adjust angle
  void optimizeModule(float desiredSpeed, float desiredAngle, float currentAngle,
                      float &outSpeed, float &outAngle)
  {
    float delta = desiredAngle - currentAngle;

    while (delta > PI)  delta -= 2.0f * PI;
    while (delta < -PI) delta += 2.0f * PI;

    if (fabsf(delta) > PI / 2.0f) {
      outSpeed = -desiredSpeed;
      outAngle = currentAngle + delta - (delta > 0.0f ? PI : -PI);
    } else {
      outSpeed = desiredSpeed;
      outAngle = currentAngle + delta;
    }
  }

private:
  float rx_[3], ry_[3];
};

// ============================================================
// KOMUNIKASI — 1 PACKET, TANPA ID
// Format: [0xAA] [angle:4B] [speed:4B] [checksum:1B] = 10 byte
// ============================================================
#define CMD_HEADER    0xAA
#define SEND_INTERVAL 20   // ms (~50 Hz)

struct __attribute__((packed)) DriveCmd {
  float angle;
  float speed;
};

void sendDriveCmd(float angle, float speed)
{
  DriveCmd cmd = { angle, speed };
  uint8_t buf[sizeof(DriveCmd)];
  memcpy(buf, &cmd, sizeof(cmd));

  uint8_t checksum = 0;
  for (size_t i = 0; i < sizeof(DriveCmd); i++) checksum ^= buf[i];

  Serial3.write(CMD_HEADER);
  Serial3.write(buf, sizeof(DriveCmd));
  Serial3.write(checksum);
}

// ============================================================
// GLOBALS
// ============================================================
Kinematic3 kinematic(WHEEL_DISTANCE);
uint32_t lastButtons = 0;
int currentSpeedMult = 1;
unsigned long lastSend = 0;
float currentAngles[3] = { 0.0f, 0.0f, 0.0f };  // sudut steering sekarang (rad)

// ============================================================
// SETUP & LOOP
// ============================================================
void setup()
{
  Serial.begin(115200);
  Serial3.begin(115200);
  usb.begin();
}

void loop()
{
  usb.Task();
  if (!joystick.available()) return;

  // Tombol B: ganti speed multiplier
  uint32_t buttons = joystick.getButtons();
  bool B_now = buttons & 0x04;
  bool B_before = lastButtons & 0x04;

  if (B_now && !B_before) {
    currentSpeedMult++;
    if (currentSpeedMult > 3) currentSpeedMult = 1;
    Serial.print("SPEED: "); Serial.println(currentSpeedMult);
  }
  lastButtons = buttons;

  // Baca axis
  int rawVx = joystick.getAxis(AXIS_LY) - AXIS_CENTER;
  int rawVy = joystick.getAxis(AXIS_LX) - AXIS_CENTER;
  int rawW  = joystick.getAxis(AXIS_RX) - AXIS_CENTER;

  if (abs(rawVx) < AXIS_DEADZONE) rawVx = 0;
  if (abs(rawVy) < AXIS_DEADZONE) rawVy = 0;
  if (abs(rawW)  < AXIS_DEADZONE) rawW  = 0;

  float halfRange = (float)(AXIS_MAX - AXIS_CENTER);
  float vx = constrain((float)rawVx / halfRange, -1.0f, 1.0f) * MAX_CMD_SPEED * (currentSpeedMult / 3.0f);
  float vy = constrain((float)rawVy / halfRange, -1.0f, 1.0f) * MAX_CMD_SPEED * (currentSpeedMult / 3.0f);
  float w  = constrain((float)rawW  / halfRange, -1.0f, 1.0f) * MAX_CMD_SPEED * (currentSpeedMult / 3.0f);

  // Hitung kinematik 3 roda
  Kinematic3::ModuleState ms = kinematic.calculate(vx, vy, w);

  // Normalize overspeed
  float peak = ms.speed[0];
  if (ms.speed[1] > peak) peak = ms.speed[1];
  if (ms.speed[2] > peak) peak = ms.speed[2];
  if (peak > MAX_CMD_SPEED && peak > 0.0f) {
    float scale = MAX_CMD_SPEED / peak;
    ms.speed[0] *= scale; ms.speed[1] *= scale; ms.speed[2] *= scale;
  }

  float a1 = Kinematic3::radToDeg(ms.angle[0]);
  float a2 = Kinematic3::radToDeg(ms.angle[1]);
  float a3 = Kinematic3::radToDeg(ms.angle[2]);
  float s1 = ms.speed[0];
  float s2 = ms.speed[1];
  float s3 = ms.speed[2];

  // Terapkan steering optimization per module
  float optSpeed[3], optAngle[3];
  for (int i = 0; i < 3; i++) {
    kinematic.optimizeModule(ms.speed[i], ms.angle[i], currentAngles[i],
                             optSpeed[i], optAngle[i]);
    currentAngles[i] = optAngle[i];  // update sudut sekarang
  }

  float oA1 = Kinematic3::radToDeg(optAngle[0]);
  float oA2 = Kinematic3::radToDeg(optAngle[1]);
  float oA3 = Kinematic3::radToDeg(optAngle[2]);
  float oS1 = optSpeed[0];
  float oS2 = optSpeed[1];
  float oS3 = optSpeed[2];

  // Kirim hanya 1 module (module 1 / depan) untuk testing sekarang
  if (millis() - lastSend >= SEND_INTERVAL) {
    lastSend = millis();
    sendDriveCmd(oA1, oS1);  // <-- cuma module 1, sudah optimized

    // Debug 3 module di Serial USB
    Serial.print("M1:"); Serial.print(oA1,1); Serial.print("° "); Serial.print(oS1,1);
    Serial.print(" | M2:"); Serial.print(oA2,1); Serial.print("° "); Serial.print(oS2,1);
    Serial.print(" | M3:"); Serial.print(oA3,1); Serial.print("° "); Serial.print(oS3,1);
    Serial.print(" LX: "); Serial.print(joystick.getAxis(AXIS_LY));
    Serial.print(" LY: "); Serial.print(joystick.getAxis(AXIS_LX));
    Serial.print(" RX: "); Serial.println(joystick.getAxis(AXIS_RX));
  }
}